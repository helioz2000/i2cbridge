/**
 * @file i2cbridge.cpp
 *
 * https://github.com/helioz2000/i2cbridge
 *
 * Author: Erwin Bejsta
 * August 2020
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>

#include <string>
#include <iostream>

#include <libconfig.h++>
#include <mosquitto.h>

#include "i2ctag.h"
#include "mqtt.h"
#include "hardware.h"
#include "i2cbridge.h"
#include "hardware/vimon.h"
#include "hardware/MCP9808.h"
#include "hardware/ADS1115.h"

using namespace std;
using namespace libconfig;

const char *build_date_str = __DATE__ " " __TIME__;
const int version_major = 1;
const int version_minor = 2;

#define CFG_DEFAULT_FILENAME "i2cbridge.cfg"
#define CFG_DEFAULT_FILEPATH "/etc/"

#define MAIN_LOOP_INTERVAL_MINIMUM 50     // milli seconds
#define MAIN_LOOP_INTERVAL_MAXIMUM 2000   // milli seconds

#define MQTT_BROKER_DEFAULT "127.0.0.1"
#define MQTT_CLIENT_ID "i2cbridge"
#define MQTT_RECONNECT_INTERVAL 10			// seconds between reconnect attempts

// Calibration data for power management analogs
#define PDU_BAT_V_SCALE_FACTOR 4.0	//Divider 3k/1k (16V->4V)
//#define PDU_I_V_SCALE_FACTOR 1.252947   //Divider 750K/3K (5V -> 4V)

// Zero offset adjustment (if ACS712 is not exactly at 50% of Vs)
#define PDU_I1_ZERO_V_OFFSET 5.0
#define PDU_I2_ZERO_V_OFFSET 5.0
#define PDU_I3_ZERO_V_OFFSET 5.0
#define PDU_I4_ZERO_V_OFFSET 5.0
#define PDU_I5_ZERO_V_OFFSET -65.0		// mV offset for zero point 

#define PDU_I1_V_SCALE_FACTOR 1.0
#define PDU_I2_V_SCALE_FACTOR 1.0
#define PDU_I3_V_SCALE_FACTOR 1.0
#define PDU_I4_V_SCALE_FACTOR 1.0
#define PDU_I5_V_SCALE_FACTOR 1.0

//#define PDU_I1_ZERO_OFFSET 2547.5	// mV for zero point
//#define PDU_I2_ZERO_OFFSET 2527.0	// mV for zero point
//#define PDU_I3_ZERO_OFFSET 2534.0
//#define PDU_I4_ZERO_OFFSET 2536.5
//#define PDU_I5_ZERO_OFFSET 2500.0
#define PDU_5V_SCALE_FACTOR 1.33032226	// Divider 0.9725/2.9458K (5449mV -> 4096mV)

// ACS712 V->I conversion factors
#define ACS712_30A_MV_PER_A 67		//mV per A for ACS712 current sensor
#define ACS712_20A_MV_PER_A 100
#define ACS712_5A_MV_PER_A 188		// 


static string cpu_temp_topic = "";
static string cfgFileName;
static string processName;
bool exitSignal = false;
bool debugEnabled = false;
bool runningAsDaemon = false;
char *info_label_text;

bool mqttDebugEnabled = false;
time_t mqtt_connect_time = 0;			// time the connection was initiated
time_t mqtt_next_connect_time = 0;		// time when next connect is scheduled
bool mqtt_connection_in_progress = false;
bool mqtt_retain_default = false;

useconds_t mainloopinterval = 250;	// milli seconds
struct timespec lastAccTime;		// last accumulation run
double accPwr;						// power readout accumulator (reset)
double accPwrChg, accPwrDsc;		// power accumulator (no reset)

updatecycle *updateCycles = NULL;	// array of update cycle definitions
I2Ctag *i2cReadTags = NULL;			// array of all I2C tags

int i2cDebugLevel = 0;
int i2cTagCount = -1;
uint32_t i2cTransactionDelay = 0;	// delay between modbus transactions
#define I2C_DEVICEID_MAX 254		// highest permitted I2C device ID
#define I2C_DEVICEID_MIN 1			// lowest permitted I2C device ID

#pragma mark Proto types
void subscribe_tags(void);
void mqtt_connection_status(bool status);
void mqtt_topic_update(const struct mosquitto_message *message);
void mqtt_subscribe_tags(void);
void setMainLoopInterval(int newValue);
bool i2c_read_process();
float i2c_pdu_voltage(int channel);
bool mqtt_publish_tag(I2Ctag *tag);
void mqtt_clear_tags(bool publish_noread, bool clear_retain);

MQTT mqtt(MQTT_CLIENT_ID);
Config cfg;			// config file
Hardware hw(false);	// no screen

// VI montoring board
VImon vimon;

// ADCs
ADS1115 pwr_adc1(ADS1115_ADDRESS_ADDR_GND);	// ADC 1 on power management board
ADS1115 pwr_adc2(ADS1115_ADDRESS_ADDR_VDD);	// ADC 2 on power management board
// ..._ADDR_GND / VDD / SDA / SCL


// Temperature sensors
//#define TEMP_ENV_I2C_ADDR 0x19
//MCP9808 tmp_env(TEMP_ENV_I2C_ADDR);
//MCP9808 tmp_rack(0x19);

/**
 * log to console and syslog for daemon
 */
template<typename... Args> void log(int priority, const char * f, Args... args) {
	if (runningAsDaemon) {
		syslog(priority, f, args...);
	} else {
		fprintf(stderr, f, args...);
		fprintf(stderr, "\n");
	}
}

/**
 * Handle OS signals
 */
void sigHandler(int signum)
{
	char signame[10];
	switch (signum) {
		case SIGTERM:
			strcpy(signame, "SIGTERM");
			break;
		case SIGHUP:
			strcpy(signame, "SIGHUP");
			break;
		case SIGINT:
			strcpy(signame, "SIGINT");
			break;

		default:
			break;
	}

	log(LOG_INFO, "Received %s", signame);
	exitSignal = true;
}

void timespec_diff(struct timespec *start, struct timespec *stop, struct timespec *result) {
	if ((stop->tv_nsec - start->tv_nsec) < 0) {
		result->tv_sec = stop->tv_sec - start->tv_sec - 1;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
	} else {
		result->tv_sec = stop->tv_sec - start->tv_sec;
	result->tv_nsec = stop->tv_nsec - start->tv_nsec;
	}
	return;
}

void timespec_set(struct timespec *src, struct timespec *dst) {
	dst->tv_nsec = src->tv_nsec;
	dst->tv_sec = src->tv_sec;
}

#pragma mark -- Config File functions

/**
 * Read configuration file.
 * @return true if success
 */
bool readConfig (void)
{
//	if (i2cDebugLevel > 0) {
//		printf("%s\n", __func__);
//		fflush(stdout);
//	}

	// Read the file. If there is an error, report it and exit.
	try
	{
		cfg.readFile(cfgFileName.c_str());
	}
	catch(const FileIOException &fioex)
	{
		std::cerr << "I/O error while reading file <" << cfgFileName << ">." << std::endl;
		return false;
	}
	catch(const ParseException &pex)
	{
		std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
				<< " - " << pex.getError() << std::endl;
		return false;
	}

	//log (LOG_INFO, "CFG file read OK");
	if (i2cDebugLevel > 0) {
		printf("%s ", __func__);
		std::cerr << cfgFileName << " read OK" <<endl;
		fflush(stdout);
	}

	try {
		setMainLoopInterval(cfg.lookup("mainloopinterval"));
	} catch (const SettingNotFoundException &excp) {
	;
	} catch (const SettingTypeException &excp) {
		std::cerr << "Error in config file <" << excp.getPath() << "> is not an integer" << std::endl;
		return false;
	}

	// Read MQTT broker from config
	try {
		mqtt.setBroker(cfg.lookup("mqtt.broker"));
	} catch (const SettingNotFoundException &excp) {
		mqtt.setBroker(MQTT_BROKER_DEFAULT);
	} catch (const SettingTypeException &excp) {
		std::cerr << "Error in config file <" << excp.getPath() << "> is not a string" << std::endl;
		return false;
	}

	try {
		mqtt.setUsername(cfg.lookup("mqtt.username"));
	} catch (const SettingNotFoundException &excp) {
	;
	} catch (const SettingTypeException &excp) {
 		std::cerr << "Error in config file <" << excp.getPath() << "> is not a string" << std::endl;
		return false;
 	}

	try {
		mqtt.setPassword(cfg.lookup("mqtt.password"));
	} catch (const SettingNotFoundException &excp) {
	;
	} catch (const SettingTypeException &excp) {
		std::cerr << "Error in config file <" << excp.getPath() << "> is not a string" << std::endl;
		return false;
	}

//	if (i2cDebugLevel > 0) {
//		printf("%s: Done\n", __func__);
//		fflush(stdout);
//	}
	return true;
}

/**
 * Get integer value from config file
 */
bool cfg_get_int(const std::string &path, int &value) {
	if (!cfg.lookupValue(path, value)) {
		std::cerr << "Error in config file <" << path << ">" << std::endl;
		return false;
	}
	return true;
}

/**
 * Get string value from config file
 */
bool cfg_get_str(const std::string &path, std::string &value) {
	if (!cfg.lookupValue(path, value)) {
		std::cerr << "Error in config file <" << path << ">" << std::endl;
		return false;
	}
	return true;
}

#pragma mark -- Processing

/**
 * Process all variables
 * @return true if at least one variable was processed
 * Note: the return value from this function is used
 * to measure processing time
 */
bool process() {
	bool retval = false;
	if (mqtt.isConnected()) {
		if (i2c_read_process()) retval = true;
	}
	return retval;
}

/** Process accumulator values
 * called at regulat intervals to integrate accumulator values
 * @param
 * @return true on success
 */
bool processAccumulators () {
	int readResult;
	float milliVolts, milliAmps;
	double Wh, W, elapsedseconds;
	struct timespec thistime, elapsedtime;

	// read Voltage and Current
	readResult = vimon.getMilliVolts(0, &milliVolts);
	if (readResult != 0) return false;
	readResult = vimon.getBipolarMilliAmps(&milliAmps);
	if (readResult != 0) return false;
	// calculate time since last call
	clock_gettime(CLOCK_MONOTONIC, &thistime);
	timespec_diff(&lastAccTime, &thistime, &elapsedtime);
	timespec_set(&thistime, &lastAccTime);		// update lassAccTime
	W = (milliVolts * milliAmps) * 1.0e-6;		// Watts
	// tiem since last measurement
	elapsedseconds = (double)elapsedtime.tv_sec + (double)elapsedtime.tv_nsec * 1.0e-9;
	Wh = (W / 3600) * elapsedseconds;
	// add Ws to accumulator
	accPwr += Wh;
	if (Wh > 0.0) {
		accPwrChg += Wh;
	} else {
		accPwrDsc += abs(Wh);
	}
//	printf("%s - %f %f %f\n", __func__, accPwr, accPwrCount, Ws);
	return true;
}

bool init_values(void)
{
	char info1[80], info2[80], info3[80], info4[80];

    // get hardware info
    hw.get_model_name(info1, sizeof(info1));
    hw.get_os_name(info2, sizeof(info2));
    hw.get_kernel_name(info3, sizeof(info3));
    hw.get_ip_address(info4, sizeof(info4));
    info_label_text = (char *)malloc(strlen(info1) +strlen(info2) +strlen(info3) +strlen(info4) +5);
    sprintf(info_label_text, "%s\n%s\n%s\n%s", info1, info2, info3, info4);
    if (!runningAsDaemon) {
	    printf(info_label_text);
        }
    //printf(info_label_text);
    return true;
}

#pragma mark MQTT

/** Initialise the tag database (tagstore)
 * @return false on failure
 */
bool mqtt_init_tags(void) {
	std::string strValue;

	if (!cfg.exists("mqtt_tags")) {	// optional
		log(LOG_NOTICE,"configuration - parameter \"mqtt_tags\" does not exist");
		return false;
		}
	return true;
}

void mqtt_connect(void) {
	if (mqttDebugEnabled)
		printf("%s - attempting to connect to mqtt broker %s.\n", __func__, mqtt.broker());
	mqtt.connect();
	mqtt_connection_in_progress = true;
	mqtt_connect_time = time(NULL);
	mqtt_next_connect_time = 0;
	//printf("%s - Done\n", __func__);
}

/**
 * Initialise the MQTT broker and register callbacks
 */
bool mqtt_init(void) {
	//if (!runningAsDaemon) printf("%s\n", __FUNCTION__);
	bool bValue;
	if (!runningAsDaemon) {
		if (cfg.lookupValue("mqtt.debug", bValue)) {
			mqttDebugEnabled = bValue;
			mqtt.setConsoleLog(mqttDebugEnabled);
			if (mqttDebugEnabled) printf("%s - mqtt debug enabled\n", __func__);
		}
	}
	if (cfg.lookupValue("mqtt.retain_default", bValue))
		mqtt_retain_default = bValue;
	mqtt.registerConnectionCallback(mqtt_connection_status);
	mqtt.registerTopicUpdateCallback(mqtt_topic_update);
	mqtt_connect();
	return true;
}

/**
 * Subscribe tags to MQTT broker
 * Iterate over tag store and process every "subscribe" tag
 */
void mqtt_subscribe_tags(void) {
	//mqtt.unsubscribe("vk2ray/pwr/pl20/batv");

	//printf("%s %s - Start\n", __FILE__, __func__);
/*	Tag* tp = ts.getFirstTag();
	while (tp != NULL) {
		if (tp->isSubscribe()) {
			//printf("%s %s: %s\n", __FILE__, __func__, tp->getTopic());
			mqtt.subscribe(tp->getTopic());
		}
		tp = ts.getNextTag();
	}
	//printf("%s - Done\n", __func__);
*/
}

/**
 * callback function for MQTT
 * MQTT notifies a change in connection status by calling this function
 * This function is registered with MQTT during initialisation
 */
void mqtt_connection_status(bool status) {
	//printf("%s %s - %d\n", __FILE__, __func__, status);
	// subscribe tags when connection is online
	if (status) {
		log(LOG_INFO, "Connected to MQTT broker [%s]", mqtt.broker());
		mqtt_next_connect_time = 0;
		mqtt_connection_in_progress = false;
		mqtt.setRetain(mqtt_retain_default);
		mqtt_subscribe_tags();
	} else {
		if (mqtt_connection_in_progress) {
			mqtt.disconnect();
			// Note: the timeout is determined by OS network stack
			unsigned long timeout = time(NULL) - mqtt_connect_time;
			log(LOG_INFO, "mqtt connection timeout after %lds", timeout);
			mqtt_connection_in_progress = false;
		} else {
			log(LOG_WARNING, "Disconnected from MQTT broker [%s]", mqtt.broker());
		}
		// trigger reconnect unless we are exiting
		if (!exitSignal) {
			mqtt_next_connect_time = time(NULL) + MQTT_RECONNECT_INTERVAL;	// current time
			log(LOG_INFO, "mqtt reconnect scheduled in %d seconds", MQTT_RECONNECT_INTERVAL);
		}
	}
	//printf("%s %s - done\n", __FILE__, __func__);
}

/**
 * callback function for MQTT
 * MQTT notifies when a subscribed topic has received an update
 * this function will udate the corresponding tag
 * @param message: mqtt message
 */
void mqtt_topic_update(const struct mosquitto_message *message) {
//	if(mqttDebugEnabled) {
//		printf("%s %s - %s %s\n", __FILE__, __func__, message->topic, (const char*)message->payload );
//	};
/*	Tag *tp = ts.getTag(message->topic);
	if (tp == NULL) {
		fprintf(stderr, "%s: <%s> not  in ts\n", __func__, message->topic);
	} else {
		tp->setValueIsRetained(message->retain);
		if (message->payload != NULL) {
			tp->setValue((const char*)message->payload);	// This will trigger a callback to modbus_write_request
		}
	}*/
}

/**
 * Publish tag to MQTT
 * @param tag: I2C tag to publish
 *
 */
bool mqtt_publish_tag(I2Ctag *tag) {
	if(mqttDebugEnabled) {
		printf("%s %s - %s\n", __FILE__, __func__, tag->getTopic());
	}
	if (!mqtt.isConnected()) return false;
	if (tag->getTopicString().empty()) return true;	// don't publish if topic is empty
	// Publish value if read was OK
	if (!tag->isNoread()) {
		mqtt.publish(tag->getTopic(), tag->getFormat(), tag->getScaledValue(), tag->getPublishRetain());
		//printf("%s %s - %s \n", __FILE__, __FUNCTION__, tag->getTopic());
		return true;
	}
	//printf("%s - NoRead: %s \n", __FUNCTION__, tag->getTopic());
	// Handle Noread
	if (!tag->noReadIgnoreExceeded()) return true;		// ignore noread, do nothing
	// noreadignore is exceeded, need to take action
	switch (tag->getNoreadAction()) {
	case 0:	// publish null value
		mqtt.clear_retained_message(tag->getTopic());
		break;
	case 1:	// publish noread value
		mqtt.publish(tag->getTopic(), tag->getFormat(), tag->getNoreadValue(), tag->getPublishRetain());
		break;
	default:
		// do nothing (default, -1)
		break;
	}

	return true;
}

/**
 * Publish noread value to all tags (normally done on program exit)
 * @param publish_noread: publish the "noread" value of the tag
 * @param clear_retain: clear retained value from broker's persistance store
 */
void mqtt_clear_tags(bool publish_noread = true, bool clear_retain = true) {

	int index = 0, tagIndex = 0;
	int *tagArray;
	I2Ctag i2cTag;
	//printf("%s %s", __FILE__, __func__);

	// Iterate over all update cycles
	//mqtt.setRetain(false);
	while (updateCycles[index].ident >= 0) {
		// ignore if cycle has no tags to process
		if (updateCycles[index].tagArray == NULL) {
			index++; continue;
		}
		// get array for tags
		tagArray = updateCycles[index].tagArray;
		// read each tag in the array
		tagIndex = 0;
		while (tagArray[tagIndex] >= 0) {
			i2cTag = i2cReadTags[tagArray[tagIndex]];
			if (publish_noread) {}
				mqtt.publish(i2cTag.getTopic(), i2cTag.getFormat(), i2cTag.getNoreadValue(), i2cTag.getPublishRetain());
				//mqtt_publish_tag(mbTag, true);			// publish noread value
			if (clear_retain) {}
				mqtt.clear_retained_message(i2cTag.getTopic());	// clear retained status
			tagIndex++;
		}
		index++;
	}	// while

}

#pragma mark I2C

/** Read analog current input based on ACS712
 * @param rawAnalog the raw analog value from ADS1115
 * @param vScaleFactor: scaling of ADC reading (e.g. voltage divider)
 * @param mVperA: mV per A from ACS712 data sheet
 * @param vOffset: compensation if ACS712 zero point is not exactly 50% of 5V supply
 * @return current in mA
 * Note: the zero point is calculated by measuring the ACS712 supply voltage
 *		 and using 50% of that voltage measurement as the zero point.
 *
 */
float current_reading(int rawAnalog, double vScaleFactor, float mVperA, double vOffset =  0.0) {
    double mVscaled, mVunscaled, mA;
	double zeroOffset = (i2c_pdu_voltage(6) / 2) + vOffset;
    mVunscaled = (double)rawAnalog * ADS1115_MV_4P096;
    mVscaled = mVunscaled * vScaleFactor;
   	mA = (mVscaled - zeroOffset) / (mVperA / 1000);
	// sanity check - discard out of range readings
	if (mA > 20000.0) mA = 0.0;
	if (mA < -20000.0) mA = 0.0;
    return (float)(mA);
}

/**
 * raw analog reading
 */
double i2c_raw_voltage(int channel) {
	float mVunscaled, raw;
	switch (channel) {
		case 0:		// Voltage
			raw = pwr_adc1.getConversionP0GND();
			break;
		case 1:		// Current 1
			raw = pwr_adc2.getConversionP0GND();
			break;
		case 2:		// Current 2
			raw = pwr_adc2.getConversionP1GND();
			break;
		case 3:		// Current 3
			raw = pwr_adc2.getConversionP2GND();
			break;
		case 4:		// Current 4
			raw = pwr_adc2.getConversionP3GND();
			break;
		case 5:		// Current 5
			raw = pwr_adc1.getConversionP1GND();
			break;
		case 6:		// 5V SMPS
			raw = pwr_adc1.getConversionP2GND();
			break;
		case 7:		//3V3 SMPS
			raw = pwr_adc1.getConversionP3GND();
			break;
		default:
			raw = 0.0;
	}
	mVunscaled = (double)raw * ADS1115_MV_4P096;
	return mVunscaled;
}

/**
 * Get voltage reading from power distribution unit (PDU)
 * @param channel: 0 = battery, 6 = 5V SMPS, 7 = 3V3 SMPS
 * @returns: voltage reading in mV
 */
float i2c_pdu_voltage(int channel) {
	int rawAnalog;
	double mVunscaled, mVscaled;
    // read battery voltages from Power management board
	mVunscaled = i2c_raw_voltage(channel);
	switch (channel) {
		case 0:		// Battery Voltage
			mVscaled = mVunscaled * PDU_BAT_V_SCALE_FACTOR;
			break;
		case 6:		// 5V SMPS
			mVscaled = mVunscaled * PDU_5V_SCALE_FACTOR;
			break;
		case 7:		// 3V3 SMPS
			mVscaled = mVunscaled;
			break;
		default:
			mVscaled = mVunscaled;
	}
//    if (!runningAsDaemon) {
//		printf("%s: battery %d  %.2f\n", __func__, channel, mVscaled);
//	}
	return mVscaled;
}

/**
 * current current reading from PDU
 * @param channel: current channel [1-5]
 * @param value: float pointer to current value in mA
 * @returns: 0 for success, -1 on failure
 */
int i2c_pdu_current(int channel, float *value) {
	int retVal;
	double val;
	double zerooffset = i2c_pdu_voltage(6) / 2;
	if ( (channel < 1) || (channel > 5) ) return -1;
	switch (channel) {
		case 1:
			val = current_reading(pwr_adc2.getConversionP0GND(), PDU_I1_V_SCALE_FACTOR, (float)ACS712_30A_MV_PER_A, PDU_I1_ZERO_V_OFFSET);
			*value = val * -1;		// invert channel, ACS712 provides negative number for current draw
			break;
		case 2:
			val = current_reading(pwr_adc2.getConversionP1GND(), PDU_I2_V_SCALE_FACTOR, (float)ACS712_30A_MV_PER_A, PDU_I2_ZERO_V_OFFSET);
			*value = val * -1;		// invert channel, ACS712 provides negative number for current draw
			break;
		case 3:
			val = current_reading(pwr_adc2.getConversionP2GND(), PDU_I3_V_SCALE_FACTOR, (float)ACS712_30A_MV_PER_A, PDU_I3_ZERO_V_OFFSET);
			*value = val * -1;		// invert channel, ACS712 provides negative number for current draw
			break;
		case 4:
			val = current_reading(pwr_adc2.getConversionP3GND(), PDU_I4_V_SCALE_FACTOR, (float)ACS712_30A_MV_PER_A, PDU_I4_ZERO_V_OFFSET);
			*value = val * -1;		// invert channel, ACS712 provides negative number for current draw
			break;
		case 5:
			*value = current_reading(pwr_adc1.getConversionP1GND(), PDU_I5_V_SCALE_FACTOR, (float)ACS712_5A_MV_PER_A, PDU_I5_ZERO_V_OFFSET);
			break;
		default: return -1;
	}
//	if (!runningAsDaemon) {
//		printf("%s: Ch%d %.2f\n", __func__, channel, *value);
//	}
	return 0;
}

/**
 * Read single tag from I2C device
 * @returns: true if successful read
 */
bool i2c_read_tag(I2Ctag *tag) {
	uint16_t registers[4];
	bool retVal = true;
	float value;
	int readResult = 0;
	int16_t rawValue;

	uint8_t slaveId = tag->getSlaveId();

	//printf("%s %s - %s\n", __FILE__, __func__, tag->getTopic());

	switch(tag->getAddress()) {
//		case 101:
//			value = tmp_env.readTempC();
//			break;
		case 200:		// PDU voltage
			value = i2c_pdu_voltage(0);
			break;
		case 201:		// PDU current channels (5)
		case 202:
		case 203:
		case 204:
		case 205:
			readResult = i2c_pdu_current(tag->getAddress()-200, &value);
			break;
		case 206:		// PDU 5V supply
			value = i2c_pdu_voltage(6);
			break;
		case 207:		// PDU 3V3 supply
			value = i2c_pdu_voltage(7);
			break;
		case 250:		// PDU raw voltage on ADC
		case 251:
		case 252:
		case 253:
		case 254:
		case 255:
		case 256:
		case 257:
			value = i2c_raw_voltage(tag->getAddress()-250);
			break;
		case 301:
			value = hw.read_cpu_temp();
			break;
		case 401:		// vimon battery voltage
			readResult = vimon.getMilliVolts(0, &value);
			break;
		case 402:		// battery mid point voltage
			readResult = vimon.getMilliVolts(1, &value);
			break;
		case 403:		// vimon battery current
			readResult = vimon.getBipolarMilliAmps(&value);
			break;
/*		case 404:		// vimon battery temperature
			readResult = vimon.getPT100temp(&value);
			//printf("%s - temp: %.2f\n", __func__, readValue);
			break; */
		case 1001:		// Power accumulator
			value = accPwr * 3600;		// convert from Ws to Wh
			//readValue = accPwr * tag->getMultiplier();
			accPwr = 0.0;	// clear accumulator
			break;
		case 1002:		// Power accumulator charge
			value = accPwrChg;
			break;
		case 1003:		// Power accumulator discharge
			value = accPwrDsc;
			break;
		default:
			printf("%s: %s - unknown address %d\n", __FILE__, __func__, tag->getAddress());
			retVal = false;
			break;
	}

	if (readResult == 0) {
		tag->setValue(value);
	} else {
		printf("%s: %s - read error on tag address %d\n", __FILE__, __func__, tag->getAddress());
	}

	if (retVal) {
		mqtt_publish_tag(tag);
	}
	return retVal;
}

/**
 * process I2C cyclic read update
 * @return false if there was nothing to process, otherwise true
 */
bool i2c_read_process() {
	int index = 0;
	int tagIndex = 0;
	int *tagArray;
	bool retval = false;
	time_t now = time(NULL);
	time_t refTime;
	while (updateCycles[index].ident >= 0) {
		// ignore if cycle has no tags to process
		if (updateCycles[index].tagArray == NULL) {
			index++; continue;
		}
		// new reference time for each read cycle
		refTime = time(NULL);		// used for group reads
		if (now >= updateCycles[index].nextUpdateTime) {
			// set next update cycle time
			updateCycles[index].nextUpdateTime = now + updateCycles[index].interval;
			// get array for tags
			tagArray = updateCycles[index].tagArray;
			// read each tag in the array
			tagIndex = 0;
			while (tagArray[tagIndex] >= 0) {
				i2c_read_tag(&i2cReadTags[tagArray[tagIndex]]);
				tagIndex++;
			}
			retval = true;
			//cout << now << " Update Cycle: " << updateCycles[index].ident << " - " << updateCycles[index].tagArraySize << " tags" << endl;
		}
		index++;
	}

	return retval;
}



/**
 * assign tags to update cycles
 * generate arrays of tags assigned ot the same updatecycle
 * 1) iterate over update cycles
 * 2) count tags which refer to update cycle
 * 3) allocate array for those tags
 * 4) fill array with index of tags that match update cycle
 * 5) assign array to update cycle
 * 6) go back to 1) until all update cycles have been matched
 */
bool i2c_assign_updatecycles () {
	int updidx = 0;
	int i2cTagIdx = 0;
	int cycleIdent = 0;
	int matchCount = 0;
	int *intArray = NULL;
	int arIndex = 0;
	// iterate over updatecycle array
	while (updateCycles[updidx].ident >= 0) {
		cycleIdent = updateCycles[updidx].ident;
		updateCycles[updidx].tagArray = NULL;
		updateCycles[updidx].tagArraySize = 0;
		// iterate over mbReadTags array
		i2cTagIdx = 0;
		matchCount = 0;
		while (i2cReadTags[i2cTagIdx].updateCycleId() >= 0) {
			// count tags with cycle id match
			if (i2cReadTags[i2cTagIdx].updateCycleId() == cycleIdent) {
				matchCount++;
				//cout << cycleIdent <<" " << mbReadTags[mbTagIdx].getAddress() << endl;
			}
			i2cTagIdx++;
		}
		// skip to next cycle update if we have no matching tags
		if (matchCount < 1) {
			updidx++;
			continue;
		}
		// -- We have some matching tags
		// allocate array for tags in this cycleupdate
		intArray = new int[matchCount+1];			// +1 to allow for end marker
		if (i2cDebugLevel > 0) {
			printf("%s: %d tags for update cycle %d\n", __func__, matchCount, updidx);
		}
		// fill array with matching tag indexes
		i2cTagIdx = 0;
		arIndex = 0;
		while (i2cReadTags[i2cTagIdx].updateCycleId() >= 0) {
			// count tags with cycle id match
			if (i2cReadTags[i2cTagIdx].updateCycleId() == cycleIdent) {
				intArray[arIndex] = i2cTagIdx;
				arIndex++;
			}
			i2cTagIdx++;
		}
		// mark end of array
		intArray[arIndex] = -1;
		// add the array to the update cycles
		updateCycles[updidx].tagArray = intArray;
		updateCycles[updidx].tagArraySize = arIndex;
		// next update index
		updidx++;
	}
	if (i2cDebugLevel > 0) {
		fflush(stdout);
	}
	return true;
}

/**
 * read tag configuration for one I2C device from config file
 */
bool i2c_config_tags(Setting& i2cTagsSettings, uint8_t deviceId) {
	int tagIndex;
	int tagAddress;
	int tagUpdateCycle;
	string strValue;
	float fValue;
	int intValue;
	bool bValue;

	int numTags = i2cTagsSettings.getLength();
	if (numTags < 1) {
		cout << "No tags Found " << endl;
		return true;		// permissible condition
	}

	for (tagIndex = 0; tagIndex < numTags; tagIndex++) {
		if (i2cTagsSettings[tagIndex].lookupValue("address", tagAddress)) {
			i2cReadTags[i2cTagCount].setAddress(tagAddress);
			i2cReadTags[i2cTagCount].setSlaveId(deviceId);
		} else {
			log(LOG_WARNING, "Error in config file, tag address missing");
			continue;		// skip to next tag
		}
		if (i2cTagsSettings[tagIndex].lookupValue("update_cycle", tagUpdateCycle)) {
			i2cReadTags[i2cTagCount].setUpdateCycleId(tagUpdateCycle);
		}
		// is topic present? -> read mqtt related parametrs
		if (i2cTagsSettings[tagIndex].lookupValue("topic", strValue)) {
			i2cReadTags[i2cTagCount].setTopic(strValue.c_str());
			i2cReadTags[i2cTagCount].setPublishRetain(mqtt_retain_default);		// set to default
			if (i2cTagsSettings[tagIndex].lookupValue("retain", bValue))		// override default if required
				i2cReadTags[i2cTagCount].setPublishRetain(bValue);
			if (i2cTagsSettings[tagIndex].lookupValue("format", strValue))
				i2cReadTags[i2cTagCount].setFormat(strValue.c_str());
			if (i2cTagsSettings[tagIndex].lookupValue("multiplier", fValue))
				i2cReadTags[i2cTagCount].setMultiplier(fValue);
			if (i2cTagsSettings[tagIndex].lookupValue("offset", fValue))
				i2cReadTags[i2cTagCount].setOffset(fValue);
			if (i2cTagsSettings[tagIndex].lookupValue("noreadvalue", fValue))
				i2cReadTags[i2cTagCount].setNoreadValue(fValue);
			if (i2cTagsSettings[tagIndex].lookupValue("noreadaction", intValue))
				i2cReadTags[i2cTagCount].setNoreadAction(intValue);
			if (i2cTagsSettings[tagIndex].lookupValue("noreadignore", intValue))
				i2cReadTags[i2cTagCount].setNoreadIgnore(intValue);
		}
		if (i2cDebugLevel > 0) {
			printf("%s: [%d] ", __func__, tagIndex);
			cout << "Tag " << i2cTagCount << " addr: " << tagAddress << " cycle: " << tagUpdateCycle;
			cout << " Topic: " << i2cReadTags[i2cTagCount].getTopicString() << endl;
		}
		i2cTagCount++;
	}
	return true;
}

/**
 * read device configuration from config file
 */

bool i2c_config_devices(Setting& i2cDeviceSettings) {
	int deviceId, numTags;
	string deviceName;
	bool deviceEnabled;

	// we need at least one slave in config file
	int numDevices = i2cDeviceSettings.getLength();
	if (numDevices < 1) {
		log(LOG_ERR, "Error in config file, no Modbus slaves found");
		return false;
	}

	if (i2cDebugLevel > 0) {
		printf("%s: Total number of devices: %d\n", __func__, numDevices);
	}

	// calculate the total number of tags for all configured slaves
	numTags = 0;
	for (int deviceIdx = 0; deviceIdx < numDevices; deviceIdx++) {
		if (i2cDebugLevel > 0) {
			printf("%s: counting tags for device: %d\n", __func__, deviceIdx);
		}
		if (i2cDeviceSettings[deviceIdx].exists("tags")) {
			if (!i2cDeviceSettings[deviceIdx].lookupValue("enabled", deviceEnabled)) {
				deviceEnabled = true;	// true is assumed if there is no entry in config file
			}
			if (deviceEnabled) {
				Setting& i2cTagsSettings = i2cDeviceSettings[deviceIdx].lookup("tags");
				numTags += i2cTagsSettings.getLength();
			}
		}
	}

	i2cReadTags = new I2Ctag[numTags+1];

	if (i2cDebugLevel > 0) {
		printf("%s: Total number of enabled tags: %d\n", __func__, numTags);
	}
	i2cTagCount = 0;
	// iterate through devices
	for (int deviceIdx = 0; deviceIdx < numDevices; deviceIdx++) {
		i2cDeviceSettings[deviceIdx].lookupValue("name", deviceName);
		if (i2cDeviceSettings[deviceIdx].lookupValue("id", deviceId)) {
			if (i2cDebugLevel > 0)
				printf("%s: processing Device %d (%s)\n", __func__, deviceId, deviceName.c_str());
		} else {
			log(LOG_ERR, "Config error - PL device ID missing in entry %d", deviceId+1);
			return false;
		}

		// get list of tags
		if (i2cDeviceSettings[deviceIdx].exists("tags")) {
			if (!i2cDeviceSettings[deviceIdx].lookupValue("enabled", deviceEnabled)) {
				deviceEnabled = true;	// true is assumed if there is no entry in config file
			}
			if (deviceEnabled) {
				Setting& i2cTagsSettings = i2cDeviceSettings[deviceIdx].lookup("tags");
				if (!i2c_config_tags(i2cTagsSettings, deviceId)) {
					return false; }
			} else {
				log(LOG_NOTICE, "I2C device %d (%s) disabled in config", deviceId, deviceName.c_str());
			}
		} else {
			log(LOG_NOTICE, "No tags defined for I2C device %d", deviceId);
			// this is a permissible condition
		}
	}
	// mark end of array
	i2cReadTags[i2cTagCount].setUpdateCycleId(-1);
	i2cReadTags[i2cTagCount].setSlaveId(I2C_DEVICEID_MAX +1);
	if (i2cDebugLevel > 0) {
		printf("%s: completed\n", __func__);
		fflush(stdout);
		fflush(stderr);
	}
	return true;
}

/**
 * read update cycles from config file
 */
bool i2c_config_updatecycles(Setting& updateCyclesSettings) {
	int idValue, interval, index;
	int numUpdateCycles = updateCyclesSettings.getLength();

	if (numUpdateCycles < 1) {
		log(LOG_ERR, "Error in config file, \"updatecycles\" missing");
		return false;
	}

	// allocate array
	updateCycles = new updatecycle[numUpdateCycles+1];

	for (index = 0; index < numUpdateCycles; index++) {
		if (updateCyclesSettings[index].lookupValue("id", idValue)) {
		} else {
			log(LOG_ERR, "Config error - cycleupdate ID missing in entry %d", index+1);
			return false;
		}
		if (updateCyclesSettings[index].lookupValue("interval", interval)) {
		} else {
			log(LOG_ERR, "Config error - cycleupdate interval missing in entry %d", index+1);
			return false;
		}
		updateCycles[index].ident = idValue;
		updateCycles[index].interval = interval;
		updateCycles[index].nextUpdateTime = time(0) + interval;
		if (i2cDebugLevel > 0) {
			printf("%s: ", __func__);
			cout << "Update " << index << " ID " << idValue; 
			cout << " Interval: " << interval << " t:" << updateCycles[index].nextUpdateTime << endl;
		}
	}
	// mark end of data
	updateCycles[index].ident = -1;
	updateCycles[index].interval = -1;

	if (i2cDebugLevel > 0) {
		fflush(stdout);
	}

	return true;
}


/**
 * read I2C configuration from config file
 */
bool i2c_config() {

	if (i2cDebugLevel > 0) {
		printf("%s\n", __func__);
		fflush(stdout);
	}

	// Configure update cycles
	try {
		Setting& updateCyclesSettings = cfg.lookup("updatecycles");
		if (!i2c_config_updatecycles(updateCyclesSettings)) {
			return false; }
	} catch (const SettingNotFoundException &excp) {
		log(LOG_ERR, "Error in config file <%s> not found", excp.getPath());
		return false;
	} catch (const SettingTypeException &excp) {
		log(LOG_ERR, "Error in config file <%s> is wrong type", excp.getPath());
		return false;
	}


	// Configure i2c devices
	try {
		Setting& i2cDeviceSettings = cfg.lookup("i2cdevices");
		if (!i2c_config_devices(i2cDeviceSettings)) {
			return false; }
	} catch (const SettingNotFoundException &excp) {
		log(LOG_ERR, "Error in config file <%s> not found", excp.getPath());
		return false;
	} catch (const SettingTypeException &excp) {
		log(LOG_ERR, "Error in config file <%s> is not a string", excp.getPath());
		return false;
	} catch (const ParseException &excp) {
		log(LOG_ERR, "Error in config file - Parse Exception");
		return false;
	} catch (...) {
		log(LOG_ERR, "pl_config <pldevices> Error in config file (exception)");
		return false;
	}
	return true;
}


/**
 * initialize I2C interface devices
 * @returns false for configuration error, otherwise true
 */
bool i2c_init() {
    // sequence is important, the I2C setup also calls
    // WiringPiSetupSys() which is required for pin IO functions

	// VI-Monitor board
	if (!vimon.initialize( ADS1115_ADDRESS_ADDR_SDA )) {
		log(LOG_ERR, "vimon init failed");
		return false;
	}

	if (i2cDebugLevel > 0) {
		printf("%s: vimon board init OK\n", __func__);
		fflush(stdout);
	}

    // Power Management board ADC 1
    pwr_adc1.initialize();
    if (!pwr_adc1.testConnection()) {
		log(LOG_ERR, "ADC1 on PDU failed to detect");
		return false;
    }

	if (i2cDebugLevel > 0) {
		printf("%s: ADC1 on PDU present\n", __func__);
		fflush(stdout);
	}

    pwr_adc1.setGain(ADS1115_PGA_4P096);
	if (!runningAsDaemon) {
		pwr_adc1.showConfigRegister();
	}

	// Power Management board ADC 1
    pwr_adc2.initialize();
    if (!pwr_adc2.testConnection()) {
		log(LOG_ERR, "ADC2 on PDU failed to detect");
		return false;
    }

	if (i2cDebugLevel > 0) {
		printf("%s: ADC2 on PDU present\n", __func__);
		fflush(stdout);
	}

    pwr_adc2.setGain(ADS1115_PGA_4P096);
    if (!runningAsDaemon) {
        pwr_adc2.showConfigRegister();
	}

	if (i2cDebugLevel > 0) {
		printf("%s: I2C hardware initialized\n", __func__);
		fflush(stdout);
	}

	if (!i2c_config()) return false;
	if (!i2c_assign_updatecycles()) return false;
	return true;
}

#pragma mark Loops

/**
 * set main loop interval to a valid setting
 * @param newValue the new main loop interval in ms
 */
void setMainLoopInterval(int newValue)
{
	int val = newValue;
	if (newValue < MAIN_LOOP_INTERVAL_MINIMUM) {
		val = MAIN_LOOP_INTERVAL_MINIMUM;
	}
	if (newValue > MAIN_LOOP_INTERVAL_MAXIMUM) {
		val = MAIN_LOOP_INTERVAL_MAXIMUM;
	}
	mainloopinterval = val;

	log(LOG_INFO, "Main Loop interval is %dms", mainloopinterval);
}

/**
 * called on program exit
 */
void exit_loop(void) 
{
	bool bValue, clearonexit = false, noreadonexit = false;

	// how to handle mqtt broker published tags 
	// clear retain status for all tags?
	if (cfg.lookupValue("mqtt.clearonexit", bValue))
		clearonexit = bValue;
	// publish noread value for all tags?
	if (cfg.lookupValue("mqtt.noreadonexit", bValue)) 
		noreadonexit = bValue;
	if (noreadonexit || clearonexit)
		mqtt_clear_tags(noreadonexit, clearonexit);
	// free allocated memory
	// arrays of tags in cycleupdates
	int *ar, idx=0;
	while (updateCycles[idx].ident >= 0) {
		ar = updateCycles[idx].tagArray;
		if (ar != NULL) delete [] ar;		// delete array if one exists
		idx++;
	}

	delete [] updateCycles;
}

/**
 * Main program loop
 */
void main_loop()
{
	bool processing_success = false;
	//clock_t start, end;
	struct timespec starttime, endtime, difftime;
	useconds_t sleep_usec;
	//double delta_time;
	useconds_t processing_time;
	useconds_t min_time = 99999999, max_time = 0;
	useconds_t interval = mainloopinterval * 1000;	// convert ms to us

	// intiate accumulator timing
	clock_gettime(CLOCK_MONOTONIC, &lastAccTime);
	// reset accumulator values
	accPwr = 0;
	accPwrChg = 0; accPwrDsc = 0;

	// first call takes a long time (10ms)
	while (!exitSignal) {
	// run processing and record start/stop time
		clock_gettime(CLOCK_MONOTONIC, &starttime);
		processAccumulators();
		processing_success = process();
		clock_gettime(CLOCK_MONOTONIC, &endtime);
		// calculate cpu time used [us]
		timespec_diff(&starttime, &endtime, &difftime);
		processing_time = (difftime.tv_nsec / 1000) + (difftime.tv_sec * 1000000);

		// store min/max times if any processing was done
		if (processing_success) {
			// calculate cpu time used [us]
			if (debugEnabled)
				printf("%s - process() took %dus\n", __func__, processing_time);
			if (processing_time > max_time) {
				max_time = processing_time;
			}
			if (processing_time < min_time) {
				min_time = processing_time;
			}
			//printf("%s - success (%dus)\n", __func__, processing_time);
		}
		// enter loop delay if needed
		// if cpu_time_used exceeds the mainLoopInterval
		// then bypass the loop delay
		if (interval > processing_time) {
			sleep_usec = interval - processing_time;  // sleep time in us
			//printf("%s - sleeping for %dus (%dus)\n", __func__, sleep_usec, processing_time);
			usleep(sleep_usec);
		}

		if (mqtt_next_connect_time > 0) {
			if (time(NULL) >= mqtt_next_connect_time) {
				mqtt_connect();
			}
		}
	}
	if (!runningAsDaemon)
		printf("CPU time for variable processing: %dus - %dus\n", min_time, max_time);
}

/** Display program usage instructions.
 * @param
 * @return
 */
static void showUsage(void) {
	if (!runningAsDaemon) {
		cout << "usage:" << endl;
		cout << processName << "-cCfgFileName -d -h" << endl;
		cout << "c = name of config file" << endl;
		cout << "d = enable debug mode" << endl;
		cout << "i = I2C debug enable" << endl;
		cout << "h = show help" << endl;
	}
}

/** Parse command line arguments.
 * @param argc argument count
 * @param argv array of arguments
 * @return false to indicate program needs to abort
 */
bool parseArguments(int argc, char *argv[]) {
	char buffer[64];
	int i, buflen;
	int retval = true;


	if (runningAsDaemon) {
		cfgFileName = std::string(CFG_DEFAULT_FILEPATH) + std::string(CFG_DEFAULT_FILENAME);
	} else {
		cfgFileName = std::string(CFG_DEFAULT_FILENAME);
	}

	if (argc > 1) {
		for (i = 1; i < argc; i++) {
			strcpy(buffer, argv[i]);
			buflen = strlen(buffer);
			if ((buffer[0] == '-') && (buflen >=2)) {
				switch (buffer[1]) {
				case 'c':
					cfgFileName = std::string(&buffer[2]);
					break;
				case 'd':
					debugEnabled = true;
					printf("Debug enabled\n");
					break;
				case 'h':
					showUsage();
					retval = false;
					break;
				case 'i':
					i2cDebugLevel = 1;
					break;
				default:
					log(LOG_NOTICE, "unknown parameter: %s", argv[i]);
					showUsage();
					retval = false;
					break;
				} // switch
				;
			} // if
		}  // for (i)
	}  // if (argc >1)
	return retval;
}

int main (int argc, char *argv[])
{
	if ( getppid() == 1) runningAsDaemon = true;

	processName = std::string(basename(argv[0]));

	if (! parseArguments(argc, argv) ) goto exit_fail;

	log(LOG_INFO,"[%s] PID: %d PPID: %d", argv[0], getpid(), getppid());
	log(LOG_INFO,"Version %d.%02d [%s] ", version_major, version_minor, build_date_str);

	// catch SIGTERM only if running as daemon (started via systemctl)
	// when run from command line SIGTERM provides a last resort method
	// of killing the process regardless of any programming errors.
	if (runningAsDaemon) {
		signal (SIGTERM, sigHandler);
	}

	// read config file
	if (!readConfig()) {
		log(LOG_ERR, "Error reading config file <%s>", cfgFileName.c_str());
		goto exit_fail;
	}

	if (!mqtt_init()) goto exit_fail;
//	printf("%s: mqtt_init done\n", __func__);
//	fflush(stdout);

	if (!init_values()) goto exit_fail;
//	printf("%s: init_values done\n", __func__);
//	fflush(stdout);

	if (!i2c_init()) goto exit_fail;
//	printf("%s: i2c_init done\n", __func__);
//	fflush(stdout);

	usleep(100000);
	main_loop();

	exit_loop();
	log(LOG_INFO, "exiting");
	exit(EXIT_SUCCESS);

exit_fail:
	log(LOG_INFO, "exit with error");
	exit(EXIT_FAILURE);
}
