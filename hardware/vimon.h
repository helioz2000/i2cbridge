/**
 * @file vimon.h
 * Author: Erwin Bejsta (2020)
 */

/*
 VI monitoring board
 This class facilitates access to VI-board readings
 The file vimon_cal.h contains the claibration data
 specific to a particular board and it components.
 The board Uses precision voltage source at 2.048V on the OpAmp
 Ch0: V1 10.0V to 16.0V = 0 to 2.048V
 Ch1: V2 or PT100 0 to 2.03V = 90 to 134 Ohm = -25 to +88 DegC
 Ch2: I1
 Ch3: I2
 Current sensor INA180A1 gain is 20
 Example: 50mV/50A on Shunt = 1V on Analog Input for 50A
          50mA per mV
 */

#ifndef _VIMON_H_
#define _VIMON_H_

#include <unistd.h>
#include <string>

#include "ADS1115.h"

class VImon {
public:
	VImon();
	~VImon();

/*
 initialise the VI board ADC
 - must be called before any other functions 
 - can only be called once 
 - returns true on success
*/
	bool initialize(uint8_t address =0);

/*
 read and store raw analog value
 using this function can avoind rapid subsequent reading from
 the ADC. All 4 channels are read and the results ares stored
 in "rawValue[4]" (public).
 The "get...." functions can optinally use the raw values or perform teir own read
 - usecs specifies the delay between readings
 */
	void readRaw();

/*
 get function read various analog values
 - valid channels: 0-1 for Voltage, 2-3 for Current
 - return 0 on success, -1 on failure
 - "useRaw" defaults to 0. If set to 1 the function
   will use the values previously read with "readRaw()"
   this feature can be used to avoid successive reading
   of the ADC channels which can cause small reading variations
   due to rapid MUX switching.
*/
	int getRawValue(int channel, int16_t *value);
	int getUnscaledMilliVolts(int channel, float *value, bool useRaw =0);
	int getMilliVolts(int channel, float *value, bool useRaw =0);
	int getMilliAmps(int channel, float *value, bool useRaw =0);
	int getBipolarMilliAmps(float *value, bool useRaw =0);
/*
 PT100 is optional and replaces Voltage 2
 - connected on CH 1
 */
	int getPT100ohm(float* value, bool useRaw =0);
	int getPT100temp(float *value, bool useRaw =0);
	int getTemperature(float *value, bool useRaw =0);

/*
 returns true when the ADS1115 is present on the I2C bus
 */
	bool testConnection();

/*
 for testing - read and print readiangs for all channels
 - useRaw will give a consistent reading as displayed
   details come from a single reading.
 */
	void readAllChannels(std::string& retStr, bool useRaw =0);

/*
 storage for raw readings
 */
	int16_t rawValue[4];

private:
	ADS1115 *_adc;
	bool _init_done;
};

#endif /* _VIMON_H_ */
