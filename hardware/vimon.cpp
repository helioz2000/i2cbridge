/*
 VI monitoring board ADC configuration
 Uses precision voltage source for 2.048V on the OpAmp
 Ch0: V1 10.0V to 16.0V = 0 to 2.048V
 Ch1: V2 or PT100 0 to 2.03V = 90 to 134 Ohm = -25 to +88 DegC
 Ch2: I1
 Ch3: I2
 Current sensor INA180A1 gain is 20
 Example: 50mV/50A on Shunt = 1V on Analog Input for 50A
		  50mA per mV
 */


#include <stdio.h>
#include <unistd.h>

#include "ADS1115.h"

#include "vimon_cal.h"
#include "vimon.h"

using namespace std;


VImon::VImon() {
	_adc = NULL;
}

VImon::~VImon() {
	if (_adc != NULL)
		delete _adc;
}

bool VImon::initialize(uint8_t address) {

	if (!address) {
		address = BOARD_ADDRESS;
	}
	if (_adc != NULL) {
		return false;		// fail if already initialized (can only initialize once)
	}

	_adc = new ADS1115(address);
	_adc->initialize();

	if (!this->testConnection()) {
		delete _adc;
		_adc = NULL;
		return false;
	}

    // set gain
    _adc->setGain(ADS1115_PGA_2P048);

    //_adc->showConfigRegister();
	readRaw();	// read all channels once

    return true;
}

bool VImon::testConnection() {
	if (!_adc->testConnection()) {
        perror("ADS1115 not found \n");
        return false;
    }
	return true;
}

void VImon::readRaw() {
	rawValue[0] = _adc->getConversionP0GND();
	rawValue[1] = _adc->getConversionP1GND();
	rawValue[2] = _adc->getConversionP2GND();
	rawValue[3] = _adc->getConversionP3GND();
}

int VImon::getRawValue(int channel, int16_t *value) {
	switch (channel) {
		case 0:
			*value = _adc->getConversionP0GND();
			break;
		case 1:
			*value = _adc->getConversionP1GND();
			break;
		case 2:
			*value = _adc->getConversionP2GND();
			break;
		case 3:
			*value = _adc->getConversionP3GND();
			break;
		default:
			return -1;
	}
	return 0;

}

int VImon::getUnscaledMilliVolts(int channel, float *value, bool useRaw) {
	float reading;
	switch (channel) {
		case 0:
			if (useRaw)
				reading = (float)rawValue[0] * ADS1115_MV_2P048;
			else
				reading = (float)_adc->getConversionP0GND() * ADS1115_MV_2P048;
			break;
		case 1:
			if (useRaw)
				reading = (float)rawValue[1] * ADS1115_MV_2P048;
			else
				reading = (float)_adc->getConversionP1GND() * ADS1115_MV_2P048;
			break;
		case 2:
			if (useRaw)
				reading = (float)rawValue[2] * ADS1115_MV_2P048;
			else
				reading = (float)_adc->getConversionP2GND() * ADS1115_MV_2P048;
			break;
		case 3:
			if (useRaw)
				reading = (float)rawValue[3] * ADS1115_MV_2P048;
			else
				reading = (float)_adc->getConversionP3GND() * ADS1115_MV_2P048;
			break;
		default:
			return -1;
	}
	*value = reading;
	return 0;
}

int VImon::getMilliVolts(int channel, float *value, bool useRaw) {
	float mVunscaled, mVscaled;

	if (getUnscaledMilliVolts(channel, &mVunscaled, useRaw) < 0) {
		return -1;
	}

	switch(channel) {
		case 0:
			mVscaled = (mVunscaled * V1_MV_PER_MV) + V1_OFFSET;
			break;
		case 1:
			mVscaled = (mVunscaled * V2_MV_PER_MV) + V2_OFFSET;
			break;
		default:
			return -1;
	}
	*value = mVscaled;
	return 0;
}

int VImon::getPT100temp(float *value, bool useRaw) {
	float ohm;
	if (getPT100ohm(&ohm, useRaw) < 0) {
		return -1;
	}
    *value = (ohm/PT_REFERENCE_OHM-1.0)/PT_SLOPE;
	*value += (float)PT_OFFSET_TEMP;
	return 0;
}

int VImon::getPT100ohm(float *value, bool useRaw) {
	float mVunscaled;
	if (getUnscaledMilliVolts(1, &mVunscaled, useRaw) < 0) {
		return -1;
	}
	*value = (mVunscaled * PT_OHM_PER_MV) + PT_OFFSET_OHM;
	return 0;
}

int VImon::getMilliAmps(int channel, float *value, bool useRaw) {
	float mVunscaled;
	if (getUnscaledMilliVolts(channel, &mVunscaled, useRaw) < 0) {
		return -1;
	}
	switch (channel) {
		case 2:
			*value = mVunscaled * I1_MA_PER_MV;
			break;
		case 3:
			*value = mVunscaled * I2_MA_PER_MV;
			break;
		default:
			return -1;
	}
	return 0;
}

int VImon::getBipolarMilliAmps(float *value, bool useRaw) {
	float mVunscaled, i1, i2;
	// read both current channels
	if (getUnscaledMilliVolts(2, &mVunscaled, useRaw) < 0) return -1;
	i1 = mVunscaled * I1_MA_PER_MV;
	if (getUnscaledMilliVolts(3, &mVunscaled, useRaw) < 0) return -1;
	i2 = mVunscaled * I2_MA_PER_MV;
	// positive value on i1 (charging)
	if (i1 > i2) {
		*value = i1;
	} else {
		*value = 0.0 - i2;
	}
	return 0;
}

void VImon::readAllChannels(std::string& retStr, bool useRaw) {
	float mVunscaled;
	float voltage_mv;
	float current1_ma, current2_ma;
	float pt100ohm, pt100temp;
	int i;
	char buf[64];

	if (useRaw)
		readRaw();

	retStr = "";
	// show raw values
	for (i=0; i<4; i++) {
		sprintf(buf, "%5d ",rawValue[i]);
		retStr += buf;
	}

	//printf(" : ");
	retStr += " : ";

	// show unscaled mV reading
	for (i=0; i<4; i++) {
		if (getUnscaledMilliVolts(i, &mVunscaled, useRaw) == 0) {
			sprintf(buf, "%5.0f ", mVunscaled);
		} else {
			sprintf(buf, "-err- ");
		}
		retStr += buf;
	}

	// show Voltage (CH0)
	if (getMilliVolts(0, &voltage_mv, useRaw) == 0) {
		sprintf(buf, " : %5.0f mV", voltage_mv);
	} else {
		sprintf(buf, " : -err- mV");
	}
	retStr += buf;

	// show PT100 (CH1)
	if (getPT100ohm(&pt100ohm, useRaw) == 0) {
		getPT100temp(&pt100temp, useRaw);
		sprintf(buf, " : %6.2f Ohm : %5.2f DegC", pt100ohm, pt100temp);
	} else {
		sprintf(buf, " : -err-  Ohm : -err- DegC");
	}
	retStr += buf;

	// show Current 1 (CH2)
	if (getMilliAmps(2, &current1_ma, useRaw) == 0) {
		sprintf(buf, " : %6.1f mA", current1_ma);
	} else {
		sprintf(buf, " : -err-  mA");
	}
	retStr += buf;

	// show Current 2 (CH3)
	if (getMilliAmps(3, &current2_ma, useRaw) == 0) {
		sprintf(buf, " : %6.1f mA", current2_ma);
	} else {
		sprintf(buf, " : -err-  mA");
	}
	retStr += buf;

}
