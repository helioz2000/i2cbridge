/**
 * @file modbustag.cpp
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <sys/utsname.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "i2ctag.h"

#include <stdexcept>
#include <iostream>

/*********************
 *      DEFINES
 *********************/
using namespace std;

/*********************
 * GLOBAL FUNCTIONS
 *********************/


/*********************
 * MEMBER FUNCTIONS
 *********************/

//
// Class Tag
//

I2Ctag::I2Ctag() {
	this->_address = 0;
	this->_topic = "";
	this->_slaveId = 0;
	this->_multiplier = 1.0;
	this->_offset = 0.0;
	this->_format = "%f";
	this->_noreadvalue = 0.0;
	this->_noreadaction = -1;	// do nothing
	this->_noreadignore = 0;
	this->_noreadcount = 0;
	this->_ignoreRetained = false;
	this->_value = 0.0;
//	this->_dataType = 'r';
	//printf("%s - constructor %d %s\\", __func__, this->_slaveId, this->_topic.c_str());
	//throw runtime_error("Class Tag - forbidden constructor");
}

I2Ctag::I2Ctag(const uint16_t addr) {
	this->_address = addr;
	this->_value = 0.0;
}

I2Ctag::~I2Ctag() {
	//cout << "Topic: <" << topic << ">" << endl;
	//printf("%s - destructor %d\n", __func__, address);
}

void I2Ctag::noreadNotify(void) {
	if (_noreadcount <= _noreadignore)	// if noreadignore is 0 noreadcount will still increment to 1
		_noreadcount++;					// a noreadcount > 0 indicates the tag is in noread state
}

bool I2Ctag::isNoread(void) {
	if (_noreadcount > 0) return true;
	else return false;
}

bool I2Ctag::noReadIgnoreExceeded(void) {
	if (_noreadcount > _noreadignore) return true;
	else return false;
}

void I2Ctag::setSlaveId(int newId) {
	_slaveId = newId;
}

uint8_t I2Ctag::getSlaveId(void) {
	return _slaveId;
}

void I2Ctag::setAddress(uint16_t newAddress) {
	_address = newAddress;
}

uint16_t I2Ctag::getAddress(void) {
	return _address;
}

void I2Ctag::setTopic(const char *topicStr) {
	if (topicStr != NULL) {
		_topic = topicStr;
	}
}

const char* I2Ctag::getTopic(void) {
	return _topic.c_str();
}

std::string I2Ctag::getTopicString(void) {
	return _topic;
}

void I2Ctag::setPublishRetain(bool newRetain) {
    _publish_retain = newRetain;
}

bool I2Ctag::getPublishRetain(void) {
    return _publish_retain;
}

void I2Ctag::setIgnoreRetained(bool newValue) {
	_ignoreRetained = newValue;
}

bool I2Ctag::getIgnoreRetained(void) {
	return _ignoreRetained;
}

void I2Ctag::setFormat(const char *formatStr) {
	if (formatStr != NULL) {
		_format = formatStr;
	}
}

const char* I2Ctag::getFormat(void) {
	return _format.c_str();
}

void I2Ctag::setValue(double newValue) {
	_value = newValue;
	_lastUpdateTime = time(NULL);
}

double I2Ctag::getValue(void) {
	return _value;
}

/*
void I2Ctag::setRawValue(int16_t intValue) {
	switch(_dataType) {
		case 'r':
			_rawValue = intValue;
			break;
		case 'i':
		case 'q':
			if (intValue > 0) _rawValue = 1;
			else _rawValue = 0;
			break;
	}
	_lastUpdateTime = time(NULL);
	_noreadcount = 0;
}

int16_t I2Ctag::getRawValue(void) {
	return _rawValue;
}
*/

uint16_t I2Ctag::getBoolValue(void) {
	if (_value == 0.0)
		return false;
	else
		return true;
}

float I2Ctag::getScaledValue(void) {
	//float fValue = (float) _rawValue;
	double dValue = _value;
	dValue *= _multiplier;
	return dValue + _offset;
}

void I2Ctag::setMultiplier(float newMultiplier) {
	_multiplier = newMultiplier;
}

float I2Ctag::getMultiplier(void) {
	return _multiplier;
}

void I2Ctag::setOffset(float newOffset) {
	_offset = newOffset;
}

void I2Ctag::setUpdateCycleId(int ident) {
	_updatecycle_id = ident;
}

int I2Ctag::updateCycleId(void) {
	return _updatecycle_id;
}

void I2Ctag::setNoreadValue(float newValue) {
	_noreadvalue = newValue;
}

float I2Ctag::getNoreadValue(void) {
	return _noreadvalue;
}

void I2Ctag::setNoreadAction(int newValue) {
	_noreadaction = newValue;
}

int I2Ctag::getNoreadAction(void) {
	return _noreadaction;
}

void I2Ctag::setNoreadIgnore(int newValue) {
	_noreadignore = newValue;
}

int I2Ctag::getNoreadIgnore(void) {
	return _noreadignore;
}

/*
bool I2Ctag::setDataType(char newType) {
	switch (newType) {
	case 'i':
	case 'I':
		_dataType = 'i';
		break;
	case 'q':
	case 'Q':
		_dataType = 'q';
		break;
	case 'r':
	case 'R':
		_dataType = 'r';
		break;
	default:
		return false;
	}
	return true;
}

char I2Ctag::getDataType(void) {
	return _dataType;
}
*/
