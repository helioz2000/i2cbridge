/*
 * MCP9808.cpp:
 *	functions to access the I2C based MCP9808 temperature sensor
 *	Copyright (c) 2015 Erwin Bejsta
 ***********************************************************************
 *
 *    This code is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    This code is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


//#include <wiringPiI2C.h>
#include <stdio.h>
#include <byteswap.h>
#include "MCP9808.h"

#define MCP9808_DEFAULT_ADDRESS 0x18
#define MCP9808_RES_0P0625      0.0625


/** Default constructor, uses default I2C address.
 * @see MCP9808_DEFAULT_ADDRESS
 */
MCP9808::MCP9808() {
    devAddr = MCP9808_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see MCP9808_DEFAULT_ADDRESS
 */
MCP9808::MCP9808(uint8_t address) {
    if ((address < 0x18) || (address) > 0x1F) {
        fprintf(stderr, "%s - invalid I2C address.\n", __PRETTY_FUNCTION__);
    } else {
        devAddr = address;
    }
}

/*
 *	Reset to factory defaults.
 *
 *********************************************************************************
 */

uint8_t MCP9808::getResolution (void) {
    uint8_t data;
    I2Cdev::readByte (devAddr, MCP9808_RESOLUTION, &data);
    return data;
}

void MCP9808::setResolution (uint8_t res )
{
    I2Cdev::writeByte (devAddr, MCP9808_RESOLUTION, res);
}

void MCP9808::initialize (void)
{
    // reset mcp9808
    I2Cdev::writeWord (devAddr, MCP9808_CONFIG, 0);
    I2Cdev::writeWord (devAddr, MCP9808_UPPER_TEMP, 0);
    I2Cdev::writeWord (devAddr, MCP9808_LOWER_TEMP, 0);
    I2Cdev::writeWord (devAddr, MCP9808_CRIT_TEMP, 0);
    // set the resolution register (see datasheet 5.1.6 for details)
    setResolution (MCP9808_RES_CFG_0P0625);       // highest resolution = slowest conversion time
}

/*
 * testConnection:
 *	Check presence of mcp9808.
 *
 *********************************************************************************
 */

bool MCP9808::testConnection (void)
{
    uint16_t manufid, devid ;
    
    I2Cdev::readWord (devAddr, MCP9808_MANUF_ID, &manufid);
    I2Cdev::readWord (devAddr, MCP9808_DEVICE_ID, &devid);
    if ((manufid != 0x0054) || (devid != 0x0400)) {
        fprintf(stderr, "%s - Device does not identify as MCP9808.\n", __PRETTY_FUNCTION__ );
        return false;
    }
    return true;
}

float MCP9808::convertLimitTemp (uint16_t rawData)
{
    float temp;

    // convert to DegC, mask out top nibble first
    temp = (float)((rawData & 0x0FFF) >> 4);
    // fraction part - units = 0.25degC per LSB
    temp += 0.25 * (float)(rawData >> 2 & 0x3);
    
    // apply sign bit if required
    if(rawData & 0x1000) {
        temp = 256.0 - temp;
    }
    return temp;
}

float MCP9808::getLowerTempLimit (void)
{
    uint16_t rawData;
    
    I2Cdev::readWord (devAddr, MCP9808_LOWER_TEMP, &rawData) ;
    return convertLimitTemp (rawData);
}

float MCP9808::getUpperTempLimit (void)
{
    uint16_t rawData;
    
    I2Cdev::readWord (devAddr, MCP9808_UPPER_TEMP, &rawData) ;
    return convertLimitTemp (rawData);
}

float MCP9808::getCriticalTempLimit (void)
{
    uint16_t rawData;
    
    I2Cdev::readWord (devAddr, MCP9808_CRIT_TEMP, &rawData) ;
    return convertLimitTemp (rawData);
}

/*
 * readTemp_ functions:
 *	Read ambient temperature in Degree Celsius. Farenheit or Kelvin
 *********************************************************************************
 */

uint16_t MCP9808::readTempRaw (void)
{
    uint16_t rawData;
    I2Cdev::readWord (devAddr, MCP9808_AMBIENT_TEMP, &rawData) ;
    return rawData;
}

float MCP9808::readTempC (void) {
    uint16_t rawData;
    float temp;

    rawData = readTempRaw();
    // convert to DegC, mask out top nibble first
    temp = (float)((rawData & 0x0FFF) >> 4);
    // fraction part - units = selected resolution
    temp += MCP9808_RES_0P0625 * (float)(rawData & 0xF);
    
    // apply sign bit if required
    if(rawData & 0x1000) {
        temp = 256.0 - temp;
    }
    return temp;
}
/*** Farenheit ***/
float MCP9808::readTempF (void) {
    float temp;
    temp = readTempC();
    temp /= 5;
    temp *= 9;
    temp += 32;
    return temp;
}

/*** Kelvin ***/
float MCP9808::readTempK (void) {
    float temp;
    temp = readTempC();
    temp += 273.15;
    return temp;
}

