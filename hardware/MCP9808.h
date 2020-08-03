/*
 * MCP9808.H:
 *	Extend wiringPi with the MCP9808 I2C Temperature Sensor
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

#ifndef _MCP9808_H_
#define _MCP9808_H_

#include "I2CdevPi.h"


// MCP9808 Registers

#define	MCP9808_CONFIG          0x01
#define	MCP9808_UPPER_TEMP      0x02
#define	MCP9808_LOWER_TEMP      0x03
#define	MCP9808_CRIT_TEMP       0x04
#define	MCP9808_AMBIENT_TEMP	0x05
#define	MCP9808_MANUF_ID		0x06
#define	MCP9808_DEVICE_ID		0x07
#define	MCP9808_RESOLUTION      0x08

// MCP9808 Bits in CONFIG register

#define MCP9808_CFG_ALERTMODE   0x0001
#define	MCP9808_CFG_ALERTPOL	0x0002
#define	MCP9808_CFG_ALERTSEL	0x0004
#define	MCP9808_CFG_ALERTCTRL	0x0008
#define	MCP9808_CFG_ALERTSTAT	0x0008
#define	MCP9808_CFG_INTCLR		0x0020
#define	MCP9808_CFG_WINLOCKED	0x0040
#define	MCP9808_CFG_CRITLOCKED	0x0080
#define	MCP9808_CFG_SHUTDOWN	0x0100

// MCP9808 Sensor resolution
#define MCP9808_RES_CFG_0P5     0x00
#define MCP9808_RES_CFG_0P25    0x01
#define MCP9808_RES_CFG_0P125   0x02
#define MCP9808_RES_CFG_0P0625  0x03


class MCP9808 {
public:
    MCP9808();
    MCP9808(uint8_t address);
    void initialize();
    bool testConnection();
    
    uint16_t readTempRaw ();
    float readTempC ();
    float readTempF ();
    float readTempK ();

    uint8_t getResolution ();
    void setResolution (uint8_t res );
    float getLowerTempLimit (void);
    float getUpperTempLimit (void);
    float getCriticalTempLimit (void);

    
private:
 
    uint8_t devAddr;
    uint16_t buffer[2];
    float convertLimitTemp (uint16_t rawData);
};

#endif  /* _MCP9808_H_ */

/*
#ifdef __cplusplus
extern "C" {
#endif

    extern int mcp9808Setup (const int i2cAddress);
    extern float mcp9808ReadTempC (int mcp9808handle);
    extern float mcp9808ReadTempF (int mcp9808handle);
    extern float mcp9808ReadTempK (int mcp9808handle);
    
#ifdef __cplusplus
}
#endif
*/