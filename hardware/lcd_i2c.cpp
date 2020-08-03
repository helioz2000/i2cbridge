// ----------------------------------------------------------------
// Copyright Erwin Bejsta 2017
// Version 1.0 - 18 Feb 2017
//      First Release for APRSlog on VK2RAY
// ----------------------------------------------------------------
#include "lcd_i2c.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

// LCD Commands
// ---------------------------------------------------------------------------
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
// ---------------------------------------------------------------------------
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off and cursor control
// ---------------------------------------------------------------------------
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
// ---------------------------------------------------------------------------
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
// ---------------------------------------------------------------------------
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00

// convenience macros
#define LCD_I2C_BACKLIGHT_ON() lcd_i2c_backlight(1)
#define LCD_I2C_BACKLIGHT_OFF() lcd_i2c_backlight(0)
// Display, cursor and blink on/off control.
#define LCD_I2C_DISPLAY_CONTROL(display, cursor, blink) lcd_i2c_write_i ( LCD_DISPLAYCONTROL | (display << 2) | (cursor << 1) | (blink) )
#define LCD_I2C_CURSOR_ON() LCD_I2C_DISPLAY_CONTROL(1,1,0)
#define LCD_I2C_CURSOR_OFF() LCD_I2C_DISPLAY_CONTROL(1,0,0)
#define LCD_I2C_CURSOR_BLINK_ON() LCD_I2C_DISPLAY_CONTROL(1,1,1)
#define LCD_I2C_CURSOR_BLINK_OFF() LCD_I2C_DISPLAY_CONTROL(1,1,0)

// pcf8574 bit positions of the lcd control pins
// bits 4 to 7 are the data bits
#define LCD_I2C_RS 0
#define LCD_I2C_RW 1
#define LCD_I2C_E  2
#define LCD_I2C_BACKLIGHT 3

// convenience macros
// write current value of output variable to device
#define LCD_I2C_WRITE() wiringPiI2CWrite(fd, output);
// enable
#define LCD_I2C_E_HI()  output |= (1<<LCD_I2C_E)
#define LCD_I2C_E_LO()  output &=~ (1<<LCD_I2C_E)
// set data or instruction mode
#define LCD_I2C_RS_D() output |= (1<<LCD_I2C_RS)
#define LCD_I2C_RS_I() output &=~ (1<<LCD_I2C_RS)
// set the data nibble -
// note: will need to change this for devices with different wiring.
#define LCD_I2C_DATA_NIBBLE(x) output= (x<<4 | (output & 0x0f))
//! set the lcd data ram address to that passed, only lower 7 bits are used.
#define LCD_I2C_SET_DD_RAM_ADDRESS(address) lcd_i2c_write_i(LCD_SETDDRAMADDR | ((address) & 0x7f) )

// default i2c address
#define LCD_I2C_PCF8574_DEFAULT_ADDRESS 0x3F

LCD_I2C::LCD_I2C ()
{
    i2c_address = LCD_I2C_PCF8574_DEFAULT_ADDRESS;
}

// -----------------------------------------------------
// private methods
// -----------------------------------------------------
void LCD_I2C::lcd_i2c_e_assert()
{
    LCD_I2C_E_LO();
    LCD_I2C_WRITE();
    LCD_I2C_E_HI();
    LCD_I2C_WRITE();
    LCD_I2C_E_LO();
    LCD_I2C_WRITE();
}

//! Write passed data to device
//! Note: RS must be set prior to calling
void LCD_I2C::lcd_i2c_write(uint8_t data)
{
   // high nibble
   LCD_I2C_DATA_NIBBLE((data >> 4));
   lcd_i2c_e_assert();
   // low nibble
   LCD_I2C_DATA_NIBBLE((data & 0x0f));
   lcd_i2c_e_assert();
   // delay for command to be executed
   delayMicroseconds(50);
}

// -----------------------------------------------------
// public methods
// -----------------------------------------------------

int LCD_I2C::lcd_i2c_setup(int rows, int columns, uint8_t address)
{
    if (address > 0) {
        i2c_address = address;
    }
    output=0;
    fd=wiringPiI2CSetup(i2c_address);
    rows=rows;
    cols=columns;
    return fd;
}

void LCD_I2C::lcd_i2c_init()
{
    LCD_I2C_E_LO();
    LCD_I2C_RS_I();
    LCD_I2C_WRITE();
    
    // software reset
    LCD_I2C_DATA_NIBBLE(0x3);
    lcd_i2c_e_assert();
    delay(5); // ms
    lcd_i2c_e_assert();
    delayMicroseconds(150);
    lcd_i2c_e_assert();
    delayMicroseconds(150);
    
    // set 4 bit mode
    LCD_I2C_DATA_NIBBLE(0x02);
    lcd_i2c_e_assert();
    
    // entry mode - 0x06 is display shift on, increment address counter
    lcd_i2c_write_i(LCD_ENTRYMODESET | LCD_ENTRYLEFT);
    
    // set cursor
    LCD_I2C_CURSOR_BLINK_OFF();
    LCD_I2C_CURSOR_OFF();
    
    // clear and home
    lcd_i2c_clear();
    lcd_i2c_home();
}

void LCD_I2C::lcd_i2c_gotoxy(uint8_t xpos, uint8_t ypos)
{
    // note: on two line devices, second line begins at address 0x40
    // XXX so this will work with 1 and 2 line devices, but probabaly not with 4.
    LCD_I2C_SET_DD_RAM_ADDRESS( ypos*0x40 + xpos  );
    x=xpos;
    y=ypos;
}

void LCD_I2C::lcd_i2c_printf(char* format, ...)
{
    va_list args;
    char *spp=NULL;
    
    va_start(args, format);
    int r=vasprintf(&spp, format, args);
    if(r!=-1){
        lcd_i2c_puts(spp);
        free(spp);
    }
    va_end(args);
}

void LCD_I2C::lcd_i2c_write_i(uint8_t data)
{
   LCD_I2C_RS_I();
   lcd_i2c_write(data);
}

void LCD_I2C::lcd_i2c_write_d(uint8_t data)
{
   LCD_I2C_RS_D();
   lcd_i2c_write(data);
}

void LCD_I2C::lcd_i2c_clear()
{
    lcd_i2c_write_i(LCD_CLEARDISPLAY);
    delay(2); // delay for command to take effect
}

void LCD_I2C::lcd_i2c_home()
{
    lcd_i2c_write_i(LCD_RETURNHOME);
    x=0;
    y=0;
    delay(2); // delay for command to take effect
}

void LCD_I2C::lcd_i2c_putc(char c)
{
    lcd_i2c_write_d(c);
    //    printf("c: %c (%i,%i)\n",c, lcd->x, lcd->y);
    if(++x == cols){
        x=0;
        if(++y == rows){
            y=0;
        }
    }
    lcd_i2c_gotoxy(x,y);
}

void LCD_I2C::lcd_i2c_puts(const char* str)
{
    while(*str!='\x0'){
        lcd_i2c_putc(*str++);
    }
}

/**
 * Turn backlight on or off
 * @param on Turn on if non-zero, off otherwise
 */
void LCD_I2C::lcd_i2c_backlight(uint8_t on)
{
    if(on)
        output |= (1<<LCD_I2C_BACKLIGHT);
    else
        output &=~ (1<<LCD_I2C_BACKLIGHT);
    LCD_I2C_WRITE();
}
