// ----------------------------------------------------------------
// Copyright Erwin Bejsta 2017
// Version 1.0 - 18 Feb 2017
//      First Release for APRSlog on VK2RAY
// ----------------------------------------------------------------

#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

class LCD_I2C {
public:
    // Constructor
    LCD_I2C ();
    /**
     *  Initialise i2c and lcd_i2c_t structure.
     * @param rows number of LCD display rows
     * @param columns number of LCD display columns
     * @param address I2C address of PCF8574, pass 0 to use default address
     * @return The 'file descriptor' for the device (as returned by wiringPiI2CSetup) on success, -1 on error with errno set
     */
    int lcd_i2c_setup(int rows, int columns, uint8_t address);
    
    /**
     * Initalise the lcd by doing a software reset, clearing and positioning cursor in home position
     */
    void lcd_i2c_init();
    
    /**
     * Write an instruction byte to the lcd
     * @param data The instruction byte
     */
    void lcd_i2c_write_i(uint8_t data);
    
    /**
     * Write a data byte to the lcd
     * @param data The data byte
     */
    void lcd_i2c_write_d(uint8_t data);
    
    /**
     * Move the cursor location to specified coords.
     * Note that no range checking is done
     * @param xpos The x coord
     * @param ypos The y coord
     */
    void lcd_i2c_gotoxy(uint8_t xpos, uint8_t ypos);
    
    /**
     * Write to lcd with printf like functionallity
     * @param format_p The format string, followed by the parameters
     */
    void lcd_i2c_printf(char* format_p, ...);
    
    /**
     * Clear the lcd
     * @param lcd Pointer to lcd_i2c_t structure
     */
    void lcd_i2c_clear();
    
    /**
     * Move curson to home position
     * @param lcd Pointer to lcd_i2c_t structure
     */
    void lcd_i2c_home();
    
    /**
     * Display passed character at current position
     * @param c The character to be displayed
     */
    void lcd_i2c_putc(char c);
    
    /**
     * Display passed string at current position
     * @param str Pointer to the string to be displayed
     */
    void lcd_i2c_puts(const char* str);
    
    /** 
     * Turn backlight on or off
     * @param on Turn on if non-zero, off otherwise
     */
    void lcd_i2c_backlight(uint8_t on);

private:
    
    void lcd_i2c_e_assert();
    void lcd_i2c_write(uint8_t data);
    
    uint8_t i2c_address;
    // i2c file descriptor/identifier
    int fd;
    // current pcf8574  output
    uint8_t output;
    // number of rows and columns
    uint8_t rows, cols;
    // row and column coord of current cursor position
    // for a 1602 display, bottom right is (15,1), home is (0,0)
    uint8_t x,y;
    
};

#endif
