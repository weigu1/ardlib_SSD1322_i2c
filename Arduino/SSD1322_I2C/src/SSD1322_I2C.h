/*
$Id:$

SSD1322_I2C LCD library! (for NHD-2.7-12864WDW3-M)

Changes on PCB:
No 5V! Connect 5V Pin of Header with 3V
Connect DispPin 19-20 (both 3V: 6800 // mode)
Change Reset from Disp Pin 17 to 16
Gnd to Disp Pin 17
BL to Disp Pin 18

Copyright (C) 2018 Guy WEILER www.weigu.lu

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

some of this code was written by <cstone@pobox.com> originally and
some of this code was written by Limor Fried, Adafruit Industries
it is in the public domain.
*/

#ifndef __SSD1322_I2C_H__
#define __SSD1322_I2C_H__

#include "Arduino.h"

//#define MCP23017_addr 0x21
#define MCP23017_PortA 0x12  // data
#define MCP23017_PortB 0x13  // res, bl, e, rw, di, 3PB

#define swap_it(a, b) { uint8_t t = a; a = b; b = t; }

#define BLACK 1
#define WHITE 0

#define LCDWIDTH                 128
#define LCDHEIGHT                64

// Commands see data sheet SSD1322
#define CMD_ENABLE_GRAYSCALE_TABLE 0x00
#define CMD_SET_COLUMN_ADDR        0x15 // +2B start,end (default 0x00,0x3F)
#define CMD_WRITE_RAM              0x5C
#define CMD_READ_RAM               0x5D
#define CMD_SET_ROW_ADDR           0x75 // +2B start,end (default 0x00,0x4F)
#define CMD_SET_REMAP              0xA0 // +1B (default 0x00) 1Bit in 2 Byte
#define CMD_SET_START_LINE         0xA1 // +1B (default 0x00)
#define CMD_SET_OFFSET             0xA2 // +1B (default 0x00)
#define CMD_ENTIRE_DISPLAY_OFF     0xA4
#define CMD_ENTIRE_DISPLAY_ON      0xA5
#define CMD_NORMAL_DISPLAY         0xA6
#define CMD_INVERSE_DISPLAY        0xA7
#define CMD_ENABLE_PARTIAL_DISPLAY 0xA8 // +2B start,end
#define CMD_EXIT_PARTIAL_DISPLAY   0xA9
#define CMD_FUNCTION_SELECTION     0xAB // +1B (0x00 ext VDD, 0x01 int VDD)
#define CMD_DISPLAY_OFF            0xAE // SLEEP_MODE_ON
#define CMD_DISPLAY_ON             0xAF // SLEEP_MODE_OFF
#define CMD_SET_PHASE_LENGTH       0xB1 // +1B (default 0x53)
#define CMD_CLKDIV_OSCFREQ         0xB3 // +1B (default 0x41)
#define CMD_SET_VSL                0xB4 // +2B
#define CMD_SETGPIO                0xB5 // +1B
#define CMD_SET_SEC_RECH_PERIOD    0xB6 // +1B
#define CMD_GRAYSCALE_TABLE        0xB8 // +15B
#define CMD_SELECT_DEF_GRAYSCALE_T 0xB9
#define CMD_SET_PRECHARGE_VOLT     0xBB // +1B
#define CMD_SET_VCOMH_VOLTAGE      0xBE // +1B
#define CMD_SET_CONTRAST_CURRENT   0xC1 // +1B (default 0x40)
#define CMD_MASTER_CONTRAST_CURR   0xC7 // +1B
#define CMD_SET_MUX_RATIO          0xCA // +1B
#define CMD_SET_COMMAND_LOCK       0xFD // +1B

class SSD1322_I2C {
 public:
  // 8 Data Pins on MCP23017 PORTA
  SSD1322_I2C(uint8_t rst, uint8_t bl, uint8_t en, uint8_t rw, uint8_t di):
              rst(rst), bl(bl), en(en), rw (rw),di(di){}
  SSD1322_I2C(uint8_t rst, uint8_t bl, uint8_t en, uint8_t rw, uint8_t di, uint8_t i2caddr):
              rst(rst), bl(bl), en(en), rw (rw),di(di), i2caddr (i2caddr){}
  void pinPortB(uint8_t c, boolean l);
  void setPort(uint8_t port, uint8_t c);
  uint8_t getPort(uint8_t port);
  void command(uint8_t c);
  void datawrite(uint8_t c);
  uint8_t dataread();
  void setColumn(uint8_t xStart, uint8_t xEnd);
  void setRow(uint8_t yStart, uint8_t yEnd);
  void init(void);
  void begin(void);
  void begin(uint8_t contrast);
  void begin(int x, int y, uint8_t contrast);
  void setbrightness(uint8_t val);
  void clear(void);
  void setall(void);
  void setpixel(uint8_t x, uint8_t y, uint8_t color);
  void readRAM(uint8_t x, uint8_t y);
  void setbyte(uint8_t x, uint8_t p, uint8_t b);
  void set6bit(uint8_t x, uint8_t p, uint8_t b);
  void drawline(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
  void fillcircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
  void drawcircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
  void drawrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
  void fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
  void displaylogo();
  void printchar(uint8_t x, uint8_t line, char c);
  void clearchar(uint8_t x, uint8_t line, char c);
  void drawstring(uint8_t x, uint8_t line, const char *str);
  void clearstring(uint8_t x, uint8_t line, const char *str);
  void drawbitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w,
                  uint8_t h, uint8_t color);

  void setCursor(uint8_t col, uint8_t row);
  void print(uint8_t x, uint8_t line, const char *c);
  void print(uint8_t line, const char *c);
  void print(const char *c);
  void print(int i);
  void write(char c);

  private:
  uint8_t rst, bl, en, rw, di;
  int gx, gy;
  uint8_t i2caddr = 0x20;
};

#endif
