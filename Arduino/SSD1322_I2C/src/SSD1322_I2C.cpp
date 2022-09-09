/* SSD1322_I2C LCD library! (for NHD-2.7-12864WDW3-M)

Changes on PCB:
No 5V! Connect 5V Pin of Header with 3V
Connect DispPin 19-20 (both 3V)
Change Reset from Disp Pin 17 to 16
Gnd to Disp Pin 17
BL to Disp Pin 18

create logo with gimp (128x64) and save as *.xbm. Convert with xbm2SSD1322_I2C.py


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


Display mapping: 256x128 in 480x128
First Column = 28 Last Column = 92 (2 Pixel/Byte; 28+64-1)
First Row = 0 Last Row = 63

 */

#ifdef __AVR__
  #include <avr/pgmspace.h>
  #include <util/delay.h>
#endif

#include <stdlib.h>
#include "SSD1322_I2C.h"
#include "logo.h" //creative-lab logo
#include <Wire.h> // I2C

/*#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif*/

uint8_t is_reversed = 0;
const extern uint8_t PROGMEM font[]; // a 5x7 font table

void SSD1322_I2C::pinPortB(uint8_t c, boolean l) {
  uint8_t PortB_MCP23017 = 0;
  Wire.beginTransmission(i2caddr);
  Wire.write(MCP23017_PortB);             // PortB
  Wire.endTransmission();
  Wire.requestFrom(i2caddr, (uint8_t)1);     // get PortB
  PortB_MCP23017 = Wire.read();
  if (l) { PortB_MCP23017 |= (1 << c); }  // HIGH
  else { PortB_MCP23017 &= ~(1 << c); } // LOW
  Wire.beginTransmission(i2caddr);
  Wire.write(MCP23017_PortB);
  Wire.write(PortB_MCP23017);
  Wire.endTransmission();
}

void SSD1322_I2C::setPort(uint8_t port, uint8_t c) {
  Wire.beginTransmission(i2caddr);
  Wire.write(port);  // Port address
  Wire.write(c);     // data
  Wire.endTransmission();
}

uint8_t SSD1322_I2C::getPort(uint8_t port) {
  uint8_t data;
  Wire.beginTransmission(i2caddr);
  Wire.write(port);      // Port address
  Wire.endTransmission();
  Wire.requestFrom(i2caddr, 1);     // get Port
  return( Wire.read());
}

void SSD1322_I2C::command(uint8_t c) {
  pinPortB(di,LOW);	// command
  setPort(MCP23017_PortA, c);
  pinPortB(rw, LOW);	// write
  pinPortB(en, HIGH);       // rising edge
  pinPortB(en, LOW);       // needed!
}

void SSD1322_I2C::datawrite(uint8_t c) {
  pinPortB(di, HIGH);	// data
  setPort(MCP23017_PortA, c);
  pinPortB(rw, LOW);	// write
  pinPortB(en, HIGH);
  pinPortB(en, LOW);        //needed!
}

uint8_t SSD1322_I2C::dataread() {
  Wire.beginTransmission(i2caddr); // PortA = Input
  Wire.write(0x00);             // IODIRA register (Bank = 0)
  Wire.write(0xFF);             // set all of port A to input
  Wire.endTransmission();
  pinPortB(di, HIGH);	// data
  pinPortB(rw, HIGH);	// read
  pinPortB(en, LOW);
  pinPortB(en, HIGH);
  //uint8_t c = PINB;
  uint8_t c = getPort(MCP23017_PortA);
  pinPortB(en, LOW);
  Wire.beginTransmission(i2caddr);
  Wire.write(0x00);             // IODIRA register (Bank = 0)
  Wire.write(0x00);             // set all of port A to outputs
  Wire.endTransmission();
  return c;
}

void SSD1322_I2C::setColumn(uint8_t xStart, uint8_t xEnd) {
  command(CMD_SET_COLUMN_ADDR);
  datawrite(xStart);    //column start; 28 is left-most column
  datawrite(xEnd);      //column end; 91 is right-most column
}
void SSD1322_I2C::setRow(uint8_t yStart, uint8_t yEnd) {
  command(CMD_SET_ROW_ADDR);
  datawrite(yStart);    //row start; 0 is top row
  datawrite(yEnd);      //row end; 63 is bottom row
}

void SSD1322_I2C::init(void) {
  #ifdef ESP8266
    Wire.begin(4, 5); // D3 and D4 on ESP8266
  #else
    Wire.begin();     // wake up I2C bus
  #endif // ifdef ESP8266
  Wire.setClock(400000L);       // 400kHz
  Wire.beginTransmission(i2caddr);
  Wire.write(0x00);             // IODIRA register (Bank = 0)
  Wire.write(0x00);             // set all of port A to outputs
  Wire.endTransmission();
  Wire.beginTransmission(i2caddr);
  Wire.write(0x01);             // IODIRB register (Bank = 0)
  Wire.write(0x07);             // 5 out, 3 in
  Wire.endTransmission();       // res, bl, e, rw, di, 3pb
  pinPortB(bl, HIGH);
  pinPortB(en, LOW);
  pinPortB(rst, LOW); // hardware reset the display
  delay(200);
  pinPortB(rst, HIGH);
  delay(200);
  command(CMD_DISPLAY_OFF);
  command(CMD_CLKDIV_OSCFREQ);
  datawrite(0x91); // Set Clock as 135 Frames/Sec (default 0x41?)
  command(CMD_SET_MUX_RATIO);
  datawrite(0x3F); // 1/64 Duty (0x0F~0x5F) def 5F?
  command(CMD_SET_OFFSET);
  datawrite(0x00); // Shift Mapping RAM Counter (0x00~0x5F)
  command(CMD_SET_START_LINE);
  datawrite(0x00); // Set Mapping RAM Display Start Line (0x00~0x5F)
  command(CMD_FUNCTION_SELECTION);//function selection 0xAB
  datawrite(0x01);
  command(CMD_SET_REMAP);//set re-map 0xA0
  datawrite(0x16);
  datawrite(0x11);
  command(CMD_MASTER_CONTRAST_CURR);//master contrast current 0xC7
  datawrite(0x0F);
  command(CMD_SET_CONTRAST_CURRENT);//set contrast current 0xC1
  datawrite(0x9F);
  command(CMD_SET_PHASE_LENGTH);//set phase length 0xB1
  datawrite(0xF2);
  command(CMD_SET_PRECHARGE_VOLT);//set pre-charge voltage 0xBB
  datawrite(0x1F);
  command(CMD_SET_VSL);//set VSL 0xB4
  datawrite(0xA0);
  datawrite(0xFD);
  command(CMD_SET_VCOMH_VOLTAGE);//set VCOMH 0xBE
  datawrite(0x04);
  command(CMD_EXIT_PARTIAL_DISPLAY);
  command(CMD_NORMAL_DISPLAY);
  command(CMD_DISPLAY_ON);//display ON 0xAF
}

void SSD1322_I2C::begin() {
  init();
}
void SSD1322_I2C::begin(uint8_t contrast) {
  init();
}

void SSD1322_I2C::begin(int x, int y, uint8_t contrast) {
  gx = x*6;
  gy = y*8+7;
  init();
}

void SSD1322_I2C::clear(void) {
  setColumn(28,91);
  setRow(0,63);
  command(CMD_WRITE_RAM);
  pinPortB(di, HIGH);	// data
  for(unsigned int i=0;i<4096;i++) {
    setPort(MCP23017_PortA, 0x00);
    pinPortB(rw, LOW);	// write
    pinPortB(en, HIGH);
    pinPortB(en, LOW);        //needed!
    pinPortB(rw, LOW);	// write
    pinPortB(en, HIGH);
    pinPortB(en, LOW);        //needed!
    yield();
  }
}

void SSD1322_I2C::setall(void) {
  setColumn(28,91);
  setRow(0,63);
  command(CMD_WRITE_RAM);
  pinPortB(di, HIGH);	// data
  for(unsigned int i=0;i<4096;i++) {
    setPort(MCP23017_PortA, 0xFF);
    pinPortB(rw, LOW);	// write
    pinPortB(en, HIGH);
    pinPortB(en, LOW);        //needed!
    pinPortB(rw, LOW);	// write
    pinPortB(en, HIGH);
    pinPortB(en, LOW);        //needed!
    yield();
  }
}

void SSD1322_I2C::setpixel(uint8_t x, uint8_t y, uint8_t color) {
  if ((x >= LCDWIDTH) || (y >= LCDHEIGHT)) return;
  uint8_t m,n;
  setColumn(x/2 + 28,x/2 + 28);
  setRow(y,y);
  command(CMD_READ_RAM);
  m = dataread();
  m = dataread();
  n = dataread();
  command(CMD_WRITE_RAM);
  pinPortB(di, HIGH);	// data
  if ((x%2) == 0) {
    if (color == WHITE) {
      setPort(MCP23017_PortA, 0XFF);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
     }
    else {
      setPort(MCP23017_PortA, 0X00);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
     }
    datawrite(n);
  }
  else {
    datawrite(m);
    if (color == WHITE) {
      setPort(MCP23017_PortA, 0XFF);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
     }
    else {
      setPort(MCP23017_PortA, 0X00);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
     }
  }
}

void SSD1322_I2C::readRAM(uint8_t x, uint8_t y) {
  uint8_t m,n;
  x = x + 28;
  setColumn(x,91);
  setRow(y,63);
  command(CMD_READ_RAM);
  m = dataread();
  m = dataread();
  n = dataread();
  Serial.print("ReadRAM ");
  Serial.print(x-28);
  Serial.print(" ");
  Serial.print(y);
  Serial.print('\t');
  Serial.print(m);
  Serial.print(" ");
  Serial.println(n);
}

void SSD1322_I2C::setbyte(uint8_t x, uint8_t y, uint8_t b) {
  uint8_t i;
  setColumn(x/2 + 28,91);
  setRow(y,63);
  command(CMD_WRITE_RAM);
  pinPortB(di, HIGH);	// data
  for(i=0;i<8;i++){
    if (((b<<i)&0x80)==0x80) {
      setPort(MCP23017_PortA, 0XFF);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
    }
    else {
      setPort(MCP23017_PortA, 0x00);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
    }
  }
}

void SSD1322_I2C::set6bit(uint8_t x, uint8_t y, uint8_t b) {
  uint8_t i;
  setColumn(x/2 + 28,91);
  setRow(y,63);
  command(CMD_WRITE_RAM);
  pinPortB(di, HIGH);	// data
  for(i=2;i<8;i++){
    if (((b<<i)&0x80)==0x80) {
      setPort(MCP23017_PortA, 0XFF);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
    }
    else {
      setPort(MCP23017_PortA, 0X00);
      pinPortB(rw, LOW);	// write
      pinPortB(en, HIGH);
      pinPortB(en, LOW);        //needed!
    }
  }
}

void SSD1322_I2C::displaylogo(void) {
  setColumn(28,91);
  setRow(0,63);
  command(CMD_WRITE_RAM);
  for(unsigned int i=0;i<1024;i++) {
    setbyte((i%16)*8,i/16, pgm_read_byte(&logo[i]));
    yield();
  }
}

void SSD1322_I2C::drawline(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
		      uint8_t color) {  // bresenham's algorithm - thx wikpedia
  uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap_it(x0, y0);
    swap_it(x1, y1);}
  if (x0 > x1) {
    swap_it(x0, x1);
    swap_it(y0, y1);}
  uint8_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);
  int8_t err = dx / 2;
  int8_t ystep;
  if (y0 < y1) ystep = 1;
  else ystep = -1;
  for (; x0<=x1; x0++) {
    if (steep) setpixel(y0, x0, color);
    else setpixel(x0, y0, color);
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  yield();
  }
}

void SSD1322_I2C::drawrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
		      uint8_t color) {
  for (uint8_t i=x; i<x+w; i++) {
    setpixel(i, y, color);
    setpixel(i, y+h-1, color);
    yield();
  }
  for (uint8_t i=y; i<y+h; i++) {
    setpixel(x, i, color);
    setpixel(x+w-1, i, color);
    yield();
  }
}

void SSD1322_I2C::fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
		      uint8_t color) {
  for (uint8_t i=x; i<x+w; i++) {
    for (uint8_t j=y; j<y+h; j++) setpixel(i, j, color);
    yield();
  }
}

void SSD1322_I2C::drawcircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;
  setpixel(x0, y0+r, color);
  setpixel(x0, y0-r, color);
  setpixel(x0+r, y0, color);
  setpixel(x0-r, y0, color);
  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;}
    x++;
    ddF_x += 2;
    f += ddF_x;
    setpixel(x0 + x, y0 + y, color);
    setpixel(x0 - x, y0 + y, color);
    setpixel(x0 + x, y0 - y, color);
    setpixel(x0 - x, y0 - y, color);
    setpixel(x0 + y, y0 + x, color);
    setpixel(x0 - y, y0 + x, color);
    setpixel(x0 + y, y0 - x, color);
    setpixel(x0 - y, y0 - x, color);
    yield();
  }
}

void SSD1322_I2C::fillcircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;
  for (uint8_t i=y0-r; i<=y0+r; i++) setpixel(x0, i, color);
  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;}
    x++;
    ddF_x += 2;
    f += ddF_x;
    for (uint8_t i=y0-y; i<=y0+y; i++) {
      setpixel(x0+x, i, color);
      setpixel(x0-x, i, color);
    }
    for (uint8_t i=y0-x; i<=y0+x; i++) {
      setpixel(x0+y, i, color);
      setpixel(x0-y, i, color);
    }
    yield();
  }
}

/*void  SSD1322_I2C::printchar(uint8_t x, uint8_t y, char c) {
  for (uint8_t i =0; i<5; i++ ) {
     setbyte(x/2+28, y+i, pgm_read_byte(font+(c*5)+i));
  }
}*/

void  SSD1322_I2C::printchar(uint8_t x, uint8_t y, char c) {
  char data[8];
  for (int i=7; i>-1; i--) {
    data[i] = pgm_read_byte(font+(c*8)+i);
    set6bit(x, y-i,data[i]);
    yield();
  }
}


void  SSD1322_I2C::clearchar(uint8_t x, uint8_t y, char c) {
  for (int i=7; i>-1; i--) {
    set6bit(x, y-i,0x00);
    yield();
  }
}

void SSD1322_I2C::drawstring(uint8_t x, uint8_t line, const char *c) {
  while (c[0] != 0) {
    printchar(x, line, c[0]);
    c++;
    x += 6; // 6 pixels wide
    if (x + 6 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      line++;
    }
    if (line >= (LCDHEIGHT/8))
      return;        // ran out of space :(
  }
}

void SSD1322_I2C::clearstring(uint8_t x, uint8_t line, const char *c) {
  while (c[0] != 0) {
    clearchar(x, line, c[0]);
    c++;
    x += 6; // 6 pixels wide
    if (x + 6 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      line++;
    }
    if (line >= (LCDHEIGHT/8))
      return;        // ran out of space :(
  }
}

void SSD1322_I2C::drawbitmap(uint8_t x, uint8_t y, const uint8_t *bitmap,
                        uint8_t w, uint8_t h, uint8_t color) {
  for (uint8_t j=0; j<h; j++) {
    for (uint8_t i=0; i<w; i++ ) {
      if (pgm_read_byte(bitmap + i + (j/8)*w) & (1<<(j%8))) {
        setpixel(x+i, y-1+j, color);
        yield();
      }
      yield();
    }
  }
}

void SSD1322_I2C::setCursor(uint8_t col, uint8_t row) {
  gx = col*6;
  gy = row*8+7;
}

void SSD1322_I2C::print(uint8_t x, uint8_t line, const char *c) {
  x = x*6;
  uint8_t y = line*8+7;
  while (c[0] != 0) {
    printchar(x, y, c[0]);
    c++;
    x += 6; // 6 pixels wide
    if (x + 6 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      y +=8;
    }
    if (y >= (LCDHEIGHT)) return;        // ran out of space :(
  }
}

void SSD1322_I2C::print(uint8_t line, const char *c) {
  uint8_t x = 0;
  uint8_t y = line*8+7;
  while (c[0] != 0) {
    printchar(x, y, c[0]);
    c++;
    x += 6; // 6 pixels wide
    if (x + 6 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      y +=8;
    }
    if (y >= (LCDHEIGHT)) return;        // ran out of space :(
  }
}

void SSD1322_I2C::print(const char *c) {
  uint8_t x = gx;
  uint8_t y = gy;
  while (c[0] != 0) {
    printchar(x, y, c[0]);
    c++;
    gx=gx+6;
    x += 6; // 6 pixels wide
    if (x + 6 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      y +=8;
      gy++;
    }
    if (y >= (LCDHEIGHT)) return;        // ran out of space :(
  }
}

void SSD1322_I2C::print(int i) {
  char *c;
  uint8_t x = gx;
  uint8_t y = gy;
  itoa(i,c,10);
  while (c[0] != 0) {
    printchar(x, y, c[0]);
    c++;
    gx=gx+6;
    x += 6; // 6 pixels wide
    if (x + 6 >= LCDWIDTH) {
      x = 0;    // ran out of this line
      y +=8;
      gy++;
    }
    if (y >= (LCDHEIGHT/8))
      return;        // ran out of space :(
  }
}
/*void SSD1322_I2C::write(char c) {
  uint8_t x = gx;
  uint8_t line = gy;
  for (uint8_t i =0; i<5; i++ ) {
     setbyte(x, line, pgm_read_byte(font+(c*5)+i));
     x++;
     gx++;
  }
}*/

void SSD1322_I2C::write(char c) {
  uint8_t x = gx;
  uint8_t y = gy;
  char data[8];
  for (int i=7; i>-1; i--) {
    data[i] = pgm_read_byte(font+(c*8)+i);
    set6bit(x, y-i,data[i]);
    gx++;
  }
}
/*void  SSD1322_I2C::printchar(uint8_t x, uint8_t y, char c) {
  char data[8];
  for (int i=7; i>-1; i--) {
    data[i] = pgm_read_byte(font+(c*8)+i);
    set6bit(x, y-i,data[i]);
  }
}*/
