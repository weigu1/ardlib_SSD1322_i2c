/* Display test program for SSD1322 I2C LCD library! (for Newheaven 128x64)
   more info on www.weigu.lu

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
it is in the public domain.*/

#include "SSD1322_I2C.h"

const uint8_t TEENSY_LED =  11;    // LED connected to digital pin 13
int m,n;
//uint8_t c, p;

SSD1322_I2C glcd(7, 6, 5, 4, 3); // rst, bl, en, rw. di

void setup() {
  Serial.begin(115200);         // start serial for output
  Serial.print("Hello");
  pinMode(TEENSY_LED, OUTPUT);
  glcd.begin();
  glcd.clear();  
} 

void loop() {
  digitalWrite(TEENSY_LED, LOW);
  delay(10);
  digitalWrite(TEENSY_LED, HIGH);
  delay(10);
  digitalWrite(TEENSY_LED, LOW);
  glcd.displaylogo();
  long p = millis();
  glcd.setall();      //fill black
  long o = millis();
  Serial.print(o-p);
  glcd.clear();
  testpixel();        // draw many pixel
  testdrawline();     // draw many lines
  testdrawrect();     // draw multiple rectangles
  testfillrect();     // draw multiple filled rectangles  
  glcd.clear();
  testdrawcircle();   // draw mulitple circles  
  glcd.clear();
  testfillcircle();   // multiple black circles  
  glcd.clear();
  testprintchar();     // draw the 128 characters in the font  
}

void testpixel() {
  for (uint8_t i=0; i<128; i+=4) {
    for (uint8_t j=0; j<64; j+=4) { glcd.setpixel(i, j, WHITE); }
  }
  for (uint8_t i=0; i<128; i+=4) {
    for (uint8_t j=0; j<64; j+=4) { glcd.setpixel(i, j, BLACK); }
  }
}

void testdrawline() {
  for (uint8_t i=0; i<128; i+=8) { glcd.drawline(0, 0, i, 63, WHITE); }
  for (uint8_t i=0; i<64; i+=8) { glcd.drawline(0, 0, 127, i, WHITE); }
  for (uint8_t i=0; i<128; i+=8) { glcd.drawline(i, 63, 0, 0, BLACK); }
  for (uint8_t i=0; i<64; i+=8) { glcd.drawline(127, i, 0, 0, BLACK); }
}

void testdrawrect(void) {
  for (uint8_t i=0; i<10; i+=2) glcd.drawrect(i, i, 128-i*2, 64-i*2, BLACK);
  for (uint8_t i=0; i<10; i+=2) glcd.drawrect(i, i, 128-i*2, 64-i*2, WHITE);
}

void testfillrect(void) {
  glcd.fillrect(40, 20, 20, 20, BLACK);
  glcd.fillrect(40, 20, 20, 20, WHITE);
}

void testdrawcircle(void) {
  for (uint8_t i=0; i<30; i+=5) { glcd.drawcircle(63, 31, i, WHITE);}
  for (uint8_t i=0; i<30; i+=5) { glcd.drawcircle(63, 31, i, BLACK);}
}

void testfillcircle(void) {
  for (uint8_t i=0; i<128; i+=20) { glcd.fillcircle(i, 32, 10, WHITE); }
  for (uint8_t i=0; i<128; i+=20) { glcd.fillcircle(i, 32, 10, BLACK); }
}

void testprintchar(void) {
  for (uint8_t i=0; i < 128; i++) { glcd.printchar((i % 21) * 6, (i/21)*8+7, i); }
}
