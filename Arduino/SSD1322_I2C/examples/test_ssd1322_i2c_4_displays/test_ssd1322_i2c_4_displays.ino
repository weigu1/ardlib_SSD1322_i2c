/* Display test program for SSD1322 I2C LCD library! (for Newheaven 128x64)
   Using two displays with different addresses
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

SSD1322_I2C oled1(7, 6, 5, 4, 3); // rst, bl, en, rw, di (default = 0x20)
SSD1322_I2C oled2(7, 6, 5, 4, 3, 0x21); // rst, bl, en, rw, di, i2caddr
SSD1322_I2C oled3(7, 6, 5, 4, 3, 0x22); // rst, bl, en, rw, di, i2caddr
SSD1322_I2C oled4(7, 6, 5, 4, 3, 0x23); // rst, bl, en, rw, di, i2caddr

void setup() {
  Serial.begin(115200);         // start serial for output
  delay(500);
  Serial.print("Hello");
  pinMode(LED_BUILTIN, OUTPUT);
  oled1.begin();
  oled1.clear();
  oled1.print(1,1,"First display");
  oled1.print(1,3,"Default addr. 0x20");
  oled2.begin();
  oled2.clear();
  oled2.print(1,1,"Second display");
  oled2.print(1,3,"Address 0x21");
  oled3.begin();
  oled3.clear();
  oled3.print(1,1,"Third display");
  oled3.print(1,3,"Address 0x22");
  oled4.begin();
  oled4.clear();
  oled4.print(1,1,"Fourth display");
  oled4.print(1,3,"Address 0x23");
} 

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  yield();
}
