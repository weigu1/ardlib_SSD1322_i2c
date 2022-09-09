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

SSD1322_I2C oled(7, 6, 5, 4, 3); // rst, bl, en, rw, di
//SSD1322_I2C oled(7, 6, 5, 4, 3, 0x20); // rst, bl, en, rw, di, i2caddress

void setup() {
  Serial.begin(115200);         // start serial for output
  delay(500);
  Serial.print("Hello");
  pinMode(LED_BUILTIN, OUTPUT);
  oled.begin();
  //oled.clear();
  oled.print(1,1,"Hi");
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}
