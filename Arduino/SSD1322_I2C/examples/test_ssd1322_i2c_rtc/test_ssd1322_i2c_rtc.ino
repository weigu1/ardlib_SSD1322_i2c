/*
  RTC test program for SSD1322 I2C LCD library! (for Newheaven 128x64)
  Display date and time from RTC
  For this include RTClib from  NeiroN JeeLabs
  
  If setting RTC you have to add 18 or 108 to year (2018->126=108+18)
  (i could not figure out why :()
   
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
  it is in the public domain.test_ssd1322_i2c_rtc
*/

#include "SSD1322_I2C.h"
#include <RTClib.h> // Arduino include RTCLib NeiroN JeeLabs (or platformio.ini)

const uint8_t TEENSY_LED =  11;    // LED connected to digital pin 13

int m,n;
//uint8_t c, p;

SSD1322_I2C glcd(7, 6, 5, 4, 3); // rst, bl, en, rw. di
DS3231 g_RTC;
DateTime g_CurrTime;
char RTCbuf_date[10], RTCbuf_time[10]; //display clock

void setup() {
  Serial.begin(115200);         // start serial for output
  Serial.print("Hello");
  pinMode(TEENSY_LED, OUTPUT);
  glcd.begin();
  glcd.clear();
  //SetRTC(36,9,28,16,3,00); // 18 + 18
}

void loop() {
  //Serial.println("MAIN");
  digitalWrite(TEENSY_LED, LOW);
  delay(10);
  digitalWrite(TEENSY_LED, HIGH);
  delay(10);
  digitalWrite(TEENSY_LED, LOW);
  GetRTC(RTCbuf_date,RTCbuf_time);
  glcd.print(0,3,RTCbuf_date);
  glcd.print(13,3,RTCbuf_time);
}

void SetRTC(uint8_t y,uint8_t m,uint8_t d,uint8_t h,uint8_t mn,uint8_t s) {
  g_RTC.adjust(DateTime(y,m,d,h,mn,s));
}
void GetRTC(char *buf_date,char *buf_time) {
  g_CurrTime = g_RTC.now();
  sprintf(buf_date,"%02d/%02d/%02d",g_CurrTime.day(),g_CurrTime.month(),g_CurrTime.year()-2018);
  sprintf(buf_time,"%02d/%02d/%02d",g_CurrTime.hour(),g_CurrTime.minute(),g_CurrTime.second());
}
