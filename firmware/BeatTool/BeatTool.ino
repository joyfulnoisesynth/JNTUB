/*
  Copyright (C) 2021  Ben Reeves

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

  ==============================================================================

  Project:     JoyfulNoise Beat Tool
  File:        BeatTool.ino
  Description: 4HP Multi-Purpose Clock Utility Module

  Arduino sketch for Beat Tool.

 */

#include "BeatTool.h"

BeatToolModule module;

#define TEST_CLK_PIN 12
JNTUB::Clock testClock;
uint32_t testTime;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(1);
  testClock.setPeriod(500000); //us
  testClock.start();
  testTime = 0;

  JNTUB::Device::setUpDevice();
  module.setup();
}

void loop()
{


  testClock.update(micros());
  digitalWrite(TEST_CLK_PIN, testClock.getState());

  if (Serial.available()) {
    char action = Serial.read();
    int value = Serial.parseInt();
    if (value) {
      switch (action) {
      case 'p':
        Serial.print("Setting clock period to ");
        Serial.print(value, DEC);
        Serial.println("us");
        testClock.setPeriod(value);
        break;
      case 't':
        Serial.print("Setting time to ");
        Serial.print(value, DEC);
        Serial.println("us");
        break;
      default:
        break;
      }
    }
  }

  uint8_t output = module.loop(JNTUB::Device::getEnvironment());
  JNTUB::Device::writeOutput(output);
}
