/*
  Copyright (C) 2020  Ben Reeves

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

  Project:     JoyfulNoise Tiny Utility Board Firmware
  File:        Test.ino
  Description: Test sketch for JNTUB library

 */

#include <JNTUB.h>

#define READ_PIN A0

#define REPORT_RATE 500L
unsigned long nextReport = 0;

JNTUB::DiscreteKnob knob1(2, 10);
JNTUB::Clock clock;
bool prevGate;

void setup()
{
  JNTUB::Device::setUpDevice();
  Serial.begin(115200);
  Serial.setTimeout(1);
  clock.setPeriod(500); //ms
  clock.start();

  prevGate = false;
}

void loop()
{
  auto inputs = JNTUB::Device::getEnvironment();

  uint16_t param1 = inputs.param1;
  uint16_t param2 = inputs.param2;
  uint16_t param3 = inputs.param3;
  bool gate = inputs.gateTrg;

  knob1.update(param1);
  clock.update(millis());

  if (gate & !prevGate)
    clock.sync();

  JNTUB::Device::writeOutput(clock.getState() ? 255 : 0);

  prevGate = gate;

  if (millis() >= nextReport) {
    nextReport += REPORT_RATE;
    char buf[256];
    int len = sprintf(
      buf,
      "raw1=%i, knob1=%i, inner=%i\n",
      param1,
      knob1.getValue(),
      knob1.mapInnerValue(0, 99));
    Serial.write(buf, len);
  }

  if (Serial.available()) {
    char action = Serial.read();
    int value = Serial.parseInt();
    if (value) {
      switch (action) {
      case 'p':
        Serial.print("Setting clock period to ");
        Serial.print(value, DEC);
        Serial.println("ms");
        clock.setPeriod(value);
        break;
      case 'n':
        Serial.print("Setting num knob values to ");
        Serial.println(value, DEC);
        knob1.setNumValues(value);
        break;
      case 'h':
        Serial.print("Setting knob hysteresis to ");
        Serial.println(value, DEC);
        knob1.setHysteresis(value);
        break;
      default:
        break;
      }
    }
  }
}
