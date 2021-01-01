/*
  Project:     JoyfulNoise Tiny Utility Board Firmware
  File:        Test.ino
  Description: Test sketch for JNTUB library

  ==============================================================================

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
 */

#include <JNTUB.h>

#define READ_PIN A0

#define REPORT_RATE 500L
unsigned long nextReport = 0;

uint8_t val = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  //Io::analogWriteOut(val);
  //delay(50);
  //++val;

  uint16_t param1 = JNTUB::readParam1();
  uint16_t param2 = JNTUB::readParam2();
  uint16_t param3 = JNTUB::readParam3();

  bool gate = JNTUB::readGateTrg();

  uint16_t output_val;
  if (gate) {
    // Output the min of the three inputs.
    output_val = param1;
    if (param2 < output_val)
      output_val = param2;
    if (param3 < output_val)
      output_val = param3;
  } else {
    // Output the max of the three inputs
    output_val = param1;
    if (param2 > output_val)
      output_val = param2;
    if (param3 > output_val)
      output_val = param3;
  }

  JNTUB::analogWriteOut(map(output_val, 0, 1023, 0, 255));

  if (millis() >= nextReport) {
    nextReport += REPORT_RATE;

    char buf[256];
    int len = sprintf(buf, "Param1: %i, Param2: %i, Param3: %i, Gate: %i\n", param1, param2, param3, gate);
    Serial.write(buf, len);

    uint16_t actualVal = analogRead(READ_PIN);
    len = sprintf(buf, "PWM val: %i, actual val: %i\n", output_val, actualVal);
    Serial.write(buf, len);
  }
}
