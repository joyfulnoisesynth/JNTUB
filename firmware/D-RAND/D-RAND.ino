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

  Project:     JoyfulNoise D-RAND
  File:        D-RAND.ino
  Description: 4HP Voltage-Controlled Random Generator

  The JoyfulNoise D-RAND is an 8-bit random voltage generator. On every trigger
  received, a new random voltage is generated between some Low and High bounds.
  These Low and High bounds range from -5V to 5V and are voltage-controllable.

  Many thanks to Ross Fish at Moffenzeef Modular for designing the Deviant
  module which inspired this.

  ----------
  PARAMETERS
  ----------

  PARAM 1 - Low
    Sets the lower bound of the random output.

  PARAM 2 - High
    Sets the upper bound of the random output.

  PARAM 3 - Slew
    Adds digital slew to the output values.

  GATE/TRG - Trigger New Value

  -----
  JNTUB
  -----

  D-RAND is based on the JoyfulNoise Tiny Utility Board (JNTUB), a
  reprogrammable 4HP module with a standard set of inputs and one 8-bit
  analog output.

 */

// JoyfulNoise Tiny Utility Board Library
#include <JNTUB.h>

JNTUB::EdgeDetector trigger;

const uint16_t SLEW_RATE_CURVE[] = {
  0,  // no slew
  150,  // 150ms slew
  1000,  // 1s slew
};
JNTUB::CurveKnob<uint16_t> slewRateKnob(SLEW_RATE_CURVE, NELEM(SLEW_RATE_CURVE));

// Used to time the slew
JNTUB::Stopwatch stopwatch;

uint8_t prevVal;
uint8_t targetVal;

void setup()
{
  JNTUB::setUpFastPWM();
  prevVal = 128;
  targetVal = 128;
}

void loop()
{
  stopwatch.update(millis());
  trigger.update(digitalRead(JNTUB::PIN_GATE_TRG));
  slewRateKnob.update(analogRead(JNTUB::PIN_PARAM3));

  uint16_t lowRaw = analogRead(JNTUB::PIN_PARAM1);
  uint16_t highRaw = analogRead(JNTUB::PIN_PARAM2);

  uint8_t low = map(lowRaw, 0, 1023, 0, 255);
  uint8_t high = map(highRaw, 0, 1023, 0, 255);

  // Prevent low from being greater than high
  if (low > high)
    low = high;

  if (trigger.isRising()) {
    prevVal = targetVal;
    targetVal = random(low, high);
    stopwatch.reset();
  }

  uint16_t slewTimeMs = slewRateKnob.getValue();
  uint32_t timeSinceLastTrg = stopwatch.getTime();

  if (slewTimeMs == 0 || timeSinceLastTrg >= slewTimeMs) {
    JNTUB::analogWriteOut(targetVal);
  } else {
    // Continue slewing to target value
    uint8_t currentVal = map(
        timeSinceLastTrg, 0, slewTimeMs, prevVal, targetVal);
    JNTUB::analogWriteOut(currentVal);
  }
}
