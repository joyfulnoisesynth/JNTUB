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
  File:        D-RAND.cpp
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

  PARAM 3 - Bias
    Biases the output towards either end of the configured output range.
      - 12 o'clock = voltages evenly distributed between Low and High
      - All the way left = always outputs Low
      - All the way right = always outputs High


  GATE/TRG - Trigger New Value

  -----
  JNTUB
  -----

  D-RAND is based on the JoyfulNoise Tiny Utility Board (JNTUB), a
  reprogrammable 4HP module with a standard set of inputs and one 8-bit
  analog output.

 */

#include <Arduino.h>

#include "D-RAND.h"

/**
 * =============================================================================
 *                           MODULE IMPLEMENTATION
 * =============================================================================
 */

class D_Rand {
private:
  JNTUB::EdgeDetector trigger;
  uint8_t output;

public:
  D_Rand() {}

  void setup()
  {
    output = 128;
  }

  uint8_t loop(const JNTUB::Environment &env)
  {
    trigger.update(env.gateTrg);

    uint16_t lowRaw = env.param1;
    uint16_t highRaw = env.param2;

    uint8_t low = map(lowRaw, 0, 1023, 0, 255);
    uint8_t high = map(highRaw, 0, 1023, 0, 255);

    // Prevent low from being greater than high
    if (low > high)
      low = high;

    if (trigger.isRising()) {
      int unbiased = random(low, high);

      uint16_t biasRaw = env.param3;
      int16_t biasScaled = map(biasRaw, 0, 1023, -256, 255);

      // Pick a random number between 0 and bias to add to the generated value
      int bias;
      if (biasScaled < 0)
        bias = random(biasScaled, 0);
      else
        bias = random(0, biasScaled);

      output = (uint8_t)(unbiased + bias);
    }

    return output;
  }
};

D_Rand instance;

D_RandModule::D_RandModule()
{
  mImpl = &instance;
}

void D_RandModule::setup()
{
  mImpl->setup();
}

uint8_t D_RandModule::loop(const JNTUB::Environment &env)
{
  return mImpl->loop(env);
}
