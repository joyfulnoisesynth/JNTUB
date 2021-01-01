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
  File:        JNTUB.h
  Description: JNTUB Common Library Declarations

  Common functionality for all JNTUB-based modules.

 */

#ifndef JNTUB_H_
#define JNTUB_H_

#include <stdint.h>

namespace JNTUB {
  /*
   * =======================================================================
   * INPUTS AND OUTPUT
   * =======================================================================
   *
   * All modules based on the JoyfulNoise Tiny Utility Board have the same
   * set of inputs and one output.
   *
   * From top to bottom on the front panel, they are:
   *  - PARAM 3:
   *    - Voltage values: 0V to 5V from a single knob
   *    - Digital values: 0 to 1023
   *
   *  - PARAM 2:
   *    - Voltage values: -5V to 5V, summed from a knob and CV input
   *    - Digital values: 0 to 1023
   *
   *  - PARAM 1:
   *    - Voltage values: -5V to 5V, summed from a knob and CV input
   *    - Digital values: 0 to 1023
   *
   *  - GATE/TRG:
   *    - Voltage values: 0V or 5V, arbitrary input CV converted to gate
   *    - Digital values: 0 or 1
   *
   *  - OUT:
   *    - Voltage values: -5V to 5V (0V to 5V configurable with jumper)
   *    - Digital values: 0 to 255(*)
   *
   * (*) OUT can be used as either a PWM analog output or a digital output.
   */

  extern const uint8_t PIN_PARAM1;
  extern const uint8_t PIN_PARAM2;
  extern const uint8_t PIN_PARAM3;
  extern const uint8_t PIN_GATE_TRG;
  extern const uint8_t PIN_OUT;

  uint16_t readParam1();
  uint16_t readParam2();
  uint16_t readParam3();
  bool readGateTrg();

  /**
   * ON THE TOPIC OF PWM
   * -------------------
   *
   * JNTUB uses PWM to generate an "analog" output signal from the ATtiny.
   * The default PWM frequency on these AVRs is only around 500 Hz though!!
   * That would be quite noticeable in a modular synthesizer.
   *
   * Luckily, the PWM frequency can be increased, but it requires some
   * real fuckery with various registers and lots of datasheet reading.
   * I do my best to explain it all in the implementation, but you needn't
   * worry about the dirty details if you just want to use this library.
   */

  void digitalWriteOut(bool value);
  void analogWriteOut(uint8_t value);

  /*
   * =======================================================================
   * UTILITY CLASSES
   * =======================================================================
   */

  /**
   * A knob that selects between a finite number of "categories" or "values"
   * with built-in hysteresis.
   *
   * By "hysteresis" I mean that it does not simply do inputVal / numValues.
   * Doing so would be susceptible to noise when the input is at or near
   * one of the "cutoffs".
   *
   * Example, with numValues=4 and hysteresis=10:
   *  - The nominal cutoff from value 0 to value 1 would be 1024/4 = 128.
   *  - getValue() will not return 1 until value exceeds 138.
   *  - getValue() will not return 0 again until value goes below 118.
   */
  class DiscreteKnob {
  public:
    // numValues min: 1
    //           max: 128
    // hysteresis min: 0
    // hysteresis max: (1024 / numValues) / 2
    DiscreteKnob(uint8_t numValues, uint8_t hysteresis);

    // Call once per loop with the read analog input value.
    void update(uint16_t value);

    // Retrieve the current discrete value (0 to numValues-1).
    int getValue() const;

  private:
    uint8_t mNumValues;
    uint8_t mHysteresis;
    uint16_t mCurVal;
  };

}  //JNTUB

#endif  //JNTUB_H_
