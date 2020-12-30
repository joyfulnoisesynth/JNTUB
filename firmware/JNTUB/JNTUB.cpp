/*
  Project:     JoyfulNoise Tiny Utility Board Firmware
  File:        JNTUB.cpp
  Description: JNTUB Common Library Implementation

  Common functionality for all JNTUB-based modules.

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

#include "JNTUB.h"

#include <Arduino.h>

namespace JNTUB {

namespace Io {

#if defined(__AVR_ATtiny85__)

  const uint8_t PIN_PARAM1 = A1;  // ADC1, chip pin 7
  const uint8_t PIN_PARAM2 = A2;  // ADC2, chip pin 3
  const uint8_t PIN_PARAM3 = A3;  // ADC3, chip pin 2
  const uint8_t PIN_GATE   = 0;   // PB0, chip pin 5
  const uint8_t PIN_OUT    = 1;   // PB1/OC1A, chip pin 6

#elif defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

  // I used a regular old Arduino Uno for initial testing.
  const uint8_t PIN_PARAM1 = A3;  // ADC3, chip pin 26
  const uint8_t PIN_PARAM2 = A4;  // ADC4, chip pin 27
  const uint8_t PIN_PARAM3 = A5;  // ADC5, chip pin 28
  const uint8_t PIN_GATE   = 8;   // PB0, chip pin 14
  const uint8_t PIN_OUT    = 9;   // PB1/OC1A, chip pin 15

#else

  static_assert(false, "Pin mappings not defined for this AVR chip");

#endif

uint16_t readParam1()
{
  return analogRead(PIN_PARAM1);
}
uint16_t readParam2()
{
  return analogRead(PIN_PARAM2);
}
uint16_t readParam3()
{
  return analogRead(PIN_PARAM3);
}
bool readGate()
{
  // Gate signal is inverted at the chip's input pin.
  return !digitalRead(PIN_GATE);
}

void digitalWriteOut(bool value)
{
  digitalWrite(PIN_OUT, value);
}
void analogWriteOut(uint8_t value)
{
  analogWrite(PIN_OUT, value);
}

}  //JNTUB::Io

}  //JNTUB
