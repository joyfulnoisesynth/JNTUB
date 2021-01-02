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
  File:        JNTUB.cpp
  Description: JNTUB Common Library Implementation

  Common functionality for all JNTUB-based modules.

 */

#include "JNTUB.h"

#include <avr/io.h>
#include <Arduino.h>

#if defined(__AVR_ATtiny85__)
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#else
#error This AVR board is not supported
#endif

namespace JNTUB {

/**
 * ===========================================================================
 *
 *                             PIN MAPPINGS
 *
 * ===========================================================================
 */

#if defined(__AVR_ATtiny85__)

  const uint8_t PIN_PARAM1   = A1;  // ADC1, chip pin 7
  const uint8_t PIN_PARAM2   = A2;  // ADC2, chip pin 3
  const uint8_t PIN_PARAM3   = A3;  // ADC3, chip pin 2
  const uint8_t PIN_GATE_TRG = 0;   // PB0, chip pin 5
  const uint8_t PIN_OUT      = 1;   // PB1/OC1A, chip pin 6

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

  // I used a regular old Arduino Uno for initial testing.
  const uint8_t PIN_PARAM1   = A3;  // ADC3, chip pin 26
  const uint8_t PIN_PARAM2   = A4;  // ADC4, chip pin 27
  const uint8_t PIN_PARAM3   = A5;  // ADC5, chip pin 28
  const uint8_t PIN_GATE_TRG = 8;   // PB0, chip pin 14
  const uint8_t PIN_OUT      = 9;   // PB1/OC1A, chip pin 15

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
bool readGateTrg()
{
  return digitalRead(PIN_GATE_TRG);
}

static bool fastPWMInitialized = false;
void setUpFastPWM();

// The PWM generator is always running, but that does not necessarily mean
// that the PWM signal is being written to PIN_OUT all the time.
// Certain registers have to be set in order for the PWM signal to affect
// the state of a pin.
static bool pwmOutputEnabled = false;
static void enablePwmOutput();
static void disablePwmOutput();

void digitalWriteOut(bool value)
{
  if (pwmOutputEnabled)
    disablePwmOutput();

  digitalWrite(PIN_OUT, value);
}

void analogWriteOut(uint8_t value)
{
  if (!fastPWMInitialized)
    setUpFastPWM();

  analogWrite(PIN_OUT, value);

  if (!pwmOutputEnabled)
    enablePwmOutput();
}

static void enablePwmOutput()
{
  // Need to also make sure pin is in output mode.
  pinMode(PIN_OUT, OUTPUT);

  pwmOutputEnabled = true;
}

static void disablePwmOutput()
{
  pwmOutputEnabled = false;
}

/**
 * OK, ALL ABOUT FAST PWM
 * ----------------------
 * I will improve this explanation, I'm tired right now.
 *
 * Most (if not all) AVR boards have at least two Timer/Counters.
 *
 * The whole reason I need a custom implementation of analogWrite is
 * because Arduino's implementation chooses to base its PWM output on
 * Timer/Counter0, the same timer which the Arduino library relies upon for
 * timing functions like millis() and micros(). One way I could increase
 * the PWM frequency is to decrease the prescaler on Timer/Counter0 from 64
 * down to 1, but that would make functions like millis() and micros()
 * completely inaccurate. That's lame.
 *
 * Furthermore, Timer/Counter0 is locked to only 8MHz on ATtiny85 (that's the
 * speed of the internal oscillator). In Fast PWM mode, every PWM period
 * lasts exactly 256 counts, so 8MHz / 256 = maximum 31.25kHz PWM frequency.
 * Unless I were to set my PWM filter cutoff to something as low as 1kHz,
 * the output signal would have unavoidable jitter that could cause weird
 * audio artifacts.
 *
 * The solution to all these problems is to use Timer/Counter1 for PWM.
 * This has two great benefits:
 *
 *  - Timer/Counter0's prescaler remains unaffected, so I can use Arduino
 *    library functions to accurately keep time (which is important for music).
 *
 *  - Timer/Counter1 can have its clock source set to the internal PLL
 *    (phase-locked loop) generator rather than the 8MHz system clock. This
 *    means that I can count at 64MHz, 8 times faster!
 *
 * With a base clock of 64MHz and a prescaler of 1, that gets me
 * 64MHz / 256 = 250kHz of PWM. If I then filter that signal through a
 * ~5kHz lowpass filter, I get a signal with minimal jitter and a decent enough
 * bandwidth for nearly any kind of audio you would want to play through the
 * board (aside from maybe Ariana Grande or a dog whistle).
 */

void setUpFastPWM()
{
  /* TODO: make fast PWM work for ATmega328p and ATtiny85 */

  fastPWMInitialized = true;
}

/**
 * ===========================================================================
 *
 *                            UTILITY CLASSES
 *
 * ===========================================================================
 */

/**
 * ============================================================================
 * DiscreteKnob
 * ============================================================================
 */

DiscreteKnob::DiscreteKnob(uint8_t numValues, uint8_t hysteresis)
{
  mNumValues = numValues;
  mHysteresis = hysteresis;
  mCurVal= 0;
}

void DiscreteKnob::update(uint16_t value)
{
  uint16_t stepSize = 1024 / mNumValues;
  uint16_t curStep = mCurVal * stepSize;

  // Calculate the values that need to be passed in order to go up/down a value.
  int nextStepDown, nextStepUp;

  // If current val is 0, can't go down
  if (mCurVal == 0)
    nextStepDown = 0;
  else
    nextStepDown = curStep - stepSize - mHysteresis;
  nextStepDown = constrain(nextStepDown, 0, 1023);

  // If current val is numValues-1, can't go up
  if (mCurVal == mNumValues - 1)
    nextStepUp = 1024;
  else
    nextStepUp = curStep + stepSize + mHysteresis;
  nextStepUp = constrain(nextStepUp, 0, 1023);

  if (value < nextStepDown || value > nextStepUp) {
    mCurVal = value / stepSize;
  }
}

int DiscreteKnob::getValue() const
{
  return mCurVal;
}

/**
 * ============================================================================
 * Clock
 * ============================================================================
 */

Clock::Clock(uint32_t period)
{
  mPeriod = period;
  mDuty = (uint8_t)(PHASE_MAX / 2);
  stop();
}

uint32_t Clock::getPeriod() const
{
  return mPeriod;
}
void Clock::setPeriod(uint32_t period)
{
  mPeriod = period;
}

uint8_t Clock::getPhase() const
{
  return mCurPhase;
}

uint8_t Clock::getDuty() const
{
  return mDuty;
}
void Clock::setDuty(uint8_t duty)
{
  mDuty = duty;
}

bool Clock::getState() const
{
  return mCurState;
}
bool Clock::isRising() const
{
  return !mPrevState & mCurState;
}
bool Clock::isFalling() const
{
  return mPrevState & !mCurState;
}

void Clock::start(uint32_t time)
{
  mRunning = true;
  sync(time);
}

void Clock::stop()
{
  mRunning = 0;
  mCurPhase = 0;
  mCurState = 0;
  mPrevState = 0;
}

void Clock::sync(uint32_t time, uint8_t phase)
{
  if (!mRunning)
    return;

  mCurPhase = phase;
  updateState();
}

void Clock::update(uint32_t time)
{
  if (!mRunning)
    return;

  // TODO: detect overflow?
  uint32_t timeWithinPeriod = (time - mLastRisingEdge) % mPeriod;
  mCurPhase = map(timeWithinPeriod, 0, mPeriod-1, 0, PHASE_MAX-1);

  updateState();
}

void Clock::updateState()
{
  mPrevState = mCurState;
  mCurState = mCurPhase < mDuty;

  if (isRising())
    mLastRisingEdge = time;
}

}  //JNTUB
