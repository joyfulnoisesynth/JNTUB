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

#include <limits.h>

#include <avr/io.h>
#include <Arduino.h>

#if defined(__AVR_ATtiny85__)
#define DEBUG(...)
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
static char DEBUGBUF[256];
#define DEBUG(...) ({ \
    int len = snprintf(DEBUGBUF, sizeof(DEBUGBUF), ...); \
    Serial.write(DEBUGBUF, len); \
})
#else
#error This AVR board is not supported
#endif

namespace JNTUB {

namespace Device {

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

static void setUpFastPWM();

void setUpDevice()
{
  setUpFastPWM();
}

Environment getEnvironment()
{
  Environment env;
  env.tMillis = millis();
  env.tMicros = micros();
  env.param1 = analogRead(PIN_PARAM1);
  env.param2 = analogRead(PIN_PARAM2);
  env.param3 = analogRead(PIN_PARAM3);
  env.gateTrg = digitalRead(PIN_GATE_TRG);
  return env;
}

// The PWM generator is always running, but that does not necessarily mean
// that the PWM signal is being written to PIN_OUT all the time.
// Certain registers have to be set in order for the PWM signal to affect
// the state of a pin.
static bool pwmOutputEnabled = false;
static void enablePwmOutput();
static void disablePwmOutput();

static void digitalWriteOut(bool value);
static void analogWriteOut(uint8_t value);

void writeOutput(uint8_t value)
{
  if (value == 0)
    digitalWriteOut(0);
  else if (value == 255)
    digitalWriteOut(1);
  else
    analogWriteOut(value);
}

void digitalWriteOut(bool value)
{
  disablePwmOutput();
  digitalWrite(PIN_OUT, value);
}

void analogWriteOut(uint8_t value)
{
#if defined(__AVR_ATtiny85__) || \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
  // Output-Compare Match Register A for Timer/Counter1: sets PWM duty
  OCR1A = value;
#endif
  enablePwmOutput();
}

void enablePwmOutput()
{
#if defined(__AVR_ATtiny85__)
  // Timer/Counter1 control status register TCCR1
  // Bit COM1A1 high: PWM generator output is connected to OC1A pin
  // Bit COM1A1 low: PWM generator output is disconnected from OC1A pin
  bitSet(TCCR1, COM1A1);
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
  // Timer/Counter1 control status register TCCR1A
  // Bit COM1A1 high: PWM generator output is connected to OC1A pin
  // Bit COM1A1 low: PWM generator output is disconnected from OC1A pin
  bitSet(TCCR1A, COM1A1);
#endif

  // Need to also make sure pin is in output mode.
  pinMode(PIN_OUT, OUTPUT);
}

void disablePwmOutput()
{
#if defined(__AVR_ATtiny85__)
  // Timer/Counter1 control status register TCCR1
  // Bit COM1A1 high: PWM generator output is connected to OC1A pin
  // Bit COM1A1 low: PWM generator output is disconnected from OC1A pin
  bitClear(TCCR1, COM1A1);
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
  // Timer/Counter1 control status register TCCR1A
  // Bit COM1A1 high: PWM generator output is connected to OC1A pin
  // Bit COM1A1 low: PWM generator output is disconnected from OC1A pin
  bitClear(TCCR1A, COM1A1);
#endif
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
 *  - On ATtiny85, Timer/Counter1 can have its clock source set to the internal
 *    PLL (phase-locked loop) generator rather than the 8MHz system clock. This
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
#if defined(__AVR_ATtiny85__)

  // Enable PLL
  bitSet(PLLCSR, PLLE);

  // Datasheet recommends waiting 100us to avoid initial noise in the PLL
  // getting locked
  delayMicroseconds(100);

  // Wait for PLL to be locked (denoted by PLOCK bit in PLLCSR)
  while (!bitRead(PLLCSR, PLOCK));

  // Set PCKE to use PLL as the clock for Timer/Counter1
  bitSet(PLLCSR, PCKE);

  // Timer/Counter1 control status register TCCR1:
  // PWM1A - Enable PWM generator A for Timer/Counter1
  // CS1[3:0] - Prescaler select
  //   - 0001 - Prescaler = 1
  TCCR1 = 1<<PWM1A | 1<<CS10;

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

  // ATmega328p has no PLL, so I'm limited to the 16MHz internal clock.
  // Compared to the 64MHz PLL clock on ATTiny85, that's 4x slower,
  // so that'll be only 62.5kHz PWM. Oh well, it's just for testing.

  // ATmega328p's Timer/Counter1 has more PWM modes than that of the ATTiny85.
  // The ATTiny85 only has one PWM mode, and it's equivalent on ATmega328p is
  // "Fast PWM mode."
  // Also, ATmega328's Timer/Counter1 is a 16-bit counter, so we'll need to
  // specify that we only want 8 bit 
  // Both of these things are set via the WGM1[3:0] bits of TCCR1A/TCCR1B
  //    WMG1[3:0] = 0101 --> Fast PWM, 8-bit

  // Timer/Counter1 control status register is split between two registers:
  // TCCR1A and TCCR1B.
  //  - TCCR1A:
  //    * WGM1[1:0] - Waveform Generation Mode, bottom two bits (rest in TCCR1B)
  //  - TCCR1B:
  //    * WGM1[3:2] - Waveform Generation Mode, top two bits (rest in TCCR1A)
  //    * CS1[2:0] - Clock Select
  //        - 0001 = Internal clock, no prescaler
  TCCR1A = 1<<WGM10;
  TCCR1B = 1<<WGM12 | 1<<CS10;

#endif
}

} //JNTUB::Device

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

DiscreteKnob::DiscreteKnob(uint16_t numValues, uint8_t hysteresis)
  : mMaxVal(0), mHysteresis(0), mCurVal(0), mPrevVal(0), mCurValRaw(0),
    mStep(0), mCurLower(0), mCurUpper(0)
{
  setNumValues(numValues);
  setHysteresis(hysteresis);
}

void DiscreteKnob::setNumValues(uint16_t numValues)
{
  numValues = constrain(numValues, 1, 256);
  mMaxVal = numValues - 1;
  mStep = 1024 / (mMaxVal + 1);
  mCurVal = mCurValRaw / mStep;
  updateThresholds();
}

void DiscreteKnob::setHysteresis(uint8_t hysteresis)
{
  mHysteresis = constrain(hysteresis, 0, mStep / 2);
  updateThresholds();
}

void DiscreteKnob::update(uint16_t value)
{
  mCurValRaw = value;

  if (value < mCurLower || value > mCurUpper) {
    mPrevVal = mCurVal;
    mCurVal = value / mStep;
    updateThresholds();
  }
}

void DiscreteKnob::updateThresholds()
{
  uint16_t lower = mCurVal * mStep;

  if (mCurVal == 0) {
    mCurLower = 0;
  } else {
    mCurLower = lower - mHysteresis;
  }

  if (mCurVal == mMaxVal) {
    mCurUpper = 1023;
  } else {
    mCurUpper = lower + mStep + mHysteresis;
  }
}

uint8_t DiscreteKnob::getValue() const
{
  return mCurVal;
}

uint16_t DiscreteKnob::getValueRaw() const
{
  return mCurValRaw;
}

bool DiscreteKnob::valueChanged() const
{
  return mPrevVal != mCurVal;
}

uint32_t DiscreteKnob::mapValue(uint32_t lower, uint32_t upper) const
{
  return map(mCurVal, 0, mMaxVal, lower, upper);
}

uint32_t DiscreteKnob::mapInnerValue(uint32_t lower, uint32_t upper) const
{
  return map(mCurValRaw, mCurLower, mCurUpper, lower, upper);
}

/**
 * ============================================================================
 * EdgeDetector
 * ============================================================================
 */

EdgeDetector::EdgeDetector()
{
  mState = 0;
  mPrevState = 0;
}

void EdgeDetector::update(bool value)
{
  mPrevState = mState;
  mState = value;
}

bool EdgeDetector::isRising() const
{
  return !mPrevState & mState;
}

bool EdgeDetector::isFalling() const
{
  return mPrevState & !mState;
}

/**
 * ============================================================================
 * Stopwatch
 * ============================================================================
 */

Stopwatch::Stopwatch()
{
  mCurTime = 0;
  mStartTime = 0;
}

void Stopwatch::reset()
{
  mStartTime = mCurTime;
}

void Stopwatch::update(uint32_t time)
{
  mCurTime = time;
}

uint32_t Stopwatch::getTime() const
{
  // Check for overflow
  if (mCurTime < mStartTime) {
    return UINT_MAX - mStartTime + mCurTime;
  }
  return mCurTime - mStartTime;
}

uint32_t Stopwatch::getRealTime() const
{
  return mCurTime;
}

uint32_t Stopwatch::getStartTime() const
{
  return mStartTime;
}

void Stopwatch::setStartTime(uint32_t startTime)
{
  mStartTime = startTime;
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
  return mCurPhase < mDuty;
}
bool Clock::isRising() const
{
  return !mPrevState && getState();
}
bool Clock::isFalling() const
{
  return mPrevState && !getState();
}

void Clock::start()
{
  mRunning = true;
  // Reset to start of period
  sync(0);
}

void Clock::stop()
{
  mRunning = false;
  mCurPhase = 0;
}

void Clock::sync(uint8_t phase)
{
  if (!mRunning)
    return;

  mCurPhase = phase;

  // Adjust the start time of the current period, otherwise updates will
  // proceed as if phase had not been modified
  uint32_t timeWithinPeriod = mCurPhase * mPeriod / PHASE_MAX;
  mStopwatch.setStartTime(mStopwatch.getRealTime() - timeWithinPeriod);
}

void Clock::update(uint32_t time)
{
  mStopwatch.update(time);

  if (!mRunning || mPeriod == 0)
    return;

  mPrevState = getState();

  uint32_t timeWithinPeriod = mStopwatch.getTime() % mPeriod;
  mCurPhase = timeWithinPeriod * PHASE_MAX / mPeriod;

  // Start counting a new period if one is just starting
  if (isRising())
    mStopwatch.reset();
}

}  //JNTUB
