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

  Project:     JoyfulNoise Tiny Utility Board Firmware
  File:        JNTUB.cpp
  Description: JNTUB Common Library Implementation

  Common functionality for all JNTUB-based modules.

 */

#include "JNTUB.h"

#include <limits.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

namespace JNTUB {

#if defined(__AVR_ATtiny85__)

  static const uint8_t PIN_OUT = 1;  // PB1/OC1A, chip pin 6

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

  static const uint8_t PIN_OUT = 9;  // PB1/OC1A, chip pin 15

#endif

/**
 * ===========================================================================
 *
 *                           PWM / AUDIO / TIMERS
 *
 * ===========================================================================
 */

// The PWM generator is always running, but its output is not connected
// to any pin on the MCU unless the proper bit(s) is/are set.
static inline void enablePwmOutput()
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

static inline void disablePwmOutput()
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

/**
 * MORE DETAIL ON GETTING FASTER PWM
 * ---------------------------------
 *
 * Most (if not all) AVR boards have at least two Timer/Counters.
 *
 * Arduino's implementation chooses to base its PWM output on Timer/Counter0,
 * the same timer which the Arduino library relies upon for timing functions
 * like millis() and micros(). One way I could increase the PWM frequency is to
 * decrease the prescaler on Timer/Counter0 from 64 down to 1, but that would
 * make functions like millis() and micros() completely inaccurate. That's lame.
 *
 * Furthermore, Timer/Counter0 is locked to only 8MHz on ATTiny85 (that's the
 * speed of the internal oscillator). Every PWM period lasts exactly 256 counts,
 * so 8MHz / 256 = maximum 31.25kHz PWM frequency. Unless I were to set my PWM
 * filter cutoff to something as low as 1kHz, the output signal would have
 * unavoidable jitter that could cause weird audio artifacts.
 *
 * The solution to all these problems is to use Timer/Counter1 for PWM.
 * This has two great benefits:
 *
 *  - Timer/Counter0's prescaler remains unaffected, so I can use Arduino
 *    library functions to accurately keep time (which is important for
 *    some modules).
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

void setUpFastPWM(PWMRate rate)
{
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
  TCCR1 = 1<<PWM1A;

  // Timer/Counter1 control status register TCCR1:
  // CS1[3:0] - Prescaler select
  switch(rate) {
    case PWM_RATE_250_KHZ:
      // CS1[3:0]: 0001 - Prescaler = 1
      TCCR1 |= 1<<CS10;
      break;
    case PWM_RATE_125_KHZ:
      // CS1[3:0]: 0010 - Prescaler = 2
      TCCR1 |= 2<<CS10;
      break;
    case PWM_RATE_62_KHZ:
      // CS1[3:0]: 0011 - Prescaler = 4
      TCCR1 |= 3<<CS10;
      break;
     default:
      break;
  }

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

  // ATmega328p has no PLL, so I'm limited to the 16MHz internal clock.
  // Compared to the 64MHz PLL clock on ATTiny85, that's 4x slower,
  // so that'll be only 62.5kHz PWM. Oh well, it's just for testing.

  // ATmega328p's Timer/Counter1 has more PWM modes than that of the ATTiny85.
  // The ATTiny85 only has one PWM mode, and its equivalent on ATmega328p is
  // "Fast PWM mode."
  // Also, ATmega328's Timer/Counter1 is a 16-bit counter, so we'll need to
  // specify that we only want 8 bit PWM.
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

  enablePwmOutput();
  pinMode(PIN_OUT, OUTPUT);
}

/**
 * ============================================================================
 * Precise PWM
 * ============================================================================
 */

void setUp10BitPWM()
{
  // Enable Timer/Counter1's overflow interrupt so we can change the value
  // we output to the PWM generator every PWM period.
#if defined(__AVR_ATtiny85__)
  // TIMSK – Timer/Counter Interrupt Mask Register.
  //  - TOIE1: Timer/Counter1 Overflow Interrupt Enable
  bitSet(TIMSK, TOIE1);

  // Precise PWM requires executing an interrupt service routine on every period
  // of the PWM generator. That's _really_ frequent. In fact, at 250 kHz PWM
  // and 8 MHz clock rate, the ISR takes up more than 100% of the CPU time.
  // So we unfortunately need to reduce the PWM rate if we're going to have
  // any cycles left over for the actual code.
  //
  // The PWM ISR takes around 50 clock cycles, so we need to have at least 64
  // clock cycles per PWM period if we want _any_ cycles reserved for main
  // computation. I'm shooting for 128 cycles per period to give even more
  // headroom. With the ATtiny configured for 16 MHz clock rate, that still
  // allows for 125 KHz PWM and results in an effective clock rate of just over
  // 8 MHz.
#if F_CPU == 16000000
  // 16 MHz / 125 KHz = 128 cycles
  setUpFastPWM(PWM_RATE_125_KHZ);
#elif F_CPU == 8000000
  // 8 MHz / 62.5 KHz = 128 cycles
  setUpFastPWM(PWM_RATE_62_KHZ);
#elif F_CPU == 1000000
  // 1 MHz / 7812 Hz = ~128 cycles
  setUpFastPWM(PWM_RATE_7_KHZ);
#endif // F_CPU

#else
#error Precise PWM not implemented for this board
#endif
}

static volatile uint8_t pwmVals[4] = { 0, 0, 0, 0 };

void analogWriteOutPrecise(uint16_t value)
{
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) {
    if (value > 255) {
      pwmVals[i] = 255;
      value -= 256;
    } else {
      pwmVals[i] = value;
      value = 0;
    }
  }
  interrupts();
}

// This is an extremely performance-critical piece of code.
// Every ISR comes with at minimum 42 cycles of overhead:
//  4 cycles to save ISP.
//  3 cycles to JMP to the ISR code.
//  16 cycle prologue to push registers to stack
//    (may be as low as 12 if some registers don't need saving).
//  19 cycle epilogue to pop registers off the stack and return
//    (may be as low as 15).
//
// The code below compiles down to 12 cycles, giving the entire ISR
// a runtime of no more than 54 cycles.
ISR(TIMER1_OVF_vect)
{
  static uint8_t cycle = 0;

  OCR1A = pwmVals[++cycle & 0x03];
  // Compiles to:
  // lds     r30, 0x0060     ; 2 cycles
  // subi    r30, 0xFF       ; 1 cycle
  // sts     0x0060, r30     ; 2 cycles
  // andi    r30, 0x03       ; 1 cycle
  // ldi     r31, 0x00       ; 1 cycle
  // subi    r30, 0x1B       ; 1 cycle
  // sbci    r31, 0xFF       ; 1 cycle
  // ld      r24, Z          ; 2 cycles
  // out     0x2e, r24       ; 1 cycle
  //                         ; = 12 cycles
}

/**
 * ============================================================================
 * Timer Interrupts
 * ============================================================================
 */

#define TIMER0_PRESCALE_1 (1<<CS00)
#define TIMER0_PRESCALE_8 (2<<CS00)
#define TIMER0_PRESCALE_64 (3<<CS00)

void setUpTimerInterrupt(SampleRate rate)
{
#if defined(__AVR_ATtiny85__)

  TCCR0A = 3<<WGM00;  // Fast PWM
  TCCR0B = 1<<WGM02;  // Overflow on TOP

#if F_CPU == 16000000

  switch(rate) {
    case SAMPLE_RATE_40_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 49;   // Divide by 50 (2M / 50 = 40k)
      break;
    case SAMPLE_RATE_20_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 99;   // Divide by 100 (2M / 100 = 20k)
      break;
    case SAMPLE_RATE_10_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 199;   // Divide by 200 (2M / 200 = 10k)
      break;
    case SAMPLE_RATE_8_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 249;  // Divide by 250 (2M / 250 = 8k)
      break;
    case SAMPLE_RATE_4_KHZ:
      TCCR0B |= TIMER0_PRESCALE_64;
      OCR0A = 61;   // Divide by 62 (250k / 62 = 4,032) **
    case SAMPLE_RATE_1_KHZ:
      TCCR0B |= TIMER0_PRESCALE_64;
      OCR0A = 249;   // Divide by 250 (250k / 250 = 1k)
    default:
      break;
  }

#elif F_CPU == 8000000

  switch(rate) {
    case SAMPLE_RATE_40_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 24;   // Divide by 25 (1M / 25 = 40k)
      break;
    case SAMPLE_RATE_20_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 49;   // Divide by 50 (1M / 50 = 20k)
      break;
    case SAMPLE_RATE_10_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 99;   // Divide by 100 (1M / 100 = 10k)
      break;
    case SAMPLE_RATE_8_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 124;  // Divide by 125 (1M / 125 = 8k)
      break;
    case SAMPLE_RATE_4_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 249;  // Divide by 250 (1M / 125 = 4k)
      break;
    case SAMPLE_RATE_1_KHZ:
      TCCR0B |= TIMER0_PRESCALE_64;
      OCR0A = 124;  // Divide by 125 (125k / 125 = 1k)
      break;
    default:
      break;
  }

#elif F_CPU == 1000000

  switch(rate) {
    case SAMPLE_RATE_40_KHZ:
      TCCR0B |= TIMER0_PRESCALE_1;
      OCR0A = 24;   // Divide by 25 (1M / 25 = 40k)
      break;
    case SAMPLE_RATE_20_KHZ:
      TCCR0B |= TIMER0_PRESCALE_1;
      OCR0A = 49;   // Divide by 50 (1M / 50 = 20k)
      break;
    case SAMPLE_RATE_10_KHZ:
      TCCR0B |= TIMER0_PRESCALE_1;
      OCR0A = 99;   // Divide by 100 (1M / 100 = 10k)
      break;
    case SAMPLE_RATE_8_KHZ:
      TCCR0B |= TIMER0_PRESCALE_1;
      OCR0A = 124;  // Divide by 125 (1M / 125 = 8k)
      break;
    case SAMPLE_RATE_4_KHZ:
      TCCR0B |= TIMER0_PRESCALE_1;
      OCR0A = 249;  // Divide by 250 (1M / 125 = 4k)
      break;
    case SAMPLE_RATE_1_KHZ:
      TCCR0B |= TIMER0_PRESCALE_8;
      OCR0A = 124;  // Divide by 125 (125k / 125 = 1k)
      break;
    default:
      break;
  }

#else
#error setUpTimerInterrupt not implemented for this clock frequency
#endif // F_CPU

  bitSet(TIMSK, OCIE0A);  // Enable compare match int.
  bitClear(TIMSK, TOIE0); // Disable overflow int.

#else
#error setUpTimerInterrupt not implemented for this board
#endif
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
  mCurPhase = 0;
  mRunning = false;
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
}

void Clock::stop()
{
  mRunning = false;
}

void Clock::sync(uint8_t phase)
{
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

/**
 * ============================================================================
 * FastClock
 * ============================================================================
 */

FastClock::FastClock(uint16_t updateRateHz)
{
  // 16-bit microsPerSample limits sample rate to 16 Hz
  // (1,000,000 / 15 = 66,666; 1,000,000 / 16 = 62,500).
  updateRateHz = constrain(updateRateHz, 16, UINT16_MAX);
  mMicrosPerUpdate = 1000000UL / updateRateHz;

  mRate = 0;
  mDuty = PHASE_MAX / 2;
  mCycles = 0;
  mCurPhase = 0;
  mRunning = false;
  mPrevState = false;
}

uint32_t FastClock::microsToRate(uint32_t micros)
{
  uint32_t phasePerMicro = PHASE_MAX / micros;
  return mMicrosPerUpdate * phasePerMicro;
}

uint32_t FastClock::getRate() const
{
  return mRate;
}

void FastClock::setRate(uint32_t rate)
{
  mRate = rate;
}

uint32_t FastClock::getDuty() const
{
  return mDuty;
}

void FastClock::setDuty(uint32_t duty)
{
  mDuty = duty;
}

uint32_t FastClock::getNumCycles() const
{
  return mCycles;
}

void FastClock::start()
{
  mRunning = true;
}

void FastClock::stop()
{
  mRunning = false;
}

void FastClock::sync(uint32_t phase=0)
{
  mCurPhase = phase;
}

bool FastClock::getState() const
{
  return mCurPhase < mDuty;
}

bool FastClock::isRising() const
{
  return getState() & !mPrevState;
}

bool FastClock::isFalling() const
{
  return !getState() & mPrevState;
}

uint32_t FastClock::getPhase() const
{
  return mCurPhase;
}

void FastClock::update()
{
  if (mRunning) {
    mPrevState = getState();
    mCurPhase += mRate;
    if (mCurPhase >= PHASE_MAX) {
      mCurPhase -= PHASE_MAX;
      ++mCycles;
    }
  }
}

}  //JNTUB
