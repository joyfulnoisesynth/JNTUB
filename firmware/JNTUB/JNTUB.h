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
  File:        JNTUB.h
  Description: JNTUB Common Library Declarations

  Common functionality for all JNTUB-based modules.

 */

#ifndef JNTUB_H_
#define JNTUB_H_

#include <Arduino.h>
#include <stdint.h>

#define NELEM(a) (sizeof(a))/(sizeof(a[0]))

namespace JNTUB {

  /*
   * =======================================================================
   *                         INPUTS AND OUTPUT
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

#if defined(__AVR_ATtiny85__)

#if defined(HW_VER_0_1)
  static const uint8_t PIN_PARAM1    = A2;  // ADC2, chip pin 3
  static const uint8_t PIN_PARAM2    = A1;  // ADC1, chip pin 7
  static const uint8_t PIN_PARAM3    = A3;  // ADC3, chip pin 2
#elif defined(HW_VER_0_2)
  static const uint8_t PIN_PARAM1    = A1;  // ADC1, chip pin 7
  static const uint8_t PIN_PARAM2    = A2;  // ADC2, chip pin 3
  static const uint8_t PIN_PARAM3    = A3;  // ADC3, chip pin 2
#else
  static const uint8_t PIN_PARAM1    = A1;  // ADC1, chip pin 7
  static const uint8_t PIN_PARAM2    = A3;  // ADC3, chip pin 2
  static const uint8_t PIN_PARAM3    = A2;  // ADC2, chip pin 3
#endif

  static const uint8_t PIN_GATE_TRG  = 0;   // PB0, chip pin 5

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

  // I used a regular old Arduino Uno for initial testing.
  static const uint8_t PIN_PARAM1    = A3;  // ADC3, chip pin 26
  static const uint8_t PIN_PARAM2    = A4;  // ADC4, chip pin 27
  static const uint8_t PIN_PARAM3    = A5;  // ADC5, chip pin 28
  static const uint8_t PIN_GATE_TRG  = 8;   // PB0, chip pin 14

#else
#error This AVR board is not supported
#endif

  /**
   * =======================================================================
   * PWM OUTPUT
   * =======================================================================
   *
   * JNTUB uses Pulse-Width Modulation to generate an "analog" output signal
   * from the ATTiny. Unfortunately, the Arduino analogWrite() function
   * utilizes a PWM generator that runs off a slow-running internal timer,
   * so it only achieves ~500 Hz PWM. That's totally unusable for audio,
   * and not really acceptable for clean CV output either.
   *
   * Luckily, the ATTiny has a second timer which can generate a much higher
   * PWM frequency (250 kHz). Setting that up requires some low-level register
   * manipulation, and using it requires us to write our own analogWrite().
   */
  enum PWMRate : uint32_t {
    PWM_RATE_250_KHZ = 250000,
    PWM_RATE_125_KHZ = 125000,
    PWM_RATE_62_KHZ = 62500,
    PWM_RATE_31_KHZ = 31250,
    PWM_RATE_15_KHZ = 15625,
    PWM_RATE_7_KHZ = 7812,  // ** 7812.5
  };
  void setUpFastPWM(PWMRate rate=PWM_RATE_250_KHZ);  // call once during setup()
  void analogWriteOut(uint8_t value);
  void digitalWriteOut(bool value);

  /**
   * =======================================================================
   * PRECISE PWM OUTPUT
   * =======================================================================
   *
   * "But Ben!" you exclaim, "the ATtiny85 only has 8-bit PWM!"
   * Indeed. However, we can squeeze more precision out of it by sort of
   * manually generating the PWM waveform ourselves.
   *
   * See David Johnson-Davies' wonderful article on the topic:
   * "10 or 12-bit DAC from the ATtiny85"
   * http://www.technoblogy.com/show?1NGL
   *
   * If you look at the implementation, I do something similar but much
   * more efficient than David's solution, so that I can keep a really
   * fast PWM rate and still have CPU cycles left over for main program code.
   *
   * CAVEATS:
   *
   *    If you enable 10-bit PWM, the PWM frequency will be reduced from what
   *    you would normally get with setUpFastPWM(). But it's still quite fast.
   *    **I recommend only using it with 16 MHz system clock rate**,
   *    but you can use it with any of the three clock rates:
   *      - At 16 MHz system clock: PWM rate is 125 kHz
   *      - At 8 MHz system clock: PWM rate is 62.5 kHz
   *      - At 16 MHz system clock: PWM rate is 125 kHz
   *
   *    Generating 10-bit PWM uses just under half of the available CPU cycles,
   *    so expect the effective clock rate to be half of what it is nominally
   *    (so with 10-bit PWM and 16MHz clock rate, effective clock rate is 8MHz).
   *
   * IMPORTANT NOTE IF YOU USE THIS WITH TIMER INTERRUPTS:
   *
   *    If you use 10-bit PWM and you have another ISR in your program (like
   *    TIMER_INTERRUPT), you should probably start the ISR by re-enabling
   *    interrupts (they are turned off before running the ISR). If you don't,
   *    your ISR may starve the 10-bit software PWM generator. This isn't
   *    catastrophic, but it will result in glitchy PWM waveforms.
   *
   *    Note that enabling interrupts during an ISR is potentially risky
   *    business! If your ISR runs for long enough, it is possible that it
   *    gets triggered again before it finishes, causing wacky recursion.
   *    Don't let this happen (if you do, let me know how it goes).
   */
  void setUp10BitPWM();  // call once during setup()
  void analogWriteOutPrecise(uint16_t value);

  /**
   * =======================================================================
   * TIMER INTERRUPT / AUDIO OUTPUT
   * =======================================================================
   *
   * For modules that want to perform some action at a rapid and precise
   * interval (such as outputting audio samples), we can utilize the MCU's
   * Timer/Counter0 to generate a regular interrupt.
   *
   * NOTE: USING TIMER/COUNTER0 FOR TIMER INTERRUPTS WILL CAUSE TIMING
   * FUNCTIONS (millis(), micros()) TO BE INACCURATE.
   */

  enum SampleRate : uint16_t {
    SAMPLE_RATE_40_KHZ = 40000,  // 200 cycles / sample (assuming 8MHz clock)
    SAMPLE_RATE_20_KHZ = 20000,  // 400 cycles / sample (assuming 8MHz clock)
    SAMPLE_RATE_10_KHZ = 10000,  // 800 cycles / sample (assuming 8MHz clock)
    SAMPLE_RATE_8_KHZ = 8000,   // 1000 cycles / sample (assuming 8MHz clock)
    SAMPLE_RATE_4_KHZ = 4000,   // 2000 cycles / sample (assuming 8MHz clock)
    SAMPLE_RATE_1_KHZ = 1000,   // 8000 cycles / sample (assuming 8MHz clock)
  };
  void setUpTimerInterrupt(SampleRate rate);  // call once during setup()

  // Implement ISR(TIMER_INTERRUPT) {} for the timer/audio service routine.
  // Call JNTUB::analagWriteOut() to output an audio sample.
  #define TIMER_INTERRUPT TIMER0_COMPA_vect

  /*
   * =======================================================================
   * UTILITY FUNCTIONS
   * =======================================================================
   */

  #define absdiff(a, b) ((a < b) ? (b - a) : (a - b))

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
    // numValues max: 256
    // hysteresis min: 0
    // hysteresis max: (1024 / numValues) / 2
    DiscreteKnob(uint16_t numValues, uint8_t hysteresis);

    void setNumValues(uint16_t numValues);
    void setHysteresis(uint8_t hysteresis);

    // Call once per loop with the read analog input value.
    void update(uint16_t value);

    // Retrieve the current discrete value (0 to numValues-1).
    uint8_t getValue() const;

    // Retrieve the raw value of the knob.
    uint16_t getValueRaw() const;

    // Whether the value changed since last loop.
    bool valueChanged() const;

    // Map the knob's value from its discrete range onto some output range.
    uint32_t mapValue(uint32_t lower, uint32_t upper) const;

    // Map the "inner value" of the knob to some output range.
    // The inner value is how far between the current bounds the knob is.
    // For example, if numValues is 2 and the knob is at 25%, then
    // the inner value is 50% (halfway between 0 and 512)
    uint32_t mapInnerValue(uint32_t lower, uint32_t upper) const;

  public:
    uint8_t mMaxVal;
    uint8_t mHysteresis;
    uint8_t mCurVal;
    uint8_t mPrevVal;
    uint16_t mCurValRaw;
    uint16_t mStep;
    uint16_t mCurLower;  // start point of curVal in the input range (0 to 1023)
    uint16_t mCurUpper;  // end point of curVal in the input range (0 to 1023)

    void updateThresholds();
  };

  /**
   * A knob that maps the input range of signals (0 to 1023) onto an arbitrary
   * piecewise-linear curve specified by an array.
   *
   * Optimized for inputs that don't change drastically or frequently.
   *
   * Note: the higher the granularity, the more suceptible to noise it is.
   */
  template<typename T>
  class CurveKnob {
  public:
    DiscreteKnob mSegmentKnob;
    DiscreteKnob mHysteresisKnob;
    const T *mCurve;

  public:
    CurveKnob(
        const T *curve,
        uint8_t size,
        uint8_t granularity=255,
        uint8_t hysteresis=0)
      : mSegmentKnob(size-1, 0),
        mHysteresisKnob(granularity, hysteresis),
        mCurve(curve)
    {}

    void setCurve(const T *curve)
    {
      mCurve = curve;
    }

    // Call once per loop with the read analog input value.
    void update(uint16_t value)
    {
      mSegmentKnob.update(value);
      mHysteresisKnob.update(mSegmentKnob.mapInnerValue(0, 1023));
    }

    // Retrieve the current mapped value (curve[0] to curve[size-1]).
    T getValue() const
    {
      uint8_t segment = mSegmentKnob.getValue();
      return mHysteresisKnob.mapValue(mCurve[segment], mCurve[segment+1]);
    }

    uint16_t getValueRaw() const
    {
      return mSegmentKnob.getValueRaw();
    }
  };

  /**
   * Indexable, FIFO ring buffer.
   */
  template<typename T, unsigned N>
  class RingBuffer {
  private:
    T buf[N];
    uint16_t front;

  public:
    RingBuffer()
    {
      front = 0;
    }

    inline void fill(T value)
    {
      memset(&buf[0], 0, N * sizeof(T));
    }

    inline T & extend()
    {
      ++front;
      if (front >= N)
        front = 0;
      return buf[front];
    }

    inline void push(T item)
    {
      extend() = item;
    }

    inline T & operator[](uint16_t i)
    {
       if (i > front)
         return buf[N-(i-front)];
       return buf[front-i];
    }
  };


  /**
   * Simple class, reports rising and falling edges.
   */
  class EdgeDetector {
  public:
    EdgeDetector();

    void update(bool value);

    bool isRising() const;
    bool isFalling() const;
  public:
    uint8_t mState: 1,
            mPrevState: 1;
  };

  /**
   * Measures time between events and handles timer overflow.
   */
  class Stopwatch {
  public:
    Stopwatch();

    // Restart counting from 0
    void reset();

    // Call every loop with the current real time
    void update(uint32_t time);

    // Get time since the last reset. If real time has overflowed since the
    // last reset, still reports the correct time.
    // Does not account for multiple overflows occurring since reset.
    uint32_t getTime() const;

    // Get the current real time
    uint32_t getRealTime() const;

    // Get the real time at which the stopwatch was reset
    uint32_t getStartTime() const;

    // Manually reset stopwatch's start time
    void setStartTime(uint32_t);

  public:
    uint32_t mStartTime;
    uint32_t mCurTime;
  };

  /**
   * A clock generator that also reports phase.
   */
  class Clock {
  public:
    Stopwatch mStopwatch;
    uint32_t mPeriod;
    uint8_t mDuty;
    uint8_t mCurPhase;
    uint8_t mRunning;
    uint8_t mPrevState;

  public:
    Clock(uint32_t period=0);

    uint32_t getPeriod() const;
    void     setPeriod(uint32_t period);

    static const uint16_t PHASE_MAX = 256;
    uint8_t  getPhase() const;

    // Clock is high when 0 <= phase < duty.
    // Clock is low when duty <= phase < PHASE_MAX.
    uint8_t  getDuty() const;
    void     setDuty(uint8_t duty);

    bool     getState() const;
    bool     isRising() const;
    bool     isFalling() const;

    // NOTE: call update() before modifying clock parameters
    void     start();
    void     stop();
    void     sync(uint8_t phase=0);
    void     update(uint32_t time);
  };

  /**
   * A more performance-sensitive clock. Meant to be updated on a regular
   * timer interrupt.
   */
  class FastClock {
    private:
      // How many microseconds elapse per tick
      const uint16_t mMicrosPerTick;
      // Phase accumulator
      uint32_t mCurPhase;
      // Sets how much phase accumulates per tick
      volatile uint32_t mRate;
      // Phase at which the clock switches from high to low
      volatile uint32_t mDuty;
      // How many periods the clock has completed since construction
      volatile uint32_t mCycles;
      // Whether or not the clock is advancing
      bool mRunning: 1;
      // What state the clock was in last tick
      bool mPrevState: 1;

    public:
      // The clock's phase has 30-bit granularity.
      // The phase advances from 0 to 2^30 - 1 and then wraps around to start
      // the next period.
      static const uint8_t PHASE_BITS = 30;
      static const uint32_t PHASE_MAX = (uint32_t)1<<PHASE_BITS;

      // Construct the FastClock specifying how frequently it will be updated.
      FastClock(uint16_t tickRateHz);

      /* ----------------------------------------------- */
      /* Callable from main code with interrupts enabled */
      /* ----------------------------------------------- */

      // **SLOW** (32-bit multiply)
      static uint16_t calculateMicrosPerTick(uint16_t tickRateHz);

      uint16_t getMicrosPerTick() const;

      // **SLOW** (32-bit divide)
      uint32_t microsToRate(uint32_t micros) const;

      /* ---------------------------------------------------------------- */
      /* Callable during timer interrupt, or when interrupts are disabled */
      /* ---------------------------------------------------------------- */

      uint32_t getRate() const;
      void     setRate(uint32_t rate);

      // Clock is high when 0 <= phase < duty.
      // Clock is low when duty <= phase < PHASE_MAX.
      uint32_t getDuty() const;
      void     setDuty(uint32_t duty);

      uint32_t getNumCycles() const;

      void     start();
      void     stop();
      void     sync(uint32_t phase=0);

      /* ------------------------------------ */
      /* Only callable during timer interrupt */
      /* ------------------------------------ */

      uint32_t getPhase() const;

      bool     getState() const;
      bool     isRising() const;
      bool     isFalling() const;

      void     tick();
  };

  /**
    * A more performance-sensitive stopwatch. Meant to be updated on a regular
    * timer interrupt.
    */
  class FastStopwatch {
  private:
    // How many microseconds elapse per tick
    const uint16_t mMicrosPerTick;
    // How many ticks the stopwatch has gone through since last reset
    volatile uint32_t mTicks;
    // Whether the stopwatch is advancing
    bool mRunning;

  public:
    // Construct the FastStopwatch specifying how frequently it will be updated.
    FastStopwatch(uint16_t tickRateHz);

    /* ----------------------------------------------- */
    /* Callable from main code with interrupts enabled */
    /* ----------------------------------------------- */

    uint16_t getMicrosPerTick() const;

    // **SLOW** (32-bit multiply)
    uint32_t getTimeMicros() const;

    /* ---------------------------------------------------------------- */
    /* Callable during timer interrupt, or when interrupts are disabled */
    /* ---------------------------------------------------------------- */

    void     start();
    void     stop();
    void     reset();

    uint32_t getNumTicks() const;

    /* ------------------------------------ */
    /* Only callable during timer interrupt */
    /* ------------------------------------ */

    void     tick();

  };

  /**
   * Uses the elapsed time between rising and falling edges of an input signal
   * to approximate its period and duty cycle.
   *
   * For example, here is how to use this class to synchronize a FastClock
   * (fastClock) to some incoming gate signal (gate):
   *
   *    timer interrupt:
   *      approximator.tick(gate);
   *      // This keeps the FastClock's edges in lockstep with the input signal.
   *      if (approximator.isRising())
   *        fastClock.sync(0);
   *      else if (approximator.isFalling())
   *        fastClock.sync(fastClock.getDuty());
   *
   *    main code:
   *      uint32_t rate, duty;
   *      approximator.calculateRateAndDuty(&rate, &duty);
   *      // This makes the FastClock's phase roughly track that of the input.
   *      noInterrupts();
   *      fastClock.setRate(rate);
   *      fastClock.setDuty(duty);
   *      interrupts();
   */
  class FastClockApproximator {
  private:
    FastStopwatch mStopwatch;
    EdgeDetector mEdgeDetector;
    // Number of ticks for which the input signal was high during its last
    // completed period.
    volatile uint32_t mHighTicks;
    // Number of ticks for which the input signal was low during its last
    // completed period.
    volatile uint32_t mLowTicks;
    // Store the number of high ticks here until the current period ends.
    uint32_t mTmpTicks;

  public:
    FastClockApproximator(uint16_t tickRateHz);

    /* ----------------------------------------------- */
    /* Callable from main code with interrupts enabled */
    /* ----------------------------------------------- */

    // **SLOW** (32-bit divide and multiply)
    void calculateRateAndDuty(uint32_t *rateOut, uint32_t *dutyOut) const;

    /* ---------------------------------------------------------------- */
    /* Callable during timer interrupt, or when interrupts are disabled */
    /* ---------------------------------------------------------------- */

    bool     isRising() const;
    bool     isFalling() const;

    uint32_t getHighTicks() const;
    uint32_t getLowTicks() const;

    /* ------------------------------------ */
    /* Only callable during timer interrupt */
    /* ------------------------------------ */

    void     tick(bool gate);

  };

  /**
   * Takes an incoming signal and uses some heuristics to determine whether the
   * signal is a repeating clock signal or just a series of unrelated
   * gates/triggers.
   *
   * The signal will be recognized as a clock signal if:
   *  - There are 3 consecutive periods with roughly* the same length, AND
   *  - The duty cycle of all 3 periods is greater than some threshold (1/8)
   *
   *  * See MARGIN_OF_ERROR_MICROS.
   *
   * Once the input signal has been recognized as a clock signal, it won't
   * be unrecognized until:
   *  - The input signal fails to go high again for more than 4 periods, or for
   *      more than 2 seconds, whichever is longer.
   */
  class ClockDetector {
  private:
    FastClockApproximator mApproximator;

    // The number of ticks that elapse in one millisecond.
    const uint8_t mTicksPerMillis;

    // Number of ticks for which the input was high during the last period
    uint32_t mPrevHighTicks;
    // Number of ticks for which the input was low during the last period
    uint32_t mPrevLowTicks;

    // Counts the number of ticks that have elapsed since the last rising
    // edge of the input signal.
    uint32_t mTicksInPeriod;

    mutable bool mIsClock;

    // When mIsClock is false, this counts the number of periods in a row that
    // have been roughly the same length and whose duty cycle has been
    // sufficiently large.
    uint8_t mNumGoodPeriods;

    // MARGIN_OF_ERROR_MILLIS = 32 (i.e., 2^5)
    static const uint8_t MARGIN_OF_ERROR_BITS = 5;

  public:
    ClockDetector(uint16_t tickRateHz);

    /* ----------------------------------------------- */
    /* Callable from main code with interrupts enabled */
    /* ----------------------------------------------- */

    bool isClock() const;

    // **SLOW** (32-bit divide and multiply)
    void getRateAndDuty(uint32_t *rateOut, uint32_t *dutyOut) const;

    /* ---------------------------------------------------------------- */
    /* Callable during timer interrupt, or when interrupts are disabled */
    /* ---------------------------------------------------------------- */

    bool isRising() const;
    bool isFalling() const;

    /* ------------------------------------ */
    /* Only callable during timer interrupt */
    /* ------------------------------------ */

    void tick(bool gate);

  private:
    bool mostRecentPeriodWasGood() const;
  };

}  //JNTUB

#endif  //JNTUB_H_
