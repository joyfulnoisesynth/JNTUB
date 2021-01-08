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

  extern const uint8_t PIN_PARAM1;
  extern const uint8_t PIN_PARAM2;
  extern const uint8_t PIN_PARAM3;
  extern const uint8_t PIN_GATE_TRG;
  extern const uint8_t PIN_OUT;

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
  void setUpFastPWM();  // call once during setup()
  void analogWriteOut(uint8_t value);
  void digitalWriteOut(bool value);

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
   *
   * The generator is agnostic of timing units and operating environment.
   * In other words, it need not run on an AVR board, and you can express
   * clock periods as microseconds, milliseconds, whatever you want.
   *
   * To start the clock, call start() with the current real time unit
   * (e.g., on Arduino, you could pass in the value of millis()).
   * Every loop, call update() with the new current real time unit.
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

}  //JNTUB

#endif  //JNTUB_H_
