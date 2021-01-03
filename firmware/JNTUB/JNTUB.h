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

  /**
   * Contains all information needed to run a JNTUB-based module.
   * This abstraction allows JNTUB firmwares to theoretically run on any
   * platform (e.g., perhaps on VCVRack soon??).
   */
  struct Environment {
    // Execution environment
    uint32_t tMillis;
    uint32_t tMicros;

    // Module parameters
    uint16_t param1;  // 0 to 1023
    uint16_t param2;  // 0 to 1023
    uint16_t param3;  // 0 to 1023
    bool gateTrg;     // 0 or 1
  };

  /**
   * Inherit from this interface to implement a JNTUB-based module.
   */
  class Module {
  public:
    // Called once upon module powerup.
    virtual void setup() = 0;

    // Called endlessly in a loop.
    // Returns the analog value to write out to the module output.
    virtual uint8_t loop(const Environment &env) = 0;
  };

  namespace Device {

    // Depending on the software platform on which the module runs,
    // acquiring the input parameters / execution environment and producing
    // the output will work differently. These interfaces abstract that away.
    void setUpDevice();
    Environment getEnvironment();
    void writeOutput(uint8_t value);

  }  //JNTUB::Device

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
    // hysteresis min: 0
    // hysteresis max: (1024 / numValues) / 2
    DiscreteKnob(uint8_t numValues, uint8_t hysteresis);

    // Call once per loop with the read analog input value.
    void update(uint16_t value);

    // Retrieve the current discrete value (0 to numValues-1).
    uint8_t getValue() const;

    // Retrieve the raw value of the knob.
    uint16_t getValueRaw() const;

    // Map the "inner value" of the knob to some output range.
    // The inner value is how far between the current bounds the knob is.
    // For example, if numValues is 2 and the knob is at 25%, then
    // the inner value is 50%.
    uint16_t mapInnerValue(uint16_t lower, uint16_t upper) const;

  private:
    uint8_t mNumValues;
    uint8_t mHysteresis;
    uint8_t mCurVal;
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
   */
  class CurveKnob {
  public:
    CurveKnob(const uint16_t *curve, uint8_t size);

    // Call once per loop with the read analog input value.
    void update(uint16_t value);

    // Retrieve the current mapped value (curve[0] to curve[size-1]).
    uint16_t getValue() const;

  private:
    DiscreteKnob mKnob;
    const uint16_t *mCurve;
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
  private:
    uint32_t mPeriod;
    uint32_t mLastRisingEdge;
    uint8_t mDuty;
    uint8_t mCurPhase;
    uint8_t mRunning: 1,
            mCurState: 1,
            mPrevState: 1;

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

    // NOTE: only call one of these per loop.
    // TODO: fix this logic
    void     start(uint32_t time);
    void     stop();
    void     sync(uint32_t time, uint8_t phase=0);
    void     update(uint32_t time);

  private:
    void     updateState(uint32_t time);
  };

}  //JNTUB

#endif  //JNTUB_H_
