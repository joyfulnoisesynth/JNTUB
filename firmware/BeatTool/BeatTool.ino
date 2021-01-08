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

  Project:     JoyfulNoise Beat Tool
  File:        BeatTool.ino
  Description: 4HP Multi-Purpose Clock Utility Module

  Beat Tool generates and processes clock signals. It can be used as a
  voltage-controlled clock multiplier, clock divider, burst generator,
  or clock generator.

  ----------
  PARAMETERS
  ----------

  PARAM 3 - Mode
    Knob selects between 4 different modes:
      - Divide Mode
      - Multiply Mode
      - Burst Mode
      - Clock Mode

  PARAM 2 - Range / Repeat
    Exact function varies depending on mode. See each mode's comment.
    Generally, modifies the range of time values that the Rate knob sweeps.

  PARAM 1 - Rate
    Exact function varies depending on mode. See each mode's comment.
    Generally, modifies the rate at which gates are generated.

  -----
  JNTUB
  -----

  Beat Tool is based on the JoyfulNoise Tiny Utility Board (JNTUB), a
  reprogrammable 4HP module with a standard set of inputs and one 8-bit
  analog output.

 */

// JoyfulNoise Tiny Utility Board Library
#include <JNTUB.h>

// Hysteresis amount for discrete knobs
#define HYSTERESIS_AMT 5

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#define DEBUG(...) Serial.print(__VA_ARGS__)
#else
#define DEBUG(...)
#endif

/**
 * =============================================================================
 *                                 CLOCK MODE
 * =============================================================================
 *
 * Clock mode generates a simple square wave clock at a voltage-controlled rate.
 * A trigger at the GATE/TRG input synchronizes the clock / resets it.
 *
 * ------------------
 *     PARAMETERS
 * ------------------
 *
 * PARAM 1: Rate
 *    Controls the frequency of the generated clock. The range of frequencies
 *    follows a piecewise-linear exponential approximation curve defined by
 *    CLOCK_RATE_CURVES.
 *
 * PARAM 2: Range
 *    Controls the range of values that the Rate input sweeps through.
 *
 * GATE/TRG: Clock Sync
 *    Immediately resets the clock to the beginning of its cycle.
 */

const uint32_t CLOCK_RATE_CURVES[][3] = {
  { // LFO range
    10000000, // 10s
    3000000,  // 3s
    1000000,  // 1s
  },
  { // Slow musical tempos
    2000000,  // 2s (30 bpm)
    1000000,  // 1s (60bpm)
    666000 // 90 bpm
  },
  { // Musical tempos
    1000000,  // 1s period (60bpm)
    600000,       // 600ms period (~100bpm)
    333000        // 250ms period (180bpm)
  },
  { // Fast musical tempos
    333000,       // 180 bpm
    250000,       // 240 bpm
    166000        // 360 bpm
  },
  { // Audible range
    250000,  // 40 Hz
    2273,    // ~440 Hz
    1000     // 1 kHz
  }
};

class ClockMode {
private:
  JNTUB::CurveKnob<uint32_t> rateKnob;
  JNTUB::DiscreteKnob rangeKnob;
  JNTUB::Clock clock;
  bool prevSync;
  uint32_t mTime;

public:
  ClockMode()
    : rateKnob(CLOCK_RATE_CURVES[0], NELEM(CLOCK_RATE_CURVES[0])),
      rangeKnob(NELEM(CLOCK_RATE_CURVES), HYSTERESIS_AMT)
  {}

  void setup()
  {
    clock.start();
    prevSync = false;
  }

  bool loop(uint32_t time, uint16_t rateIn, uint16_t rangeIn, bool syncIn)
  {
    mTime = time;
    rateKnob.update(rateIn);
    rangeKnob.update(rangeIn);
    clock.update(time);

    uint8_t whichCurve = rangeKnob.getValue();
    rateKnob.setCurve(CLOCK_RATE_CURVES[whichCurve]);

    if (syncIn && !prevSync)
      clock.sync();
    prevSync = syncIn;

    clock.setPeriod(rateKnob.getValue());

    return clock.getState();
  }

  void debug() const
  {
    DEBUG(">>>CLOCK MODE");

    DEBUG(", rate=");
    DEBUG(rateKnob.getValue(), DEC);

    DEBUG(", rateRaw=");
    DEBUG(rateKnob.getValueRaw(), DEC);

    DEBUG(", time=");
    DEBUG(mTime, DEC);

    DEBUG('\n');
    DEBUG("        knob");

    DEBUG(", n=");
    DEBUG(rateKnob.mSegmentKnob.mMaxVal + 1, DEC);

    DEBUG(", val=");
    DEBUG(rateKnob.mSegmentKnob.mCurVal, DEC);

    DEBUG(", valRaw=");
    DEBUG(rateKnob.mSegmentKnob.mCurValRaw, DEC);

    DEBUG(", step=");
    DEBUG(rateKnob.mSegmentKnob.mStep, DEC);

    DEBUG(", lower=");
    DEBUG(rateKnob.mSegmentKnob.mCurLower, DEC);

    DEBUG(", upper=");
    DEBUG(rateKnob.mSegmentKnob.mCurUpper, DEC);

    DEBUG(", inner=");
    DEBUG(rateKnob.mSegmentKnob.mapInnerValue(0, 99), DEC);

    DEBUG(", curve[a]=");
    DEBUG(rateKnob.mCurve[rateKnob.mSegmentKnob.mCurVal], DEC);

    DEBUG(", curve[b]=");
    DEBUG(rateKnob.mCurve[rateKnob.mSegmentKnob.mCurVal+1], DEC);

    DEBUG('\n');
  }
};

/**
 * =============================================================================
 *                                 BURST MODE
 * =============================================================================
 *
 * Burst mode generates a burst of gates. The gates generated are not
 * synchronized to any incoming clock signal, unfortunately, because the
 * GATE/TRG input must be used to trigger the burst. Instead, the burst rate
 * is continuously variable via PARAM 1.
 *
 * ------------------
 *     PARAMETERS
 * ------------------
 *
 * PARAM 1: Rate
 *    Varies the time between each burst. The range of tempos follows a
 *    piecewise-linear exponential approximation function defined by
 *    BURST_RATE_CURVE.
 *
 * PARAM 2: Repeats
 *    Varies the number of gates that are generated in a burst.
 *    The available values are defined by BURST_REPEATS.
 *
 * GATE/TRG: Trigger Burst
 */

const uint8_t BURST_REPEAT_OPTIONS[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16
};

const uint32_t BURST_RATE_CURVE[] = {
  100000,  // 0.1 second period
  1000000  // 1 second period
};

class BurstMode {
private:
  JNTUB::DiscreteKnob repeatKnob;
  JNTUB::CurveKnob<uint32_t> rateKnob;
  JNTUB::Clock clock;
  JNTUB::EdgeDetector trigger;
  uint8_t numGatesSent;

public:
  BurstMode()
    : repeatKnob(NELEM(BURST_REPEAT_OPTIONS), HYSTERESIS_AMT),
      rateKnob(BURST_RATE_CURVE, NELEM(BURST_RATE_CURVE))
  {}

  void setup()
  {
    numGatesSent = 0;
  }

  bool loop(uint32_t time, uint16_t rateIn, uint16_t repeatsIn, bool trgIn)
  {
    rateKnob.update(rateIn);
    repeatKnob.update(repeatsIn);
    clock.update(time);
    trigger.update(trgIn);

    // Update rate
    uint16_t periodMicros = rateKnob.getValue();
    clock.setPeriod(periodMicros);

    // Start a new burst on trigger
    if (trigger.isRising()) {
      clock.start();
      numGatesSent = 0;
    }

    if (clock.isFalling())
      ++numGatesSent;

    // Check if burst done
    uint8_t repeatOpt = repeatKnob.getValue();
    uint8_t numRepeats = BURST_REPEAT_OPTIONS[repeatOpt];

    if (numGatesSent >= numRepeats) {
      clock.stop();
      numGatesSent = 0;
    }

    return clock.getState();
  }
};

/**
 * =============================================================================
 *                          CLOCK DIVIDE / MULTIPLY MODE
 * =============================================================================
 *
 * Divide and multiply mode have the same theory of operation.
 * The GATE/TRG input acts as a clock signal input, and OUT is derived by
 * dividing or multiplying the incoming clock.
 *
 * Clock division with whole number divisors is exact; transitions are
 * synchronous with incoming rising and falling clock edges (well, as
 * synchronous as it can be, limited by the MCU's internal system clock).
 *
 * Clock division with fractional divisors, or clock multiplication with any
 * multiplier, is approximate; the perceived clock speed is calculated in real
 * time and may become briefly inaccurate if the actual clock speed changes or
 * there is inconsistency / swing. Perhaps you can use this to your creative
 * advantage in your patches :-)
 *
 * ------------------
 *     PARAMETERS
 * ------------------
 *
 * PARAM 1: Rate
 *    Controls the rate of division or multiplication. The knob/CV steps
 *    through 8 different division/multiplication factors. The values of
 *    these factors are controlled by the Range input.
 *
 * PARAM 2: Range
 *    Controls the range of values that the Rate input sweeps through.
 *    There are 4 different ranges:
 *
 *    Range 0:
 *      x1    x5/4  x4/3  x3/2  x8/5  x5/3  x5/2  x12/5
 *      x1    x4/5  x3/4  x2/3  x5/8  x3/5  x2/5  x5/12
 *
 *    Range 1:
 *      x1    x2    x3    x4    x5    x6    x7    x8
 *      /1    /2    /3    /4    /5    /6    /7    /8
 *
 *    Range 2:
 *      x1    x2    x3    x4    x6    x8    x16   x32
 *      /1    /2    /3    /4    /6    /8    /16   /32
 *
 *    Range 4:
 *      x1    x2    x4    x8    x16   x32   x64   x128
 *      /1    /2    /4    /8    /16   /32   /64   /128
 *
 * GATE/TRG: Clock In
 */

struct Multiplier {
  uint8_t numerator;
  uint8_t denominator;

  Multiplier reciprocal() const
  {
    return { denominator, numerator };
  }
};

const Multiplier MULTIPLIERS[][8] = {
  { {1, 1}, {5, 4}, {4, 3}, {3, 2}, {8, 5}, {5, 3}, {5, 2}, {12, 5} },
  { {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}, {8, 1} },
  { {1, 1}, {2, 1}, {3, 1}, {4, 1}, {6, 1}, {8, 1}, {16, 1}, {32, 1} },
  { {1, 1}, {2, 1}, {4, 1}, {8, 1}, {16, 1}, {32, 1}, {64, 1}, {128, 1} },
};

/**
 * Uses the phase of an input clock to derive a multiplied
 * version of the input clock.
 *
 * Implements a Clock-like interface so it can be used as input to
 * another ClockMultiplier or ClockDivider.
 */
template<typename ClockT>
class ClockMultiplier {
private:
  const ClockT *mClkIn;
  uint8_t mMult;
  uint8_t mCurPhase;
  bool mPrevState;

public:
  ClockMultiplier(const ClockT *clkIn, uint8_t mult=1)
  {
    mClkIn = clkIn;
    mMult = mult;
    mCurPhase = 0;
    mPrevState = 0;
  }

  void setMultiplier(uint8_t mult)
  {
    mMult = mult;
  }
  uint8_t getDuty() const
  {
    return mClkIn->getDuty();
  }
  uint8_t getPhase() const
  {
    return mCurPhase;
  }
  bool getState() const
  {
    return mCurPhase < mClkIn->getDuty();
  }
  bool isRising() const
  {
    return !mPrevState && getState();
  }
  bool isFalling() const
  {
    return mPrevState && !getState();
  }

  // Do not call until clkIn has been updated.
  void update()
  {
    mPrevState = getState();
    mCurPhase = (mClkIn->getPhase() * mMult) % JNTUB::Clock::PHASE_MAX;
  }
};

/**
 * Uses the edges of an input clock to derive a divided
 * version of the input clock.
 *
 * Implements a Clock-like interface so it can be used as input to
 * another ClockMultiplier or ClockDivider.
 */
template<typename ClockT>
class ClockDivider {
public:
  const ClockT *mClkIn;
  uint8_t mDiv;
  // How many edges have occurred in the input clock since the last edge
  // of the divided clock? Flip-flop the output every mDiv edges.
  uint8_t mNumEdges;
  uint8_t mCurPhase;
  bool mPrevState;

public:
  ClockDivider(const ClockT *clkIn, uint8_t div=1)
  {
    mClkIn = clkIn;
    mDiv = div;
    mCurPhase = 0;
    mPrevState = 0;
    mNumEdges = mDiv * 2;
  }

  void setDivisor(uint8_t div)
  {
    mDiv = div;
  }
  uint8_t getDuty() const
  {
    return mClkIn->getDuty();
  }
  uint8_t getPhase() const
  {
    return mCurPhase;
  }
  bool getState() const
  {
    return mCurPhase < mClkIn->getDuty();
  }
  bool isRising() const
  {
    return !mPrevState && getState();
  }
  bool isFalling() const
  {
    return mPrevState && !getState();
  }

  // Do not call until clkIn has been updated.
  void update()
  {
    mPrevState = getState();

    if (mClkIn->isRising() || mClkIn->isFalling()) {
      ++mNumEdges;
    }

    if (mNumEdges >= mDiv * 2)
      mNumEdges = 0;

    uint8_t phasePerInputHalfPeriod = JNTUB::Clock::PHASE_MAX / mDiv / 2;
    uint8_t phaseAccumulatedFromEdges = mNumEdges * phasePerInputHalfPeriod;
    uint8_t currentPhaseInInputHalfPeriod =
      mClkIn->getPhase() % (JNTUB::Clock::PHASE_MAX / 2);
    mCurPhase =
      phaseAccumulatedFromEdges + (currentPhaseInInputHalfPeriod / mDiv);
  }
};

class MultiplyMode {
private:
  JNTUB::DiscreteKnob rateKnob;
  JNTUB::DiscreteKnob rangeKnob;

  // Keep an internal running clock and continually set its period
  // to what we think the period of the input clock is.
  JNTUB::Clock clockFollower;
  JNTUB::EdgeDetector clkEdge;
  JNTUB::Stopwatch periodTimer;

  // All possible fractional multipliers/divisors are accomplished using a
  // combination of one clock multiplier and one clock divider.
  // The multiplier multiplies the input clock by the numerator,
  // and the divider divides the resulting signal by the denominator.
  using MultT = ClockMultiplier<JNTUB::Clock>;
  using DivT = ClockDivider<MultT>;
  MultT clkMultiplier;
  DivT clkDivider;

  Multiplier multiplier;

  const int TEST_CLK_OUT = 7;

public:
  MultiplyMode()
    : rateKnob(NELEM(MULTIPLIERS[0]), HYSTERESIS_AMT),
      rangeKnob(NELEM(MULTIPLIERS), HYSTERESIS_AMT),
      clkMultiplier(&clockFollower),
      clkDivider(&clkMultiplier)
  {}

  void setup()
  {
    multiplier = {1, 1};
    clockFollower.start();
  }

  bool loop(
      uint32_t time, uint16_t rateIn, uint16_t rangeIn, bool clkIn, bool divide)
  {
    rateKnob.update(rateIn);
    rangeKnob.update(rangeIn);
    clockFollower.update(time);
    clkEdge.update(clkIn);
    periodTimer.update(time);

    // Sync the clock follower up on every clock transition
    if (clkEdge.isRising()) {
      // Update period estimate and reset timer
      clockFollower.setPeriod(periodTimer.getTime());
      periodTimer.reset();
      clockFollower.sync(0);
    } else if (clkEdge.isFalling()) {
      clockFollower.sync(clockFollower.getDuty());
    }

    digitalWrite(TEST_CLK_OUT, clockFollower.getState());

    // Select the desired multiplier
    uint8_t range = rangeKnob.getValue();
    uint8_t rate = rateKnob.getValue();
    this->multiplier = MULTIPLIERS[range][rate];
    if (divide)
      this->multiplier = this->multiplier.reciprocal();

    clkMultiplier.setMultiplier(this->multiplier.numerator);
    clkMultiplier.update();
    clkDivider.setDivisor(this->multiplier.denominator);
    clkDivider.update();

    return clkDivider.getState() ? 255 : 0;
  }

  void debug()
  {
    DEBUG(">>>MULTIPLY MODE");

    DEBUG(", multiplier=");
    DEBUG(multiplier.numerator, DEC);
    DEBUG('/');
    DEBUG(multiplier.denominator, DEC);

    DEBUG(", estimatedPeriod=");
    DEBUG(clockFollower.getPeriod(), DEC);

    DEBUG(rateKnob.getValue(), DEC);

    DEBUG(", follower.phase=");
    DEBUG(clockFollower.getPhase(), DEC);

    DEBUG(", clkDiv.phase=");
    DEBUG(clkDivider.mCurPhase, DEC);

    DEBUG(", clkDiv.edges=");
    DEBUG(clkDivider.mNumEdges, DEC);

    DEBUG('\n');
  }
};

/**
 * =============================================================================
 *                           MODULE IMPLEMENTATION
 * =============================================================================
 */

#define REPORT_RATE 500000

const uint8_t NUM_MODES = 4;

JNTUB::DiscreteKnob modeKnob(NUM_MODES, HYSTERESIS_AMT);
ClockMode clockMode;
BurstMode burstMode;
MultiplyMode multiplyMode;

uint32_t lastReport;

void setup()
{
  clockMode.setup();
  burstMode.setup();
  multiplyMode.setup();

  lastReport = 0;
}

void loop()
{
  uint16_t modeIn = analogRead(JNTUB::PIN_PARAM3);
  uint16_t rangeRptIn = analogRead(JNTUB::PIN_PARAM2);
  uint16_t rateIn = analogRead(JNTUB::PIN_PARAM1);
  bool gateIn = digitalRead(JNTUB::PIN_GATE_TRG);
  uint32_t time = micros();

  modeKnob.update(modeIn);
  uint8_t mode = modeKnob.getValue();

  bool output = 0;
  switch(mode) {
    case 0:
      output = multiplyMode.loop(
          time, rateIn, rangeRptIn, gateIn, true /* divide */);
      break;
    case 1:
      output = multiplyMode.loop(
          time, rateIn, rangeRptIn, gateIn, false /* multiply */);
      break;
    case 2:
      output = burstMode.loop(time, rateIn, rangeRptIn, gateIn);
      break;
    case 3:
      output = clockMode.loop(time, rateIn, rangeRptIn, gateIn);
      break;
    default:
      break;
  };

  if (time - lastReport >= REPORT_RATE) {
    lastReport = time;
    Serial.println(mode);
    switch(mode) {
      case 0:
      case 1:
        multiplyMode.debug();
        break;
      case 3:
        clockMode.debug();
        break;
      case 2:
      default:
        break;
    }
  }

  JNTUB::digitalWriteOut(output);
}
