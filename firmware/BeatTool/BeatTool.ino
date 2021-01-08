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
#define DEBUGINIT() Serial.begin(115200)
#define DEBUG(...) Serial.print(__VA_ARGS__)
#else
#define DEBUGINIT()
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
 *    Controls the frequency of the generated clock. Sweeps linearly
 *    through a range of frequencies set by Range.
 *
 * PARAM 2: Range
 *    Controls the range of values that the Rate input sweeps through.
 *
 * GATE/TRG: Clock Sync
 *    Immediately resets the clock to the beginning of its cycle.
 */

struct Range {
  uint16_t low;
  uint16_t high;
};

// These ranges allow for more precise tuning of a clock by hand,
// but the difference from one range to the next does not have any
// nice musical properties, so CV control of the Range is probably
// not desired.

const Range CLOCK_RANGES_PRECISE[] = {

  // 1 bpm to 10 bpm
  { 60000, 6000 },

  // 10 bpm to 30 bpm
  { 6000, 2000 },

  // 30 bpm to 60 bpm
  { 2000, 1000 },

  // 60 bpm to 80 bpm
  { 1000, 750 },

  // 80 bpm to 100 bpm
  { 750, 600 },

  // 100 bpm to 120 bpm (MIDDLE)
  { 600, 500 },

  // 120 bpm to 150 bpm (MIDDLE)
  { 500, 400 },

  // 150 bpm to 180 bpm
  { 400, 333 },

  // 180 bpm to 240 bpm
  { 333, 250 },

  // 240 bpm to 360 bpm
  { 250, 167 },

  // 360 bpm to 600 bpm
  { 167, 100 },

  // 10 Hz to 100 Hz
  { 100, 10 }

};

// These ranges make for a musically interesting clock generator
// (rotating the range knob doubles or halves the clock frequency),
// but it's harder to set a precise tempo since each range is larger.

const Range CLOCK_RANGES_MUSICAL[] = {

  // 16s to 8s
  { 16383, 8192 },

  // 8s to 4s
  { 8191, 4096 },

  // 4s to 2s
  { 4095, 2048 },

  // 29 bpm to 59 bpm
  { 2047, 1024 },

  // 59 bpm to 117 bpm (MIDDLE)
  { 1023, 512 },

  // 117 bpm to 234 bpm (MIDDLE)
  { 511, 256 },

  // 234 bpm to 469 bpm
  { 255, 128 },

  // 7.8 Hz to 15.6 Hz
  { 127, 64 },

  // 15.8 Hz to 31.2 Hz
  { 63, 32 },

  // 32.2 Hz to 62.5 Hz
  { 31, 16 },

  // 66.6 Hz to 125 Hz
  { 15, 8 },
};

#define CLOCK_RANGES      CLOCK_RANGES_PRECISE
#define NUM_CLOCK_RANGES  NELEM(CLOCK_RANGES)

class ClockMode {
private:
  JNTUB::DiscreteKnob rangeKnob;
  JNTUB::Clock clock;
  JNTUB::EdgeDetector sync;

  Range curRange;

public:
  ClockMode()
    : rangeKnob(NUM_CLOCK_RANGES, HYSTERESIS_AMT)
  {}

  void setup()
  {
    clock.start();
  }

  bool loop(uint32_t tMillis, uint16_t rateIn, uint16_t rangeIn, bool syncIn)
  {
    rangeKnob.update(rangeIn);
    clock.update(tMillis);
    sync.update(syncIn);

    if (sync.isRising())
      clock.sync();

    curRange = CLOCK_RANGES[rangeKnob.getValue()];

    uint16_t rate = map(rateIn, 0, 1023, curRange.low, curRange.high);

    clock.setPeriod(rate);

    return clock.getState();
  }

  void debug() const
  {
    DEBUG(">>>CLOCK MODE");

    DEBUG(", rate=");
    DEBUG(clock.getPeriod(), DEC);

    DEBUG(", knob=");
    DEBUG(rangeKnob.getValue(), DEC);

    DEBUG(", range=[");
    DEBUG(curRange.low, DEC);
    DEBUG(',');
    DEBUG(curRange.high, DEC);
    DEBUG(']');

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
 *    The available values are defined by BURST_REPEAT_OPTIONS.
 *
 * GATE/TRG: Trigger Burst
 */

const uint8_t BURST_REPEAT_OPTIONS[] = {
  1, 2,  3,  4,  5,  6,  7,  8,
  9, 10, 11, 12, 13, 14, 15, 16
};

const uint16_t BURST_RATE_CURVE[] = {
  1000,  // 1/16 note at 15 bpm
  500,  // 1/16 note at 30 bpm
  250,   // 1/16 note at 60 bpm
  187,   // 1/16 note at 80 bpm
  150,   // 1/16 note at 100 bpm
  125,   // 1/16 note at 120 bpm
  100,   // 1/16 note at 150 bpm
  83,    // 1/16 note at 180 bpm
  62,    // 1/16 note at 240 bpm
};

class BurstMode {
private:
  JNTUB::DiscreteKnob repeatKnob;
  JNTUB::CurveKnob<uint16_t> rateKnob;
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
    clock.sync(128);  // set clock low
  }

  bool loop(uint32_t tMillis, uint16_t rateIn, uint16_t repeatsIn, bool trgIn)
  {
    rateKnob.update(rateIn);
    repeatKnob.update(repeatsIn);
    clock.update(tMillis);
    trigger.update(trgIn);

    // Update rate
    uint16_t periodMicros = rateKnob.getValue();
    clock.setPeriod(periodMicros);

    // Start a new burst on trigger
    if (trigger.isRising()) {
      numGatesSent = 0;
      clock.sync();
      clock.start();
    }

    if (clock.isFalling())
      ++numGatesSent;

    // Check if burst done
    uint8_t repeats = BURST_REPEAT_OPTIONS[repeatKnob.getValue()];
    if (numGatesSent >= repeats) {
      clock.stop();
    }

    return clock.getState();
  }

  void debug()
  {
    DEBUG(">>>BURST MODE");

    DEBUG(", rate=");
    DEBUG(clock.getPeriod(), DEC);

    DEBUG(", repeats=");
    DEBUG(BURST_REPEAT_OPTIONS[repeatKnob.getValue()], DEC);

    DEBUG('\n');
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
  ClockMultiplier(const ClockT *clkIn)
  {
    mClkIn = clkIn;
  }

  void setMultiplier(uint8_t mult)
  {
    mMult = mult;
  }
  void initialize()
  {
    mCurPhase = 0;
    mPrevState = 0;
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
  ClockDivider(const ClockT *clkIn)
  {
    mClkIn = clkIn;
  }

  void setDivisor(uint8_t div)
  {
    mDiv = div;
  }
  void initialize()
  {
    mCurPhase = 0;
    mPrevState = 0;
    mNumEdges = mDiv * 2;
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

    uint8_t phaseAccumulatedFromEdges =
        mNumEdges * JNTUB::Clock::PHASE_MAX / mDiv / 2;
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

  bool initialized;

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

    initialized = false;
  }

  bool loop(
      uint32_t time, uint16_t rateIn, uint16_t rangeIn, bool clkIn, bool divide)
  {
    rateKnob.update(rateIn);
    rangeKnob.update(rangeIn);
    clockFollower.update(time);
    clkEdge.update(clkIn);
    periodTimer.update(time);

    // Select the desired multiplier
    uint8_t range = rangeKnob.getValue();
    uint8_t rate = rateKnob.getValue();
    multiplier = MULTIPLIERS[range][rate];
    if (divide)
      multiplier = multiplier.reciprocal();

    clkMultiplier.setMultiplier(multiplier.numerator);
    clkDivider.setDivisor(multiplier.denominator);

    if (!initialized) {
      clkMultiplier.initialize();
      clkDivider.initialize();
      initialized = true;
    }

    // Sync the clock follower up on every clock transition
    if (clkEdge.isRising()) {
      // Update period estimate and reset timer
      clockFollower.setPeriod(periodTimer.getTime());
      periodTimer.reset();
      clockFollower.sync(0);
    } else if (clkEdge.isFalling()) {
      clockFollower.sync(clockFollower.getDuty());
    }

    // If divide mode, and if the divisor is a whole number,
    // then stop the clock follower from running, and solely rely
    // on the clock edge transitions. If the clock follower keeps running,
    // it could outpace the actual incoming clock edges and lead to
    // inexact clock division.
    if (divide && multiplier.numerator == 1)
      clockFollower.stop();
    else
      clockFollower.start();

    digitalWrite(TEST_CLK_OUT, clockFollower.getState());

    clkMultiplier.update();
    clkDivider.update();

    return clkDivider.getState();
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
  DEBUGINIT();

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
  uint32_t tMillis = millis();
  uint32_t tMicros = micros();

  modeKnob.update(modeIn);
  uint8_t mode = modeKnob.getValue();

  bool output = 0;
  switch(mode) {
    case 0:
      output = multiplyMode.loop(
          tMicros, rateIn, rangeRptIn, gateIn, true /* divide */);
      break;
    case 1:
      output = multiplyMode.loop(
          tMicros, rateIn, rangeRptIn, gateIn, false /* multiply */);
      break;
    case 2:
      output = burstMode.loop(tMillis, rateIn, rangeRptIn, gateIn);
      break;
    case 3:
      output = clockMode.loop(tMillis, rateIn, rangeRptIn, gateIn);
      break;
    default:
      break;
  };

  if (tMicros - lastReport >= REPORT_RATE) {
    lastReport = tMicros;
    DEBUG(mode, DEC);
    switch(mode) {
      case 0:
      case 1:
        multiplyMode.debug();
        break;
      case 2:
        burstMode.debug();
        break;
      case 3:
        clockMode.debug();
        break;
      default:
        break;
    }
  }

  JNTUB::digitalWriteOut(output);
}
