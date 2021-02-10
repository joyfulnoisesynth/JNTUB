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

  Project:     JoyfulNoise LFO
  File:        LFO.ino
  Description: Syncable LFO with Waveshape Select in 4HP

  This is a simple LFO generator with voltage control over rate and phase and
  manual control of waveshape. The LFO can be reset by a trigger or a gate, or
  it can be synchronized to an incoming clock signal.

  ----------
  PARAMETERS
  ----------

  PARAM 3 - Shape
    Selects between:
      * Sine
      * Triangle
      * Ramp (ascending)
      * Sawtooth (descending)
      * Square

  PARAM 2 - Phase
    Controls the position to which the LFO resets when synchronized
    (either via a one-time SYNC or by a regular CLK). Can be used to keep
    two LFOs perfectly out of phase with each other by some amount,
    for example.

  PARAM 1 - Rate
    Controls the frequency. Its exact function depends on the mode of operation
    (see GATE/TRG):

    In Normal Mode:
      Varies the rate from around 100 Hz (at 100%) to around 1 minute cycle time
      (at 0%).

    In Clocked Mode:
      Varies the rate in relation to the input clock signal. At 50%, the LFO
      is locked to 1x the input clock. At 100%, it runs at 32x the input clock,
      and at 0%, it runs at 1/32 speed of the input clock.


  GATE/TRG - Reset or Clock In
    A rising edge resets the LFO back to the position set by Phase.

    The LFO code intelligently detects whether or not the incoming signal
    is a one-off gate/trigger or a repeating clock signal. If it detects a
    clock signal, the LFO goes into Clocked Mode. Otherwise, the LFO is in
    Normal Mode.

  -----
  JNTUB
  -----

  LFO is based on the JoyfulNoise Tiny Utility Board (JNTUB), a
  reprogrammable 4HP module with a standard set of inputs and one 8-bit
  analog output.

 */

// JoyfulNoise Tiny Utility Board Library
#include <JNTUB.h>

// Wavetable data
#include "Tables.h"

template<typename T, unsigned N>
class RollingAverage {
private:
  T history[1<<N];

public:
  RollingAverage()
  {
    memset(history, 0, sizeof(history));
  }

  void update(T val)
  {
    for (auto i = (1<<N)-1; i > 0; --i) {
      history[i] = history[i-1];
    }
    history[0] = val;
  }

  T getValue() const
  {
    uint32_t result = 0;
    for (auto i = 0; i < (1<<N); ++i)
      result += history[i];
    return result >> N;
  }
};

// LFO periods in milliseconds
const uint32_t PERIOD_CURVE[] = {
  180000,
  90000,
  50000,
  20000,
  9000,
  5000,
  2000,
  900,
  500,
  250,
  167,
  100,
  25,
  10,
};

const int8_t *const SHAPES[] = {
  WT_SINE,
  WT_TRIANGLE,
  WT_RAMP,
  WT_SAWTOOTH,
  WT_SQUARE,
};

JNTUB::CurveKnob<uint32_t> rateKnob(PERIOD_CURVE, NELEM(PERIOD_CURVE));
JNTUB::DiscreteKnob shapeKnob(NELEM(SHAPES), 5);

JNTUB::FastClock lfoClock;
JNTUB::EdgeDetector trigger;

RollingAverage<uint16_t, 5> rateAvg;

// Phase offset set by the PHASE knob.
uint8_t phaseOffset;

// Wavetable selected by SHAPE knob.
const int8_t *wavetable;

#define TIMER_RATE JNTUB::SAMPLE_RATE_10_KHZ

uint32_t millisToRate(uint32_t ms)
{
  return JNTUB::FastClock::millisToRate(ms, TIMER_RATE);
}

void setup()
{
  lfoClock.start();
  lfoClock.setRate(millisToRate(140));

  phaseOffset = 0;
  wavetable = WT_TRIANGLE;

  JNTUB::setUpFastPWM();
  JNTUB::setUpTimerInterrupt(TIMER_RATE);
}

void loop()
{
  uint16_t shapeRaw = analogRead(JNTUB::PIN_PARAM3);
  uint16_t phaseRaw = analogRead(JNTUB::PIN_PARAM2);
  uint16_t rateRaw = analogRead(JNTUB::PIN_PARAM1);

  shapeKnob.update(shapeRaw);

  /* rateAvg.update(rateRaw); */
  /* rateKnob.update(rateAvg.getValue()); */
  rateKnob.update(rateRaw);

  uint32_t periodMs = rateKnob.getValue();
  uint32_t rate = millisToRate(periodMs);
  noInterrupts();
  lfoClock.setRate(rate);
  interrupts();

  phaseOffset = map(phaseRaw, 0, 1023, 0, 255);

  uint8_t shapeSelect = shapeKnob.getValue();
  wavetable = SHAPES[shapeSelect];
}

ISR(TIMER_INTERRUPT)
{
  lfoClock.update();

  bool trgRaw = digitalRead(JNTUB::PIN_GATE_TRG);
  trigger.update(trgRaw);
  if (trigger.isRising())
    lfoClock.sync();

  uint8_t phase = lfoClock.getPhase() >> (JNTUB::FastClock::PHASE_BITS - 8);
  phase += phaseOffset;

  int8_t output = pgm_read_byte_near(wavetable + phase);

  JNTUB::analogWriteOut(output + 128);
}
