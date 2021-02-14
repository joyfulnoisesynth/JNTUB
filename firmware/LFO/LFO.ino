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

#define USE_10_BIT_PWM

#define TIMER_RATE JNTUB::SAMPLE_RATE_8_KHZ

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

JNTUB::FastClock lfoClock(TIMER_RATE);
JNTUB::EdgeDetector trigger;

// Phase offset set by the PHASE knob.
volatile uint8_t phaseOffset;

// Wavetable selected by SHAPE knob.
volatile const int8_t *wavetable;

void setup()
{
  lfoClock.start();

  phaseOffset = 0;
  wavetable = WT_TRIANGLE;

  JNTUB::setUpTimerInterrupt(TIMER_RATE);

#ifdef USE_10_BIT_PWM
  // At slow speeds, the 8-bitness of the PWM output becomes quite apparent.
  // So we'll utilize the JNTUB library's 10-bit PWM procedure.
  // Since we only store 8-bit values in the wavetables though, we'll have
  // to interpolate in order to fully utilize those 10 bits.
  JNTUB::setUp10BitPWM();
#else
  JNTUB::setUpFastPWM();
#endif
}

void loop()
{
  uint16_t shapeRaw = analogRead(JNTUB::PIN_PARAM3);
  uint16_t phaseRaw = analogRead(JNTUB::PIN_PARAM2);
  uint16_t rateRaw = analogRead(JNTUB::PIN_PARAM1);

  shapeKnob.update(shapeRaw);
  rateKnob.update(rateRaw);

  uint32_t periodMs = rateKnob.getValue();
  uint32_t rate = lfoClock.microsToRate(periodMs * 1000);

  uint8_t phase = map(phaseRaw, 0, 1023, 0, 255);

  uint8_t shapeSelect = shapeKnob.getValue();

  noInterrupts();
  lfoClock.setRate(rate);
  phaseOffset = phase;
  wavetable = SHAPES[shapeSelect];
  interrupts();
}

ISR(TIMER_INTERRUPT)
{
  // Re-enable interrupts (they are disabled by default when entering ISRs).
  // This prevents the timer interrupt from starving the 10-bit PWM generator.
  interrupts();

  lfoClock.update();

  bool trgRaw = digitalRead(JNTUB::PIN_GATE_TRG);
  trigger.update(trgRaw);
  if (trigger.isRising())
    lfoClock.sync();

  uint32_t phase = lfoClock.getPhase();
  uint16_t phase10bit = phase >> (JNTUB::FastClock::PHASE_BITS - 10);
  uint8_t phase8bit = phase10bit >> 2;
  uint8_t phaseRemainder = phase10bit - ((uint16_t)phase8bit<<2); // 0 to 3

  // Interpolate between wavetable[index] and wavetable[index+1]
  uint8_t index = phase8bit + phaseOffset;
  int16_t valueA = (int8_t)pgm_read_byte_near(wavetable + index++) * 4;
  int16_t valueB = (int8_t)pgm_read_byte_near(wavetable + index) * 4;
  int16_t difference = valueB - valueA;

  int16_t output = valueA + (difference * (int8_t)phaseRemainder / 4);

#ifdef USE_10_BIT_PWM
  JNTUB::analogWriteOutPrecise(output + 512);
#else
  JNTUB::analogWriteOut((output/4) + 128);
#endif
}
