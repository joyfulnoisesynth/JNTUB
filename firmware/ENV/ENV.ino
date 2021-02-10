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

  Project:     JoyfulNoise ENV
  File:        ENV.ino
  Description: Attack/Decay Envelope Generator in 4HP

  ENV is a simple attack/decay envelope generator with voltage control over
  attack and decay and continuously variable envelope shape.

  ----------
  PARAMETERS
  ----------

  PARAM 3 - Shape

    Controls the envelope curve. Continuously varies between:

      0%:         _____   .
                 |        |
                 |        |____

      25%:         .___   .
                  /       |
                 |         \___

      50%:            /   \
                    /       \
                  /           \

      75%:            .   ___.
                      |       \
                  ___/         |

      100%:           .   _____
                      |        |
                  ____|        |

  PARAM 2 - Decay Time

  PARAM 3 - Attack Time

  GATE/TRG - Trigger Envelope

  -----
  JNTUB
  -----

  ENV is based on the JoyfulNoise Tiny Utility Board (JNTUB), a
  reprogrammable 4HP module with a standard set of inputs and one 8-bit
  analog output.

 */

// JoyfulNoise Tiny Utility Board Library
#include <JNTUB.h>

// Envelope shapes
#include "Tables.h"

#define TIMER_RATE JNTUB::SAMPLE_RATE_10_KHZ

uint32_t microsToRate(uint32_t us)
{
  return JNTUB::FastClock::microsToRate(us, TIMER_RATE);
}

/**
 * Blends between two values with no divides.
 * For blend=0, returns a
 * For blend=64, returns average of a and b
 * For blend=128, returns b
 */
uint8_t blend(uint8_t a, uint8_t b, uint8_t blend)
{
  if (blend > 128)
    blend = 128;
  // Guaranteed to not overflow. UINT8_MAX * 64 + UINT8_MAX * 64 == 2^15 - 1
  uint16_t aScaled = a * (128-blend);
  uint16_t bScaled = b * blend;
  return (aScaled + bScaled) >> 7;
}

/**
 * An envelope generator with customizable envelope shape (curve).
 *
 * The template type CurveT should provide a getValue(x) method that returns
 * 0 for x=0 and 255 for x=255.
 */
template<typename CurveT>
class Envelope {
private:
  const CurveT *mCurve;

  JNTUB::FastClock mClock;
  uint32_t mPrevClockPhase;
  uint32_t mAttackRate;
  uint32_t mDecayRate;
  uint8_t mCurVal;
  enum {
    IDLE,
    RISE,
    FALL,
  } mState;

public:
  Envelope(const CurveT *curve)
  {
    mCurve = curve;
    mPrevClockPhase = 0;
    mAttackRate = 0;
    mDecayRate = 0;
    mCurVal = 0;
    mState = IDLE;
  }

  void setAttack(uint32_t attackMicros)
  {
    mAttackRate = microsToRate(attackMicros);
  }

  void setDecay(uint32_t decayMicros)
  {
    mDecayRate = microsToRate(decayMicros);
  }

  inline void trigger()
  {
    mState = RISE;
    mClock.setRate(mAttackRate);
    mClock.sync();
    mClock.start();
  }

  inline void update()
  {
    mPrevClockPhase = mClock.getPhase();
    mClock.update();
    uint32_t curClockPhase = mClock.getPhase();

    if (mState == RISE) {
      // If clock finished its period, switch to FALL.
      if (curClockPhase < mPrevClockPhase) {
        mState = FALL;
        mClock.setRate(mDecayRate);
        mClock.sync();
      }
    } else if (mState == FALL) {
      // If clock finished its period, switch to IDLE.
      if (curClockPhase < mPrevClockPhase) {
        mState = IDLE;
        mClock.stop();
      }
    }

    uint8_t index = curClockPhase >> (JNTUB::FastClock::PHASE_BITS - 8);

    if (mState == RISE) {
      mCurVal = mCurve->getValue(index);
    } else if (mState == FALL) {
      mCurVal = 255 - mCurve->getValue(index);
    } else {
      mCurVal = 0;
    }
  }

  uint8_t getValue() const
  {
    return mCurVal;
  }
};

/**
 * Blends between two curves stored in program space.
 */
class BlendedCurve {
private:
  const uint8_t *mCurveA;
  const uint8_t *mCurveB;
  uint8_t mBlend;
  bool mFlip;

public:
  BlendedCurve(const uint8_t *curveA, const uint8_t *curveB)
  {
    mCurveA = curveA;
    mCurveB = curveB;
    mFlip = false;
  }

  void setCurves(const uint8_t *curveA, const uint8_t *curveB)
  {
    mCurveA = curveA;
    mCurveB = curveB;
  }

  void setBlend(uint8_t blend)
  {
    mBlend = blend;
  }

  void setFlip(bool flip)
  {
    mFlip = flip;
  }

  uint8_t getValue(uint8_t index) const
  {
    if (mFlip)
      index = 255 - index;
    uint8_t valueA = pgm_read_byte_near(mCurveA + index);
    uint8_t valueB = pgm_read_byte_near(mCurveB + index);
    uint8_t blended = blend(valueA, valueB, mBlend);
    if (mFlip)
      blended = 255 - blended;
    return blended;
  }
};

// Attack/decay times in microseconds.
const uint32_t TIME_CURVE[] = {
  0,
  16000,
  48000,
  128000,
  512000,
  2048000,
  8192000,
  16384000,
  32768000,
};

const uint8_t *const SHAPE_CURVES[] PROGMEM = {
  CURVE_SQUARE,
  CURVE_INVERSE,
  CURVE_EXPONENTIAL,
  CURVE_QUADRATIC,
  CURVE_LINEAR,
};

JNTUB::DiscreteKnob shapeKnobLeft(NELEM(SHAPE_CURVES)-1, 0);
JNTUB::DiscreteKnob shapeKnobRight(NELEM(SHAPE_CURVES)-1, 0);

JNTUB::CurveKnob<uint32_t> attackKnob(TIME_CURVE, NELEM(TIME_CURVE));
JNTUB::CurveKnob<uint32_t> decayKnob(TIME_CURVE, NELEM(TIME_CURVE));

JNTUB::EdgeDetector trigger;

BlendedCurve blendedCurve(CURVE_LINEAR, CURVE_LINEAR);
Envelope<BlendedCurve> env(&blendedCurve);

void setup()
{
  JNTUB::setUpFastPWM();
  JNTUB::setUpTimerInterrupt(TIMER_RATE);
}

void loop()
{
  uint16_t shapeRaw = analogRead(JNTUB::PIN_PARAM3);
  uint16_t decayRaw = analogRead(JNTUB::PIN_PARAM2);
  uint16_t attackRaw = analogRead(JNTUB::PIN_PARAM1);

  uint8_t curveSelect;
  uint8_t blend;
  if (shapeRaw < 512) {
    shapeKnobLeft.update(shapeRaw * 2);
    curveSelect = shapeKnobLeft.getValue();
    blend = shapeKnobLeft.mapInnerValue(0, 128);
    // To the left of 12 o'clock, the envelope shape is upside down
    // compared to the corresponding position to the right of 12 o'clock.
    blendedCurve.setFlip(true);
  } else {
    shapeKnobRight.update((1024-shapeRaw) * 2);
    curveSelect = shapeKnobRight.getValue();
    blend = shapeKnobRight.mapInnerValue(0, 128);
    blendedCurve.setFlip(false);
  }
  const uint8_t *curveA = pgm_read_word(&SHAPE_CURVES[curveSelect]);
  const uint8_t *curveB = pgm_read_word(&SHAPE_CURVES[curveSelect+1]);
  blendedCurve.setCurves(curveA, curveB);
  blendedCurve.setBlend(blend);

  attackKnob.update(attackRaw);
  decayKnob.update(decayRaw);
  noInterrupts();
  env.setAttack(attackKnob.getValue());
  env.setDecay(decayKnob.getValue());
  interrupts();
}

ISR(TIMER_INTERRUPT)
{
  env.update();

  bool trgRaw = digitalRead(JNTUB::PIN_GATE_TRG);
  trigger.update(trgRaw);
  if (trigger.isRising())
    env.trigger();

  JNTUB::analogWriteOut(env.getValue());
}
