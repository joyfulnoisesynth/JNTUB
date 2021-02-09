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

  Project:     JoyfulNoise D-VCO
  File:        D-VCO.ino
  Description: Digital, 4-Voice, Morphing Wavetable VCO in 4HP

 */

// JoyfulNoise Tiny Utility Board Library
#include <JNTUB.h>

// Wavetables and note data
#include "Tables.h"

#define isneg(a) (a < 0)
#define neg(a) (~a + 1)

/**
 * Blends between two values with no divides.
 * For blend=0, returns a
 * For blend=64, returns average of a and b
 * For blend=128, returns b
 *
 * XXX: Turns out this is too slow! Curse you, multiplication!!
 * Attempt 1 to fix: make fastBlend where I do a manual less-than-8-bit
 * multiplication by hand with shifts and adds.
 * If that doesn't work, I will store the blended wavetable in memory
 * compute the whole thing during loop().
 */
int8_t blend(int8_t a, int8_t b, uint8_t blend)
{
  if (blend > 128)
    blend = 128;
  // Guaranteed to not overflow. INT8_MAX * 64 + INT8_MAX * 64 == 2^14 - 1
  int16_t aScaled = a * (128-blend);
  int16_t bScaled = b * blend;
  return (aScaled + bScaled) >> 7;
  //return (blend > 64) ? b : a;
}

/**
 * Quickly multiply an unsigned byte by some small value no greater than max.
 */
uint16_t fastMultiplyUnsigned(uint8_t op, uint8_t multiplicand, uint8_t max)
{
  // result = op * 2^0 * (multiplicand & 2^0) +
  //          op * 2^1 * (multiplicand & 2^1) +
  //          etc...
  uint8_t N = 1;
  int16_t opTimesN = op;
  int16_t result = 0;

  while (N <= max) {
    result += (multiplicand & N) ? opTimesN : 0;
    N = N << 1;
    opTimesN = opTimesN << 1;
  }

  return result;
}

/**
 * Quickly multiply a signed byte by some small signed value no greater
 * than +max and no less than -max.
 */
int16_t fastMultiplySigned(int8_t op, int8_t multiplicand, int8_t max)
{
  bool negateResult = 0;
  if (isneg(op)) {
    op = neg(op);
    negateResult ^= 1;
  }
  if (isneg(multiplicand)) {
    multiplicand = neg(multiplicand);
    negateResult ^= 1;
  }
  uint16_t result = fastMultiplyUnsigned(op, multiplicand, max);
  return negateResult ? neg(result) : result;
}

/**
 * Blends between two values with no divides or multiplies.
 * For blend=0, returns a
 * For blend=16, returns average of a and b
 * For blend=32, returns b
 */
int8_t fastBlend(int8_t a, int8_t b, uint8_t blend)
{
  if (blend > 32)
    blend = 32;

  int16_t aScaled = fastMultiplySigned(a, 32 - blend, 32);
  int16_t bScaled = fastMultiplySigned(b, blend, 32);
  return (aScaled + bScaled) >> 5;
}


/**
 * Single-voice wavetable oscillator.
 */
template<typename WavetableT>
class Oscillator {
private:
  // Some indexable wavetable.
  const WavetableT *mWavetable;

  // Phase accumulator.
  uint16_t mPhase;

  // Pitch of the oscillator expressed as fraction of periods per sample.
  uint16_t mPitch;

public:
  Oscillator()
  {
    mPhase = 0;
    mPitch = 0;
    mWavetable = nullptr;
  }

  inline void setPitch(uint16_t pitch)
  {
    mPitch = pitch;
  }

  inline void setPhase(uint16_t phase)
  {
    mPhase = phase;
  }

  inline void setWavetable(const WavetableT *wavetable)
  {
    mWavetable = wavetable;
  }

  inline int8_t getSample()
  {
    mPhase += mPitch;
    if (mWavetable)
      return mWavetable->getValue(mPhase);
    return 0;
  }
};

/**
 * Indexable wavetable of size 2^N stored in SRAM.
 */
template<unsigned N>
class SRAMWavetable {
private:
  int8_t mTable[1<<N];

public:
  SRAMWavetable() {
    // Fill table with zeros.
    memset(mTable, 0, sizeof(mTable));
  }

  int8_t *getPtr()
  {
    return mTable;
  }

  inline int8_t getValue(uint16_t phase) const
  {
    uint16_t index = phase >> (16-N);
    return mTable[index];
  }
};

/**
 * The wavetables to be blended between.
 */
#define NUM_WAVETABLES 4
const int8_t *const WAVETABLES[NUM_WAVETABLES] = {
  WT_SINE,
  WT_TRIANGLE,
  WT_SAWTOOTH,
  WT_SQUARE,
};

/**
 * The multiplications required to compute the blended wave shape
 * are too expensive to perform during each sample.
 * So instead, we will compute the entire blended wavetable during
 * loop() and store it in memory, and then switch over the pointer
 * when we've finished filling the in-memory wavetable.
 *
 * So we'll need to two in-memory tables to switch between.
 * One is on-deck being filled by loop() as quickly as possible,
 * and the other is being read by the oscillator(s).
 *
 * Since there's only 512 bytes of SRAM, we unfortunately can't store
 * two copies of the full wavetables. So chop em in half. 128 samples
 * is still enough to get good waves.
 */
SRAMWavetable<7> BLENDED_WAVETABLES[2];
uint8_t onDeckWavetable;

/**
 * Blends two wavetables together and stores the result in the 128-byte
 * array result.
 */
void blendWavetables(
  const int8_t *tableA,
  const int8_t *tableB,
  uint8_t blendAmt,
  int8_t *result)
{
  for (uint8_t i = 0; i < 128; ++i) {
    int8_t sampA = pgm_read_byte_near(tableA + i*2);
    int8_t sampB = pgm_read_byte_near(tableB + i*2);
    result[i] = blend(sampA, sampB, blendAmt);
  }
}


/**
 * Oscillator voice(s).
 */
#define NUM_OSCS 3
Oscillator<SRAMWavetable<7>> oscs[NUM_OSCS];

// Map input range to midi notes 0 to 119 (10 octaves)
JNTUB::DiscreteKnob pitchKnob(120, 1);

JNTUB::DiscreteKnob waveKnob(NUM_WAVETABLES-1, 1);
uint8_t currentTable;
uint8_t currentBlend;

#define DETUNE_MIN 0
#define DETUNE_MAX 10
#define MAX_SUB_OCTAVE 2
JNTUB::DiscreteKnob detuneKnob(MAX_SUB_OCTAVE + 1, 1);

JNTUB::EdgeDetector sync;

void setup()
{
  onDeckWavetable = 1;
  currentTable = 0;
  currentBlend = 0;

  JNTUB::setUpFastPWM();
  JNTUB::setUpTimerInterrupt(JNTUB::SAMPLE_RATE_20_KHZ);
}

void loop()
{
  uint16_t pitchRaw = analogRead(JNTUB::PIN_PARAM1);
  uint16_t waveRaw = analogRead(JNTUB::PIN_PARAM2);
  uint16_t detuneRaw = analogRead(JNTUB::PIN_PARAM3);
  bool syncRaw = digitalRead(JNTUB::PIN_GATE_TRG);

  pitchKnob.update(pitchRaw);
  waveKnob.update(waveRaw);
  detuneKnob.update(detuneRaw);
  sync.update(syncRaw);

  uint8_t midiNote = pitchKnob.getValue();
  uint16_t pitch = pgm_read_word(&MIDI_NOTE_PITCHES[midiNote]);
  uint8_t subOct = detuneKnob.getValue();
  uint16_t detune = detuneKnob.mapInnerValue(DETUNE_MIN, DETUNE_MAX);
  oscs[0].setPitch(pitch >> subOct);
  oscs[1].setPitch(pitch - detune);
  oscs[2].setPitch(pitch + (detune>>1));
  //oscs[3].setPitch(pitch+detune);

  if (sync.isRising()) {
    for (uint8_t i = 0; i < NUM_OSCS; ++i) {
      oscs[i].setPhase(0);
    }
  }

  uint8_t tableSelect = waveKnob.getValue();
  uint8_t blend = waveKnob.mapInnerValue(0, 128);

  if (tableSelect != currentTable || blend != currentBlend) {
    // Compute new blended wavetable
    blendWavetables(
      WAVETABLES[tableSelect],
      WAVETABLES[tableSelect+1],
      blend,
      BLENDED_WAVETABLES[onDeckWavetable].getPtr());

    currentTable = tableSelect;
    currentBlend = blend;

    // Switch wavetables
    noInterrupts();
    for (uint8_t i = 0; i < NUM_OSCS; ++i) {
      oscs[i].setWavetable(&BLENDED_WAVETABLES[onDeckWavetable]);
    }
    onDeckWavetable = !onDeckWavetable;
    interrupts();
  }

}

ISR(TIMER_INTERRUPT)
{
  int16_t sample1 = oscs[0].getSample() << 1;
  int8_t sample2 = oscs[1].getSample();
  int8_t sample3 = oscs[2].getSample();
  //int8_t sample4 = oscs[3].getSample();
  int16_t sum = (int16_t)sample1 + sample2 + sample3;
  int8_t sample = sum >> 2;
  JNTUB::analogWriteOut(sample + 128);
}
