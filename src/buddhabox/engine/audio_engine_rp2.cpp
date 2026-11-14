/**
 * @file audio_engine_rp2.cpp
 * @brief I2S audio output for the RP2350 (Raspberry Pi Pico 2 and other
 *        RP2350 boards) through the arduino-pico core's I2S library.
 *
 * ONE FILE PER CHIP: this whole file is skipped unless arduino-pico is
 * building it (ARDUINO_ARCH_RP2040, which it defines for the RP2350 as
 * well); audio_engine_stm32.cpp is the Black Pill's. Both implement the
 * same audio_engine.h, so a patch's audioCallback() runs unchanged.
 *
 * HOW IT DIFFERS FROM THE STM32 ENGINE:
 *
 *   core 0: setup(), loop()          core 1: the audio loop below
 *   (knobs, menu, OLED, serial)      audioCallback() → I2S buffers → DAC
 *
 * 1. The audio runs on the RP2350's second core, not in an interrupt.
 *    loop() keeps all of core 0, so a slow OLED or serial print can no
 *    longer delay the audio, and the audio can't starve loop() either.
 *    The patch's habit of writing single floats in loop() for the audio
 *    to read stays safe: a 32-bit write is atomic across cores too.
 * 2. There is no I2S peripheral. The I2S library builds one from a PIO
 *    block (a tiny programmable state machine for pin protocols) fed by
 *    DMA from a ring of buffers. The audio loop computes a block, then
 *    waits for a free buffer to copy it into: that wait is its spare time.
 * 3. The PIO divides the system clock to make the bit clock, and 150 MHz
 *    doesn't divide into 44.1 kHz cleanly. setSysClk() moves the system
 *    clock to 144 MHz, which does (the library's own table), for both
 *    cores.
 *
 * PINS: BCLK on BB_I2S_BCLK (default GP10), LRCLK on the next pin (the
 * PIO program drives the two clocks as a pair), DIN on BB_I2S_DOUT
 * (default GP12). Override with build flags, e.g.
 *   build_flags = -D BB_I2S_BCLK=2 -D BB_I2S_DOUT=4
 */

#if defined(ARDUINO_ARCH_RP2040)

#include "buddhabox/engine/audio_engine.h"
#include <Arduino.h>
#include <I2S.h>
#include <hardware/timer.h>

using bb::AUDIO_BLOCK_SIZE;
using bb::SAMPLE_RATE;

static I2S i2s(OUTPUT);

// One block of stereo frames, [L, R, L, R, ...], as on the STM32. Aligned
// so it can be handed to the I2S library as 32-bit words.
static int16_t block[AUDIO_BLOCK_SIZE * 2] __attribute__((aligned(4)));

// Set by begin() on core 0; the audio loop on core 1 idles until then.
static volatile bool running = false;

// ─── CPU load ────────────────────────────────────────────────────────
// Timed with the 1 MHz system timer, which both cores share, rather
// than a cycle counter: one block is 5,805 us, so 1 us is 0.02% of the
// budget, finer than the meter needs. `volatile`: core 1 writes them,
// loop() on core 0 reads them.
static volatile uint32_t lastMicros = 0;
static volatile uint32_t peakMicros = 0;
static const float kBlockMicros = 1e6f * AUDIO_BLOCK_SIZE / SAMPLE_RATE;

/**
 * The audio loop: arduino-pico runs loop1() over and over on core 1
 * once it sees one defined. Defining it here means the engine owns
 * core 1, so a patch must not define setup1() or loop1() itself.
 */
void loop1() {
  if (!running)
    return;

  // 1. Compute a block, timed.
  uint32_t start = time_us_32();
  audioCallback(block, AUDIO_BLOCK_SIZE * 2);
  uint32_t us = time_us_32() - start; // unsigned maths handles wrap
  lastMicros = us;
  if (us > peakMicros)
    peakMicros = us;

  // 2. The library sends each 32-bit word as (left << 16) | right. In
  //    memory (little-endian) that's [right, left], the reverse of our
  //    [left, right], so swap the halves of each word. A rotate by 16
  //    is one instruction.
  uint32_t *words = (uint32_t *)block;
  for (int i = 0; i < AUDIO_BLOCK_SIZE; i++)
    words[i] = (words[i] << 16) | (words[i] >> 16);

  // 3. Hand it over. write() takes what fits in the free buffers and
  //    returns; keep going until it has the lot. The waiting here is
  //    the time to spare.
  const uint8_t *bytes = (const uint8_t *)block;
  size_t left = sizeof(block);
  while (left > 0) {
    size_t sent = i2s.write(bytes, left);
    bytes += sent;
    left -= sent;
  }
}

namespace bb {
namespace AudioEngine {

float cpuLoad() { return lastMicros / kBlockMicros; }

float cpuPeak() {
  uint32_t peak = peakMicros;
  // ponytail: a block finishing between these two lines loses its peak;
  // harmless for a meter.
  peakMicros = 0;
  return peak / kBlockMicros;
}

void begin() {
  // 1. Clocks and pins. LRCLK is BB_I2S_BCLK + 1.
  i2s.setSysClk(SAMPLE_RATE);
  i2s.setBCLK(BB_I2S_BCLK);
  i2s.setDATA(BB_I2S_DOUT);
  i2s.setBitsPerSample(16);

  // 2. Four buffers of one block each: 23 ms of audio queued. Fewer
  //    gives less delay but less slack for an uneven block.
  i2s.setBuffers(4, AUDIO_BLOCK_SIZE);
  i2s.begin(SAMPLE_RATE);

  // 3. Let core 1 start.
  running = true;
}

} // namespace AudioEngine
} // namespace bb

#endif // ARDUINO_ARCH_RP2040
