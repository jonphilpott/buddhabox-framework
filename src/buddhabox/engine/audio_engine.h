/**
 * @file audio_engine.h
 * @brief I2S DMA audio output engine for the UDA1334A DAC.
 *
 * This is the hardware interface layer — it handles all the low-level details
 * of getting audio samples from your code to the DAC chip via I2S.
 *
 * HOW IT ALL CONNECTS:
 * ───────────────────
 *
 *   Your Code              STM32 Hardware           External DAC
 *   ─────────              ──────────────           ────────────
 *   audioCallback()  →  DMA Buffer  →  I2S Peripheral  →  UDA1334A  →  Speaker
 *   (fills samples)     (in RAM)       (shifts bits out)   (analog out)
 *
 * THE DMA DOUBLE-BUFFER PATTERN:
 * ─────────────────────────────
 * DMA (Direct Memory Access) is hardware that transfers data from RAM to
 * the I2S peripheral automatically, without the CPU doing anything. This
 * lets the CPU focus on computing audio samples.
 *
 * We use a buffer split into two halves:
 *
 *   [────── Half A ──────][────── Half B ──────]
 *    ^                     ^
 *    DMA plays this...     ...while you fill this
 *
 * When the DMA finishes playing Half A, it triggers an interrupt and starts
 * playing Half B. In the interrupt, you fill Half A with new samples. Then
 * when Half B finishes, you fill Half B. This ping-pong continues forever.
 *
 * The result: uninterrupted audio with no gaps or clicks, as long as your
 * audioCallback() finishes before the current half finishes playing.
 *
 * I2S PIN CONNECTIONS (Black Pill → UDA1334A):
 * ────────────────────────────────────────────
 *   PB12 → WSEL (Word Select / LRCLK) — toggles between left/right channel
 *   PB13 → BCLK (Bit Clock)           — clocks out individual bits
 *   PB15 → DIN  (Data In)             — the actual audio data bits
 *   GND  → GND
 *   3V3  → VIN  (or 5V depending on your UDA1334A board)
 *
 * USAGE:
 * ─────
 * 1. Implement the audioCallback() function (see main.cpp for example)
 * 2. Call AudioEngine::begin() in setup()
 * 3. That's it — audio starts playing automatically via DMA
 *
 * Example:
 *   // You must define this function — it's called by the DMA interrupt
 *   void audioCallback(int16_t* buffer, uint16_t length) {
 *       for (uint16_t i = 0; i < length; i += 2) {
 *           float sample = myOscillator.process();
 *           int16_t s = (int16_t)(sample * 32000.0f);
 *           buffer[i]     = s;  // Left channel
 *           buffer[i + 1] = s;  // Right channel (mono)
 *       }
 *   }
 *
 *   void setup() {
 *       AudioEngine::begin();  // Start audio output
 *   }
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

/**
 * The user-implemented audio callback function.
 *
 * You MUST define this function in your main.cpp. It's called automatically
 * by the DMA interrupt handler twice per audio block cycle (once for each
 * half of the double buffer).
 *
 * @param buffer  Pointer to the buffer half to fill with audio data.
 *                Format: stereo interleaved 16-bit samples [L, R, L, R, ...]
 *                Each sample ranges from -32768 to +32767.
 *                To convert from float: (int16_t)(floatSample * 32000.0f)
 *                (We use 32000 instead of 32767 to leave a tiny bit of
 * headroom)
 *
 * @param length  Number of int16_t values to fill (= frames * 2 for stereo).
 *                With AUDIO_BLOCK_SIZE=256, length will be 512.
 *
 * IMPORTANT: This runs in interrupt context! Keep it fast:
 *   - No Serial.print() or other I/O
 *   - No analogRead() (too slow)
 *   - No dynamic memory allocation (malloc/new)
 *   - Keep computations minimal and predictable
 */
extern void audioCallback(int16_t *buffer, uint16_t length);

namespace bb {

namespace AudioEngine {
/** Initialize I2S + DMA and begin audio playback. Call once in setup(). */
void begin();

/**
 * How much of the CPU the last audioCallback() used.
 *
 * Measured with the Cortex-M4's cycle counter around each callback. Use
 * it to find out whether a patch fits: keep it below about 0.8 to leave
 * room for loop() and the USB serial port.
 *
 * @return  0.0 (idle) to 1.0 (fully loaded); above 1.0 the callback is
 *          too slow and the audio will drop out
 *
 * Example:
 *   void loop() {
 *     Serial.println(AudioEngine::cpuLoad() * 100.0f); // percent
 *     delay(500);
 *   }
 */
float cpuLoad();

/**
 * The highest load of any audioCallback() since the last call to
 * cpuPeak(), then reset.
 *
 * cpuLoad() shows only the latest block, so a print once a second can
 * miss a short spike. A spike above 1.0 is a click; the peak catches it.
 *
 * @return  The same scale as cpuLoad(): above 1.0 a block was late
 *
 * Example:
 *   void loop() {
 *     Serial.println(AudioEngine::cpuPeak() * 100.0f); // worst, percent
 *     delay(1000);
 *   }
 */
float cpuPeak();
} // namespace AudioEngine

} // namespace bb
