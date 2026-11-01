/**
 * @file audio_engine.cpp
 * @brief I2S DMA audio output implementation for STM32F401CC + UDA1334A.
 *
 * This file contains all the low-level hardware setup for the I2S peripheral,
 * DMA controller, and GPIO pins. You shouldn't need to modify this unless
 * you're changing the pin assignments or audio format.
 *
 * HARDWARE OVERVIEW:
 * ─────────────────
 * The STM32F4 has a dedicated I2S peripheral (built into the SPI2 block)
 * that serializes 16-bit audio samples and clocks them out to the DAC.
 *
 * The DMA (Direct Memory Access) controller handles the data transfer
 * from our buffer in RAM to the I2S peripheral — the CPU doesn't need
 * to be involved at all during playback.
 *
 * We configure DMA in "circular" mode so it automatically loops back to
 * the start of the buffer when it reaches the end, creating the continuous
 * double-buffer ping-pong described in audio_engine.h.
 */

#include "buddhabox/engine/audio_engine.h"
#include <Arduino.h>

// STM32 HAL (Hardware Abstraction Layer) headers for I2S and DMA
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_i2s.h>

// The DMA buffer size and sample rate live in namespace bb (dsp_common.h).
using bb::AUDIO_BLOCK_SIZE;
using bb::SAMPLE_RATE;

// ─── DMA Buffer
// ──────────────────────────────────────────────────────────────
//
// The DMA buffer holds TWO blocks of stereo audio:
//   Size = AUDIO_BLOCK_SIZE * 2 (stereo) * 2 (double buffer)
//
// With AUDIO_BLOCK_SIZE=256:
//   256 * 2 * 2 = 1024 int16_t values = 2048 bytes
//
// The DMA plays through this linearly while we fill the inactive half.
static int16_t dmaBuffer[AUDIO_BLOCK_SIZE * 2 * 2];

// ─── HAL Handle Structures ──────────────────────────────────────────────────
//
// The STM32 HAL uses "handle" structs to track the state of each peripheral.
// These are passed to HAL functions and updated by interrupt handlers.
static I2S_HandleTypeDef hi2s2;
static DMA_HandleTypeDef hdma_i2s2_tx;

// ─── DMA Interrupt Callbacks ────────────────────────────────────────────────
//
// The HAL calls these when DMA reaches the halfway point and the end of the
// buffer respectively. We use them to fill the half that just finished
// playing.
//
// extern "C" is needed because the HAL is written in C, and C++ name mangling
// would prevent the HAL from finding these functions.

/**
 * Called when DMA finishes sending the FIRST half of the buffer.
 * We fill the first half with new samples while the second half plays.
 */
// ─── CPU Load Measurement ───────────────────────────────────────────────────
//
// The Cortex-M4 has a free-running cycle counter (DWT->CYCCNT) that ticks
// once per CPU clock. Reading it before and after audioCallback() tells us
// exactly how many cycles the DSP took. Dividing by the cycles available
// per block (the time it takes the DAC to play one half-buffer) gives the
// load: 0.5 = half the CPU, 1.0 = the callback no longer keeps up and the
// audio will glitch.
//
// `volatile` because the interrupt writes them and loop() reads them.
static volatile uint32_t lastCycles = 0;
static volatile uint32_t peakCycles = 0; // worst since cpuPeak() last ran

// Cycles in one block, set in begin(): an integer so the interrupt can
// compare against it without any float maths.
static uint32_t blockCycles = 0xFFFFFFFFu;

/** Run the user's callback, time it, and flag an overrun. */
static void runCallback(int16_t *buffer) {
  uint32_t start = DWT->CYCCNT;
  audioCallback(buffer, AUDIO_BLOCK_SIZE * 2);
  uint32_t cycles = DWT->CYCCNT - start; // unsigned maths handles wrap
  lastCycles = cycles;
  if (cycles > peakCycles)
    peakCycles = cycles;
#if BB_OVERRUN_LED
  // The block took longer than the DAC took to play one: light the LED
  // (active-low: LOW is on). Written once, so a patch that overruns
  // every block doesn't pay for a digitalWrite() each time.
  static bool lit = false;
  if (!lit && cycles > blockCycles) {
    lit = true;
    digitalWrite(LED_BUILTIN, LOW);
  }
#endif
}

/** Cycles available per block: CPU clock / sample rate * frames. */
static float cyclesPerBlock() {
  return (float)SystemCoreClock / (float)SAMPLE_RATE * (float)AUDIO_BLOCK_SIZE;
}

extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  runCallback(dmaBuffer);
}

/**
 * Called when DMA finishes sending the SECOND half of the buffer.
 * We fill the second half with new samples while the first half plays.
 */
extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  runCallback(dmaBuffer + AUDIO_BLOCK_SIZE * 2);
}

/**
 * DMA1 Stream4 interrupt handler — routes the interrupt to the HAL.
 */
extern "C" void DMA1_Stream4_IRQHandler(void) {
  HAL_DMA_IRQHandler(hi2s2.hdmatx);
}

namespace bb {
namespace AudioEngine {

/**
 * Manually configure the I2S clock (PLLI2S).
 *
 * CRITICAL: The STM32F401 has a dedicated PLL (Phase Locked Loop) for the I2S
 * peripheral. Unlike most peripherals that run off the standard system clock,
 * I2S requires this specific high-precision clock to generate the Bit Clock
 * (BCLK) and Word Select (LRCLK). If this PLL is not initialized, the I2S
 * peripheral will stay idle and never trigger DMA requests, resulting in
 * silence.
 */
void configureI2SClock() {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  // Configure PLLI2S to generate 44.1kHz clock.
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

float cpuLoad() { return (float)lastCycles / cyclesPerBlock(); }

float cpuPeak() {
  uint32_t peak = peakCycles;
  // ponytail: an interrupt between these two lines loses one block's
  // peak; harmless for a meter, use LDREX/STREX if it ever matters.
  peakCycles = 0;
  return (float)peak / cyclesPerBlock();
}

void begin() {
  // ── Step 0: Enable the cycle counter used by cpuLoad() ─────────────
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable DWT unit
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  blockCycles = (uint32_t)cyclesPerBlock();

#if BB_OVERRUN_LED
  // ── Step 0a: Overrun LED, off until a block runs late ──────────────
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // active-low: HIGH is off
#endif

  // ── Step 0b: Configure I2S Clock (PLLI2S) ──────────────────────────
  // Essential for the I2S peripheral to drive its internal bit clock.
  configureI2SClock();

  // ── Step 1: Enable peripheral clocks ────────────────────────────────
  __HAL_RCC_SPI2_CLK_ENABLE();  // I2S2 shares the SPI2 clock domain
  __HAL_RCC_DMA1_CLK_ENABLE();  // DMA controller clock
  __HAL_RCC_GPIOB_CLK_ENABLE(); // GPIO Port B (where our I2S pins live)

  // ── Step 2: Configure GPIO pins for I2S ────────────────────────────
  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &gpio);

  // ── Step 3: Configure I2S2 peripheral ──────────────────────────────
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = SAMPLE_RATE;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;

  HAL_I2S_Init(&hi2s2);

  // ── Step 4: Configure DMA for I2S2 TX ──────────────────────────────
  hdma_i2s2_tx.Instance = DMA1_Stream4;
  hdma_i2s2_tx.Init.Channel = DMA_CHANNEL_0;
  hdma_i2s2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_i2s2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_i2s2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_i2s2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_i2s2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_i2s2_tx.Init.Mode = DMA_CIRCULAR;
  hdma_i2s2_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_i2s2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  HAL_DMA_Init(&hdma_i2s2_tx);
  __HAL_LINKDMA(&hi2s2, hdmatx, hdma_i2s2_tx);

  // ── Step 5: Enable DMA interrupt ───────────────────────────────────
  // Priority 2: lower than USB (1) and SysTick (0). Lower numbers win
  // on Cortex-M. audioCallback() runs inside this interrupt, so if it
  // ranked above USB, a patch near 100% CPU would starve USB and the
  // board would never appear as a serial port. USB's handlers are short,
  // and the DMA keeps playing the other half-buffer while they run.
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  // ── Step 6: Start DMA playback ─────────────────────────────────────
  memset(dmaBuffer, 0, sizeof(dmaBuffer));
  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)dmaBuffer, AUDIO_BLOCK_SIZE * 2 * 2);
}

} // namespace AudioEngine

} // namespace bb
