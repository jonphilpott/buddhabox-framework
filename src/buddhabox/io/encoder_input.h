/**
 * @file encoder_input.h
 * @brief One rotary encoder, counted in hardware by timer TIM4.
 *
 * A rotary encoder (the clicky knob that turns forever, as on a menu)
 * has two switches, A and B, that open and close a quarter of a step
 * apart as you turn it. The order they change in gives the direction:
 *
 *   clockwise:      A B = 00 → 10 → 11 → 01 → 00   (one full cycle)
 *   anticlockwise:  A B = 00 → 01 → 11 → 10 → 00
 *
 * Each change is an "edge". Most encoders make 4 edges per click
 * (detent); some make 2.
 *
 * WHY A HARDWARE TIMER?
 * ─────────────────────
 * A brisk turn makes an edge every 2–5 ms. If you read the pins from
 * loop() and loop() stalls for longer than that, you miss edges and can
 * even read the direction backwards. loop() does stall: the audio
 * interrupt takes most of each block at high CPU load, and an Oled
 * display() takes about 27 ms.
 *
 * The STM32's general-purpose timers have an "encoder mode" that reads
 * A and B themselves and counts up or down on every edge. Counting
 * costs no CPU and misses nothing however long loop() stalls; we read
 * the count whenever convenient.
 *
 * WHAT ABOUT CONTACT BOUNCE?
 * ──────────────────────────
 * Quadrature counting largely cancels bounce by itself. While one
 * contact bounces, the other is steady, so the count goes +1, -1, +1,
 * -1 and settles where it should. On top of that, the timer's digital
 * input filter ignores glitches shorter than about 10 µs.
 *
 * WIRING (fixed pins):
 * ────────────────────
 *   Encoder A   → PB6   (TIM4 channel 1)
 *   Encoder B   → PB7   (TIM4 channel 2)
 *   Encoder C   → GND   (the middle pin of the three)
 *   Switch legs → any free pin and GND; read it with SwitchInput
 *
 * The internal pull-ups are turned on, so no resistors are needed. If
 * the count runs the wrong way, swap A and B. These pins are fixed:
 * only TIM4 reaches them, and only these pins reach TIM4 without
 * taking analog pins. Don't use analogWrite() on PB6/PB7 as well,
 * since it would reprogram TIM4.
 *
 * Example usage:
 *   EncoderInput enc;             // 4 edges per detent
 *   SwitchInput  push(PB5);       // the encoder's push switch
 *
 *   void setup() {
 *       enc.begin();
 *       push.begin();
 *   }
 *
 *   void loop() {
 *       int turn = enc.delta();   // detents since the last call
 *       if (turn != 0) menuItem += turn;
 *       push.update();
 *       if (push.triggered()) selectItem(menuItem);
 *   }
 */

#pragma once

#include <Arduino.h>
#include <cstdint>
#include <stm32f4xx_hal.h>

namespace bb {

/**
 * One rotary encoder on PB6/PB7, counted by timer TIM4.
 *
 * See the file header above for how it works and how to wire it.
 *
 * Example:
 *   EncoderInput enc;        // in setup(): enc.begin();
 *   int turn = enc.delta();  // in loop(): +1 per click clockwise
 */
class EncoderInput {
public:
  /**
   * Construct an EncoderInput. Nothing touches the hardware until
   * begin().
   *
   * @param edgesPerDetent  Edges the encoder makes per click: 4 for most
   *                        (e.g. EC11, PEC11), 2 for some. If one click
   *                        moves two steps, double this; if two clicks
   *                        move one step, halve it. 0 is treated as 1.
   *
   * Example:
   *   EncoderInput enc;      // 4 edges per click
   *   EncoderInput enc2(2);  // half-step encoder
   */
  explicit EncoderInput(uint8_t edgesPerDetent = 4)
      : edges_(edgesPerDetent ? edgesPerDetent : 1), last_(0), pending_(0) {}

  /**
   * Set up PB6/PB7 and start TIM4 counting. Call once in setup().
   *
   * Example:
   *   void setup() { enc.begin(); }
   */
  void begin() {
    // Step 1: turn on the clocks. Every STM32 peripheral starts
    // switched off to save power, and its registers ignore writes
    // until its clock is on.
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    // Step 2: hand PB6/PB7 to TIM4. "Alternate function" mode routes a
    // pin to a peripheral instead of plain GPIO; AF2 is TIM3-5 on F4
    // chips. The pull-ups hold A and B high while the encoder's
    // contacts are open, as with SwitchInput.
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    // Step 3: configure the timer. We write the registers directly:
    // it is five lines, and the HAL encoder API would need its own
    // module enabled in the core. Register names are from the STM32F4
    // reference manual (RM0383), chapter "General-purpose timers".
    TIM4->CR1 = 0; // stop the timer while we set it up

    // CKD = 10: the input filter samples at timer clock / 4 (24 MHz on
    // the F411), so the filter below covers a longer glitch.
    TIM4->CR1 = TIM_CR1_CKD_1;

    // CC1S = CC2S = 01: channels 1 and 2 take their inputs from their
    // own pins (TI1 = PB6, TI2 = PB7). IC1F = IC2F = 1111: the
    // strongest filter, which needs 8 equal samples at fDTS / 32 before
    // it accepts a change, about 10 µs here.
    TIM4->CCMR1 =
        TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;

    // Not inverted (CC1P = CC2P = 0). Setting CC1P reverses direction.
    TIM4->CCER = 0;

    // SMS = 011: encoder mode 3, count on edges of both A and B, so
    // the counter moves 4 per full cycle. The timer works out the
    // direction from which input changed and the level of the other.
    TIM4->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

    // Count the full 16-bit range, start from 0 and go.
    TIM4->ARR = 0xFFFF;
    TIM4->CNT = 0;
    last_ = 0;
    pending_ = 0;
    TIM4->CR1 |= TIM_CR1_CEN;
  }

  /**
   * Clicks turned since the last call: positive for clockwise,
   * negative for anticlockwise, 0 if the knob hasn't moved.
   *
   * Call it from one place only (normally loop()), because each call
   * consumes the movement. Edges short of a whole click are kept for
   * next time, so slow turns aren't lost. It only reads a register, so
   * it is also safe from the audio callback.
   *
   * @return Signed number of clicks since the previous call.
   *
   * Example:
   *   volume = constrain(volume + enc.delta(), 0, 10);
   */
  int delta() {
    // 1. Edges since last time. The counter is 16 bits and wraps from
    //    65535 to 0. Subtracting as uint16_t and casting the result to
    //    int16_t gives the right signed difference across the wrap
    //    (0 - 65535 = 1 in 16-bit arithmetic), provided fewer than
    //    32768 edges happened between calls, i.e. always.
    uint16_t now = static_cast<uint16_t>(TIM4->CNT);
    pending_ += static_cast<int16_t>(static_cast<uint16_t>(now - last_));
    last_ = now;

    // 2. Whole clicks. C++ integer division rounds toward zero, so the
    //    remainder keeps its sign: -3 edges stays -3, not +1.
    int clicks = pending_ / edges_;
    pending_ -= clicks * edges_;
    return clicks;
  }

private:
  int edges_;     // edges per click (detent)
  uint16_t last_; // counter value at the previous delta()
  int pending_;   // edges counted but not yet a whole click
};

} // namespace bb
