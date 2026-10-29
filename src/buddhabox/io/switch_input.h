/**
 * @file switch_input.h
 * @brief Debounced digital switch input with trigger/hold/release detection.
 *
 * Reads a momentary switch (or button) wired to ground and provides
 * clean, debounced event signals — a pressed trigger, a released
 * trigger, and a held-down state query. Designed for controlling:
 *
 *   - Envelope gates: sustain a note while held, release on lift
 *   - One-shot triggers: fire an envelope, advance a sequence step
 *   - Toggle events: flip between modes on each press
 *   - Manual mixItUp() calls: trigger a scene change from hardware
 *
 * WIRING (active-low with internal pull-up):
 * ──────────────────────────────────────────
 * Connect one leg of the switch to the GPIO pin, the other leg to
 * GND. The internal pull-up resistor holds the pin HIGH when the
 * switch is open, and the switch pulls it LOW when pressed.
 *
 *   3.3V ──[internal pull-up]── PA1 ──┤ switch ├── GND
 *
 * This is the simplest possible wiring: no external resistors needed.
 * Call begin() in setup() to configure the pin as INPUT_PULLUP.
 *
 * WHY DEBOUNCING IS NECESSARY:
 * ────────────────────────────
 * Mechanical switches have tiny metal contacts that physically bounce
 * when they make or break contact. In the first ~5–20ms after a press,
 * the signal can flutter between HIGH and LOW dozens of times. Without
 * debouncing, a single button press can register as many presses.
 *
 * We use a time-based approach with millis():
 *   1. Track when the raw pin reading last changed.
 *   2. Only accept the reading as "settled" if it has been stable for
 *      at least debounce_ms milliseconds.
 *   3. Once settled, update the canonical state and fire the
 *      appropriate one-shot trigger flag.
 *
 * This is more robust than a counter-based approach because it works
 * correctly regardless of how often update() is called.
 *
 * CALLING FROM THE AUDIO CALLBACK:
 * ──────────────────────────────────
 * triggered() and released() are safe to call from the audio callback
 * (ISR context). They read and clear a single bool flag. On ARM
 * Cortex-M4, bool reads and writes are atomic — no locking needed.
 * update() must only be called from loop(), as digitalRead() is
 * not suitable for ISR use.
 *
 * Example usage:
 *   SwitchInput btn(PA1);         // Switch on PA1, default 20ms debounce
 *
 *   void setup() {
 *       btn.begin();              // Configure pin as INPUT_PULLUP
 *   }
 *
 *   void loop() {
 *       btn.update();             // Must be called regularly from loop()
 *       delay(10);
 *   }
 *
 *   // In audio callback — gate a note envelope:
 *   if (btn.triggered()) {
 *       env.trigger();            // Attack on press
 *   }
 *   if (btn.released()) {
 *       env.release();            // Release on lift (if envelope supports it)
 *   }
 *   // Or: hold gate open while pressed (for sustain-style envelopes):
 *   float gate = btn.isHeld() ? 1.0f : 0.0f;
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

namespace bb {

/**
 * Debounced digital switch input with trigger/hold/release detection.
 *
 * See the file header above for how it works and usage examples.
 */
class SwitchInput {
public:
  /**
   * Construct a SwitchInput.
   *
   * @param pin          The digital GPIO pin the switch is wired to.
   *                     One switch leg goes to this pin, the other to GND.
   *                     On the Black Pill, any GPIO pin works (PA1–PA15,
   *                     PB0–PB15, PC13, etc.).
   * @param debounce_ms  Debounce window in milliseconds. The pin reading
   *                     must be stable for this long before a state change
   *                     is accepted. Default 20ms covers most switches;
   *                     increase to 50ms for noisy or cheap switches.
   *
   * Example:
   *   SwitchInput btn(PA1);          // Default 20ms debounce
   *   SwitchInput btn2(PB3, 50);     // Noisier switch, longer window
   */
  SwitchInput(uint8_t pin, uint32_t debounce_ms = 20)
      : pin_(pin), debounce_ms_(debounce_ms), debounced_state_(false),
        raw_reading_(false), trigger_flag_(false), release_flag_(false),
        last_change_ms_(0) {}

  /**
   * Configure the GPIO pin. Must be called once in setup() before
   * any calls to update().
   *
   * Sets the pin to INPUT_PULLUP mode — the internal pull-up resistor
   * holds the pin HIGH when the switch is open, eliminating the need
   * for an external pull-up resistor.
   *
   * Example:
   *   void setup() {
   *       btn.begin();
   *   }
   */
  void begin() {
    // INPUT_PULLUP enables the microcontroller's internal pull-up
    // resistor. The pin reads HIGH (open) and LOW (pressed/grounded).
    pinMode(pin_, INPUT_PULLUP);
  }

  /**
   * Read the switch and update the debounced state. Call this from
   * loop() at a regular interval. 5–20ms polling (50–200 Hz) is ideal;
   * faster is fine, slower may add perceptible latency.
   *
   * Do NOT call from the audio callback — digitalRead() is not safe
   * in ISR context and adds unpredictable latency.
   *
   * This method uses millis() for timing so it works correctly at any
   * polling rate. The debounce window is measured in real wall-clock
   * time, not update() call counts.
   *
   * Example:
   *   void loop() {
   *       btn.update();
   *       delay(10);   // ~100 Hz polling
   *   }
   */
  void update() {
    // Active-low: pin is LOW when pressed (switch connects pin to GND),
    // HIGH when open (internal pull-up wins). We invert so that
    // pressed = true, which is more intuitive in calling code.
    bool reading = (digitalRead(pin_) == LOW);
    uint32_t now = millis();

    // Step 1: detect any change in the raw reading and start the
    // debounce timer. Any further bouncing resets the timer.
    if (reading != raw_reading_) {
      raw_reading_ = reading;
      last_change_ms_ = now;
    }

    // Step 2: check if the raw reading has been stable long enough
    // to be accepted as a genuine state change.
    if ((now - last_change_ms_) >= debounce_ms_) {
      if (reading != debounced_state_) {
        // The reading has settled and disagrees with current state:
        // this is a confirmed press or release event.
        debounced_state_ = reading;

        if (debounced_state_) {
          // Transition: open → pressed. Set the one-shot trigger.
          // This flag will be consumed by the next triggered() call.
          trigger_flag_ = true;
        } else {
          // Transition: pressed → open. Set the one-shot release flag.
          release_flag_ = true;
        }
      }
    }
  }

  /**
   * Returns true once per button press (falling edge), then clears.
   *
   * This fires exactly once when a confirmed press is detected,
   * regardless of how long the button is held or how many times
   * triggered() is polled. Subsequent calls return false until
   * the next press.
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @return  true on the first call after a confirmed press,
   *          false otherwise.
   *
   * Example:
   *   // In audio callback — one-shot note trigger:
   *   if (btn.triggered()) {
   *       osc.setFrequency(440.0f);
   *       env.trigger();
   *   }
   */
  bool triggered() {
    if (trigger_flag_) {
      trigger_flag_ = false;
      return true;
    }
    return false;
  }

  /**
   * Returns true once per button release (rising edge), then clears.
   *
   * Useful for sustain-style envelopes where you want the note to
   * hold while the button is down and begin releasing on lift-off.
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @return  true on the first call after a confirmed release,
   *          false otherwise.
   *
   * Example:
   *   // In audio callback — sustain while held, release on lift:
   *   if (btn.triggered()) { env.trigger(); }
   *   if (btn.released())  { env.release(); }
   */
  bool released() {
    if (release_flag_) {
      release_flag_ = false;
      return true;
    }
    return false;
  }

  /**
   * Returns the current debounced switch state.
   *
   * true  = switch is currently pressed (held down)
   * false = switch is open (not pressed)
   *
   * Useful for gating: multiply a signal by this cast to float
   * to open/close a gate while the button is held.
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @return  true if the switch is currently held down.
   *
   * Example:
   *   // Hard gate: silence output unless button is held
   *   float gate = btn.isHeld() ? 1.0f : 0.0f;
   *   float out = osc.process() * env.process() * gate;
   */
  bool isHeld() const { return debounced_state_; }

private:
  uint8_t pin_;          // GPIO pin number
  uint32_t debounce_ms_; // Debounce window in milliseconds

  // Debounced (accepted) state: true = pressed. Updated only after
  // the raw reading has been stable for debounce_ms_.
  bool debounced_state_;

  // The last raw reading from digitalRead(). May be bouncing.
  bool raw_reading_;

  // One-shot flags set by update(), consumed by triggered()/released().
  // Using volatile to ensure the compiler doesn't cache these across
  // the ISR boundary.
  volatile bool trigger_flag_;
  volatile bool release_flag_;

  // Timestamp (millis) of the last change in raw_reading_.
  // Used to measure how long the reading has been stable.
  uint32_t last_change_ms_;
};

} // namespace bb
