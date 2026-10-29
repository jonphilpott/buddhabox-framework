/**
 * @file led.h
 * @brief Driver for the onboard user LED (PC13) on the Black Pill.
 *
 * The STM32F401CC Black Pill has a single user-controllable LED wired to
 * pin PC13. The LED is ACTIVE LOW: driving the pin LOW turns it ON, and
 * driving it HIGH turns it OFF. This is a common STM32 board convention
 * — the LED cathode connects to the GPIO pin, and the anode is pulled to
 * 3.3V through a resistor.
 *
 * This class provides:
 *   - Instant on / off / toggle control.
 *   - Non-blocking blink at a configurable interval (driven by update()).
 *   - A clean way to signal firmware status without touching audio code.
 *
 * IMPORTANT — DO NOT CALL FROM AUDIO ISR:
 * digitalWrite() and millis() are not safe inside the DMA audio callback.
 * Call update() from loop() only.
 *
 * Example usage:
 *   UserLED led;
 *
 *   void setup() {
 *       led.begin();
 *       led.blink(500);  // Blink every 500 ms while audio initialises
 *   }
 *
 *   void loop() {
 *       led.update();    // Must be called each loop to service blink
 *   }
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

namespace bb {

/**
 * Driver for the onboard user LED (PC13) on the Black Pill.
 *
 * See the file header above for how it works and usage examples.
 */
class UserLED {
public:
  // PC13 is the onboard user LED on the WeAct Black Pill board.
  // It is active LOW: LOW = illuminated, HIGH = extinguished.
  static constexpr uint8_t PIN = PC13;

  /**
   * Construct a UserLED object.
   *
   * The LED is not configured until begin() is called. No pin I/O
   * occurs in the constructor so it is safe to instantiate globally.
   */
  UserLED() : blinking_(false), interval_(500), lastToggle_(0) {}

  /**
   * Initialise the LED pin as an output and ensure the LED is off.
   *
   * Call once from setup() before any other method.
   *
   * Example:
   *   void setup() {
   *       led.begin();
   *   }
   */
  void begin() {
    pinMode(PIN, OUTPUT);
    off();
  }

  /**
   * Turn the LED on immediately and cancel any active blink.
   *
   * Example:
   *   led.on();  // LED lights up, stays solid
   */
  void on() {
    blinking_ = false;
    // Active LOW: pull the pin LOW to light the LED.
    digitalWrite(PIN, LOW);
  }

  /**
   * Turn the LED off immediately and cancel any active blink.
   *
   * Example:
   *   led.off();  // LED goes dark
   */
  void off() {
    blinking_ = false;
    // Active LOW: pull the pin HIGH to extinguish the LED.
    digitalWrite(PIN, HIGH);
  }

  /**
   * Toggle the LED between on and off, and cancel any active blink.
   *
   * Useful for event-driven toggling (e.g. one press = on, next = off).
   *
   * Example:
   *   if (btn.triggered()) { led.toggle(); }
   */
  void toggle() {
    blinking_ = false;
    // Read the current output latch and invert it.
    // Because the LED is active LOW, inverting the pin level flips
    // the visible state: LOW→HIGH = off, HIGH→LOW = on.
    digitalWrite(PIN, !digitalRead(PIN));
  }

  /**
   * Start a non-blocking blink at the specified interval.
   *
   * The LED will alternate on/off every interval_ms milliseconds.
   * You MUST call update() from loop() for the blink to advance.
   *
   * @param interval_ms  Half-period in milliseconds.
   *                     LED is ON for interval_ms, then OFF for
   *                     interval_ms, repeating indefinitely.
   *                     Default: 500 ms (1 Hz blink).
   *                     Range: 1 – 2^32 ms.
   *
   * Example:
   *   led.blink(250);   // Fast blink: 2 Hz
   *   led.blink(1000);  // Slow heartbeat: 0.5 Hz
   */
  void blink(uint32_t interval_ms = 500) {
    interval_ = interval_ms;
    blinking_ = true;
    lastToggle_ = millis();
    // Drive the LED on immediately so the first state is visible
    // right away rather than waiting for the first interval to pass.
    digitalWrite(PIN, LOW);
  }

  /**
   * Service the non-blocking blink timer.
   *
   * Call this once per loop() iteration. If blinking is not active
   * this returns immediately with no side effects, so it is always
   * safe to call.
   *
   * Do NOT call from the audio callback — millis() and digitalWrite()
   * are not ISR-safe and will cause audio glitches.
   *
   * Example:
   *   void loop() {
   *       led.update();
   *       sensor.update();
   *       btn.update();
   *   }
   */
  void update() {
    if (!blinking_)
      return;

    uint32_t now = millis();

    // Unsigned subtraction handles the millis() rollover (~49 days)
    // correctly: even when now wraps back to 0, (now - lastToggle_)
    // remains a valid positive duration in unsigned arithmetic.
    if (now - lastToggle_ >= interval_) {
      lastToggle_ = now;
      digitalWrite(PIN, !digitalRead(PIN));
    }
  }

  /**
   * Query whether a blink is currently running.
   *
   * @return  true if blink() was called and has not been cancelled
   *          by on(), off(), or toggle().
   *
   * Example:
   *   if (!led.isBlinking()) { led.blink(200); }
   */
  bool isBlinking() const { return blinking_; }

private:
  bool blinking_;       // Whether non-blocking blink is active
  uint32_t interval_;   // Blink half-period in milliseconds
  uint32_t lastToggle_; // millis() timestamp of last state change
};

} // namespace bb
