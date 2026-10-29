/**
 * @file matrix_keypad.h
 * @brief 5×5 diode matrix keypad — debounced input with per-key
 *        trigger/hold/release detection and N-key rollover.
 *
 * A 5×5 matrix keypad has 25 momentary switches arranged in a grid,
 * sharing 5 row wires and 5 column wires (10 pins total). A diode is
 * placed in series with each switch to allow multiple simultaneous
 * keypresses without producing phantom "ghost" keypresses.
 *
 * HOW MATRIX SCANNING WORKS:
 * ────────────────────────────────────────────────────────────────────
 * Instead of one GPIO per switch (which needs 25 pins), the matrix
 * reuses wires in a grid. The scan loop:
 *
 *   For each row (0–4):
 *     1. Drive that row pin LOW (all others stay HIGH).
 *     2. Wait 10µs for the signal to settle on PCB traces.
 *     3. Read each column pin. A LOW reading means the switch at
 *        (active_row, that_column) is pressed — the switch connects
 *        the column's pull-up to the driven-LOW row.
 *     4. Restore the row pin to HIGH before moving on.
 *
 * WHY DIODES PREVENT GHOSTING:
 * ────────────────────────────────────────────────────────────────────
 * Without diodes, pressing keys (R0,C0), (R0,C1), and (R1,C0)
 * simultaneously creates a ghost current path:
 *
 *   C1(HIGH) → Switch(R0,C1) → R0 bus → Switch(R0,C0) → C0
 *            → Switch(R1,C0) → R1(LOW)
 *
 * This makes C1 appear LOW when scanning R1, falsely detecting key
 * (R1,C1) as pressed. Each diode (cathode at the row wire, anode at
 * the switch/column side) blocks this reverse current: when R0 is
 * HIGH (not being scanned), the diode at any (R0,*) key is
 * reverse-biased, so no ghost current can flow.
 *
 * WIRING (STM32F401CC Black Pill — I2C-safe pin assignment):
 * ────────────────────────────────────────────────────────────────────
 *   Row pins   : PA3, PA4, PA5, PA6, PA7
 *   Column pins: PB0, PB1, PB4, PB5, PB14
 *
 *   Row pins   : OUTPUT, driven LOW one at a time during scanning.
 *                All other rows held HIGH between scans.
 *   Column pins: INPUT_PULLUP (HIGH when open, LOW when pressed).
 *   Diode      : cathode to row wire, anode to switch (one per key).
 *
 *   I2C1 (PB6/PB7 or PB8/PB9), I2C2 (PB10/PB3), and I2C3 (PA8)
 *   are deliberately avoided so an I2C peripheral can coexist.
 *
 * KEY NUMBERING:
 * ────────────────────────────────────────────────────────────────────
 *   key = row × 5 + col   (0–24)
 *
 *    Col:  0   1   2   3   4
 *   Row 0: 0   1   2   3   4
 *   Row 1: 5   6   7   8   9
 *   Row 2:10  11  12  13  14
 *   Row 3:15  16  17  18  19
 *   Row 4:20  21  22  23  24
 *
 * CALLING FROM THE AUDIO CALLBACK:
 * ────────────────────────────────────────────────────────────────────
 * triggered(), released(), and isHeld() are safe to call from the
 * audio callback (ISR context). They read/clear volatile bool flags.
 * On ARM Cortex-M4, bool reads and writes are atomic — no locking
 * needed. update() must only be called from loop().
 *
 * Example usage:
 *   const uint8_t rows[5] = {PA3, PA4, PA5, PA6, PA7};
 *   const uint8_t cols[5] = {PB0, PB1, PB4, PB5, PB14};
 *   MatrixKeypad pad(rows, cols);
 *
 *   void setup() { pad.begin(); }
 *
 *   void loop() {
 *       pad.update();
 *       delay(10);   // ~100 Hz polling
 *   }
 *
 *   // In audio callback:
 *   if (pad.triggered(0)) { env.trigger(); } // key 0 = row 0, col 0
 *   if (pad.released(0))  { env.release(); }
 *   float gate = pad.isHeld(6) ? 1.0f : 0.0f; // key 6 = row 1, col 1
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

namespace bb {

/**
 * 5×5 diode matrix keypad — debounced input with per-key trigger/hold/release
 * detection and N-key rollover.
 *
 * See the file header above for how it works and usage examples.
 */
class MatrixKeypad {
public:
  /** Number of rows in the keypad grid. */
  static constexpr uint8_t ROWS = 5;
  /** Number of columns in the keypad grid. */
  static constexpr uint8_t COLS = 5;
  /** Total number of keys (ROWS × COLS). */
  static constexpr uint8_t NUM_KEYS = ROWS * COLS;

  /**
   * Construct a MatrixKeypad.
   *
   * @param row_pins    Array of 5 GPIO pin numbers for the row wires.
   *                    Configured as OUTPUT; driven LOW one at a time
   *                    during scanning, HIGH otherwise.
   * @param col_pins    Array of 5 GPIO pin numbers for the column
   *                    wires. Configured as INPUT_PULLUP.
   * @param debounce_ms Debounce window in milliseconds. A key state
   *                    must be stable for this long before it is
   *                    accepted. Default 20ms suits most keypad
   *                    membranes; increase to 50ms for noisier or
   *                    cheaper contacts.
   *
   * Example:
   *   const uint8_t rows[5] = {PA3, PA4, PA5, PA6, PA7};
   *   const uint8_t cols[5] = {PB0, PB1, PB4, PB5, PB14};
   *   MatrixKeypad pad(rows, cols);        // 20ms debounce
   *   MatrixKeypad pad(rows, cols, 30);    // 30ms for membrane pads
   */
  MatrixKeypad(const uint8_t row_pins[ROWS], const uint8_t col_pins[COLS],
               uint32_t debounce_ms = 20)
      : debounce_ms_(debounce_ms) {
    for (uint8_t i = 0; i < ROWS; i++)
      row_pins_[i] = row_pins[i];
    for (uint8_t i = 0; i < COLS; i++)
      col_pins_[i] = col_pins[i];
    for (uint8_t i = 0; i < NUM_KEYS; i++) {
      raw_state_[i] = false;
      debounced_state_[i] = false;
      last_change_ms_[i] = 0;
      trigger_flags_[i] = false;
      release_flags_[i] = false;
    }
  }

  /**
   * Configure GPIO pins. Call once from setup() before update().
   *
   * Row pins are set OUTPUT HIGH. Column pins are set INPUT_PULLUP.
   * The diodes in the keypad prevent short-circuit current between
   * row outputs when multiple keys are held, so driving non-active
   * rows HIGH is safe.
   *
   * Example:
   *   void setup() { pad.begin(); }
   */
  void begin() {
    for (uint8_t r = 0; r < ROWS; r++) {
      // Start HIGH — only the actively-scanned row goes LOW.
      // Diodes ensure non-scanned rows cannot create ghost paths
      // even when driven HIGH alongside a LOW scanning row.
      pinMode(row_pins_[r], OUTPUT);
      digitalWrite(row_pins_[r], HIGH);
    }
    for (uint8_t c = 0; c < COLS; c++) {
      // INPUT_PULLUP: column reads HIGH when open, LOW when the
      // active row's key closes the circuit to the LOW row pin.
      pinMode(col_pins_[c], INPUT_PULLUP);
    }
  }

  /**
   * Scan the keypad and update per-key debounced states.
   * Call from loop() at a regular interval (5–20ms is ideal).
   *
   * For each row the method:
   *   1. Drives the row LOW.
   *   2. Waits 10µs for voltage to settle on PCB traces.
   *   3. Reads each column pin.
   *   4. Restores the row to HIGH.
   *   5. Applies per-key millis()-based debounce and fires trigger /
   *      release one-shot flags on confirmed transitions.
   *
   * Do NOT call from the audio callback — digitalRead() and millis()
   * are not suitable for ISR use and would cause audio underruns.
   *
   * Example:
   *   void loop() {
   *       pad.update();
   *       delay(10);   // ~100 Hz polling
   *   }
   */
  void update() {
    uint32_t now = millis();

    for (uint8_t r = 0; r < ROWS; r++) {
      // Step 1: Drive this row LOW. All other rows remain HIGH.
      // The diodes guarantee that a pressed key on a HIGH row cannot
      // feed reverse current into this LOW row's column wires.
      digitalWrite(row_pins_[r], LOW);

      // Step 2: Let the pin voltage settle. Stray capacitance on the
      // keypad traces means the column level takes a few microseconds
      // to fully respond after the row changes state.
      delayMicroseconds(10);

      // Step 3: Read every column while this row is active.
      for (uint8_t c = 0; c < COLS; c++) {
        // Active-low: column goes LOW when a switch in this row is
        // pressed. Invert so that pressed == true.
        bool pressed = (digitalRead(col_pins_[c]) == LOW);
        uint8_t key = r * COLS + c; // Flat index 0–15

        // Debounce step A: detect raw state changes and reset timer.
        // Any further bounce resets the timer, extending the window.
        if (pressed != raw_state_[key]) {
          raw_state_[key] = pressed;
          last_change_ms_[key] = now;
        }

        // Debounce step B: accept state only after the reading has
        // been stable for the full debounce_ms_ window.
        if ((now - last_change_ms_[key]) >= debounce_ms_) {
          if (pressed != debounced_state_[key]) {
            debounced_state_[key] = pressed;
            if (pressed) {
              // Confirmed press: arm the one-shot trigger flag.
              trigger_flags_[key] = true;
            } else {
              // Confirmed release: arm the one-shot release flag.
              release_flags_[key] = true;
            }
          }
        }
      }

      // Step 4: Restore this row to HIGH before scanning the next.
      // Leaving it LOW would cause false positives on subsequent
      // column reads, since any shared key would appear pressed.
      digitalWrite(row_pins_[r], HIGH);
    }
  }

  /**
   * Returns true once per key press (falling edge), then clears.
   *
   * Fires exactly once when a confirmed press is detected, regardless
   * of how long the key is held or how often triggered() is polled.
   * Subsequent calls return false until the next press event.
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @param key  Key index 0–24 (row × 5 + col). Key 0 = top-left,
   *             key 24 = bottom-right of the standard 5×5 layout.
   * @return     true on the first call after a confirmed press;
   *             false otherwise, or if key >= NUM_KEYS.
   *
   * Example:
   *   if (pad.triggered(0)) { env.trigger(); } // key 0: row 0, col 0
   */
  bool triggered(uint8_t key) {
    if (key >= NUM_KEYS)
      return false;
    if (trigger_flags_[key]) {
      trigger_flags_[key] = false;
      return true;
    }
    return false;
  }

  /**
   * Returns true once per key release (rising edge), then clears.
   *
   * Useful for sustain-style patches where the note holds while a key
   * is depressed and begins its release phase on lift-off.
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @param key  Key index 0–24.
   * @return     true on the first call after a confirmed release;
   *             false otherwise, or if key >= NUM_KEYS.
   *
   * Example:
   *   if (pad.triggered(5)) { env.trigger(); }
   *   if (pad.released(5))  { env.release(); }
   */
  bool released(uint8_t key) {
    if (key >= NUM_KEYS)
      return false;
    if (release_flags_[key]) {
      release_flags_[key] = false;
      return true;
    }
    return false;
  }

  /**
   * Returns the current debounced state of a key.
   *
   * true  = key is currently held down
   * false = key is open (not pressed)
   *
   * Useful for continuous gating: multiply a signal by this cast to
   * float to open or close a gate while the key is held.
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @param key  Key index 0–24.
   * @return     true if the key is currently held.
   *
   * Example:
   *   // Gate output only while key 4 (row 0, col 4) is held:
   *   float gate = pad.isHeld(4) ? 1.0f : 0.0f;
   *   float out  = osc.process() * env.process() * gate;
   */
  bool isHeld(uint8_t key) const {
    if (key >= NUM_KEYS)
      return false;
    return debounced_state_[key];
  }

private:
  uint8_t row_pins_[ROWS]; // GPIO pin numbers for the 5 row wires
  uint8_t col_pins_[COLS]; // GPIO pin numbers for the 5 column wires
  uint32_t debounce_ms_;   // Debounce window in milliseconds

  // Raw (un-debounced) reading from the most recent scan pass.
  // May still be mid-bounce; never used directly in application code.
  bool raw_state_[NUM_KEYS];

  // Accepted (debounced) per-key state. Updated only after raw_state_
  // has been stable for the full debounce_ms_ window.
  bool debounced_state_[NUM_KEYS];

  // Timestamp (millis) of the last raw_state_ change for each key.
  // Compared against now in update() to measure how long the reading
  // has been stable.
  uint32_t last_change_ms_[NUM_KEYS];

  // One-shot event flags: set by update() on confirmed transitions,
  // consumed (read+cleared) by triggered() / released().
  // volatile prevents the compiler from caching values across the
  // ISR boundary when queried from the audio callback.
  volatile bool trigger_flags_[NUM_KEYS];
  volatile bool release_flags_[NUM_KEYS];
};

} // namespace bb
