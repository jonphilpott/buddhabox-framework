/**
 * @file mpr121_input.h
 * @brief MPR121 capacitive touch controller — standalone I2C driver
 *        with per-electrode trigger/hold/release detection.
 *
 * The MPR121 is a 12-electrode capacitive touch sensor IC from NXP.
 * It communicates over I2C and handles signal conditioning, baseline
 * tracking, and threshold detection entirely in hardware — so NO
 * software debounce is required. The chip does it for you via
 * configurable touch/release threshold hysteresis.
 *
 * This driver talks directly to the MPR121 via Arduino's Wire library.
 * No external library dependency is needed.
 *
 * HOW CAPACITIVE TOUCH WORKS:
 * ────────────────────────────────────────────────────────────────────
 * Each electrode continuously measures capacitance. The chip maintains
 * a slowly-updated "baseline" per electrode to track environmental
 * drift (temperature, humidity). When a finger touches a pad, the
 * capacitance rises above the baseline by enough to cross the touch
 * threshold. When it falls back below the release threshold, a release
 * is registered.
 *
 * The gap between touch_thresh and release_thresh (the hysteresis
 * window) prevents rapid toggling when a finger is hovering near the
 * threshold — the same role debounce plays in mechanical switches.
 *
 * WIRING (I2C):
 * ────────────────────────────────────────────────────────────────────
 *   MPR121 VCC → 3.3V
 *   MPR121 GND → GND
 *   MPR121 SDA → PB9  (I2C1 SDA, alternate mapping)
 *   MPR121 SCL → PB8  (I2C1 SCL, alternate mapping)
 *   MPR121 ADD → GND  → I2C address 0x5A (default)
 *              → 3.3V → 0x5B
 *              → SDA  → 0x5C
 *              → SCL  → 0x5D
 *
 * I2C pull-up resistors (4.7kΩ to 3.3V) are required on SDA and SCL
 * if not already present on your MPR121 breakout board.
 *
 * IMPORTANT — DO NOT CALL update() FROM THE AUDIO ISR:
 * ────────────────────────────────────────────────────────────────────
 * Wire I2C transactions block for several hundred microseconds. This
 * would cause audio buffer underruns. Call update() from loop()
 * instead. The triggered(), released(), and isHeld() methods are
 * safe to call from the audio callback — they only read volatile
 * bool flags that update() sets.
 *
 * Example usage:
 *   MPR121Input touch;                   // address 0x5A, thresh 12/6
 *
 *   void setup() {
 *       if (!touch.begin()) {
 *           Serial.println("MPR121 not found!");
 *       }
 *   }
 *
 *   void loop() {
 *       touch.update();          // poll ~100x/sec from loop()
 *       delay(10);
 *   }
 *
 *   // In audio callback — electrode 0 gates an envelope:
 *   if (touch.triggered(0)) { env.trigger(); }
 *   if (touch.released(0))  { env.release(); }
 *
 *   // Electrode 5 as a hard gate:
 *   float gate = touch.isHeld(5) ? 1.0f : 0.0f;
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

namespace bb {

// ── MPR121 Register Addresses ─────────────────────────────────────────────
// Named constants for every register we use. This makes the init
// sequence readable without a datasheet open. All addresses are from
// the MPR121 datasheet (NXP document AN3944).
namespace mpr121_reg {

// Touch status: two bytes whose combined bits 0-11 are the touch state
// for electrodes 0-11. Bit N = 1 means electrode N is being touched.
constexpr uint8_t TOUCHSTATUS_L = 0x00;
constexpr uint8_t TOUCHSTATUS_H = 0x01;

// Baseline filter registers.
// The MPR121 tracks a slowly-drifting "baseline" for each electrode so
// that environmental changes (humidity, temperature) don't cause false
// triggers. These registers tune how fast that baseline can change.
//   MHD = Maximum Half Delta: largest step the filter can take per cycle
//   NHD = Noise Half Delta: step size when signal is in the noise band
//   NCL = Noise Count Limit: how many noise-band samples before moving
//   FDL = Filter Delay Count Limit: samples to hold before first move
constexpr uint8_t MHDR = 0x2B; // Max half-delta, rising
constexpr uint8_t NHDR = 0x2C; // Noise half-delta, rising
constexpr uint8_t NCLR = 0x2D; // Noise count limit, rising
constexpr uint8_t FDLR = 0x2E; // Filter delay count, rising
constexpr uint8_t MHDF = 0x2F; // Max half-delta, falling
constexpr uint8_t NHDF = 0x30; // Noise half-delta, falling
constexpr uint8_t NCLF = 0x31; // Noise count limit, falling
constexpr uint8_t FDLF = 0x32; // Filter delay count, falling
constexpr uint8_t NHDT = 0x33; // Noise half-delta, touched state
constexpr uint8_t NCLT = 0x34; // Noise count limit, touched state
constexpr uint8_t FDLT = 0x35; // Filter delay count, touched state

// Per-electrode touch and release thresholds.
// Registers are interleaved: TOUCHTH_0=0x41, RELEASETH_0=0x42,
//   TOUCHTH_1=0x43, RELEASETH_1=0x44, ... stride = 2 per electrode.
constexpr uint8_t TOUCHTH_0 = 0x41;
constexpr uint8_t RELEASETH_0 = 0x42;

// Debounce register. Upper nibble = release debounce, lower = touch.
// Each count adds one sample period of extra confirmation time.
constexpr uint8_t DEBOUNCE = 0x5B;

// Analog Front End configuration.
// CONFIG1: charge/discharge current for the sensing circuit.
// CONFIG2: charge time, sample frequency (ESI field).
constexpr uint8_t CONFIG1 = 0x5C;
constexpr uint8_t CONFIG2 = 0x5D;

// Electrode Configuration Register.
// Write 0x00 to enter stop mode (required before changing config).
// Write 0x8F to enable all 12 electrodes with baseline tracking.
constexpr uint8_t ECR = 0x5E;

// Auto-configuration: the chip measures each electrode's parasitics
// and adjusts charge current automatically. The UP/LOW/TARGET limits
// describe the target range as a fraction of the supply voltage.
constexpr uint8_t AUTOCONFIG0 = 0x7B;
constexpr uint8_t UPLIMIT = 0x7D;
constexpr uint8_t LOWLIMIT = 0x7E;
constexpr uint8_t TARGETLIMIT = 0x7F;

// Soft reset: write the magic value 0x63 to reset all registers.
constexpr uint8_t SOFTRESET = 0x80;

// Out-of-Range Status registers.
// A set bit means that electrode's auto-configuration failed — the
// charge current could not be calibrated within LOWLIMIT..UPLIMIT.
// That electrode will be unresponsive until auto-config succeeds or
// is disabled.
//   OORS_L bits 7-0: ELE7-ELE0
//   OORS_H bits 3-0: ELE11-ELE8
constexpr uint8_t OORS_L = 0x02;
constexpr uint8_t OORS_H = 0x03;

// Baseline Value registers, one per electrode (ELE0=0x1E, stride 1).
// Each register holds the upper 8 bits of the 10-bit baseline value.
// Multiply by 4 to recover the actual baseline count.
constexpr uint8_t BASELINE_0 = 0x1E;

// Filtered data registers, two bytes per electrode (ELE0=0x04, stride 2).
// The 10-bit filtered reading = (high_byte << 2) | (low_byte & 0x03).
constexpr uint8_t FILTERED_L_0 = 0x04;

} // namespace mpr121_reg

// ── MPR121Input ───────────────────────────────────────────────────────────

/**
 * MPR121 capacitive touch controller — standalone I2C driver with per-electrode
 * trigger/hold/release detection.
 *
 * See the file header above for how it works and usage examples.
 */
class MPR121Input {
public:
  /** Number of capacitive electrodes on the MPR121. */
  static constexpr uint8_t NUM_ELECTRODES = 12;

  /**
   * Construct an MPR121Input.
   *
   * @param addr            I2C address of the MPR121. Set by the ADD
   *                        pin on the breakout board:
   *                          GND → 0x5A (default)
   *                          VCC → 0x5B
   *                          SDA → 0x5C
   *                          SCL → 0x5D
   * @param touch_thresh    Touch threshold, applied to all 12 electrodes
   *                        (0–255). Higher = less sensitive. Default 12
   *                        is appropriate for bare copper pads ~1 cm².
   *                        Reduce (e.g. 6) for thick plastic overlays.
   * @param release_thresh  Release threshold (0–255). Must be less than
   *                        touch_thresh. The gap between them is the
   *                        hysteresis window that prevents "chatter"
   *                        when a finger hovers near the boundary.
   *                        Default 6.
   *
   * Example:
   *   MPR121Input touch;              // 0x5A, threshold 12/6
   *   MPR121Input touch(0x5B, 8, 4); // alternate address, sensitive
   */
  MPR121Input(uint8_t addr = 0x5A, uint8_t touch_thresh = 12,
              uint8_t release_thresh = 6)
      : addr_(addr), touch_thresh_(touch_thresh),
        release_thresh_(release_thresh), current_touched_(0) {
    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
      trigger_flags_[i] = false;
      release_flags_[i] = false;
    }
  }

  /**
   * Initialize the MPR121 over I2C. Call once in setup().
   *
   * Performs a soft reset, configures the baseline filter, sets
   * per-electrode thresholds, configures the analog front-end, enables
   * auto-configuration for 3.3V operation, and starts all 12 electrodes.
   *
   * Avoid touching the electrode pads during begin() — the chip
   * captures the initial baseline reading immediately on startup,
   * and a finger on a pad at that moment will corrupt the baseline.
   *
   * @return  true if the chip responded on the I2C bus.
   *          false if no response (check wiring, address, power).
   *
   * Example:
   *   void setup() {
   *       if (!touch.begin()) {
   *           Serial.println("MPR121 not found — check wiring!");
   *           while (1) {}
   *       }
   *   }
   */
  bool begin() {
    // Set pins explicitly before begin() — Wire defaults may vary
    // across STM32 core versions. PB9=SDA, PB8=SCL is the I2C1
    // alternate mapping on the Black Pill.
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.begin();

    // Step 1: Soft reset. Writing 0x63 to register 0x80 resets all
    // internal registers to power-on defaults. This is important when
    // the MCU reboots while the MPR121 stays powered — without reset,
    // it may be stuck in a previous configuration.
    writeReg(mpr121_reg::SOFTRESET, 0x63);
    delay(1); // 1ms for the reset to settle (from datasheet)

    // Step 2: Enter stop mode. Most configuration registers can only
    // be written when the chip is stopped (ECR = 0). The soft reset
    // already does this, but being explicit here is good practice.
    writeReg(mpr121_reg::ECR, 0x00);

    // Verify the chip is on the bus by reading ECR back. If Wire
    // can't reach the device, readReg() returns 0xFF (I2C bus high).
    if (readReg(mpr121_reg::ECR) != 0x00) {
      return false;
    }

    // Step 3: Configure the baseline tracking filter.
    //
    // The baseline is a slowly-updated reference for each electrode.
    // It tracks gradual environmental changes (temperature, humidity)
    // but must ignore fast changes from touches. These register values
    // are from the MPR121 application note AN3944 and are a reliable
    // starting point. Adjust MHDR/MHDF if your environment has very
    // slow or very fast drift.
    writeReg(mpr121_reg::MHDR, 0x01);
    writeReg(mpr121_reg::NHDR, 0x01);
    writeReg(mpr121_reg::NCLR, 0x0E);
    writeReg(mpr121_reg::FDLR, 0x00);
    writeReg(mpr121_reg::MHDF, 0x01);
    writeReg(mpr121_reg::NHDF, 0x05);
    writeReg(mpr121_reg::NCLF, 0x01);
    writeReg(mpr121_reg::FDLF, 0x00);
    // When touched, baseline tracking is slowed to near-zero so a
    // sustained press doesn't gradually erode the touch signal.
    writeReg(mpr121_reg::NHDT, 0x00);
    writeReg(mpr121_reg::NCLT, 0x00);
    writeReg(mpr121_reg::FDLT, 0x00);

    // Step 4: Set per-electrode touch and release thresholds.
    //
    // The threshold registers are interleaved: address 0x41 is E0
    // touch, 0x42 is E0 release, 0x43 is E1 touch, etc. (stride=2).
    // Touch > release creates the hysteresis gap. This is the primary
    // "debounce" mechanism for capacitive sensing.
    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
      writeReg(mpr121_reg::TOUCHTH_0 + i * 2, touch_thresh_);
      writeReg(mpr121_reg::RELEASETH_0 + i * 2, release_thresh_);
    }

    // Step 5: Debounce register — additional sample count confirmation
    // before accepting a touch or release event. 0x00 means rely solely
    // on the threshold hysteresis above. Increase (e.g. 0x11) if you
    // still see chatter in electrically noisy environments.
    writeReg(mpr121_reg::DEBOUNCE, 0x00);

    // Step 6: Analog Front End (AFE) configuration.
    //   CONFIG1 = 0x10: 16μA charge/discharge current. This is the
    //     recommended default for most electrode sizes (1–2 cm²).
    //   CONFIG2 = 0x20: 0.5μs charge time, 16ms sample interval (ESI).
    //     Longer ESI = more averaging = better noise rejection, but
    //     adds latency. At ESI=16ms, touch latency is ~32ms worst case.
    writeReg(mpr121_reg::CONFIG1, 0x10);
    writeReg(mpr121_reg::CONFIG2, 0x20);

    // Step 7: Auto-configuration for 3.3V operation.
    //
    // The auto-config system measures each electrode's parasitics and
    // adjusts the charge current automatically. This compensates for
    // electrodes of different shapes, sizes, and trace lengths without
    // manual tuning. The UP/LOW/TARGET limits are calculated as
    // fractions of Vdd = 3.3V:
    //   UPLIMIT  = (Vdd - 0.7) / Vdd * 256 ≈ 200
    //   LOWLIMIT = UPLIMIT * 0.65            ≈ 130
    //   TARGET   = UPLIMIT * 0.90            ≈ 180
    //
    // AUTOCONFIG0 = 0x0B: enable both auto-config and auto-reconfig.
    // Auto-reconfig re-runs configuration on any electrode that fails.
    writeReg(mpr121_reg::AUTOCONFIG0, 0x0B);
    writeReg(mpr121_reg::UPLIMIT, 200);
    writeReg(mpr121_reg::LOWLIMIT, 130);
    writeReg(mpr121_reg::TARGETLIMIT, 180);

    // Step 8: Enable all 12 electrodes and start sensing.
    //
    // ECR = 0x8F breaks down as:
    //   CL    = 10  (5-point baseline: init from first filtered sample)
    //   ELEPROX_EN = 00 (no proximity electrode)
    //   ELE_EN = 1111 (enable electrodes 0-11, i.e. 12 = 0xC = 1100,
    //                  but 0x8F lower nibble = 1111 = 15 electrodes...
    //                  actually 0x8F = 1000 1111: CL=10, ELE_EN=1111
    //                  which enables electrodes 0-11, exactly 12.)
    //
    // Do NOT touch any pads from this point until update() has been
    // called a few times — the baseline needs to stabilize first.
    writeReg(mpr121_reg::ECR, 0x8F);

    return true;
  }

  /**
   * Read the current touch state from the chip and update per-electrode
   * trigger and release flags.
   *
   * Call from loop() at a regular interval. 10ms (100Hz) is ideal —
   * fast enough to feel responsive, slow enough to keep I2C overhead
   * negligible. The MPR121's internal sample rate is ~62Hz at default
   * settings, so polling faster than ~30Hz gives no additional benefit.
   *
   * DO NOT call from the audio callback. Wire I2C transactions block
   * for several hundred microseconds and will cause audio underruns.
   *
   * Example:
   *   void loop() {
   *       touch.update();
   *       delay(10);
   *   }
   */
  void update() {
    uint16_t touched = readTouched();

    // Compare each electrode's new state to its previous state.
    // A 0→1 transition is a touch (trigger); 1→0 is a release.
    // The flags are set here and consumed by triggered()/released().
    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
      bool was = (current_touched_ >> i) & 1;
      bool now = (touched >> i) & 1;

      if (!was && now) {
        // Rising edge: electrode just became touched.
        trigger_flags_[i] = true;
      } else if (was && !now) {
        // Falling edge: electrode just became released.
        release_flags_[i] = true;
      }
    }

    current_touched_ = touched;
  }

  /**
   * Returns true once per touch event on the given electrode, then
   * clears. Identical semantics to SwitchInput::triggered().
   *
   * Safe to call from the audio callback (ISR context) — reads and
   * clears a single volatile bool. On ARM Cortex-M4, bool reads and
   * writes are atomic, so no locking is required.
   *
   * @param electrode  Electrode index to query (0–11).
   * @return           true on the first call after a touch event;
   *                   false otherwise, or if electrode >= 12.
   *
   * Example:
   *   if (touch.triggered(0)) { env.trigger(); }
   */
  bool triggered(uint8_t electrode) {
    if (electrode >= NUM_ELECTRODES)
      return false;
    if (trigger_flags_[electrode]) {
      trigger_flags_[electrode] = false;
      return true;
    }
    return false;
  }

  /**
   * Returns true once per release event on the given electrode, then
   * clears. Identical semantics to SwitchInput::released().
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @param electrode  Electrode index to query (0–11).
   * @return           true on the first call after a release event;
   *                   false otherwise, or if electrode >= 12.
   *
   * Example:
   *   if (touch.triggered(0)) { env.trigger(); }
   *   if (touch.released(0))  { env.release(); }
   */
  bool released(uint8_t electrode) {
    if (electrode >= NUM_ELECTRODES)
      return false;
    if (release_flags_[electrode]) {
      release_flags_[electrode] = false;
      return true;
    }
    return false;
  }

  /**
   * Returns the current touch state of the given electrode.
   *
   * true  = electrode is currently being touched (held)
   * false = electrode is open (not touched)
   *
   * Safe to call from the audio callback (ISR context) — reads a
   * single bit from current_touched_ which is a uint16_t updated
   * atomically by update().
   *
   * @param electrode  Electrode index to query (0–11).
   * @return           true if the electrode is currently touched.
   *
   * Example:
   *   // Gate a signal while pad 3 is held:
   *   float gate = touch.isHeld(3) ? 1.0f : 0.0f;
   *   float out = osc.process() * env.process() * gate;
   */
  /**
   * Print a diagnostic report over Serial for all 12 electrodes.
   *
   * Reports for each electrode:
   *   - OOR (Out of Range): whether auto-configuration failed. If true,
   *     the electrode will not respond to touch until auto-config is
   *     re-run or disabled.
   *   - Baseline: the 10-bit reference level the chip calibrated to.
   *     A value of 0 or very high usually confirms auto-config failure.
   *   - Filtered: the current 10-bit filtered reading from the electrode.
   *     Delta = Baseline - Filtered; a touch produces a negative delta
   *     that crosses the release/touch thresholds.
   *
   * Call from setup() after begin(), not from the audio callback.
   *
   * Example:
   *   if (!touch.begin()) { Serial.println("not found"); }
   *   touch.diagnosticDump();
   */
  void diagnosticDump() {
    uint8_t oors_l = readReg(mpr121_reg::OORS_L);
    uint8_t oors_h = readReg(mpr121_reg::OORS_H);

    Serial.println("MPR121 Diagnostics:");
    Serial.println("ELE  OOR  Baseline  Filtered  Delta");

    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
      // OOR bit: low byte covers ELE0-7, high byte covers ELE8-11
      bool oor = (i < 8) ? ((oors_l >> i) & 1) : ((oors_h >> (i - 8)) & 1);

      // Baseline: one byte per electrode, upper 8 bits of 10-bit value
      uint16_t baseline = (uint16_t)readReg(mpr121_reg::BASELINE_0 + i) << 2;

      // Filtered: two bytes per electrode (little-endian 10-bit value)
      uint8_t filt_l = readReg(mpr121_reg::FILTERED_L_0 + i * 2);
      uint8_t filt_h = readReg(mpr121_reg::FILTERED_L_0 + i * 2 + 1);
      uint16_t filtered = ((uint16_t)(filt_h & 0x03) << 8) | filt_l;

      int16_t delta = (int16_t)baseline - (int16_t)filtered;

      Serial.print("  ");
      if (i < 10)
        Serial.print(" ");
      Serial.print(i);
      Serial.print("    ");
      Serial.print(oor ? "Y" : "N");
      Serial.print("    ");
      if (baseline < 100)
        Serial.print(" ");
      if (baseline < 1000)
        Serial.print(" ");
      Serial.print(baseline);
      Serial.print("      ");
      if (filtered < 100)
        Serial.print(" ");
      if (filtered < 1000)
        Serial.print(" ");
      Serial.print(filtered);
      Serial.print("      ");
      Serial.println(delta);
    }
  }

  bool isHeld(uint8_t electrode) const {
    if (electrode >= NUM_ELECTRODES)
      return false;
    return (current_touched_ >> electrode) & 1;
  }

private:
  uint8_t addr_;           // I2C bus address (0x5A–0x5D)
  uint8_t touch_thresh_;   // Touch threshold applied to all electrodes
  uint8_t release_thresh_; // Release threshold applied to all electrodes

  // 12-bit touch state: bit N = 1 means electrode N is currently
  // touched. Updated by update(), read by isHeld().
  uint16_t current_touched_;

  // One-shot event flags, one per electrode.
  // Set by update() on edge transitions, consumed by triggered() /
  // released(). volatile prevents the compiler from caching these
  // values across the ISR boundary.
  volatile bool trigger_flags_[NUM_ELECTRODES];
  volatile bool release_flags_[NUM_ELECTRODES];

  /**
   * Write one byte to an MPR121 register over I2C.
   *
   * @param reg    Register address (see mpr121_reg namespace).
   * @param value  Byte to write.
   */
  void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  /**
   * Read one byte from an MPR121 register over I2C.
   *
   * Uses a "repeated start" (endTransmission(false)) to send the
   * register address and then immediately read without releasing the
   * bus. This is required by the MPR121's I2C register read protocol.
   *
   * @param reg  Register address.
   * @return     Register value, or 0xFF if no data received (error).
   */
  uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    // false = repeated start (no STOP condition between write and read)
    Wire.endTransmission(false);
    Wire.requestFrom(addr_, (uint8_t)1);
    if (Wire.available()) {
      return Wire.read();
    }
    return 0xFF; // No response — device not on bus
  }

  /**
   * Read the 12-bit touch status from the MPR121 in a single burst.
   *
   * Registers 0x00 (low byte) and 0x01 (high byte) hold the touch
   * state for electrodes 0-11. Reading both in one I2C transaction
   * ensures both bytes are from the same sample instant (atomic read).
   *
   * Bits 12-15 of the high byte are out-of-range / overflow flags —
   * we mask them off with 0x0FFF to keep only electrode bits.
   *
   * @return  12-bit touch state. Bit N = 1 → electrode N is touched.
   */
  uint16_t readTouched() {
    Wire.beginTransmission(addr_);
    Wire.write(mpr121_reg::TOUCHSTATUS_L); // Start at register 0x00
    Wire.endTransmission(false);           // Repeated start
    Wire.requestFrom(addr_, (uint8_t)2);   // Read 2 bytes

    uint16_t lo = Wire.available() ? Wire.read() : 0;
    uint16_t hi = Wire.available() ? Wire.read() : 0;

    // Combine: high byte is bits 15-8, low byte is bits 7-0.
    // Mask to 12 bits to discard the out-of-range status flags.
    return ((hi << 8) | lo) & 0x0FFF;
  }
};

} // namespace bb
