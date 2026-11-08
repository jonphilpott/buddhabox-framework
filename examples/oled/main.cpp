/**
 * @file main.cpp
 * @brief A knob-controlled tone with its pitch and the CPU load shown on
 *        a 128x64 I2C OLED.
 *
 * Shows the usual way to drive a display: read controls and redraw the
 * screen from loop(), on a millis() timer, never from audioCallback().
 * The tone plays even with no display attached; begin() just reports
 * that it found nothing.
 *
 * Wiring:
 *   Audio  PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN (UDA1334A)
 *   Knob   10k linear pot: outer legs to 3V3 and GND, wiper to PA1
 *   OLED   VCC -> 3V3, GND -> GND, SCL -> PB8, SDA -> PB9
 *
 * For a 1.3" SH1106 module, construct the display as `Oled oled(true);`.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/oled.h"
#include "buddhabox/osc/oscillator.h"

using namespace bb;

Oscillator osc;
AnalogInput knob(PA1, 0.15f); // a little less smoothing: feels quicker
Oled oled;                    // 0.96" SSD1306 at 0x3C
bool haveScreen = false;

/** Fill one block of audio (interrupt context: no I2C, no Serial). */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    float sample = osc.process();
    buffer[i] = buffer[i + 1] =
        (int16_t)(clampf(sample, -1.0f, 1.0f) * 32000.0f);
  }
}

/**
 * Draw one frame. Everything is redrawn each time; text cells paint
 * their own background, so only the bars need erasing first.
 */
void drawScreen(float hz, float level, float cpu) {
  // Title: dark text on a lit bar.
  oled.rect(0, 0, 128, 8, true);
  oled.text(34, 0, "BuddhaBox", false);

  // Pitch, then the knob's position as a bar: an outline, its inside
  // erased, then the filled part.
  int x = oled.text(0, 16, "Pitch ");
  x = oled.number(x, 16, hz, 1);
  oled.text(x, 16, " Hz   "); // trailing spaces wipe a longer old value
  oled.rect(0, 26, 128, 8);
  oled.rect(1, 27, 126, 6, true, false);
  oled.rect(1, 27, (int)(level * 126.0f), 6, true);

  // CPU load, the same way.
  x = oled.text(0, 42, "CPU ");
  x = oled.number(x, 42, cpu * 100.0f, 1);
  oled.text(x, 42, " %   ");
  oled.rect(0, 52, 128, 8);
  oled.rect(1, 53, 126, 6, true, false);
  oled.rect(1, 53, (int)(minf(cpu, 1.0f) * 126.0f), 6, true);
}

void setup() {
  Serial.begin(115200);
  osc.setWaveform(Waveform::SINE);
  osc.setAmplitude(0.5f);

  haveScreen = oled.begin();
  if (!haveScreen)
    Serial.println("No OLED found: check SCL/SDA wiring and the address");

  AudioEngine::begin();
}

void loop() {
  // Controls: read often, they're cheap.
  knob.update();
  float level = knob.read();
  // 110 Hz to 880 Hz: three octaves, so 36 semitones across the knob.
  float hz = 110.0f * semitonesToRatio(level * 36.0f);
  osc.setFrequency(hz);

  // Display: 20 frames a second is smooth enough and leaves loop()
  // free most of the time (each frame blocks for a few milliseconds).
  static uint32_t lastFrame = 0;
  if (haveScreen && millis() - lastFrame >= 50) {
    lastFrame = millis();
    drawScreen(hz, level, AudioEngine::cpuLoad());
    oled.display();
  }
}
