/**
 * @file main.cpp
 * @brief A tone played from a knob and an encoder, with its pitch, the
 *        encoder and the CPU load shown on a 128x64 I2C OLED.
 *
 * Shows the usual way to drive a display: read controls and redraw the
 * screen from loop(), on a millis() timer, never from audioCallback().
 * The tone plays even with no display attached; begin() just reports
 * that it found nothing.
 *
 * Controls:
 *   Knob     sweeps the pitch smoothly over three octaves
 *   Encoder  transposes it in semitones, -24 to +24
 *   Push     (the encoder's switch) resets the transpose to 0
 *
 * Wiring:
 *   Audio    PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN (UDA1334A)
 *   Knob     linear pot: outer legs to 3V3 and GND, wiper to PA1
 *   OLED     VCC -> 3V3, GND -> GND, SCL -> PB8, SDA -> PB9
 *   Encoder  A -> PB6, B -> PB7, C (middle pin) -> GND
 *   Push     one leg -> PB5, the other -> GND
 *
 * For a 1.3" SH1106 module, construct the display as `Oled oled(true);`.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/encoder_input.h"
#include "buddhabox/io/oled.h"
#include "buddhabox/io/switch_input.h"
#include "buddhabox/osc/oscillator.h"

using namespace bb;

Oscillator osc;
AnalogInput knob(PA1, 0.15f); // a little less smoothing: feels quicker
EncoderInput encoder;         // PB6/PB7, 4 edges per click
SwitchInput push(PB5);
Oled oled; // 0.96" SSD1306 at 0x3C
bool haveScreen = false;
int transpose = 0; // semitones, set by the encoder

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
  int x = oled.text(0, 12, "Pitch ");
  x = oled.number(x, 12, hz, 1);
  oled.text(x, 12, " Hz   "); // trailing spaces wipe a longer old value
  oled.rect(0, 21, 128, 8);
  oled.rect(1, 22, 126, 6, true, false);
  oled.rect(1, 22, (int)(level * 126.0f), 6, true);

  // The encoder's transpose, with a sign, and a box that fills while
  // the push switch is held.
  x = oled.text(0, 33, "Transpose ");
  x = oled.text(x, 33, transpose > 0 ? "+" : (transpose < 0 ? "-" : " "));
  x = oled.number(x, 33, (float)(transpose < 0 ? -transpose : transpose), 0);
  oled.text(x, 33, "   ");
  oled.rect(118, 32, 10, 9);
  oled.rect(119, 33, 8, 7, true, push.isHeld());

  // CPU load, the same way as the pitch.
  x = oled.text(0, 45, "CPU ");
  x = oled.number(x, 45, cpu * 100.0f, 1);
  oled.text(x, 45, " %   ");
  oled.rect(0, 54, 128, 8);
  oled.rect(1, 55, 126, 6, true, false);
  oled.rect(1, 55, (int)(minf(cpu, 1.0f) * 126.0f), 6, true);
}

void setup() {
  Serial.begin(115200);
  osc.setWaveform(Waveform::SINE);
  osc.setAmplitude(0.5f);
  encoder.begin();
  push.begin();

  haveScreen = oled.begin();
  if (!haveScreen)
    Serial.println("No OLED found: check SCL/SDA wiring and the address");

  AudioEngine::begin();
}

void loop() {
  // 1. Controls: read often, they're cheap. The encoder counts in
  //    hardware, so even a slow frame can't make it miss a click.
  knob.update();
  push.update();
  transpose = (int)clampf((float)(transpose + encoder.delta()), -24.0f, 24.0f);
  if (push.triggered())
    transpose = 0;

  // 2. Pitch: the knob covers 110 Hz to 880 Hz (three octaves, 36
  //    semitones), and the encoder shifts that up or down.
  float level = knob.read();
  float hz = 110.0f * semitonesToRatio(level * 36.0f + (float)transpose);
  osc.setFrequency(hz);

  // 3. Display: 20 frames a second is smooth enough and leaves loop()
  //    free most of the time (each frame blocks for a few milliseconds).
  static uint32_t lastFrame = 0;
  if (haveScreen && millis() - lastFrame >= 50) {
    lastFrame = millis();
    drawScreen(hz, level, AudioEngine::cpuLoad());
    oled.display();
  }
}
