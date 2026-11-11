/**
 * @file oled.h
 * @brief Driver for 128x64 monochrome I2C OLED displays (SSD1306, SH1106).
 *
 * The cheap 0.96" (SSD1306) and 1.3" (SH1106) OLED modules are 128 x 64
 * pixels, one bit per pixel. This driver keeps a copy of the screen in
 * RAM (a "frame buffer", 1 KB), lets you draw pixels, lines, rectangles
 * and text into it, and sends it to the display when you call display().
 * It uses only Arduino's Wire library: no Adafruit or U8g2 dependency.
 *
 * WIRING (I2C, the 4-pin modules):
 * ────────────────────────────────────────────────────────────────────
 *   OLED VCC → 3.3V
 *   OLED GND → GND
 *   OLED SCL → PB8  (I2C1 SCL, alternate mapping)
 *   OLED SDA → PB9  (I2C1 SDA, alternate mapping)
 *
 * These are the same pins MPR121Input uses: I2C is a bus, so both
 * devices can share the two wires (they answer to different addresses).
 * The modules already carry pull-up resistors on SDA and SCL.
 *
 * HOW THE SCREEN MEMORY IS LAID OUT:
 * ────────────────────────────────────────────────────────────────────
 * The display's memory is not row by row. It is split into 8 horizontal
 * strips called "pages", each 8 pixels tall. A page holds 128 bytes, one
 * per column, and each byte is a vertical stack of 8 pixels with bit 0
 * at the top:
 *
 *   page 0 = rows 0..7,  page 1 = rows 8..15, ... page 7 = rows 56..63
 *   pixel (x, y) is bit (y % 8) of byte buf_[y / 8][x]
 *
 * This driver keeps the same layout in RAM so a page can be copied to
 * the display byte for byte.
 *
 * SPEED, AND WHY ONLY CHANGED PAGES ARE SENT:
 * ────────────────────────────────────────────────────────────────────
 * I2C at 400 kHz moves about 40 KB/s, so a full 1 KB screen takes about
 * 27 ms, and Wire blocks loop() until it is done. Audio is unaffected:
 * it runs in its own interrupt. To keep loop() responsive, the driver
 * remembers which pages were drawn on since the last display() (a "dirty"
 * flag per page) and sends only those. Updating one line of text costs
 * about 3.5 ms. Redraw 10 to 20 times a second; a knob display needs no
 * more.
 *
 * IMPORTANT — DO NOT CALL FROM THE AUDIO ISR:
 * Drawing is cheap but display() talks I2C. Call everything from loop().
 *
 * Example usage:
 *   Oled oled;                     // SSD1306 at 0x3C; Oled(true) = SH1106
 *
 *   void setup() {
 *     if (!oled.begin())
 *       Serial.println("No OLED found - check wiring");
 *   }
 *
 *   void loop() {
 *     oled.clear();
 *     oled.text(0, 0, "Cutoff");
 *     oled.number(0, 10, 440.0f, 1);   // "440.0"
 *     oled.rect(0, 20, 100, 8);        // outline
 *     oled.rect(0, 20, 60, 8, true);   // filled: a level bar
 *     oled.display();
 *     delay(50);  // only in this sketch; real patches use millis()
 *   }
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

namespace bb {

/**
 * The 5 x 7 pixel font for printable ASCII (space to '~'), 5 bytes per
 * character. Each byte is one column of the glyph, bit 0 at the top, the
 * same layout as the display's pages. Lives in flash (475 bytes).
 */
static const uint8_t OLED_FONT[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x08, 0x04, 0x08, 0x10, 0x08}, // ~
};

/**
 * A 128 x 64 monochrome OLED on I2C, SSD1306 or SH1106 controller.
 *
 * Draw into the frame buffer with clear(), pixel(), line(), rect(),
 * text() and number(), then call display() to send the changes. Every
 * drawing call takes an `on` flag: true lights pixels, false darkens
 * them, so you can draw dark text on a filled bar. Coordinates start at
 * (0, 0) in the top-left corner; anything off-screen is clipped.
 *
 * See the file header above for wiring and timing.
 *
 * Example:
 *   Oled oled;          // 0.96" SSD1306 module at address 0x3C
 *   Oled big(true);     // 1.3" SH1106 module
 *   Oled other(false, 0x3D);  // address jumper moved
 *
 *   oled.begin();
 *   oled.text(0, 0, "Hello");
 *   oled.display();
 */
class Oled {
public:
  static constexpr int WIDTH = 128;
  static constexpr int HEIGHT = 64;
  static constexpr int PAGES = HEIGHT / 8;
  // Text cells: 5-pixel glyph + 1 blank column, 7 rows + 1 blank row.
  static constexpr int CHAR_W = 6;
  static constexpr int CHAR_H = 8;

  /**
   * Construct the driver. No I2C traffic until begin().
   *
   * @param sh1106   false for an SSD1306 (most 0.96" modules), true for
   *                 an SH1106 (most 1.3" modules). The wrong choice shows
   *                 as a picture shifted 2 pixels with noise at one edge.
   * @param address  7-bit I2C address: 0x3C on nearly every module, 0x3D
   *                 if its address jumper has been moved. (Some modules
   *                 print the 8-bit form, 0x78 or 0x7A, on the back.)
   *
   * Example:
   *   Oled oled;        // SSD1306 at 0x3C
   */
  explicit Oled(bool sh1106 = false, uint8_t address = 0x3C)
      : sh1106_(sh1106), address_(address), dirty_(0) {
    memset(buf_, 0, sizeof(buf_));
  }

  /**
   * Start I2C on PB8/PB9 at 400 kHz, set the display up and clear it.
   * Called in the first 100 ms after power-up, it first waits out the
   * rest of them so the panel has finished resetting.
   *
   * @return true if the display answered, false if nothing is at the
   *         address (check wiring, power and the address). Safe to call
   *         with no display attached: it returns false, it doesn't hang.
   *
   * Example:
   *   if (!oled.begin()) Serial.println("No OLED");
   */
  bool begin() {
    // Step 0: give the panel time to wake. The board is running its
    // code within a few milliseconds of power-up, but the module's own
    // reset circuit holds the controller in reset a little longer, and
    // commands sent then are lost: the screen stays dark until the next
    // reset (which is why it works after an upload, when the display
    // stays powered, but not after plugging in). 100 ms from boot is
    // plenty; called later, this waits for nothing.
    if (millis() < 100)
      delay(100 - millis());

    // Same pins as MPR121Input, so the two can share the bus.
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.begin();
    Wire.setClock(400000); // "fast mode", which both chips support

    // Step 1: is anyone there? An empty transmission is just the
    // address byte; endTransmission() returns 0 if a device acknowledged.
    Wire.beginTransmission(address_);
    if (Wire.endTransmission() != 0)
      return false;

    // Step 2: the setup commands. These are the values from the
    // controllers' datasheets for a 128x64 panel, the same ones
    // Adafruit's and U8g2's drivers send. They work on both chips.
    // Each command sits beside its value, so formatting is off here.
    // clang-format off
    static const uint8_t init[] = {
        0xAE,       // display off while we configure it
        0xD5, 0x80, // clock divider: the recommended default
        0xA8, 0x3F, // multiplex ratio: 64 rows (0x3F = 63)
        0xD3, 0x00, // no vertical offset
        0x40,       // start at RAM row 0
        0x8D, 0x14, // SSD1306: charge pump on. The SH1106 runs its
                    // own pump by default and reads these two bytes
                    // as column-address commands, which display()
                    // overwrites, so they do no harm.
        0x20, 0x02, // SSD1306: page addressing mode. The SH1106 only
                    // has this mode; to it these are harmless too.
        0xA1,       // flip horizontally and...
        0xC8,       // ...vertically, so (0,0) is top-left with the
                    // header pins at the top
        0xDA, 0x12, // COM pin layout for 64-row panels
        0x81, 0xCF, // contrast (brightness), 0..255
        0xD9, 0xF1, // pre-charge period, for the internal pump
        0xDB, 0x40, // VCOMH deselect level
        0xA4,       // show RAM contents (0xA5 would light every pixel)
        0xA6,       // normal, not inverted (0xA7 inverts)
    };
    // clang-format on
    command(init, sizeof(init));

    // Step 3: wipe whatever the panel's RAM held at power-up, then
    // turn the panel on. clear() marks every page dirty.
    clear();
    display();
    static const uint8_t on[] = {0xAF};
    command(on, 1);
    return true;
  }

  /**
   * Blank the frame buffer. Takes effect at the next display().
   *
   * Example:
   *   oled.clear();   // start each frame from black
   */
  void clear() {
    memset(buf_, 0, sizeof(buf_));
    dirty_ = 0xFF; // all 8 pages changed
  }

  /**
   * Set or clear one pixel. Off-screen coordinates are ignored.
   *
   * @param x   Column, 0 (left) to 127.
   * @param y   Row, 0 (top) to 63.
   * @param on  true lights the pixel, false darkens it.
   *
   * Example:
   *   oled.pixel(64, 32);   // dot in the middle
   */
  void pixel(int x, int y, bool on = true) {
    // One unsigned comparison checks both "< 0" and ">= WIDTH": a
    // negative int converted to unsigned becomes a huge number.
    if ((unsigned)x >= WIDTH || (unsigned)y >= HEIGHT)
      return;
    uint8_t bit = 1 << (y & 7); // y % 8: which bit in the column byte
    if (on)
      buf_[y >> 3][x] |= bit; // y / 8: which page
    else
      buf_[y >> 3][x] &= ~bit;
    dirty_ |= 1 << (y >> 3);
  }

  /**
   * Draw a straight line between two points, ends included.
   *
   * @param x0, y0  Start point.
   * @param x1, y1  End point. Either end may be off-screen; the visible
   *                part is drawn.
   * @param on      true draws lit pixels, false dark ones.
   *
   * Example:
   *   oled.line(0, 63, 127, 0);   // diagonal, bottom-left to top-right
   */
  void line(int x0, int y0, int x1, int y1, bool on = true) {
    // Bresenham's algorithm: step one pixel at a time along the line,
    // keeping a running error term that says when the other coordinate
    // should also step. Integer-only, so no float maths per pixel.
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
      pixel(x0, y0, on);
      if (x0 == x1 && y0 == y1)
        break;
      int e2 = 2 * err;
      if (e2 >= dy) { // time to step in x
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) { // time to step in y
        err += dx;
        y0 += sy;
      }
    }
  }

  /**
   * Draw a rectangle, outlined or filled.
   *
   * @param x, y  Top-left corner.
   * @param w, h  Width and height in pixels. Zero or negative draws
   *              nothing.
   * @param fill  false (default) draws a 1-pixel outline; true fills it.
   * @param on    true draws lit pixels, false dark ones. A filled dark
   *              rectangle erases an area.
   *
   * Example:
   *   oled.rect(0, 56, 128, 8);                 // empty bar
   *   oled.rect(0, 56, (int)(level * 128), 8, true);  // filled part
   *   oled.rect(0, 0, 128, 8, true, false);     // erase the top line
   */
  void rect(int x, int y, int w, int h, bool fill = false, bool on = true) {
    if (w <= 0 || h <= 0)
      return;
    if (fill) {
      for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
          pixel(i, j, on);
      return;
    }
    int x1 = x + w - 1, y1 = y + h - 1;
    line(x, y, x1, y, on);   // top
    line(x, y1, x1, y1, on); // bottom
    line(x, y, x, y1, on);   // left
    line(x1, y, x1, y1, on); // right
  }

  /**
   * Draw a string in the built-in 5 x 7 font.
   *
   * Each character fills a 6 x 8 cell, background included, so new text
   * fully replaces old text drawn at the same place: no need to erase
   * a changing number first. 21 characters fit across, 8 lines down.
   * Characters outside printable ASCII show as '?'.
   *
   * @param x, y  Top-left of the first character. Any y works; y = 0,
   *              8, 16 ... line up with the display's pages and are the
   *              cheapest to send.
   * @param s     The text (a null-terminated C string).
   * @param on    true for lit text on black, false for black text on a
   *              lit cell (a highlight).
   * @return      The x just after the last character, to continue on
   *              the same line.
   *
   * Example:
   *   int x = oled.text(0, 0, "Freq ");
   *   oled.number(x, 0, hz, 1);      // "Freq 440.0"
   *   oled.text(0, 8, "MENU", false); // inverted
   */
  int text(int x, int y, const char *s, bool on = true) {
    for (; *s; s++) {
      char c = *s;
      if (c < ' ' || c > '~')
        c = '?';
      const uint8_t *glyph = OLED_FONT[c - ' '];
      // Column by column; column 5 is the blank gap (0), and bit 7 of
      // every column is the blank row under the glyph.
      for (int col = 0; col < CHAR_W; col++) {
        uint8_t bits = col < 5 ? glyph[col] : 0;
        for (int row = 0; row < CHAR_H; row++)
          pixel(x + col, y + row, ((bits >> row) & 1) ? on : !on);
      }
      x += CHAR_W;
    }
    return x;
  }

  /**
   * Draw a number with a fixed count of decimal places.
   *
   * Formats without printf: on this toolchain printf's %f support is
   * left out to save flash, so snprintf("%f") prints nothing.
   *
   * @param x, y      Top-left, as text().
   * @param value     The number. Beyond about ±2 billion / 10^decimals
   *                  it is clipped.
   * @param decimals  Digits after the point, 0 to 4.
   * @param on        As text().
   * @return          The x just after the last character.
   *
   * Example:
   *   oled.number(0, 0, 3.14159f, 2);   // "3.14"
   *   oled.number(0, 8, -12.0f, 0);     // "-12"
   */
  int number(int x, int y, float value, int decimals = 2, bool on = true) {
    // 1. Scale to a whole number and round: 3.14159 with 2 decimals
    //    becomes 314. Work on the magnitude and add the sign later.
    static const uint32_t scales[] = {1, 10, 100, 1000, 10000};
    decimals = decimals < 0 ? 0 : (decimals > 4 ? 4 : decimals);
    uint32_t scale = scales[decimals];
    bool negative = value < 0.0f;
    float mag = (negative ? -value : value) * scale + 0.5f;
    uint32_t n = mag < 4.0e9f ? (uint32_t)mag : 4000000000u;
    bool showSign = negative && n > 0; // -0.001 rounds to "0.00"

    // 2. Write digits backwards from the end of a buffer: n % 10 is the
    //    last digit, n / 10 drops it. Put the point in after `decimals`
    //    digits, and keep going until the whole part is written.
    char buf[16];
    char *p = buf + sizeof(buf) - 1;
    *p = '\0';
    int written = 0;
    do {
      if (decimals > 0 && written == decimals)
        *--p = '.';
      *--p = '0' + n % 10;
      n /= 10;
      written++;
    } while (n > 0 || written <= decimals);
    if (showSign)
      *--p = '-';
    return text(x, y, p, on);
  }

  /**
   * Send every page drawn on since the last display() to the screen.
   *
   * Blocks while I2C runs: about 3.4 ms per changed page, 27 ms for a
   * full screen. Call it from loop() only, at 10 to 20 frames a second.
   *
   * Example:
   *   if (millis() - last >= 50) {   // 20 frames a second
   *     last = millis();
   *     drawScreen();
   *     oled.display();
   *   }
   */
  void display() {
    for (int page = 0; page < PAGES; page++) {
      if (!(dirty_ & (1 << page)))
        continue;
      // Point the display at the start of this page: the page number,
      // then the column split into two 4-bit commands. The SH1106 has
      // 132 columns of RAM for a 128-pixel panel, centred, so its
      // first visible column is 2.
      uint8_t col = sh1106_ ? 2 : 0;
      uint8_t cmds[] = {(uint8_t)(0xB0 | page),         // page address
                        (uint8_t)(0x00 | (col & 0x0F)), // column, low 4
                        (uint8_t)(0x10 | (col >> 4))};  // column, high 4
      command(cmds, sizeof(cmds));

      // Send the 128 bytes in chunks. Each I2C write starts with a
      // control byte: 0x40 says "the rest is pixel data". 16 bytes per
      // chunk stays well inside Wire's 32-byte buffer.
      for (int x = 0; x < WIDTH; x += 16) {
        Wire.beginTransmission(address_);
        Wire.write(0x40);
        Wire.write(&buf_[page][x], 16);
        Wire.endTransmission();
      }
    }
    dirty_ = 0;
  }

private:
  /** Send command bytes. The control byte 0x00 means "commands follow". */
  void command(const uint8_t *cmds, size_t count) {
    Wire.beginTransmission(address_);
    Wire.write(0x00);
    Wire.write(cmds, count);
    Wire.endTransmission();
  }

  bool sh1106_;
  uint8_t address_;
  uint8_t dirty_;             // bit n set = page n changed
  uint8_t buf_[PAGES][WIDTH]; // the frame buffer, 1 KB
};

} // namespace bb
