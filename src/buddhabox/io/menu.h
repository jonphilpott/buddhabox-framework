/**
 * @file menu.h
 * @brief A scrolling settings menu on an Oled, driven by an encoder.
 *
 * Most small instruments end up with the same menu: a list of named
 * values, a highlighted row, and an encoder to change things. This is
 * that menu, so a project only has to list its settings.
 *
 * HOW IT IS PLAYED:
 *   turn    move the highlight up and down the list
 *   press   start editing the highlighted item: its value shows as
 *           <value>, and turning now changes it; press again to finish
 *   press on an action item (such as "Randomize"): do it at once
 *
 * HOW THE CODE FITS TOGETHER: Menu does no reading of pins itself. Your
 * loop() reads the encoder and the switch and hands the result to
 * update(), which says what happened. That keeps Menu usable with any
 * input: an EncoderInput, two buttons, MIDI, or a test.
 *
 * Every value is a whole number with a range and a step. Integers keep
 * the maths simple (the encoder adds or subtracts steps, so nothing
 * drifts); your code turns them into Hz, seconds or whatever it needs.
 *
 * Example usage:
 *   enum { CUTOFF, WAVE, RANDOMIZE, kItems };   // item numbers
 *   const char *const waves[] = {"Saw", "Square"};
 *   MenuItem items[kItems] = {
 *       {"Cutoff", 50, 0, 100, 2},             // name, value, min, max, step
 *       {"Wave", 0, 0, 1, 1, waves},           // shown as Saw / Square
 *       {"Randomize"},                         // step 0: an action
 *   };
 *   Menu menu(items, kItems);
 *
 *   void loop() {
 *       push.update();
 *       Menu::Event e = menu.update(encoder.delta(), push.triggered());
 *       if (e == Menu::CHANGED)
 *           applySettings();       // reads items[CUTOFF].value etc.
 *       if (e == Menu::ACTION && menu.selected() == RANDOMIZE)
 *           randomize();
 *       menu.draw(oled, 16, 6);    // six rows from y = 16
 *       oled.display();
 *   }
 */

#pragma once

#include "buddhabox/io/oled.h"

namespace bb {

/**
 * One line of a Menu. Fill it in with braces; fields left out are zero,
 * which is what makes `{"Randomize"}` an action.
 *
 * - name:   shown on the left
 * - value:  the current setting; read it in your code
 * - min, max, step: the range, and how much one encoder click changes it
 * - labels: optional words to show instead of the number, one per value
 *           from min to max (labels[value - min])
 * - suffix: optional text after the number, such as " Hz" or "/16"
 *
 * An item with step 0 is an action: pressing on it makes update()
 * return Menu::ACTION instead of starting an edit, and no value shows.
 *
 * Example:
 *   MenuItem tempo = {"Tempo", 120, 60, 200, 1, nullptr, " bpm"};
 */
struct MenuItem {
  const char *name;
  int value, min, max, step;
  const char *const *labels;
  const char *suffix;
};

/**
 * A scrolling list of MenuItems, edited with an encoder and drawn on an
 * Oled. Holds a pointer to your array, so your code reads values
 * straight from it.
 *
 * Example:
 *   MenuItem items[] = {{"Level", 70, 0, 100, 2}, {"Reset"}};
 *   Menu menu(items, 2);
 */
class Menu {
public:
  /** What update() reports. */
  enum Event {
    NONE,    ///< nothing to act on (or only the highlight moved)
    CHANGED, ///< the edited item's value changed
    ACTION   ///< an action item was pressed
  };

  /**
   * @param items  Your array of items. Menu keeps the pointer, so the
   *               array must outlive it (a global is simplest).
   * @param count  How many items are in it
   *
   * Example:
   *   Menu menu(items, sizeof(items) / sizeof(items[0]));
   */
  Menu(MenuItem *items, int count) : items_(items), count_(count) {}

  /**
   * Apply one round of input. Call it every loop().
   *
   * @param turn     Encoder clicks since last time (EncoderInput::delta()):
   *                 positive moves down the list or raises the value
   * @param pressed  true on the loop() the switch was pressed
   *                 (SwitchInput::triggered())
   * @return  CHANGED if a value changed, ACTION if an action item was
   *          pressed (selected() says which item), otherwise NONE
   *
   * Example:
   *   if (menu.update(encoder.delta(), push.triggered()) == Menu::CHANGED)
   *     applySettings();
   */
  Event update(int turn, bool pressed) {
    Event event = NONE;

    // 1. Turning: change the value being edited, or move the highlight.
    if (turn != 0) {
      if (editing_) {
        MenuItem &m = items_[selected_];
        int v = clampInt(m.value + turn * m.step, m.min, m.max);
        if (v != m.value) {
          m.value = v;
          event = CHANGED;
        }
      } else {
        selected_ = clampInt(selected_ + turn, 0, count_ - 1);
      }
    }

    // 2. Pressing: an action fires; anything else enters or leaves
    //    editing.
    if (pressed) {
      if (items_[selected_].step == 0)
        event = ACTION;
      else
        editing_ = !editing_;
    }
    return event;
  }

  /** @return  The highlighted item's index */
  int selected() const { return selected_; }

  /** @return  true while the highlighted item is being edited */
  bool editing() const { return editing_; }

  /**
   * Draw the menu: one row of 8 pixels per item, the highlighted row
   * inverted, scrolled so the highlight is always in view. Call
   * oled.display() afterwards, as for any drawing.
   *
   * @param oled    The display
   * @param y       Top of the first row, in pixels
   * @param rows    How many rows to draw; (64 - y) / 8 fills the rest
   *                of the screen
   * @param valueX  Where values start, in pixels from the left; names
   *                longer than about valueX / 6 characters run into them
   *
   * Example:
   *   menu.draw(oled, 16, 6); // below a 16-pixel header
   */
  void draw(Oled &oled, int y, int rows, int valueX = 80) const {
    // Scroll only as far as needed to keep the highlight on the last
    // visible row.
    int first = selected_ - rows + 1;
    first = first < 0 ? 0 : first;
    for (int r = 0; r < rows; r++) {
      int item = first + r;
      int top = y + r * 8;
      bool on = item != selected_; // the highlight is drawn inverted
      oled.rect(0, top, 128, 8, true, !on);
      if (item >= count_)
        continue;
      const MenuItem &m = items_[item];
      oled.text(2, top, m.name, on);
      if (m.step == 0)
        continue; // an action: no value

      bool edit = editing_ && item == selected_;
      int x = edit ? oled.text(valueX - 6, top, "<", on) : valueX;
      if (m.labels)
        x = oled.text(x, top, m.labels[m.value - m.min], on);
      else
        x = oled.number(x, top, (float)m.value, 0, on);
      if (m.suffix)
        x = oled.text(x, top, m.suffix, on);
      if (edit)
        oled.text(x, top, ">", on);
    }
  }

private:
  static int clampInt(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  MenuItem *items_;
  int count_;
  int selected_ = 0;
  bool editing_ = false;
};

} // namespace bb
