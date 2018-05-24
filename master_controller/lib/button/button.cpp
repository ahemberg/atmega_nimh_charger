#include "button.h"

void Button::read_button() {
  state = digitalRead(pin);
  if (state != last_state) {
    last_debounce = millis();
  }

  if ((millis() - last_debounce) > debounce_delay) {
    if (state == LOW) {
      pushed = true;
    }
    last_debounce = millis();
  }
  last_state = state;
}

bool Button::get_state() {
  return pushed;
};

void Button::reset_state() {
  pushed = false;
}
