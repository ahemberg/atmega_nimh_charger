#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
public:
  Button(int _pin) : pin(_pin) {};
  void read_button();
  bool get_state();
  void reset_state();
protected:
private:
  int pin;
  bool state = false, last_state = false, pushed = false;
  unsigned long last_debounce = 0, debounce_delay = 70;
};

#endif
