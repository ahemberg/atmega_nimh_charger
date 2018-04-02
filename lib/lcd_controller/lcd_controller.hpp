#ifndef LCD_CONTROLLER_H
#define LCD_CONTROLLER_H

#include <Arduino.h>
#include <LiquidCrystal.h>

struct lcd_pins {
  int reset;
  int enable;
  int d_pins[4];
};


class LcdController {
public:
  bool p = true;
  char page = 0;
  LiquidCrystal* lcd;

  LcdController(LiquidCrystal* _lcd) :
  lcd(_lcd) {};
  LcdController(LiquidCrystal* _lcd, int width, int height) :
  lcd(_lcd) {lcd->begin(width, height);};

  void page_a(float curr, float ch_v, float b_v);
  void page_b(float b_t, float a_t);
  void page_c(int year, int month, int day, int hour, int minute);
  void increment_page();
  void decrement_page();


};

#endif
