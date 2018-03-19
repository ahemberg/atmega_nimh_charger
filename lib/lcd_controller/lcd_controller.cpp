
#include "lcd_controller.hpp"

void LcdController::page_a(float curr, float ch_v, float b_v) {
  lcd->clear();
  lcd->setCursor(0,0);
  lcd->print("Ch:");
  lcd->print(curr);
  lcd->print("mA");
  lcd->setCursor(0,1);
  lcd->print("Cv:");
  lcd->print(ch_v);
  lcd->setCursor(9,1);
  lcd->print("Bv:");
  lcd->print(b_v);
}

void LcdController::page_b(float b_t, float a_t) {
  lcd->clear();
  lcd->setCursor(0,0);
  lcd->print("Batt t:");
  lcd->print(b_t);
  lcd->print("C");
  lcd->setCursor(0,1);
  lcd->print("Amb t:");
  lcd->print(a_t);
  lcd->print("C");
}

void LcdController::page_c(int year, int month, int day, int hour, int minute) {
  lcd->clear();
  lcd->setCursor(0,0);
  lcd->print(year);
  lcd->print("-");
  lcd->print(month);
  lcd->print("-");
  lcd->print(day);
  lcd->print(" ");
  lcd->print(hour);
  lcd->print(" ");
  lcd->print(minute);
}

void LcdController::increment_page() {
  page++;
  if (page > 2) {
    page = 0;
  }
}
