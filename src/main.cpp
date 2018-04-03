#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <RTClib.h>
#include <pin_definitions.h>
#include <LiquidCrystal.h>
#include <lcd_controller.hpp>
#include <pid_regulator.hpp>
#include <thermistor_controller.hpp>
#include <charge_controller.h>

#define SHUNT_R 2.595
#define AVGS 10
#define BAUDRATE 9600

#define KP 2.0
#define KI 0.0 //0.000005
#define KD 4.0

struct button {
  int pin;
  bool state;
  bool last_state;
  bool pushed;
  unsigned long last_debounce;
};

RTC_DS3231 rtc;

ThermistorController batt_thermistor(
  BATT_THERMISTOR_PIN,
  0.5522033026e-3,
  3.378163036e-4,
  -3.876640607e-7,
  10e3
);

ThermistorController amb_thermistor(
  AMB_THERMISTOR_PIN,
  0.6944098729e-3,
  3.124809880e-4,
  -2.784875147e-7,
  10e3
);

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
LcdController lcc(&lcd, LCD_COLUMNS, LCD_ROWS);

button button_1 = {BTN_1, false, false, false, 0};
button button_2 = {BTN_2, false, false, false, 0};
button button_3 = {BTN_3, false, false, false, 0};
button button_4 = {BTN_4, false, false, false, 0};

//charge_values charge_param;
PidRegulator spid = PidRegulator(KP, KI, KD);

ChargeController cc = ChargeController(
  CHARGE_PIN,
  BATT_VOLTAGE_PIN,
  SHUNT_VOLTAGE_PIN,
  &spid,
  &amb_thermistor,
  &batt_thermistor
);

unsigned long loop_top, last_lcd_update;

void read_button(button* btn) {
  unsigned long debounce_delay = 70;
  btn->state = digitalRead(btn->pin);
  if (btn->state != btn->last_state) {
    btn->last_debounce = millis();
  }

  if ((millis() - btn->last_debounce) > debounce_delay) {
    if (btn->state == LOW) {
      btn->pushed = true;
    }
    btn->last_debounce = millis();
  }
  btn->last_state = btn->state;
}

void setup() {
    pinMode(BATT_VOLTAGE_PIN, INPUT);
    pinMode(SHUNT_VOLTAGE_PIN, INPUT);
    pinMode(BATT_THERMISTOR_PIN, INPUT);
    pinMode(AMB_THERMISTOR_PIN, INPUT);

    pinMode(BTN_1, INPUT_PULLUP);
    pinMode(BTN_2, INPUT_PULLUP);
    pinMode(BTN_3, INPUT_PULLUP);
    pinMode(BTN_4, INPUT_PULLUP);

    pinMode(CHARGE_PIN, OUTPUT);

    Serial.begin(BAUDRATE);
    rtc.begin();

    cc.charge_current = 50;

    last_lcd_update = millis();
}

void loop() {
    DateTime now = rtc.now();
    loop_top = millis();

    cc.control_loop();

    //Plotter
    Serial.print(cc.current);
    Serial.print(",");
    Serial.print(cc.charge_current);
    Serial.print(",");
    Serial.println();

    read_button(&button_1);
    read_button(&button_2);
    read_button(&button_3);
    read_button(&button_4);

    //Update display
    if (loop_top - last_lcd_update > 1000) {
      lcd.noDisplay();
      switch (lcc.page) {
        case 0:
        lcc.page_a(cc.current, cc.charge_voltage, cc.batt_voltage);
        break;
        case 1:
        lcc.page_b(cc.batt_temp, cc.amb_temp);
        break;
        case 2:
        lcc.page_c(now.year(), now.month(), now.day(), now.hour(), now.minute());
        break;
      }
      lcd.display();
      last_lcd_update = loop_top;
    }

    if (button_2.pushed) {
      lcc.increment_page();
      button_2.pushed = false;
    }

    if (button_3.pushed) {
      lcc.decrement_page();
      button_3.pushed = false;
    }

    if (button_4.pushed) cc.charge_current += 10;
    if (button_1.pushed) cc.charge_current -= 10;

    //Clear button pushes
    button_1.pushed = false;
    button_4.pushed = false;

}
