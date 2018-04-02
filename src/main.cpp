#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <pid_regulator.hpp>
#include <lcd_controller.hpp>
#include <thermistor_controller.hpp>
#include <charge_controller.h>
#include <RTClib.h>
#include <pin_definitions.h>


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

struct charge_values {
    int charger_duty = 0;
    float charge_voltage, batt_voltage, set_voltage;
    float current = 0, charge_current = 0;
    float batt_temp = 0, amb_temp = 0;
    unsigned long last_batt_measurement = 0, measurement_start = 0;
    bool batt_measuring = false;
    float d_bv[100], d_te[100];
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

charge_values charge_param;
PidRegulator spid = PidRegulator(KP, KI, KD);

float output;

unsigned long loop_top, last_lcd_update, ll;

float read_voltage(int pin) {
  int ad_val = analogRead(pin);
  return ad_val * (5.0/1023.0);
}

float read_voltage_drop(int pin) {
  return 5.0 - read_voltage(pin);
}

void read_battery_voltage(charge_values *cv) {
  if (cv->batt_measuring) {
    if (millis() - cv->measurement_start < 1000) return;

    cv->batt_voltage = read_voltage_drop(BATT_VOLTAGE_PIN);
    analogWrite(CHARGE_PIN, cv->charger_duty);
    cv->batt_measuring = false;
    cv->last_batt_measurement = millis();
  } else {
    if (millis() - cv->last_batt_measurement > 60000) {
      cv->batt_measuring = true;
      cv->measurement_start = millis();
      analogWrite(CHARGE_PIN, 0);
    }
  }
}

float calc_current(float voltage_drop, float r_shunt) {
  return 1000.0*(voltage_drop/r_shunt);
}
// TODO Implement this in charge controller
float measure_charge_current(int num_avgs, int bv_pin, int sv_pin, float shunt_r) {
  float cv, sv;
  for (int i = 0; i < num_avgs; i++) {
    cv += read_voltage_drop(bv_pin);
    sv += read_voltage_drop(sv_pin);
  }

  cv /= AVGS;
  sv /= AVGS;

  return calc_current(sv-cv, shunt_r);
}

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

    analogWrite(CHARGE_PIN, charge_param.charger_duty);

    charge_param.last_batt_measurement = 60000;
    charge_param.set_voltage = 1.5;
    charge_param.charge_current = 50;
    last_lcd_update = millis();
}

void loop() {
    DateTime now = rtc.now();
    loop_top = millis();
    charge_param.batt_temp = batt_thermistor.read_ntc_temp();
    charge_param.amb_temp = amb_thermistor.read_ntc_temp();

    if (charge_param.batt_temp > 45) {
      charge_param.charge_current = 0.0;
    }

    read_battery_voltage(&charge_param);

    if (!charge_param.batt_measuring) {
      charge_param.charge_voltage = read_voltage_drop(CHARGE_PIN);
      charge_param.current = measure_charge_current(
        AVGS, BATT_VOLTAGE_PIN, SHUNT_VOLTAGE_PIN, SHUNT_R
      );

      output = spid.simplePid(charge_param.charge_current, charge_param.current);
      charge_param.charger_duty += output;

      if (charge_param.charger_duty > 255) charge_param.charger_duty = 255;
      if (charge_param.charger_duty < 0) charge_param.charger_duty = 0;

      analogWrite(CHARGE_PIN, charge_param.charger_duty);
    }

    //Plotter
    Serial.print(charge_param.current);
    Serial.print(",");
    Serial.print(charge_param.charge_current);
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
        lcc.page_a(charge_param.current, charge_param.charge_voltage, charge_param.batt_voltage);
        break;
        case 1:
        lcc.page_b(charge_param.batt_temp, charge_param.amb_temp);
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

    if (button_4.pushed) {
      charge_param.charge_current += 10;
    }

    if (button_1.pushed) {
      charge_param.charge_current -= 10;
    }

    //Clear button pushes
    button_1.pushed = false;
    button_4.pushed = false;

}
