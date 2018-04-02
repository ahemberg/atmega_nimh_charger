#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <pid_regulator.hpp>
#include <lcd_controller.hpp>
#include <Thermistor_controller.hpp>
#include <RTClib.h>

struct button {
  int pin;
  bool state;
  bool last_state;
  bool pushed;
  unsigned long last_debounce;
};

#define SHUNT_VOLTAGE_PIN A0
#define BATT_VOLTAGE_PIN A1
#define BATT_THERMISTOR_PIN A2
#define AMB_THERMISTOR_PIN A3
#define CHARGE_PIN 9

#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2
#define LCD_COLUMNS 16
#define LCD_ROWS 2

#define BTN_1 6
#define BTN_2 7
#define BTN_3 8
#define BTN_4 10

#define SHUNT_R 2.595
#define AVGS 10
#define BAUDRATE 9600

#define KP 2.0
#define KI 0.0 //0.000005
#define KD 4.0


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

int duty = 0;
float tot_voltage, charge_voltage, batt_voltage, current, set_voltage;
float charge_current, batt_temp, amb_temp;
float output;
float d_bv[100], d_te[100];

bool foo = false;

unsigned long last_volt_measurement, loop_top, last_lcd_update, ll;

PidRegulator spid = PidRegulator(KP, KI, KD);

float read_voltage(int pin) {
  int ad_val = analogRead(pin);
  return ad_val * (5.0/1023.0);
}

float read_voltage_drop(int pin) {
  return 5.0 - read_voltage(pin);
}

float read_battery_voltage(int cap_diedown_ms, int batt_voltage_pin, int charge_pin, int curr_duty) {
  analogWrite(charge_pin, 0);
  delay(cap_diedown_ms);
  float bv = read_voltage_drop(batt_voltage_pin);
  analogWrite(charge_pin, curr_duty);
  return bv;
}

float calc_current(float voltage_drop, float r_shunt) {
  return 1000.0*(voltage_drop/r_shunt);
}

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
    if (btn->state == HIGH) {
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

    pinMode(BTN_1, INPUT);
    pinMode(BTN_2, INPUT);
    pinMode(BTN_3, INPUT);
    pinMode(BTN_4, INPUT);

    pinMode(CHARGE_PIN, OUTPUT);

    Serial.begin(BAUDRATE);
    rtc.begin();

    analogWrite(9, duty);

    batt_voltage = read_voltage_drop(BATT_VOLTAGE_PIN);

    set_voltage = 1.5;
    charge_current=200;
    last_volt_measurement = millis();
    last_lcd_update = millis();
}

void loop() {
    DateTime now = rtc.now();
    loop_top = millis();
    batt_temp = batt_thermistor.read_ntc_temp();
    amb_temp = amb_thermistor.read_ntc_temp();

    if (batt_temp > 45) {
      charge_current = 0.0;
    }

    //Measure battery voltage once per minute
    if (loop_top - last_volt_measurement > 60500) {
        batt_voltage = read_battery_voltage(
          1000, BATT_VOLTAGE_PIN, CHARGE_PIN, duty
        );
        last_volt_measurement = millis();
    }

    current = measure_charge_current(
      AVGS, BATT_VOLTAGE_PIN, SHUNT_VOLTAGE_PIN, SHUNT_R
    );

    output = spid.simplePid(charge_current, current);
    duty += output;

    if (duty > 255) duty = 255;
    if (duty < 0) duty = 0;

    analogWrite(CHARGE_PIN, duty);

    //Plotter
    //Serial.print(current);
    //Serial.print(",");
    //Serial.print(charge_current);
    //Serial.print(",");
    //Serial.println();

    read_button(&button_1);
    read_button(&button_2);
    read_button(&button_3);
    read_button(&button_4);

    Serial.print(button_1.pushed);
    Serial.print(",");
    Serial.print(button_2.pushed);
    Serial.print(",");
    Serial.print(button_3.pushed);
    Serial.print(",");
    Serial.print(button_4.pushed);
    Serial.println();

    button_1.pushed = false;
    button_2.pushed = false;
    button_3.pushed = false;
    button_4.pushed = false;


    //Update display
    if (loop_top - last_lcd_update > 10000) {
      lcd.noDisplay();
      switch (lcc.page) {
        case 0:
        lcc.page_a(current, charge_voltage, batt_voltage);
        break;
        case 1:
        lcc.page_b(batt_temp, amb_temp);
        break;
        case 2:
        lcc.page_c(now.year(), now.month(), now.day(), now.hour(), now.minute());
        break;
      }
      lcc.increment_page();

      lcd.display();
      last_lcd_update = loop_top;
    }
}
