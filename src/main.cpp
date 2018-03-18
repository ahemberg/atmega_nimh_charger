#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <pid_regulator.hpp>
#include <lcd_controller.hpp>
#include <Thermistor_controller.hpp>

#define SHUNT_VOLTAGE_PIN A0
#define BATT_VOLTAGE_PIN A1
#define BATT_THERMISTOR_PIN A2
#define AMB_THERMISTOR_PIN A3
#define CHARGE_PIN 9

#define SHUNT_R 2.595
#define AVGS 10
#define BAUDRATE 9600

#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2
#define LCD_COLUMNS 16
#define LCD_ROWS 2

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

int duty = 0;
float tot_voltage, charge_voltage, batt_voltage, current, set_voltage;
float charge_current, batt_temp, amb_temp;
float output;

unsigned long last_volt_measurement, loop_top, last_lcd_update, ll;

float kp = 2.0, ki = 0*0.000005, kd = 4;
PidRegulator spid = PidRegulator(kp, ki, kd);

float read_voltage(int pin) {
  int ad_val = analogRead(pin);
  return ad_val * (5.0/1023.0);
}

float read_voltage_drop(int pin) {
  return 5.0 - read_voltage(pin);
}

void setup() {
    pinMode(BATT_VOLTAGE_PIN, INPUT);
    pinMode(SHUNT_VOLTAGE_PIN, INPUT);
    pinMode(BATT_THERMISTOR_PIN, INPUT);
    pinMode(AMB_THERMISTOR_PIN, INPUT);

    pinMode(CHARGE_PIN, OUTPUT);

    Serial.begin(BAUDRATE);

    analogWrite(9, duty);
    batt_voltage = analogRead(BATT_VOLTAGE_PIN);
    batt_voltage = 5.0 - (batt_voltage*5.0)/(1023.0);

    set_voltage = 1.5;
    charge_current=200;
    last_volt_measurement = millis();
    last_lcd_update = millis();
}

void loop() {

    loop_top = millis();
    batt_temp = batt_thermistor.read_ntc_temp();
    amb_temp = amb_thermistor.read_ntc_temp();

    if (batt_temp > 45) {
      charge_current = 0.0;
    }

    //Measure battery voltage once per minute
    if (loop_top - last_volt_measurement > 60500) {
        //Turn off and measure battery voltage
        analogWrite(CHARGE_PIN, 0);
        //Let cap die down
        delay(1000);
        batt_voltage = analogRead(A1);
        analogWrite(CHARGE_PIN, duty);
        last_volt_measurement = millis();
        batt_voltage = 5.0 - (batt_voltage*5.0)/(1023.0);
    }

    for (int i = 0; i < AVGS; i++) {
        charge_voltage += analogRead(BATT_VOLTAGE_PIN);
        tot_voltage += analogRead(SHUNT_VOLTAGE_PIN);
    }

    charge_voltage = 5.0 - (charge_voltage*5.0)/(1023.0*AVGS);
    tot_voltage = 5.0 - (tot_voltage*5.0)/(1023.0*AVGS);
    current = round(((tot_voltage-charge_voltage)/SHUNT_R)*1000);

    output = spid.simplePid(charge_current, current);

    if (output > 0) {
        output = ceil(output);
    } else {
        output = floor(output);
    }

    duty += output;

    if (duty > 255) duty = 255;
    if (duty < 0) duty = 0;

    analogWrite(CHARGE_PIN, duty);

    //Plotter
    Serial.print(current);
    Serial.print(",");
    Serial.print(charge_current);
    Serial.print(",");
    Serial.println();

    //Update display
    if (loop_top - last_lcd_update > 10000) {
      lcd.noDisplay();
      lcd.clear();
      if (lcc.p) {
        lcc.page_a(current, charge_voltage, batt_voltage);
      } else {
        lcc.page_b(batt_temp, amb_temp);
      }

      lcc.p = !lcc.p;

      lcd.display();
      last_lcd_update = loop_top;
    }
}
