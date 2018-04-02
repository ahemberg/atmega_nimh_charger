
#ifndef LCD_CONTROLLER_H
#define LCD_CONTROLLER_H

#include <Arduino.h>
#include <thermistor_controller.hpp>
#include <pid_regulator.hpp>

struct charge_values {
    int charger_duty = 0;
    float charge_voltage, batt_voltage, set_voltage;
    float current = 0, charge_current = 0;
    float batt_temp = 0, amb_temp = 0;
    unsigned long last_batt_measurement = 0, measurement_start = 0;
    bool batt_measuring = false;
    float d_bv[100], d_te[100];
};

class ChargeController {
public:
  int charger_duty = 0, charge_pin, bv_pin, cv_pin;
  float charge_voltage, batt_voltage, set_voltage;
  float current = 0, charge_current = 0;
  float batt_temp = 0, amb_temp = 0;
  unsigned long last_batt_measurement = 60000, measurement_start = 0;
  bool batt_measuring = false;
  float d_bv[100], d_te[100];

  ThermistorController *amb_thermistor;
  ThermistorController *batt_thermistor;

  ChargeController(
    int _charge_pin,
    int _bv_pin,
    int _cv_pin,
    ThermistorController *amb_th,
    ThermistorController *bat_th
  ) :
  charge_pin(_charge_pin), bv_pin(_bv_pin), cv_pin(_cv_pin), amb_thermistor(amb_th), batt_thermistor(bat_th) {};

  void read_battery_voltage();

protected:
private:
  float read_voltage(int pin);
  float read_voltage_drop(int pin);
  float calc_current(float voltage_drop, float r_shunt);
};

#endif
