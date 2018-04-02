#include "charge_controller.h"

float ChargeController::read_voltage(int pin) {
  int ad_val = analogRead(pin);
  return ad_val * (5.0/1023.0);
}

float ChargeController::read_voltage_drop(int pin) {
  return 5.0 - read_voltage(pin);
}

float ChargeController::calc_current(float voltage_drop, float r_shunt) {
  return 1000.0*(voltage_drop/r_shunt);
}

void ChargeController::read_battery_voltage() {
  if (batt_measuring) {
    if (millis() - measurement_start < 1000) return;

    batt_voltage = read_voltage_drop(bv_pin);
    analogWrite(charge_pin, charger_duty);
    batt_measuring = false;
    last_batt_measurement = millis();
  } else {
    if (millis() - last_batt_measurement > 60000) {
      batt_measuring = true;
      measurement_start = millis();
      analogWrite(charge_pin, 0);
    }
  }
}
