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

float ChargeController::measure_charge_current(int num_avgs, int bv_pin, int sv_pin, float shunt_r) {
  float cv, sv;
  for (int i = 0; i < num_avgs; i++) {
    cv += read_voltage_drop(bv_pin);
    sv += read_voltage_drop(sv_pin);
  }

  cv /= num_avgs;
  sv /= num_avgs;

  return calc_current(sv-cv, shunt_r);
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

void ChargeController::control_loop() {

  batt_temp = batt_thermistor->read_ntc_temp();
  amb_temp = amb_thermistor->read_ntc_temp();

  read_battery_voltage();
  if (batt_measuring) return;

  charge_voltage = read_voltage_drop(cv_pin);
  current = measure_charge_current(
    10, bv_pin, cv_pin, 2.595 // TODO: Fix this -> should be generic
  );

  charger_duty += pid->simplePid(charge_current, current);

  if (charger_duty > 255) charger_duty = 255;
  if (charger_duty < 0) charger_duty = 0;

  analogWrite(charge_pin, charger_duty);
}
