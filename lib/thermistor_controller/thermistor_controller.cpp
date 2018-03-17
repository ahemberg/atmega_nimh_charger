#include "thermistor_controller.hpp"

float ThermistorController::temp_from_r(float r_ntc) {;
  return 1 / (ntc_param.A + ntc_param.B*log(r_ntc) + ntc_param.C*pow(log(r_ntc),3));
}

float ThermistorController::ntc_resistance(float vout) {
  return (vout*ntc_param.r1)/(5.0 - vout);
}

float ThermistorController::kelvin_to_c(float kelvin) {
  return kelvin - 273.15;
}

float ThermistorController::read_ntc_temp() {
  float v_meas = (analogRead(ntc_param.pin)*5.0)/1023.0;
  float r_ntc = ntc_resistance(v_meas);
  return kelvin_to_c(
    temp_from_r(r_ntc)
  );
}
