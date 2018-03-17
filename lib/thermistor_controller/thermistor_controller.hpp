#ifndef TEMP_CONTROL_H
#define TEMP_CONTTROL_H

#include <Arduino.h>

struct ntc_info
{
   int pin;
   float A;
   float B;
   float C;
   float r1;
};

class ThermistorController {
  public:
    ntc_info ntc_param;
    ThermistorController(int pin, float A, float B, float C, float r1)
    : ntc_param({pin, A, B, C, r1}) {};

    float temp_from_r(float r_ntc);
    float ntc_resistance(float vout);
    float kelvin_to_c(float kelvin);
    float read_ntc_temp();

};

#endif
