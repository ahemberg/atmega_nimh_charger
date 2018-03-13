#include <Arduino.h>

class PidRegulator {
  public:
    float last_time, last_input, kp, ki, kd, err_sum;
    PidRegulator (float k_p, float k_i, float k_d) : kp(k_p), ki(k_i), kd(k_d) {};
    float simplePid(float setpoint, float input);
};
