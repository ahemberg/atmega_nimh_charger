#include "pid_regulator.h"

float PidRegulator::simplePid(float setpoint, float input) {
    unsigned long now = millis();
    float time_change = (float)(now - this->last_time);
    float error = setpoint - input;
    this->err_sum += (error * time_change);

    float d_input = input - this->last_input;

    this->last_input = input;
    this->last_time = now;

    return this->kp * error + this->ki * err_sum - this->kd * d_input;
}
