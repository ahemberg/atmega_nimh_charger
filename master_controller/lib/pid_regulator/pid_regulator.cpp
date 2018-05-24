#include "pid_regulator.hpp"

float PidRegulator::simplePid(float setpoint, float input) {
    unsigned long now = millis();
    float time_change = (float)(now - last_time);
    float error = setpoint - input;
    err_sum += (error * time_change);

    float d_input = input - last_input;

    last_input = input;
    last_time = now;

    float output = kp * error + ki * err_sum - kd * d_input;

    if (output > 0) {
        output = ceil(output);
    } else {
        output = floor(output);
    }
    return output;
}
