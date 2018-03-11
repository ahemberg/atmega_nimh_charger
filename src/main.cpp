#include <Arduino.h>
#include <math.h>

#define R_SENSE 2.65
#define AVGS 10
#define CHARGE_PIN 9

float temp;
int adval;
int duty = 0;
float tot_voltage, charge_voltage, batt_voltage, current, set_voltage;
float charge_current, curr_avg, max_curr;

void setup() {
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(9, OUTPUT);
    analogWrite(9, 0);
    batt_voltage = analogRead(A0);
    batt_voltage = (batt_voltage*5.0)/(1023.0);
    Serial.begin(9600);
    set_voltage = 1.5;
    charge_current=230;
    max_curr=300;
}

// TODO : GITHUB!! AND READ UP ON RC FILTER!!!

unsigned long last_time;
float err_sum, last_input;
float kp = 4, ki = 0*0.05, kd = 2;
float output;
float duty_per_amp;

float simplePid(float setpoint, float input) {
    unsigned long now = millis();
    float time_change = (float)(now - last_time);
    float error = setpoint - input;
    err_sum += (error * time_change);
    //if (err_sum > 1000) err_sum = 1000;
    //if (err_sum < -1000) err_sum = -1000;

    float d_input = input - last_input;

    last_input = input;
    last_time = now;

    return kp * error + ki * err_sum - kd * d_input;
}

unsigned long last_volt_measurement = millis();
unsigned long loop_top;

void loop() {

    loop_top = millis();

    if (loop_top - last_volt_measurement > 10000) {
        charge_current = random(10, 800);
        last_volt_measurement = loop_top;
    }

    /*
    //Measure battery voltage
    if (loop_top - last_volt_measurement > 10000) {
        //Turn off and measure battery voltage
        analogWrite(9, 0);
        //Let cap die down
        delay(1);
        batt_voltage = analogRead(A0);
        analogWrite(9, duty);
        last_volt_measurement = loop_top;
        batt_voltage = (batt_voltage*5.0)/(1023.0);
    }
    */
    for (int i = 0; i < AVGS; i++) {
        charge_voltage += analogRead(A0);
        tot_voltage += analogRead(A1);
    }

    charge_voltage = (charge_voltage*5.0)/(1023.0*AVGS);
    tot_voltage = (tot_voltage*5.0)/(1023.0*AVGS);
    current = round(((tot_voltage-charge_voltage)/R_SENSE)*1000);

    output = simplePid(charge_current, current);

    if (output > 0) {
        output = ceil(output);
    } else {
        output = floor(output);
    }

    duty += output;

    if (duty > 255) duty = 255;
    if (duty < 0) duty = 0;
    analogWrite(9, duty);
    // Terminal view
    /*
    Serial.print("Batt V: ");
    Serial.print(batt_voltage);
    Serial.print(". A1 V: ");
    Serial.print(tot_voltage);
    Serial.print(". Curr mA: ");
    Serial.print(current);
    Serial.print(". avg mA: ");
    Serial.print(curr_avg);
    Serial.print(". Duty: ");
    Serial.print(duty);
    Serial.println();
    */
    //Plotter
    Serial.print(current);
    Serial.print(",");
    Serial.print(charge_current);
    //Serial.print(",");
    //Serial.print(duty);
    //Serial.print(",");
    //Serial.print(output);
    //Serial.print(",");
    //Serial.print(batt_voltage*100);
    //Serial.print(",");
    //Serial.print(charge_voltage*100);
    Serial.println();


}
