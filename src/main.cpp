#include <Arduino.h>
#include <math.h>
#include <pid_regulator.h>

#define R_SENSE 2.595
#define AVGS 10
#define CHARGE_PIN 9

int duty = 0;
float tot_voltage, charge_voltage, batt_voltage, current, set_voltage;
float charge_current;
float output;

unsigned long last_volt_measurement, loop_top;

float kp = 2.0, ki = 0.000005, kd = 4;
PidRegulator spid = PidRegulator(kp, ki, kd);

void setup() {
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(CHARGE_PIN, OUTPUT);
    analogWrite(9, duty);
    batt_voltage = analogRead(A1);
    batt_voltage = 5.0 - (batt_voltage*5.0)/(1023.0);
    Serial.begin(9600);
    set_voltage = 1.5;
    charge_current=200;
    last_volt_measurement = millis();
}

void loop() {

    loop_top = millis();

    //Measure battery voltage
    if (loop_top - last_volt_measurement > 20000) {
        //Turn off and measure battery voltage
        analogWrite(CHARGE_PIN, 0);
        //Let cap die down
        delay(500);
        batt_voltage = analogRead(A1);
        analogWrite(CHARGE_PIN, duty);
        last_volt_measurement = millis();
        batt_voltage = 5.0 - (batt_voltage*5.0)/(1023.0);
    }

    for (int i = 0; i < AVGS; i++) {
        charge_voltage += analogRead(A1);
        tot_voltage += analogRead(A0);
    }

    charge_voltage = 5.0 - (charge_voltage*5.0)/(1023.0*AVGS);
    tot_voltage = 5.0 - (tot_voltage*5.0)/(1023.0*AVGS);
    current = round(((tot_voltage-charge_voltage)/R_SENSE)*1000);

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
    //Serial.print(current);
    //Serial.print(",");
    //Serial.print(charge_current);
    //Serial.print(",");
    //Serial.print(duty);
    //Serial.print(",");
    //Serial.print(output);
    //Serial.print(",");
    Serial.print(batt_voltage);
    Serial.print(",");
    Serial.print(charge_voltage);
    Serial.println();


}
