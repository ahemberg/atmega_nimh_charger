#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <pid_regulator.hpp>
#include <lcd_controller.hpp>
#include <Thermistor_controller.hpp>

#define R_SENSE 2.595
#define AVGS 10
#define CHARGE_PIN 9
#define BAUDRATE 9600

ThermistorController batt_thermistorX(
  A2,
  0.5522033026e-3,
  3.378163036e-4,
  -3.876640607e-7,
  10e3
);

ThermistorController amb_thermistorX(
  A3,
  0.6944098729e-3,
  3.124809880e-4,
  -2.784875147e-7,
  10e3
);
/*
struct ntc_info
{
   int pin;
   float A;
   float B;
   float C;
   float r1;
};
*/
ntc_info batt_thermistor = {
  A2,
  0.5522033026e-3,
  3.378163036e-4,
  -3.876640607e-7,
  10e3
};

ntc_info amb_thermistor = {
  A3,
  0.6944098729e-3,
  3.124809880e-4,
  -2.784875147e-7,
  10e3
};

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
LcdController lcc(&lcd);

int duty = 0;
float tot_voltage, charge_voltage, batt_voltage, current, set_voltage;
float charge_current, batt_temp, amb_temp;
float output;

unsigned long last_volt_measurement, loop_top, last_lcd_update, ll;
bool page_a;

float kp = 2.0, ki = 0*0.000005, kd = 4;
PidRegulator spid = PidRegulator(kp, ki, kd);

float temp_from_r(float resistance, float a, float b, float c) {;
  return 1 / (a + b*log(resistance) + c*pow(log(resistance),3));
}

float ntc_resistance(float vout, float r1) {
  return (vout*r1)/(5.0 - vout);
}

float kelvin_to_c(float kelvin) {
  return kelvin - 273.15;
}

float read_ntc_temp(ntc_info thermistor) {
  float v_meas = (analogRead(thermistor.pin)*5.0)/1023.0;
  float r_ntc = ntc_resistance(v_meas, thermistor.r1);
  return kelvin_to_c(
    temp_from_r(r_ntc, thermistor.A, thermistor.B, thermistor.C)
  );
}

void setup() {
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);

    pinMode(CHARGE_PIN, OUTPUT);

    Serial.begin(BAUDRATE);

    lcd.begin(16, 2);

    analogWrite(9, duty);
    batt_voltage = analogRead(A1);
    batt_voltage = 5.0 - (batt_voltage*5.0)/(1023.0);

    set_voltage = 1.5;
    charge_current=250;
    last_volt_measurement = millis();
    last_lcd_update = millis();
}

void loop() {

    loop_top = millis();
    batt_temp = read_ntc_temp(batt_thermistor);
    amb_temp = read_ntc_temp(amb_thermistor);

    if (batt_temp > 45) {
      charge_current = 0.0;
    }

    //Measure battery voltage once per minute
    if (loop_top - last_volt_measurement > 60500) {
        //Turn off and measure battery voltage
        analogWrite(CHARGE_PIN, 0);
        //Let cap die down
        delay(1000);
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
    Serial.print(current);
    Serial.print(",");
    Serial.print(charge_current);
    Serial.print(",");
    //Serial.print(duty);
    //Serial.print(",");
    //Serial.print(output);
    //Serial.print(",");
    //Serial.print(batt_voltage);
    //Serial.print(",");
    //Serial.print(charge_voltage);
    //Serial.print(",");
    //Serial.print(batt_temp);
    //Serial.print(",");
    //Serial.print(amb_temp);
    Serial.println();

    //Update display
    if (loop_top - last_lcd_update > 10000) {
      lcd.noDisplay();
      lcd.clear();
      if (page_a) {
        lcc.page_a(current, charge_voltage, batt_voltage);
        /*
        lcd.setCursor(0,0);
        lcd.print("Ch:");
        lcd.print(current);
        lcd.print("mA");
        lcd.setCursor(0,1);
        lcd.print("Cv:");
        lcd.print(charge_voltage);
        lcd.setCursor(9,1);
        lcd.print("Bv:");
        lcd.print(batt_voltage);
        */
      } else {
        lcc.page_b(batt_temp, amb_temp);
        /*
        lcd.setCursor(0,0);
        lcd.print("Batt t:");
        lcd.print(batt_temp);
        lcd.print("C");
        lcd.setCursor(0,1);
        lcd.print("Amb t:");
        lcd.print(amb_temp);
        lcd.print("C");
        */
      }

      page_a = !page_a;

      lcd.display();
      last_lcd_update = loop_top;
    }
}
