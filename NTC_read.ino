

/************************************************
NTC_Read.ino
Temperature Controller Software for TEC device
ELEN30013
Electronic System Implementation, Semester 2 2017

Workshop 8am Thursday, Group 7
Authors:
Prasadh
Michael
*************************************************/

#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <avr/interrupt.h>

#define INDICATOR_PIN 13
#define HEATING_PIN 9
#define COOLING_PIN 10

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
/*
  The circuit:
   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 8
   LCD R/W pin to ground
*/

static double Vref = 5.09;  //laptop 5.12;
double ntcTemp, ad590Temp;
int pwmPin;
static volatile double time_read_rising;
static volatile double time_read_falling;


double Setpoint, Input, Output;
double aggKp=3, aggKi=5, aggKd=2;
double consKp=313.0 , consKi=11.1 , consKd=1339.0;
PID myPID(&Input, &Output, &Setpoint, consKp,consKi,consKd, DIRECT);
/* 60, 2.5, 352.5  68,.4,922, */

void setup() {
  Serial.begin(115200);

  lcd.begin(16, 2);
  lcd.clear();

  myPID.SetMode(AUTOMATIC);
  pwmPin = HEATING_PIN;

  digitalWrite(INDICATOR_PIN, LOW);

}

void loop() {

  ntcTemp = Read_NTC();

 // Serial.print("NTC");
 // Serial.print("\t");
  Serial.print(ntcTemp);

 /*Serial.print("\t");
  Serial.print("Time");*/
 // Serial.print("\t");
 // Serial.println(millis()); 

  ad590Temp = Read_AD590(Vref);

/*  Serial.print("AD590");
  Serial.print("\t");
  Serial.println(ad590Temp);*/


 // Serial.print("SET");
    Serial.print("\t"); 
    Serial.println(Setpoint);
 // 

  Setpoint = Read_input();


  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SET Temp");
  lcd.setCursor(10, 0);
  lcd.print(Setpoint);
  lcd.setCursor(0, 1);
  lcd.print("NTC");
  lcd.setCursor(7, 1);
  lcd.print(ntcTemp);

  Input = ntcTemp;

  if(Setpoint < 7.0 || Setpoint > 42.0) {
     myPID.SetTunings(aggKp, aggKi, aggKd);
  } else {
     myPID.SetTunings(consKp, consKi, consKd);
  }
  
  if (Input <= Setpoint) {
    myPID.SetControllerDirection(DIRECT);
    analogWrite(COOLING_PIN, 0);
    pwmPin = HEATING_PIN;
  } else {
    myPID.SetControllerDirection(REVERSE);
    analogWrite(HEATING_PIN, 0);
    pwmPin = COOLING_PIN;
  }
  myPID.Compute();
  if(Output > 250.0) {
    Output = 250.0;
  }
  analogWrite(pwmPin, Output);
}
//************************************************
void ISR_Rising(void) {
  cli();
  time_read_rising = micros();
  attachInterrupt(digitalPinToInterrupt(2), ISR_Falling, FALLING);
  sei();
}
//************************************************
void ISR_Falling() {
  cli();
  time_read_falling = micros();
  attachInterrupt(digitalPinToInterrupt(2), ISR_Rising, RISING);
  sei();
}
//************************************************
double Read_NTC(void) {
  double Rt, ntcTemp, time_rising_prev, t_h, t_l;
  int i;
  boolean done;

  delay(5000);  // NTC settling time
  sei();

  for (i = 0; i < 5; i++) {

    time_rising_prev = micros();
    attachInterrupt(digitalPinToInterrupt(2), ISR_Rising , RISING);
    done = true;

    while (done) {
      if (time_read_rising > time_rising_prev) {
        time_rising_prev = time_read_rising;
      }

      if (time_read_falling > time_rising_prev) {
        if (time_read_rising > time_read_falling) {
          done = false;
        }
      }
    }

    t_h = time_read_falling - time_rising_prev;
    t_l = time_read_rising - time_read_falling;
    Rt = Rt + ((t_h / t_l) - 1) * 9905;
 
  }
  cli();

  Rt = Rt / i;
  //  Serial.print("Res");
  //  Serial.print("\t");
  //  Serial.print(Rt);
  //  Serial.print("\t");



  Rt = (Rt - 8940.0) / 2326.0; //Normalization
  ntcTemp = -0.255 * Rt * Rt * Rt + 2.107 * Rt * Rt - 11.99 * Rt + 23.08; //Calibration temp in celsius
  return ntcTemp;
}

//**************************************************
double Read_AD590(double Vref) {
  double value, value1, vol, ad590Temp;
  int i;

  value1 = 0;
  for (i = 0; i < 5; i++) {
    value = analogRead(A2);
    value1 = value + value1;
  }

  value = value1 / i;

  vol = value * (Vref / 1023.0);
  ad590Temp = ((vol / (10012.9)) * 1000000.0) - 273.15; //temp in celsius
  return ad590Temp;
}
//****************************************************
double Read_input() {
  double value, input, out;
  value = analogRead(A3);
  input = ((45.0 - 5.0) / 1023.0) * value + 5.0;
  out = Round_to_five(input);
  return out;
}
//***************************************************
double Round_to_five(double value) {
  double value_Nodec, deff, out;
  value_Nodec = floor(value);
  deff = value - value_Nodec;
  if (deff >= .25) {
    if (deff >= .75) {
      out = value_Nodec + 1;
    } else {
      out = value_Nodec + 0.5;
    }

  } else {
    out = value_Nodec;
  }

  return out;
}
//****************************************************
