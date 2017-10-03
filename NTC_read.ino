

#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <avr/interrupt.h>

#define INDICATOR_PIN 13

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
/*
The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 8
 * LCD R/W pin to ground
*/

static double Vref = 5.14;  //laptop 5.12;
double ntcTemp,ad590Temp,set_value;
int pwmPin = 9;
static volatile double time_read_rising;
static volatile double time_read_falling;


double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

void setup() {
 Serial.begin(115200);
 lcd.begin(16,2);
 lcd.clear();

 myPID.SetMode(AUTOMATIC);
 Setpoint = 15;

 
 digitalWrite(INDICATOR_PIN, LOW);


}

void loop() {
  
  ntcTemp = Read_NTC();

  Serial.print("NTC");
  Serial.print("\t");
  Serial.print(ntcTemp);
  Serial.print("\t");

  ad590Temp = Read_AD590(Vref);
  
  //Serial.print("AD590");
  //Serial.print("\t");
  //Serial.println(ad590Temp);

  set_value = Read_input();
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SET Temp");
  lcd.setCursor(10,0);
  lcd.print(set_value);
  lcd.setCursor(0,1);
  lcd.print("NTC");
  lcd.setCursor(7,1);
  lcd.print(ntcTemp);

  Input = ad590Temp;
  myPID.Compute();
 // Serial.println(Output);
  analogWrite(pwmPin, Output); 
}
//************************************************
void ISR_Rising(void){
  cli();
  time_read_rising = micros();
  attachInterrupt(digitalPinToInterrupt(2), ISR_Falling, FALLING);
  sei();
  }
//************************************************
void ISR_Falling(){
  cli();
  time_read_falling = micros();
  attachInterrupt(digitalPinToInterrupt(2), ISR_Rising, RISING);
  sei();
  }
//************************************************
double Read_NTC(void){
  double Rt,ntcTemp,time_rising_prev,t_h,t_l;
  int i;
  boolean done;

  delay(5000);  // NTC settling time
  sei();
  
  for (i = 0;i<5;i++){

    time_rising_prev = micros();
    attachInterrupt(digitalPinToInterrupt(2),ISR_Rising , RISING);
    done = true;
     
    while(done){
      if (time_read_rising> time_rising_prev){
        time_rising_prev = time_read_rising;
      }     
      
      if (time_read_falling > time_rising_prev) {
        if (time_read_rising > time_read_falling){
          done = false;          
          }
        }   
      }
      
       t_h = time_read_falling - time_rising_prev;
       t_l = time_read_rising - time_read_falling;
       Rt = Rt + ((t_h/t_l)-1)*9940;
  }
  cli();
  
  Rt = Rt/i;   
      Serial.print("Res");
      Serial.print("\t");
      Serial.println(Rt); 
  
 

  Rt = (Rt - 11860.0)/3464.0;   //Normalization
  ntcTemp = -0.5751*Rt*Rt*Rt + 2.164*Rt*Rt - 10.32*Rt + 20.49;   //Calibration temp in celsius
  return ntcTemp;
  }
  
//**************************************************
double Read_AD590(double Vref){
  double value,value1,vol,ad590Temp; 
  int i;

  value1 =0;
  for (i = 0;i<5; i++){
    value = analogRead(A2);
    value1 = value + value1;
    }
  
  value = value1/i;

  vol = value*(Vref/1023.0);
  ad590Temp = ((vol/(10012.9))*1000000.0)-273.15;   //temp in celsius
  return ad590Temp;
}
//****************************************************
double Read_input(){
  double value,input,out;
  value = analogRead(A3);
  input = ((45.0-5.0)/1023.0)*value + 5.0;
  out = Round_to_five(input);
  return out;
  }
 //***************************************************
 double Round_to_five(double value){
  double value_Nodec,deff,out;
  value_Nodec = floor(value);
  deff = value - value_Nodec;
  if (deff>=.25){
    if (deff>=.75){
      out = value_Nodec + 1;
      }else{
        out = value_Nodec + 0.5;
        }
    
    }else{
      out = value_Nodec;
      }

  return out;
  } 
 //****************************************************
 
