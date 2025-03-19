/******************************************************************************
* This code use for control direction of motor using serial command:
* k j a d w x
******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/

#include <Arduino.h>
#include "config.h"


/******************************************************************************
* Definitions
******************************************************************************/
#define DELTA_TIME 15000

/******************************************************************************
* Variales
******************************************************************************/
volatile int posi = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float vel = 0;
float prevTime = 0;
float g_i = 0;
float prevPos = 0;


float v1Filt = 0;
float v1Prev = 0;

char buff = 0;
/******************************************************************************
* Prototype
******************************************************************************/
void M1_foward(float pwm);
void M1_backward(float pwm);
void M1_brake(float pwm);

void readEncoder();

/******************************************************************************
* Setup
******************************************************************************/
void setup()
{
    // for DC Motor Control
    pinMode(DS,OUTPUT);
    pinMode(STcp,OUTPUT);
    pinMode(SHcp,OUTPUT);
    pinMode(M3_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M3_Enc_A, INPUT);
    pinMode(M3_Enc_B, INPUT);
    pinMode(M2_Enc_A, INPUT);
    pinMode(M2_Enc_B, INPUT);
    pinMode(M1_Enc_A, INPUT);
    pinMode(M1_Enc_B, INPUT);
    attachInterrupt(M1_Enc_A, readEncoder, RISING);
    attachInterrupt(M2_Enc_B, readEncoder, RISING);

    // end for DC Motor Control
    Serial.begin(115200);
}
/******************************************************************************
* Loop
******************************************************************************/
void loop()
{
  if(Serial.available())
  {
    buff = Serial.read();
    if(buff == 'j')
    {
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_BW_M2_FW_M1_BW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 50);
      analogWrite(M2_PWM, 50);
      analogWrite(M3_PWM, 50);
      delay(100);
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_FW_M2_BW_M1_FW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 0);
      analogWrite(M2_PWM, 0);
      analogWrite(M3_PWM, 0);
    }
    if(buff == 'k')
    {
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_FW_M2_BW_M1_FW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 50);
      analogWrite(M2_PWM, 50);
      analogWrite(M3_PWM, 50);
      delay(100);
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_FW_M2_BW_M1_FW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 0);
      analogWrite(M2_PWM, 0);
      analogWrite(M3_PWM, 0);
    }
    if(buff == 'w')
    {
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_FW_M1_FW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 50);
      analogWrite(M2_PWM, 50);
      analogWrite(M3_PWM, 0);
      delay(300);
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_FW_M2_BW_M1_FW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 0);
      analogWrite(M2_PWM, 0);
      analogWrite(M3_PWM, 0);
    }
    if(buff == 'x')
    {
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_BW_M1_BW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 50);
      analogWrite(M2_PWM, 50);
      analogWrite(M3_PWM, 0);
      delay(300);
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_FW_M2_BW_M1_FW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 0);
      analogWrite(M2_PWM, 0);
      analogWrite(M3_PWM, 0);
    }
  }
    

  

}

/******************************************************************************
* Code
******************************************************************************/
void M1_foward(float pwm)
{
    digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_FW_M1_ST);
    digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    analogWrite(M2_PWM, pwm);
}

void M1_backward(float pwm)
{
    digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_BW_M1_ST);
    digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    analogWrite(M2_PWM, pwm);
}
void M1_brake(float pwm)
{
    digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    shiftOut(DS,SHcp,MSBFIRST,B00001100);
    digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    analogWrite(M2_PWM, 0);
}
void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(M2_Enc_B);
  // int increment = 0;
  if(b>0){
    // If B is high, increment forward
    posi++;
    // increment = 1;
  }
  else{
    // Otherwise, increment backward
    posi--;
  }
  // posi = posi + increment;

  // Compute velocity with method 2
  // long currT = micros();
  // float deltaT = (currT - prevT_i);
  // velocity_i = deltaT;
  // prevT_i = currT;
}