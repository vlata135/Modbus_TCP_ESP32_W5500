/******************************************************************************
* Includes
******************************************************************************/

#include <Arduino.h>
#include "config.h"


/******************************************************************************
* Definitions
******************************************************************************/
#define DELTA_TIME 60000

/******************************************************************************
* Variales
******************************************************************************/
volatile int posi = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float vel = 0;
float prevTime = 0;
float g_i = 0;
uint32_t prevPos = 0;

/******************************************************************************
* Prototype
******************************************************************************/
void M1_foward(float pwm);
void M1_backward(float pwm);
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
    // end for DC Motor Control
    Serial.begin(115200);
}
/******************************************************************************
* Loop
******************************************************************************/
void loop()
{
   
  float curTime = micros();
  float y = 70 * (sin(((2*3.14)/10000000)*curTime) > 0);
  uint32_t delta = 0;
  // float vel = 0;
  // M1_foward(100);


  // uint32_t delta = posi - prevPos;

  // prevPos = posi;
  
  // prevTime = curTime;
  // noInterrupts();
  // float vel = velocity_i;
  // interrupts();
  // Serial.println(vel);
  
  // Serial.println(vt);
  if(curTime - prevTime >= DELTA_TIME)
  {
      // noInterrupts();
      // uint32_t curPos = posi;
      // interrupts();
      delta = posi - prevPos;

      vel = (delta*1000000*60) / (DELTA_TIME*440);
      // Serial.println(vel);

      prevPos = posi;
      prevTime = curTime;
  }

  float e = y - vel;
  g_i = g_i + (e*1000000)/DELTA_TIME;
  float u = 2*e;
  uint32_t sig = abs(u);

  if(sig > 160)
  {
    sig = 160;
  }

  if(u > 0)
  {
    M1_foward(sig);
  }
  // else
  // {
  //   M1_backward(sig);
  // }


  
  Serial.print("y:");
  Serial.print(y);
  Serial.print(" ");
  Serial.print("vel:");
  Serial.print(vel);
  Serial.println(" ");
  Serial.print("min:");
  Serial.print(150);
  Serial.print(" ");
  Serial.print("max:");
  Serial.println(0);

}

/******************************************************************************
* Code
******************************************************************************/
void M1_foward(float pwm)
{
    digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_ST_M1_FW);
    digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    analogWrite(M1_PWM, pwm);
}

void M1_backward(float pwm)
{
    digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_ST_M1_BW);
    digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    analogWrite(M1_PWM, pwm);
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(M1_Enc_B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  posi = posi + increment;

  // Compute velocity with method 2
  // long currT = micros();
  // float deltaT = (currT - prevT_i);
  // velocity_i = deltaT;
  // prevT_i = currT;
}