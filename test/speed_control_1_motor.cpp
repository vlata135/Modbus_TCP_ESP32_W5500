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
    attachInterrupt(M3_Enc_A, readEncoder, RISING);

    // end for DC Motor Control
    Serial.begin(115200);
}
/******************************************************************************
* Loop
******************************************************************************/
void loop()
{
   
  float curTime = micros();
  float y = 255 * (sin(((2*3.14)/10000000)*curTime));
  float delta = 0;
  float e = 0;
  
  if(curTime - prevTime >= DELTA_TIME)
  {
      // noInterrupts();
      // uint32_t curPos = posi;
      // interrupts();
      delta = abs(posi - prevPos);

      vel = (delta*1000000*60) / (DELTA_TIME*440);
      v1Filt = 0.8104139*v1Filt + 0.09479305 *vel + 0.09479305 *v1Prev;
      v1Prev = vel;
      
      // start calculate PID 
      float e = y - v1Filt;
      g_i = g_i + (e*1000000)/DELTA_TIME;

      float u = 8.7*e + 0.01*g_i;
      float sig = 0;
      sig = abs(u);
      // end calculate PID

      // start action
      if(sig > 255)
      {
        sig = 255;
      }

      if(y > 0)
      {
        M1_foward(y);
      }
      else
      {
        M1_backward(-y);
      }
      // else
      // {
      //   M1_foward(sig);
      // }
      // end action


      // start print log
      // Serial.print("u:");
      // Serial.print(u*1.1);
      // Serial.print(" ");
      // Serial.print("sig:");
      // Serial.print(sig);
      // Serial.print(" ");
      // Serial.print("posi:");
      // Serial.print(posi);
      // Serial.print(" ");
      // Serial.print("e:");
      // Serial.print(e);
      // Serial.print(" ");
      // Serial.print("sp:");
      // Serial.print(65);
      // Serial.print(" ");
      // Serial.print("y:");
      Serial.print(y);
      Serial.print(" ");
      // Serial.print("v1Filt:");
      Serial.print(v1Filt);
      Serial.print(" ");
      // Serial.print("vel:");
      Serial.println(vel);
      // end print log


      prevPos = posi;
      prevTime = curTime;
  }

  

  // float e = y - v1Filt;
  // g_i = g_i + (e*1000000)/DELTA_TIME;
  // float u = 20*e + 14 * g_i;
  // uint32_t sig = (uint32_t) fabs(u);

  // if(sig > 160)
  // {
  //   sig = 160;
  // }

  // if(u > 0)
  // {
  //   M1_foward(sig);
  // }
  // else
  // {
  //   M1_backward(sig);
  // }


  // Serial.print("y:");
  // Serial.print(y);
  // Serial.print(" ");
  // Serial.print("e:");
  // Serial.print(e);
  // Serial.print(" ");
  // Serial.print("v1Filt:");
  // Serial.println(v1Filt);
  // Serial.print(" ");
  // Serial.print("min:");
  // Serial.print(150);
  // Serial.print(" ");
  // Serial.print("max:");
  // Serial.println(0);

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
void M1_brake(float pwm)
{
    digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    shiftOut(DS,SHcp,MSBFIRST,B00001100);
    digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    analogWrite(M2_PWM, 0);
}
void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(M1_Enc_B);
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