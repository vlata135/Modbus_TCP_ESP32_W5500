/******************************************************************************
* Includes
******************************************************************************/

#include <Arduino.h>


/******************************************************************************
* Definitions
******************************************************************************/
#define DELTA_TIME 30000
#define M2_Enc_A 16
#define M2_Enc_B 17
#define M2_PWM 17
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
void readEncoder();





/******************************************************************************
* Setup
******************************************************************************/
void setup()
{
    // for DC Motor Control
    
    pinMode(M2_PWM, OUTPUT);
    
    pinMode(M2_Enc_A, INPUT);
    pinMode(M2_Enc_B, INPUT);
    attachInterrupt(M2_Enc_B, readEncoder, RISING);

    // end for DC Motor Control
    Serial.begin(115200);
}
/******************************************************************************
* Loop
******************************************************************************/
void loop()
{
   
  float curTime = micros();
//   float y = 255 * (sin(((2*3.14)/10000000)*curTime));
//   if( y > 0)
//   {
//     M1_foward(y);
//   }
//   else
//   {
//     M1_backward(-y);
//   }
  // float y = 100;
  float delta = 0;
  // float vel = 0;
  // M1_foward(100);


  // uint32_t delta = posi - prevPos;

  // prevPos = posi;
  // gsdfgsdf
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
      delta = abs(posi - prevPos);

      vel = (delta*1000000*60) / (DELTA_TIME*440);
      v1Filt = 0.969*v1Filt + 0.0155*vel + 0.0155*v1Prev;
      v1Prev = vel;

    //   Serial.print("y:");
    //   Serial.print(y);
    //   Serial.print(" ");
      Serial.print("v1Filt:");
      Serial.print(v1Filt);
      Serial.print(" ");
      Serial.print("vel:");
      Serial.println(vel);
      // Serial.println(curTime - prevTime);


      prevPos = posi;
      prevTime = curTime;
  }

  

  float e = 244 - vel;
  g_i = g_i + (e*1000000)/DELTA_TIME;
  float u = 20*e + 14 * g_i;
  uint32_t sig = (uint32_t) fabs(u);

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
// void M1_foward(float pwm)
// {
//     digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
//     shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_FW_M1_ST);
//     digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
//     analogWrite(M2_PWM, pwm);
// }

// void M1_backward(float pwm)
// {
//     digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
//     shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_BW_M1_ST);
//     digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
//     analogWrite(M2_PWM, pwm);
// }

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(M2_Enc_B);
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