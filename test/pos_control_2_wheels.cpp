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
volatile int posi_M1 = 0;
volatile int posi_M2 = 0;
volatile int posi_M3 = 0;

char buff;



/******************************************************************************
* Prototype
******************************************************************************/
void readEncoder_M1();
void readEncoder_M2();
void readEncoder_M3();

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
    attachInterrupt(M1_Enc_A, readEncoder_M1, RISING);
    attachInterrupt(M2_Enc_A, readEncoder_M2, RISING);
    attachInterrupt(M3_Enc_A, readEncoder_M3, RISING);


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
      shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_FW_M1_FW);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 20);
      analogWrite(M2_PWM, 20);
      analogWrite(M3_PWM, 20);
    }
    if(buff == 'k')
    {
      digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
      shiftOut(DS,SHcp,MSBFIRST,M3_ST_M2_ST_M1_ST);
      digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
      analogWrite(M1_PWM, 0);
      analogWrite(M2_PWM, 0);
      analogWrite(M3_PWM, 0);
    }
  }



  Serial.print("posi_M1:");
  Serial.print(posi_M1);
  Serial.print(" ");
  Serial.print("posi_M2:");
  Serial.print(posi_M2);
  Serial.print(" ");
  Serial.print("posi_M3:");
  Serial.print(posi_M3);
  Serial.println();
}

/******************************************************************************
* Code
******************************************************************************/
void readEncoder_M1()
{
  uint8_t b_M1 = digitalRead(M1_Enc_B);
  if(b_M1 > 0)
  {
    posi_M1++;
  }
  else
  {
    posi_M1--;
  }
}

void readEncoder_M2()
{
  uint8_t b_M2 = digitalRead(M2_Enc_B);
  if(b_M2 > 0)
  {
    posi_M2++;
  }
  else
  {
    posi_M2--;
  }
}

void readEncoder_M3()
{
  uint8_t b_M3 = digitalRead(M3_Enc_B);
  if(b_M3 > 0)
  {
    posi_M3++;
  }
  else
  {
    posi_M3--;
  }
}
