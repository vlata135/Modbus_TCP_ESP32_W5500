#include <Arduino.h>
#include <config.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MAX_SPEED_M1 60u

#define MAX_SPEED_M2 80u

#define MAX_SPEED_M3 70u


/*******************************************************************************
 * Variables
 ******************************************************************************/


volatile int posi_M1 = 0;
volatile int posi_M2 = 0;
volatile int posi_M3 = 0;

long g_prevT = 0;
float g_prev_error_M1 = 0;
float g_prev_error_M2 = 0;
float g_prev_error_M3 = 0;

float g_kp_M1 = 8;
float g_ki_M1 = 0;
float g_kd_M1 = 0.025;

float g_kp_M2 = 12;
float g_ki_M2 = 0;
float g_kd_M2 = 0.0001;

float g_kp_M3 = 8;
float g_ki_M3 = 0;
float g_kd_M3 = 0.0001;



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void readEncoder_M1();
void readEncoder_M2();
void readEncoder_M3();

/*******************************************************************************
 * Code
 ******************************************************************************/

void setup ()
{
  //set pins to output
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
  attachInterrupt(M3_Enc_A, readEncoder_M3, RISING);
  attachInterrupt(M2_Enc_A, readEncoder_M2, RISING);
  attachInterrupt(M1_Enc_A, readEncoder_M1, RISING);

  Serial.begin(115200);
}

void loop()
{
    uint8_t u8status = 0b00000000;


    // target M1
    int target_M1 = 9000;

    //time diff
    long curT = micros();
    float dT = ((float)(curT - g_prevT)) / (1.0e6);

    // Read the position
    int pos_M1 = 0;
    int pos_M2 = 0;
    int pos_M3 = 0;   
    noInterrupts(); // disable interrupts temporarily while reading
    pos_M1 = posi_M1;
    pos_M2 = posi_M2;
    pos_M3 = posi_M3; 
    interrupts(); // turn interrupts back on

    // PID calculation
    
    int error_M1 = pos_M1 - target_M1;
    int error_M2 = pos_M2 - (-pos_M1); //target of M1 is M2 cur_pos, in this case is pos_M2
    int error_M3 = pos_M3 - (pos_M1);

    double der_M1 = (error_M1 - g_prev_error_M1)/(dT);
    double der_M2 = (error_M2 - g_prev_error_M2)/(dT);
    double der_M3 = (error_M3 - g_prev_error_M3)/(dT);

    float u_M1 = g_kp_M1*error_M1 + g_kd_M1*der_M1;
    float u_M2 = g_kp_M2*error_M2 + g_kd_M2*der_M2;
    float u_M3 = g_kp_M3*error_M3 + g_kd_M2*der_M3;

    double pwm_M1 = fabs(u_M1);
    double pwm_M2 = fabs(u_M2);
    double pwm_M3 = fabs(u_M3);


    // stop increasing velocity when reach the saturation point
    
    if(pwm_M1 > MAX_SPEED_M1)
    {
      pwm_M1 = MAX_SPEED_M1;
    }
    if(pwm_M2 > MAX_SPEED_M2)
    {
      pwm_M2 = MAX_SPEED_M2;
    }
    if(pwm_M3 > MAX_SPEED_M3)
    {
      pwm_M3 = MAX_SPEED_M3;
    }
    
    //end stop increasing velocity when reach the saturation point

    // if(u_M2 > 0)
    // {
    //   digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    //   shiftOut(DS,SHcp,MSBFIRST,M2_FW);
    //   digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    //   analogWrite(M2_PWM, pwm_M2);
    // }
    // else
    // {
    //   digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    //   shiftOut(DS,SHcp,MSBFIRST,M2_BW);
    //   digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    //   analogWrite(M2_PWM, pwm_M2);
    // }

    

    if(u_M1 < 0)
    {
      u8status |= M1_FW;
    }
    else
    {
      u8status |= M1_BW;
    }
    
    if(u_M2 < 0)
    {
      u8status |= M2_FW;
    }
    else
    {
      u8status |= M2_BW;
    }

    if(u_M3 < 0)
    {
      u8status |= M3_FW;
    }
    else
    {
      u8status |= M3_BW;
    }
    

    digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
    shiftOut(DS,SHcp,MSBFIRST,u8status);
    digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
    analogWrite(M1_PWM, pwm_M1);
    analogWrite(M2_PWM, pwm_M2);
    analogWrite(M3_PWM, pwm_M3);

    

    Serial.print("Pos_M1:");
    Serial.print(pos_M1);
    Serial.print(" ");
    Serial.print("Pos_M2:");
    Serial.print(pos_M2);
    // Serial.print(" ");
    // Serial.print("Pos_M3:");
    // Serial.print(pos_M3);
    // Serial.print(" ");
    // Serial.print("u_M2:");
    // Serial.print(error_M2);
    // Serial.print("Setpoint:");
    // Serial.print(target);
    // Serial.print(" ");
    // Serial.print("Current_Position:");
    // Serial.print(pos);
    Serial.print(" ");
    Serial.print("error_M2:");
    Serial.print(error_M2);
    Serial.print(" ");
    Serial.print("error_M3:");
    Serial.print(error_M3);
    Serial.print(" ");
    Serial.print("pwm_M1:");
    Serial.print(pwm_M1);
    Serial.print(" ");
    Serial.print("pwm_M2:");
    Serial.print(pwm_M2);
    Serial.print(" ");
    Serial.print("pwm_M3:");
    Serial.print(pwm_M3);
    Serial.print(" ");
    Serial.print("Min:");
    Serial.print(4);
    Serial.print(" ");
    Serial.print("Max:");
    Serial.print(20);
    Serial.print(" ");
    Serial.print("Max:");
    Serial.print(-10);
    Serial.print(" ");
    Serial.println();


    //update prev_error
    g_prev_error_M2 = error_M2;
    g_prev_error_M1 = error_M1;
    g_prevT = curT;


}


void readEncoder_M3()
{
  int b_M3 = digitalRead(M3_Enc_B);
  if(b_M3 > 0)
  {
    posi_M3++;
  }
  else
  {
    posi_M3--;
  }
}

void readEncoder_M2()
{
  int b_M2 = digitalRead(M2_Enc_B);
  if(b_M2 > 0)
  {
    posi_M2++;
  }
  else
  {
    posi_M2--;
  }
}

void readEncoder_M1()
{
  int b_M1 = digitalRead(M1_Enc_B);
  if(b_M1 > 0)
  {
    posi_M1++;
  }
  else
  {
    posi_M1--;
  }
}