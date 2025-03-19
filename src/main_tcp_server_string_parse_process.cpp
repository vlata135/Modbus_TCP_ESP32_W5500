/******************************************************************************
* Implementing controlMotor after module recieved a frame
******************************************************************************/




/******************************************************************************
* Includes
******************************************************************************/

#include <SPI.h>
#include <Ethernet.h>
// #include "PID_v1.h"
#include "config.h"


/******************************************************************************
* Classes
******************************************************************************/
class DCmotor
{
    public:
        DCmotor(uint8_t _STcp,uint8_t _SHcp,uint8_t _DS): mySTcp(_STcp), mySHcp{_SHcp}, myDS{_DS} {}

        // update setpoint
        void updateSP(int *_SP)
        {
            for(int i = 0; i < 3; i++)
            {
                mySP[i] = _SP[i];
            }
        }
        void updateSP_vel(int *_SP_vel)
        {
          for(int i = 0; i < 3; i++)
          {
              mySP_vel[i] = _SP_vel[i];
          }
        }
        //getEncodedState
        uint8_t getEncodeState(uint8_t *pre_state)
        {
            uint8_t encoded = 0;
            for(int i = 0; i < 3; i++)
            {
                if(pre_state[i] == 1)
                {
                    encoded |= 0x02 << ((i*2 + 1));
                }
                else
                {
                    encoded |= 0x01 << ((i*2 + 1));
                }
            }
            myEncodedState = encoded;
            Serial.println(encoded);
            Serial.println(myEncodedState);

            return encoded;
        }

        //check and change state
        uint8_t checkState(int* curPos)
        {
            for(int i = 0; i < 3; i++)
            {

                if( abs(curPos[i] - mySP[i]) <= 5 )
                {
                    uint8_t temp_EncodedState = 0x03 << ( (i*2 + 1) );
                    myEncodedState &= ~(temp_EncodedState);
                }
            }
            return myEncodedState;
            // Serial.println(myEncodedState);

        }

        void controlMotor(uint8_t vel_M1, uint8_t vel_M2,uint8_t vel_M3 )
        {
            digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
            shiftOut(DS,SHcp,MSBFIRST,myEncodedState);
            digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
            analogWrite(M1_PWM, vel_M1);
            analogWrite(M2_PWM, vel_M2);
            analogWrite(M3_PWM, vel_M3);
        }
        // calVel
        float calVel(int posi, int prevPos, float intervalTime)
        {
          float delta = abs(posi - prevPos);
          // Serial.println(delta);
          float vel = (delta*1000000*60) / (intervalTime*440);
          return vel;
        }

        //error calculate
        void calculateError(float *curVel, float *e)
        {
          for (int i = 0; i < 3; i++)
          {
            e[i] = mySP_vel[i] - curVel[i];
            // Serial.println(mySP_vel[i]);

          }
        }

        //PID compute for velocity
        void PIDcompute(int posi_M1, int posi_M2, int posi_M3)
        {
          int curPos[3] = {posi_M1, posi_M2, posi_M3}; 
          uint32_t curTime = micros();
          uint32_t intervalTime = curTime - prevTime;
          if(intervalTime >= sampleTime)
          {
            float curVel[3] = 
            {
              calVel(posi_M1, prevPos_M1, intervalTime),
              calVel(posi_M2, prevPos_M2, intervalTime),
              calVel(posi_M3, prevPos_M3, intervalTime)
            };
            float e[3];
            calculateError(curVel, e);
            float u[3];
            for(int i = 0; i < 3; i++)
            {
              g_i[i] = g_i[i] + (e[i]*1000000)/((float)intervalTime);
              u[i] = 1.2*e[i] + 0.05*g_i[i];
              u[i] = constrain(u[i], 0, 255);

              // float sig = 0;
              // sig = abs(u[i])
            }
            controlMotor((uint8_t)u[0], (uint8_t)u[1], (uint8_t)u[2]);
            Serial.print(curVel[0]);
            Serial.print(" ");
            Serial.print(curVel[1]);
            Serial.print(" ");
            Serial.print(curVel[2]);
            Serial.print(" ");
            Serial.print(posi_M1);
            Serial.print(" ");
            Serial.print(posi_M2);
            Serial.print(" ");
            Serial.print(posi_M3);
            Serial.print(" ");
            Serial.println(u[0]);



            // Serial.println(e[0]);            
            prevPos_M1 = posi_M1;
            prevPos_M2 = posi_M2;
            prevPos_M3 = posi_M3;
            prevTime = curTime;
          }

        }

        //PID for position


    private:
        uint8_t mySTcp;
        uint8_t mySHcp;
        uint8_t myDS;
        uint8_t myState[3];
        uint8_t myEncodedState;

        int prevPos_M1;
        int prevPos_M2;
        int prevPos_M3;
        float curTime;
        float prevTime;
        // float intervalTime;
        uint32_t sampleTime = 20000;
        int mySP[3];
        int mySP_vel[3];
        float g_i[3];
};
/******************************************************************************
* Definition
******************************************************************************/

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // MAC address for the Ethernet shield
IPAddress ip(192, 168, 1, 105);  // Set a static IP address for the server
EthernetServer server(505);      // Set up TCP server on port 502

/******************************************************************************
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

volatile int posi;
double Setpoint, Input, Output;
double Kp=5, Ki=0, Kd=0.08;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
DCmotor dc(STcp, SHcp, DS);

/******************************************************************************
* Prototype
******************************************************************************/
void readEncoder_M1();
void readEncoder_M2();
void readEncoder_M3();


/******************************************************************************
* Code
******************************************************************************/

String dataBuffer = "";  // Buffer to hold the incoming data

uint16_t i;

uint32_t HexToDecimal(uint8_t character)
{
    uint32_t result = 0;
    if(character >= '0' && character <= '9')
	{
		result = character - '0';
	}
	else if(character >= 'A' && character <= 'F')
	{
		result = character - 'A' + 10;
	}

    return result;
}

uint8_t checkSum(String buffer)
{
    uint16_t i;
    uint32_t sum = 0 ;
    
    for(i = 1; i < 29; i = i + 2)
    {
        sum = sum + (( HexToDecimal(buffer[i] ) << 4) | HexToDecimal(buffer[i+1]));
    }
    if( (sum & 0xFF) == 0xFF)
    {
      return 1;
    }
    else
    {
      return 0;
    }
    Serial.println(sum);
}

uint8_t parseBuffer(String buffer)
{
  uint16_t i;
  Serial.print("Module no. ");
  // Serial.print(buffer[1]);
  for(i = 1; i < 3; i++)
  {
    Serial.print(HexToDecimal(buffer[i]));
  }
  Serial.println();

  /****M1 info ****/
  Serial.print("Motor 1 ");
  Serial.print("\t Direction: ");
  // Serial.print("\t");
  Serial.print(HexToDecimal(buffer[3]));
  Serial.print("\t Speed: ");
  // Serial.print("\t");
  for(i = 4; i < 8; i++)
  {
    Serial.print(HexToDecimal(buffer[i]));
    // Serial.println();
  }
  Serial.print("\t Distance: ");
  // Serial.print("\t");
  for(i = 9; i < 12; i++)
  {
    Serial.print(HexToDecimal(buffer[i]));
    // Serial.println();
  }
  /****end M1 info ****/


  /****M2 info ****/
  // Serial.print("Motor 2 ");
  // Serial.print("\t Direction: ");
  // // Serial.print("\t");
  // Serial.print(HexToDecimal(buffer[3]));
  // Serial.print("\t Speed: ");
  // // Serial.print("\t");
  // for(i = 4; i < 8; i++)
  // {
  //   Serial.print(HexToDecimal(buffer[i]));
  //   // Serial.println();
  // }
  // Serial.print("\t Distance: ");
  // // Serial.print("\t");
  // for(i = 9; i < 12; i++)
  // {
  //   Serial.print(HexToDecimal(buffer[i]));
  //   // Serial.println();
  // }
  /****end M1 info ****/

  return 0;

}


void controlMotor(String buffer)
{
  uint16_t i;
  uint16_t M1_dir;
  uint32_t M1_vel;
  uint16_t M1_pos;

  M1_dir = HexToDecimal(buffer[3]);
  M1_pos = HexToDecimal(buffer[4])*1000 +
           HexToDecimal(buffer[5])*100 +
           HexToDecimal(buffer[6])*10 +
           HexToDecimal(buffer[7]);
  
  M1_pos = 1000;
  
  int sp[3] = {700,500,700};
  int sp_vel[3] = {80,80,80};

  uint8_t dir[3] = {1,1,1};

  Serial.println("test: ");
  Serial.println(M1_vel);

  Input = posi;
  Setpoint = M1_vel;
  dc.updateSP(sp);
  dc.updateSP_vel(sp_vel);
  dc.getEncodeState(dir);

  int curPos[3] = {posi_M1,posi_M2, posi_M3};

  while(dc.checkState(curPos))
  {
    dc.PIDcompute(posi_M1, posi_M2, posi_M3);
    curPos[0] = posi_M1;
    curPos[1] = posi_M2;
    curPos[2] = posi_M3;
  }
  dc.controlMotor(0,0,0);
  posi_M1 = 0;
  posi_M2 = 0;
  posi_M3 = 0;

}

void setup() 
{
  Serial.begin(115200);         // Start the serial communication
  Ethernet.init(5);             // Initialize the Ethernet module (W5500) on CS pin 5
  Ethernet.begin(mac, ip);      // Start Ethernet with static IP
  server.begin();               // Start the TCP server
  Serial.println("Server is listening...");
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

}

void loop() 
{
  EthernetClient client = server.available();  // Listen for incoming clients
  
  if (client) 
  {
    Serial.println("Client connected");

    //while connecting ...
    while (client.connected()) 
    {
      if (client.available()) 
      {
        char c = client.read();  // Read a character from the client
        if (c == 0x03) 
        {         
          // If newline is received, print the complete string
          Serial.println("Received: " + dataBuffer);
          if(checkSum(dataBuffer))
          {
            char ret[2];
            ret[0] = dataBuffer[1];
            ret[1] = dataBuffer[2];
            server.print(dataBuffer[1]);
            server.println(dataBuffer[2]);

            // Serial.println(ret);
            Serial.println("Checksum done!");
            // parseBuffer(dataBuffer);
            controlMotor(dataBuffer);
            // Serial.print(dataBuffer[1]);
            // parseBuffer(dataBuffer);

            Serial.println();
          }
          else
          {
            Serial.println("Checksum wrong!");
          }
          dataBuffer = "";       // Clear the buffer for the next message
        }
        else
        {
          dataBuffer += c;       // Append character to the buffer
        }
      }
    }
    //end while connecting ...
    
    client.stop();  // Close the connection when the client disconnects
    Serial.println("Client disconnected");
  }
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