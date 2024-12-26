#include "config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void TaskTCP_code( void * pvParameters );
void TaskControl_code( void * pvParameters );
uint8_t HexToDecimal(uint8_t character);
void readEncoder();

uint16_t getVel(String buffer);

uint8_t checkSum(String buffer);


/*******************************************************************************
 * Variables
 * ******************************************************************************/

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // MAC address for the Ethernet shield
IPAddress ip(192, 168, 1, 177);  // Set a static IP address for the server
EthernetServer server(PORT);      // Set up TCP server on port 502

String dataBuffer= "";  // Buffer to hold the incoming data

TaskHandle_t TaskTCP;
TaskHandle_t TaskControl;

/*buffer frame*/
uint8_t  g_u8buffer_vel_str[4];
uint16_t g_u16buffer_vel_dec = 0;
/*end buffer frame*/

/* for PID controller*/
volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
bool flag = 0;
uint32_t settling = 0;
uint16_t target = 0;
bool flag_run = 0;
/* end for PID controller*/


/*******************************************************************************
 * Code
 * ******************************************************************************/

void setup()
{
    Serial.begin(115200); 
    pinMode(LED_BUILTIN, OUTPUT);

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
    attachInterrupt(M3_Enc_A, readEncoder, RISING);
    // end for DC Motor Control

    Ethernet.init(CS_PIN);             // Initialize the Ethernet module (W5500) on CS pin 5
    Ethernet.begin(mac, ip);      // Start Ethernet with static IP
    server.begin();               // Start the TCP server
    Serial.println("Server is listening...");

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore
    (
        TaskTCP_code,   /* Task function. */
        "TaskTCP",     /* name of task. */
        40000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task */
        &TaskTCP,      /* Task handle to keep track of created task */
        CORE_1
    );          /* pin task to core 1 */                  
    delay(500); 

    //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore
    (
        TaskControl_code,   /* Task function. */
        "TaskControl",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task */
        &TaskControl,      /* Task handle to keep track of created task */
        CORE_0
    );          /* pin task to core 1 */
    delay(500); 
    settling = millis();
}

void TaskTCP_code( void * pvParameters )
{
    Serial.print("TaskTCP running on core ");
    Serial.println(xPortGetCoreID());

    while(1)
    {
        EthernetClient client = server.available();
        if (client)
        {
            while (client.connected())
            {
                if (client.available())
                {
                    char c = client.read();
                    
                    dataBuffer += c;
                    if(c == 0x03)
                    {
                        if(checkSum(dataBuffer))
                        {
                            // server.println(dataBuffer);
                            flag_run = 1;
                            target = getVel(dataBuffer);
                            dataBuffer = "";
                        }
                        else
                        {
                            dataBuffer = "";
                        }

                    }
                }
            }
            client.stop();
        }
    }
}

void TaskControl_code( void * pvParameters )
{
    Serial.print("TaskControl running on core ");
    Serial.println(xPortGetCoreID());

    while(1)
    {
        if(flag_run == 1)
        {
            // int target = 1000000;
            // int target = 900*sin(6.28*millis());

            // PID constants
            float kp = 8;
            float kd = 0.025;
            float ki = 0.000001;

            // time difference
            long currT = micros();
            float deltaT = ((float) (currT - prevT))/(1.0e6);
            prevT = currT;


            // Read the position
            int pos = 0; 
            noInterrupts(); // disable interrupts temporarily while reading
            pos = posi;
            interrupts(); // turn interrupts back on

            // error
            int e = pos - target;
            
            // derivative
            float dedt = (e-eprev)/(deltaT);

            // integral
            eintegral = eintegral + e*deltaT;

            // control signal
            float u = kp*e + kd*dedt + ki*eintegral;

            // Serial.print(u);

            // motor power
            float pwr = fabs(u);
            // Serial.println(pwr);
            if( pwr > 200 )
            {
                pwr = 200;
            }

            if(u < 0)
            {
                digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
                shiftOut(DS,SHcp,MSBFIRST,M3_FW_M2_FW_M1_FW);
                digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
                analogWrite(M3_PWM, pwr);
                analogWrite(M2_PWM, pwr);
                analogWrite(M1_PWM, pwr);
            }
            else
            {
                digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
                shiftOut(DS,SHcp,MSBFIRST,M3_BW_M2_BW_M1_BW);
                digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
                analogWrite(M3_PWM, pwr);
                analogWrite(M2_PWM, pwr);
                analogWrite(M1_PWM, pwr);
            }


            // store previous error
            eprev = e;

            if(abs(target - pos) < 5 && flag == 0)
            {
                settling = millis() - settling;
                flag = 1;
            }
            if(target - pos  == 0 || target - pos == 1 || target - pos == -1)
            {
                flag_run = 0;
                digitalWrite(STcp,LOW); //ground ST_CP and hold low for as long as you are transmitting
                shiftOut(DS,SHcp,MSBFIRST,ST);
                digitalWrite(STcp,HIGH); //pull the ST_CPST_CP to save the data
                posi = 0;
                analogWrite(M3_PWM, 0);
                analogWrite(M2_PWM, 0);
                analogWrite(M1_PWM, 0);
                server.println("00");
                // analogWrite(M3_PWM, pwr);
            }
            Serial.print("Time:");
            Serial.print(millis());
            Serial.print(" ");
            Serial.print("settling:");
            Serial.print(settling);
            Serial.print(" ");
            Serial.print("Max:");
            Serial.print(1000);
            Serial.print(" ");
            // Serial.print("Setpoint:");
            // Serial.print(target);
            // Serial.print(" ");
            // Serial.print("Current_Position:");
            // Serial.print(pos);
            // Serial.print(" ");
            Serial.print("Error:");
            Serial.print(target - pos);
            Serial.print(" ");
            Serial.print("Min:");
            Serial.println(0);
        }
        else
        {
            vTaskDelay(10);
        }
        

    }

}



void loop()
{

}


uint8_t HexToDecimal(uint8_t character)
{
    uint8_t result = 0;
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

uint16_t getVel(String buffer)
{
    uint16_t u16vel = 0;
    u16vel = HexToDecimal(buffer[4])*1000 + HexToDecimal(buffer[5])*100 +
             HexToDecimal(buffer[6])*10  + HexToDecimal(buffer[7]); // 9 to 12 is index of position in data frame
    return u16vel;
    
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

void readEncoder()
{
  int b = digitalRead(M3_Enc_B);
  if(b > 0)
  {
    posi++;
  }
  else
  {
    posi--;
  }
}
