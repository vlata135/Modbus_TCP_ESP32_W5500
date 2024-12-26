/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
*********/
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>

/**
 * defination
 */
// LED pins
const int led1 = 2;
const int led2 = 4;

/**
 * Variables
 */

TaskHandle_t Task1;
TaskHandle_t Task2;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // MAC address for the Ethernet shield
IPAddress ip(192, 168, 1, 177);  // Set a static IP address for the server
EthernetServer server(502);      // Set up TCP server on port 502

char dataBuffer = '0';  // Buffer to hold the incoming data
uint16_t i;

/**
 * prototyoe
 */
void Task1code( void * pvParameters );
void Task2code( void * pvParameters );
uint8_t HexToDecimal(uint8_t character);

void setup() {
  Serial.begin(115200); 
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  Ethernet.init(5);             // Initialize the Ethernet module (W5500) on CS pin 5
  Ethernet.begin(mac, ip);      // Start Ethernet with static IP
  server.begin();               // Start the TCP server
  Serial.println("Server is listening...");

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    40000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */
    delay(500); 
}

//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters )
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(led1, HIGH);
    vTaskDelay(10);
    digitalWrite(led1, LOW);
    vTaskDelay(10);

    EthernetClient client = server.available();
    if (client)
    {
      while (client.connected())
      {
        if (client.available())
        {
          char c = client.read();
          dataBuffer = c;
        }
      }
      client.stop();
    }
  } 
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    analogWrite(led1, HexToDecimal(dataBuffer));
    if(HexToDecimal(dataBuffer) == 0xF)
    {
      server.println("Maximum");
    }
    vTaskDelay(10);
  }
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

void loop() {
  
}