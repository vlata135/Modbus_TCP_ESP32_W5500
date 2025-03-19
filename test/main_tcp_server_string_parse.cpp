#include <SPI.h>
#include <Ethernet.h>
#include <PID_v1.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // MAC address for the Ethernet shield
IPAddress ip(192, 168, 1, 177);  // Set a static IP address for the server
EthernetServer server(502);      // Set up TCP server on port 502

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
  M1_vel = HexToDecimal(buffer[4])*1000 +
           HexToDecimal(buffer[5])*100 +
           HexToDecimal(buffer[6])*10 +
           HexToDecimal(buffer[7]);
    
  Serial.println("test: ");
  Serial.println(M1_vel);

  while(pos)

  
}

void setup() 
{
  Serial.begin(115200);         // Start the serial communication
  Ethernet.init(5);             // Initialize the Ethernet module (W5500) on CS pin 5
  Ethernet.begin(mac, ip);      // Start Ethernet with static IP
  server.begin();               // Start the TCP server
  Serial.println("Server is listening...");
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
            server.println("01");
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
