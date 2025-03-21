/*
  ModbusTCP for W5x00 Ethernet library
  Basic Server code example

  (c)2020 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <SPI.h>
#include <Ethernet.h>       // Ethernet library v2 is required
#include <ModbusEthernet.h>

// Enter a MAC address and IP address for your controller below.
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177); // The IP address will be dependent on your local network:
ModbusEthernet mb;              // Declare ModbusTCP instance

void setup() 
{
  Serial.begin(115200);     // Open serial communications and wait for port to open
  Ethernet.init(5);        // SS pin
  Ethernet.begin(mac, ip);  // start the Ethernet connection
  delay(1000);              // give the Ethernet shield a second to initialize
  mb.server();              // Act as Modbus TCP server
  // mb.addReg(HREG(1));     // Add Holding register #100
  mb.addHreg(0); // add the register with address 1
  mb.addHreg(1); // add the register with address 2
  mb.addHreg(2); // add the register with address 3
  mb.addHreg(3); // add the register with address 4
  mb.addHreg(4); // add the register with address 5
  mb.addHreg(5); // add the register with address 6
  mb.Hreg(0, 100); // 
}

void loop() 
{
  mb.task();                // Server Modbus TCP queries
  Serial.println(mb.Hreg(0));
  delay(50);
}