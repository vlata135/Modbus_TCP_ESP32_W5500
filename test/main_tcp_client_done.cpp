#include <SPI.h>
#include <Ethernet.h>

// Địa chỉ MAC của W5500
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

// IP của ESP32
IPAddress ip(192, 168, 1, 177); 
// IP của máy tính chạy Hercules
IPAddress server(192, 168, 1, 100); 
// Cổng TCP trên Hercules
int port = 1234;   

EthernetClient client;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Chờ kết nối Serial (nếu cần)
  
  // Khởi tạo Ethernet
  Ethernet.init(5);   // Chân SS cho SPI
  Ethernet.begin(mac, ip);
  
  // Đợi Ethernet kết nối
  delay(1000);

  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());

  // Kết nối tới server (Hercules)
  if (client.connect(server, port)) {
    Serial.println("Connected to server!");
    client.println("Hello from Module 1");
  } else {
    Serial.println("Connection failed. Error Code: ");
    Serial.println(client.connect(server, port));
  }
}

void loop() {
  // Kiểm tra có dữ liệu từ server (Hercules)
  if (client.available()) {
    char c = client.read();
    Serial.print(c);
  }

  // Nếu mất kết nối
  if (!client.connected()) {
    Serial.println("Disconnected from server");
    client.stop();
    while (true);  // Dừng chương trình
  }
}
