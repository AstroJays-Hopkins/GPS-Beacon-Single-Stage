
#include <SPI.h>
#include <LoRa.h>

char buffer[10];

typedef struct{
    char header = 0x55;
    int32_t lat;
    char lat_dir;
    int32_t lon;
    char lon_dir;
} Packet; 

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  //packetSize must be 11
  if (packetSize == 11 && LoRa.available()) {
    char first = (char)LoRa.read();
    if(first == 0x55) {
      buffer[0] = first;
      int i=1;
      while(i<11) {
        buffer[i]=(char)LoRa.read();
        ++i;
      }
      Packet * packet_ptr = (Packet *) buffer;
    }
    
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
