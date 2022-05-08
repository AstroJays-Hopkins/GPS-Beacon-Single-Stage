#include <LoRa.h>


//buffer for packet
char buffer[16];


//struct for packet 
typedef struct{
    char header = 0x55;
    int32_t lat;
    char lat_dir;
    int32_t lon;
    char lon_dir;
    float altitude; 
    char dummy;
} Packet __attribute__((packed)); 



void setup() {
  //set up serial output
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");
  //set up LoRa
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  //packet size is 16
  if (packetSize==16) {
    // received a packet
    Serial.println("Received packet ");

    char first = (char)LoRa.read();
    //the header is always 0x55
    if(first == 0x55) {
      buffer[0] = first;
      int i=1;
      //read into buffer
      while(i<16) {
        buffer[i]=(char)LoRa.read();
        ++i;
      }
      //parse the buffer into packet
      Packet * packet_ptr = (Packet *) buffer;
      //output lat lon 
      Serial.print("lat: ");
      Serial.println(packet_ptr->lat);
      Serial.print("lat_dir: ");
      Serial.println(packet_ptr->lat_dir);
      Serial.print("lon: ");
      Serial.println(packet_ptr->lon);
      Serial.print("lon_dir: ");
      Serial.println(packet_ptr->lon_dir);
      Serial.print("altitude: ");
      Serial.println(packet_ptr->altitude);
    }

  }
  
}
