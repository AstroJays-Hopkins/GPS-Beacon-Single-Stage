#include <LoRa.h>

//buffer for struct
char buffer[11];

//struct for packet
typedef struct{
    char header = 0x55;
    int32_t lat;
    char lat_dir;
    int32_t lon;
    char lon_dir;
} Packet __attribute__((packed)); 


void setup() {
  if (!LoRa.begin(915E6)) {
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  //check for packet size 11
  if (packetSize==11) {
    // received a packet
    char first = (char)LoRa.read();

    //the header is always 0x55
    if(first == 0x55) {
      buffer[0] = first;
      int i=1;
      //read into buffer
      while(i<11) {
        buffer[i]=(char)LoRa.read();
        ++i;
      }
      //parse the buffer into packet
      Packet * packet_ptr = (Packet *) buffer;
    }
  }
}
