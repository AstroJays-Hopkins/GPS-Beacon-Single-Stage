#include <Adafruit_GPS.h>
#include <LoRa.h>

//struct for packet
typedef struct{
    char header = 0x55;
    int32_t lat;
    char lat_dir;
    int32_t lon;
    char lon_dir;
} Packet __attribute__((packed)); 

//baud rate for gps 
int baud_rate = 9600;

//defining GPS serial
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

void setup() {
  //Lora Setup
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //GPS startup
  GPS.begin(baud_rate);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
}


//format
//Divide int32_t by 10000000.0 to get decimal (fixed points)
//1 byte marker 
//4 bytes of int32_t lat
//1 byte of direction N/S
//4 bytes of int32_t lon
//1 bytes of direction E/W

void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, 11);
  LoRa.endPacket();
}

//global variables for lat and lon 
int32_t latpoint_fixed = 0;
int32_t lonpoint_fixed = 0;

void loop() {
  //check for parsing
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())){
      return;
    }
  }
  //read the parse data every second
  //send to lora 
  latpoint_fixed = GPS.latitude_fixed;
  lonpoint_fixed = GPS.longitude_fixed;

  //packet the struct
  Packet packet;
  packet.lat = latpoint_fixed;
  packet.lat_dir = GPS.lat;
  packet.lon = lonpoint_fixed;
  packet.lon_dir = GPS.lon; 

  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}
