
#include <Adafruit_GPS.h>
#include <LoRa.h>
#include "IntersemaBaro.h"

//struct for packet
typedef struct {
  char header = 0x55;
  int32_t lat;
  char lat_dir;
  int32_t lon;
  char lon_dir;
  float altitude; 
  char dummy;
}__attribute__((packed)) Packet;


//defining GPS serial
int baud_rate = 9600;
#define GPSSerial Serial1
#define Serial SERIAL_PORT_USBVIRTUAL
Adafruit_GPS GPS(&GPSSerial);




//altimiter set up
Intersema::BaroPressure_MS5607B baro(true);

// Altitude variables
float avg_alt;
float alt0;
float altitude;



void setup() {
  //Lora Setup
  Serial.begin(9600);
  while (!Serial);
  if (!LoRa.begin(915E6)) {
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  LoRa.setTxPower(20);
  Serial.println("Cranking up power!");
  //initalize baro
  baro.init();
  Serial.println("Initializing Baro!");

  //set starting altitude
  alt0 = 0;
  altitude = 0;
  avg_alt = 0;
  int num_points = 50;
  for (int i=0; i<num_points; i++)
    {
      alt0 += baro.getHeightCentiMeters()/30.48;
      delay(10);
    }
  alt0 /= num_points;
  
  
  //GPS startup
  GPS.begin(baud_rate);
  Serial.println("Initializing GPS!");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("Tuning GPS!");
  Serial.println("Setup Complete!");
}


//format
//Divide int32_t by 10000000.0 to get decimal (fixed points)
//1 byte marker
//4 bytes of int32_t lat
//1 byte of direction N/S
//4 bytes of int32_t lon
//1 bytes of direction E/W
//4 bytes of altitude
//total 15 bytes

void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, 16);
  LoRa.endPacket();
}

//global variables for lat and lon
int32_t latpoint_fixed = 0;
int32_t lonpoint_fixed = 0;
char lat_dir = 'A';
char lon_dir = 'A';


void loop() {


  altitude = baro.getHeightCentiMeters()/30.48 - alt0;
  avg_alt += (altitude - avg_alt)/5;
  //check for parsing
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
    }
  }
  //read the parse data every second
  //send to lora
  latpoint_fixed = GPS.latitude_fixed;
  lonpoint_fixed = GPS.longitude_fixed;
  lat_dir = GPS.lat;
  lon_dir = GPS.lon;
  
  //packet the struct
  Packet packet;
  packet.lat = latpoint_fixed;
  packet.lat_dir = lat_dir;
  packet.lon = lonpoint_fixed;
  packet.lon_dir = lon_dir;
  packet.altitude = avg_alt;
  Serial.print("lat: ");
  Serial.println(packet.lat);
  Serial.print("lat_dir: ");
  Serial.println(packet.lat_dir);
  Serial.print("lon: ");
  Serial.println(packet.lon);
  Serial.print("lon_dir: ");
  Serial.println(packet.lon_dir);
  Serial.print("altitude: ");
  Serial.println(packet.altitude);
  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}
