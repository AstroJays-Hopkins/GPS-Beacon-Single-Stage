
#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <LoRa.h>
#include <stdlib.h>

//Union type punning 
union Data {
  int32_t i;
  uint8_t str[4];
} data;

//lora frequency and pins
int frequnecy = 915E6;
int ss_pin = 5;
int reset_pin = 6;
int dio0_pin = 14;

//baud rate for gps 
int baud_rate = 9600;

//defining GPS serial
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

//setting up timer to ping ground station
uint32_t timer = millis();


void setup() {
  //Lora Setup
  LoRa.begin(frequnecy);
  LoRa.setPins(ss_pin, reset_pin, dio0_pin);

  //GPS startup
  GPS.begin(baud_rate);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
}


//format
//Divide int32_t by 10000000.0 to get decimal (fixed points)
//4 bytes of int32_t lat
//1 byte of direction N/S
//4 bytes of int32_t lon
//1 bytes of direction E/W

void send_to_lora(int32_t lat, char lat_dir, int32_t lon, char lon_dir) {
  //type puning 
  union Data lat_un;
  lat_un.i = lat;
  union Data lon_un;
  lon_un.i = lon;

  //writing with packet
  LoRa.beginPacket();
  LoRa.write(lon_un.str, 4);
  LoRa.write(lat_dir);
  LoRa.write(lat_un.str, 4);
  LoRa.write(lon_dir);
  LoRa.endPacket();
}

//global variables for lat and lon 
int32_t latpoint_fixed = 0;
int32_t lonpoint_fixed = 0;

void loop() {
  //check for parsing
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  //read the parse data every second
  //send to lora 
  if (millis() - timer > 1000) {
    latpoint_fixed = GPS.latitude_fixed;
    lonpoint_fixed = GPS.longitude_fixed;
    send_to_lora(latpoint_fixed,GPS.lat,lonpoint_fixed, GPS.lon);
  }
}
