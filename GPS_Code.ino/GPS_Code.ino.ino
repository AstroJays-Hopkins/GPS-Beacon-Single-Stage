
#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <LoRa.h>
#include <stdlib.h>


union Data {
  int32_t i;
  uint8_t str[4];
} data;

//start up Software Serial with rx tx pins on Feather m0 express
//RX TX
int frequnecy = 915E6;
int ss_pin = 5;
int reset_pin = 6;
int dio0_pin = 14;
int baud_rate = 9600;

#define SerialPort Serial
#define GPSSerial Serial1
uint32_t timer = millis();

Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO true


void setup() {
  SerialPort.begin(9600);
  SerialPort.print("Setting up Lora");
  LoRa.begin(frequnecy);
  LoRa.setPins(ss_pin, reset_pin, dio0_pin);
  SerialPort.print("Setting up GPS");
  GPS.begin(baud_rate);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
}

void send_to_lora(int32_t lat, int32_t lon) {
  union Data lat_un;
  lat_un.i = lat;
  union Data lon_un;
  lon_un.i = lon;
  LoRa.beginPacket();
  LoRa.write(lon_un.str, 4);
  LoRa.write(lat_un.str, 4);
  LoRa.endPacket();
}

int32_t latpoint_fixed = 0;
int32_t lonpoint_fixed = 0;

void loop() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  if (millis() - timer > 1000) {
    latpoint_fixed = GPS.latitude_fixed;
    lonpoint_fixed = GPS.longitude_fixed;
    send_to_lora(latpoint_fixed,lonpoint_fixed);
  }
}
