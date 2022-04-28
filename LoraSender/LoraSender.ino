
#include <Adafruit_GPS.h>
#include <RH_RF95.h>
#include "IntersemaBaro.h"

//struct for packet
typedef struct {
  char header = 0x55;
  int32_t lat;
  char lat_dir;
  int32_t lon;
  char lon_dir;
  float altitude; 
} Packet __attribute__((packed));


//defining GPS serial
int baud_rate = 9600;
#define GPSSerial Serial1
#define Serial SERIAL_PORT_USBVIRTUAL
Adafruit_GPS GPS(&GPSSerial);


//lora setup
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"
#define RF95_FREQ 915.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


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

  //initalize baro
  baro.init();

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


  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);


  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

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
//4 bytes for altitude

void send_to_lora(uint8_t * packet) {
  //writing with packet
  if(rf95.available()){
    rf95.send(packet, 15);
    rf95.waitPacketSent();
  }
}

//global variables for lat and lon
int32_t latpoint_fixed = 0;
int32_t lonpoint_fixed = 0;

void loop() {

  //update the altitude using linear converter
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

  //packet the struct
  Packet packet;
  packet.lat = latpoint_fixed;
  packet.lat_dir = GPS.lat;
  packet.lon = lonpoint_fixed;
  packet.lon_dir = GPS.lon;
  packet.altitude = avg_alt;
  
  Serial.println(packet.lat);
  Serial.println(packet.lon);
  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}
