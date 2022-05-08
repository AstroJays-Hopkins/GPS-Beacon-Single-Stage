# Beacon Code for Solid Rocket

## Purpose
This code is for the solid rocket recovery and ground system\
Sender: It reads data from a GPS Ultimate V3 and from a Baro MS5607B and sends it via Lora to the ground station. This code will be run on the rocket.

Reciever: It receives and parses the packet from the sender. This code will be run on the ground station.

## Packet Description
The packet sent from the rocket is structured as: \
1 byte marker (0x55 for this packet) \
4 bytes of int32_t fixed point lat \
1 byte of direction N/S \
4 bytes of int32_t fixed point lon \
1 bytes of direction E/W \
4 bytes of float for altitude \
1 bytes of dummy padding \
Divide the fixed point number by 10000000.0 to get decimal degrees equivalent \
altitude is in feet

## Use

Use it with an arduino IDE with the LoRa and the AdaFruit_GPS libraries linked here:\
https://github.com/sandeepmistry/arduino-LoRa \
https://github.com/adafruit/Adafruit_GPS

Add header\
Add IntersemaBaro.h from https://github.com/AstroJays-Hopkins/Avionics/blob/master/recovery/PRO_MICRO/IntersemaBaro.h
