
#ifndef LUDAC_LORA_H
#define LUDAC_LORA_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <sstream>

// Define LoRa pin connection
extern const int resetPin; // reset pin connection = 15
extern const int csPin;     // sck pin connection = 5
extern const int irqPin;    // DIO0 pin connection (hardware interupt pin)

extern int counter; // counter for outgoing messages

// Initiate LoRa buffer
extern char LoRa_sending_buffer[50];

// Initiate the received buffer and the lon, lat, distance containers for the received data
extern char LoRa_received_buffer[50];
extern char lat_rec[15];
extern char lon_rec[15];
extern char dis_rec[15];
extern float lat_away;
extern float lon_away;
extern float dis_away;

// Define constants for LoRa transceiving
extern byte msgCount;
extern byte localAddress;
extern byte destination;
extern byte buffer_size;
extern long lastSendTime;
extern int interval;

#endif