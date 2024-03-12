
#ifndef LUDAC_FUNCTIONS_H
#define LUDAC_FUNCTIONS_H


#include <ludac_espnow.h>
#include <ludac_lora.h>
#include <ludac_gps.h>

/* ----------------------------------- */
/* --- GENERAL FUNCTION PROTOTYPES --- */
/* ----------------------------------- */




/* ----------------------------------- */
/* --- ESPNOW FUNCTION PROTOTYPES --- */
/* ----------------------------------- */

void initLudacWIFI(); // Initialize WiFi, ESP-NOW, get MAC address, register peer

void getReadings(); // Receive location + time from peripherals (mainly Adafruit GPS)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status); // Callback triggered when data is sent via ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len); // Callback triggered when received is sent via ESP-NOW

void ludac_espnow_loop(); // Runs on every program loop




/* ------------------------------------ */
/* --- GPS/NMEA FUNCTION PROTOTYPES --- */
/* ------------------------------------ */

void initLudacGPS(); // Initialize GPS peripheral

void recParsing(char received[]); // Parse received data

float relaDistance(float lat1, float lon1, float lat2, float lon2); // Distance calculation



/* -------------------------------- */
/* --- LORA FUNCTION PROTOTYPES --- */
/* -------------------------------- */

void initLudacLORA(); // Initialize LoRa peripheral

void sendMSG(char outgoing[]); // Sending function
void onReceive(int packetSize); // Receiving function

void ludac_lora_loop(); // Runs on every program loop

#endif