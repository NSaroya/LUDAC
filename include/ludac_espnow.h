
#ifndef LUDAC_ESPNOW_H
#define LUDAC_ESPNOW_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define LENGTH 50 // char buffer array length

// MAC ADDRESSES (only the first line is important to the function of the code)
extern String s_thisAddress; // Get the MAC address of this transceiver
extern String s_broadcastAddress1; // MAC address of transceiver A (HackED sticker)
extern String s_broadcastAddress2; // MAC address of transceiver B (no sticker)
extern String s_broadcastAddress3; // MAC address of transceiver C (covered microstrip antenna)
extern String s_broadcastAddress4; // MAC address of transceiver D (weird WOER antenna)

/* Uncomment the MAC address of the transceiver you are sending to*/
extern uint8_t broadcastAddress[];
extern uint8_t broadcastAddress1[]; // MAC address of transceiver A (HackED sticker)
extern uint8_t broadcastAddress2[]; // MAC address of transceiver B (no sticker)
extern uint8_t broadcastAddress3[]; // MAC address of transceiver C (covered microstrip antenna)
extern uint8_t broadcastAddress4[]; // MAC address of transceiver D (weird WOER antenna)

// Outgoing variables
extern char out_message[LENGTH];
extern float out_time;
extern int out_packet_no;

// Incoming variables
extern char inc_message[LENGTH];
extern float inc_time;
extern int inc_packet_no;

// Success message
extern String success;

// Define a payload struct
typedef struct payload {
    char message[LENGTH];
    float time;
    int packet_no;
} payload;

extern payload outgoing; // Message prepped to send
extern payload incoming; // Message received by other receiver
extern esp_now_peer_info_t peerInfo; // To register the other receiver

#endif