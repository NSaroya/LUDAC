#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define LENGTH 50 // char buffer array length

// MAC ADDRESSES (only the first line is important to the function of the code)
String s_thisAddress = WiFi.macAddress(); // Get the MAC address of this transceiver
String s_broadcastAddress1 = "40:44:D8:08:F9:94"; // MAC address of transceiver A (HackED sticker)
String s_broadcastAddress2 = "40:22:D8:06:75:2C"; // MAC address of transceiver B (no sticker)
String s_broadcastAddress3 = "B8:D6:1A:67:F8:54"; // MAC address of transceiver C (covered microstrip antenna)
String s_broadcastAddress4 = "A0:A3:B3:89:23:E4"; // MAC address of transceiver D (weird WOER antenna)

/* Uncomment the MAC address of the transceiver you are sending to*/
//uint8_t broadcastAddress[] = {0x40, 0x44, 0xD8, 0x08, 0xF9, 0x94}; // MAC address of transceiver A (HackED sticker)
uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0x06, 0x75, 0x2C}; // MAC address of transceiver B (no sticker)
//uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}; // MAC address of transceiver C (covered microstrip antenna)
//uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}; // MAC address of transceiver D (weird WOER antenna)

// Outgoing variables
char out_message[LENGTH];
float out_time;
int out_packet_no;

// Incoming variables
char inc_message[LENGTH];
float inc_time;
int inc_packet_no;

// Success message
String success;

// Define a payload struct
typedef struct payload {
    char message[LENGTH];
    float time;
    int packet_no;
} payload;

payload outgoing; // Message prepped to send
payload incoming; // Message received by other receiver
esp_now_peer_info_t peerInfo; // To register the other receiver