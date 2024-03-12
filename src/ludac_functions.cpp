#include <ludac_espnow.h>
#include <ludac_lora.h>
#include <ludac_gps.h>
#include <ludac_functions.h>

/* ---------------------------- */
/* --- VARIABLE DEFINITIONS --- */
/* ---------------------------- */

/* ~~ESP-NOW Variables~~ */

// MAC ADDRESSES (only the first line is important to the function of the code atm, the rest might be used later though)
String s_thisAddress = WiFi.macAddress(); // Get the MAC address of this transceiver
String s_broadcastAddress1 = "40:44:D8:08:F9:94"; // MAC address of transceiver A (HackED sticker)
String s_broadcastAddress2 = "40:22:D8:06:75:2C"; // MAC address of transceiver B (no sticker)
String s_broadcastAddress3 = "B8:D6:1A:67:F8:54"; // MAC address of transceiver C (covered microstrip antenna)
String s_broadcastAddress4 = "A0:A3:B3:89:23:E4"; // MAC address of transceiver D (weird WOER antenna, doesn't receive)

// MAC ADDRESSES (uncomment the address of the device that is receiving)
//uint8_t broadcastAddress[] = {0x40, 0x44, 0xD8, 0x08, 0xF9, 0x94}; // MAC address of transceiver A (HackED sticker)
//uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0x06, 0x75, 0x2C}; // MAC address of transceiver B (no sticker)
uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}; // MAC address of transceiver C (covered microstrip antenna)
//uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}; // MAC address of transceiver D (weird WOER antenna, doesn't receive)

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

payload outgoing; // Message prepped to send
payload incoming; // Message received by other receiver
esp_now_peer_info_t peerInfo; // To register the other receiver

/* ~~LoRa Variables~~ */

const int resetPin = 15; // reset pin connection
const int csPin = 5;     // sck pin connection
const int irqPin = 2;    // DIO0 pin connection (hardware interupt pin)

int counter = 0; // counter for outgoing messages

// Initiate LoRa buffer
char LoRa_sending_buffer[50] = {0};

// Initiate the received buffer and the lon, lat, distance containers for the received data
char LoRa_received_buffer[50] = {0};
char lat_rec[15] = {0};
char lon_rec[15] = {0};
char dis_rec[15] = {0};
float lat_away;
float lon_away;
float dis_away;

// Define constants for LoRa transceiving
byte msgCount = 0;
byte localAddress = 0xBB;
byte destination = 0xAA;
byte buffer_size = 50;
long lastSendTime = 0;
int interval = 2000;

/* ~~GPS Variables~~ */

// Set up software serial connection with Adafruit Ultimate GPS. TX = 26, RX = 27
SoftwareSerial GPS_serial(26,27);
Adafruit_GPS GPS(&GPS_serial);

// Placeholder GPS data for LoRa testing when GPS doesn't find a fix 
const float fake_gps_lat = 53.530178;
const float fake_gps_lon = -113.530662;


/* ------------------------------------ */
/* --- GENERAL FUNCTION DEFINITIONS --- */
/* ------------------------------------ */



/* ----------------------------------- */
/* --- ESPNOW FUNCTION DEFINITIONS --- */
/* ----------------------------------- */

// Initialize WiFi and ESPNOW
void initLudacWIFI() {
  
  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Initialize packet number to zero
  out_packet_no = 0; 

  // Print the MAC address of this transceiver
  Serial.println("This device MAC address: " + s_thisAddress); 

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

// Function that gets called on every loop of the main program
void ludac_espnow_loop() {
  
  // Get readings from peripherals
  getReadings();

  // Load readings into "outgoing" payload
  strcpy(outgoing.message, out_message);
  outgoing.time = out_time;
  outgoing.packet_no = out_packet_no;

  // Send outgoing payload via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
   
  // Check for errors
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  // Print readings
  Serial.println("INCOMING READINGS");
  Serial.print("Time: ");
  Serial.print(inc_time);
  Serial.println();
  Serial.print("Incoming Message: ");
  Serial.print(inc_message);
  Serial.println();
  Serial.print("Outgoing String: ");
  Serial.print(out_message);
  Serial.println();
  Serial.print("Packet Number: ");
  Serial.print(inc_packet_no);
  Serial.println();

  // Wait before continuing loop (ms)
  delay(10000);
}


// Receive location + time from peripherals (mainly Adafruit GPS)
void getReadings() {
  s_thisAddress.toCharArray(out_message, sizeof(out_message));
  //strcpy(out_message, "test string"); //s_thisAddress;
  out_time = 500;
  out_packet_no++;
}

// Callback triggered when data is sent via ESP-NOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success";
  }
  else {
    success = "Delivery Fail";
  }
}

// Callback triggered when data is received via ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print("Bytes received: ");
  Serial.println(len);
  strcpy(inc_message, incoming.message);
  inc_time = incoming.time;
  inc_packet_no = incoming.packet_no;
}



/* ------------------------------------- */
/* --- GPS/NMEA FUNCTION DEFINITIONS --- */
/* ------------------------------------- */



// Initialize Adafruit GPS
void initLudacGPS() {
  GPS_serial.begin(9600); // Start GPS serial stream
}


float relaDistance(float lat1, float lon1, float lat2, float lon2) {
  
  // Pre-calculate some parameters to speed up the algorithm
  float delta_lat = (lat2-lat1)*PI/180;
  float delta_lon = (lon2-lon1)*PI/180;
  float sin_delta_lat = sin(delta_lat/2);
  float sin_delta_lon = sin(delta_lon/2);

  // Implement the GPS distance formula
  float d = 2*6378000*asin(sqrt(sin_delta_lat*sin_delta_lat+cos(lat1*PI/180)*cos(lat2*PI/180)*sin_delta_lon*sin_delta_lon));
  return d;
}

void recParsing(char received[]){
  // Parse the received char array by store the corresponding parts to their contaiers
  for(int i = 0; i<15; i++){
    lat_rec[i] = received[i];
    lat_away = atof(lat_rec);
  }

  for(int j = 0; j<15; j++){
    lon_rec[j] = received[j+15];
    lon_away = atof(lon_rec);
  }

  for(int k = 0; k<15; k++){
    dis_rec[k] = received[k+30];
    dis_away = atof(dis_rec);
  }
}



/* --------------------------------- */
/* --- LORA FUNCTION DEFINITIONS --- */
/* --------------------------------- */

void initLudacLORA() {

  // Check if debugging serial has been started
  while (!Serial);

  Serial.println("LoRa Duplexing Demo and GPS");

  // set LoRa pin connection to ESP32
  LoRa.setPins(csPin, resetPin, irqPin);

  delay(2000);

  // This should be re-written to handle LoRa connection errors
  while(!LoRa.begin(915E6))
  {
    Serial.println("LoRa init failed.");
    delay(5000);
  }

  // Set the LoRa spreading factor
  LoRa.setSpreadingFactor(8);
  Serial.println("LoRa module initiated successfully");

  // Set GPS NMEA data type and update frequency
  GPS_serial.println(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  GPS_serial.println(PMTK_SET_NMEA_UPDATE_1HZ);

}

void sendMSG(char outgoing[])
{
  // Define destination address, local address, message counter, and message size for LoRa
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(counter);
  LoRa.write(buffer_size);

  // Send the message char array
  int n = 0;

  while(n<buffer_size){
    LoRa.print(outgoing[n]);
    Serial.print(outgoing[n]);
    n++;
  }
    
  // Once the array size is reached, end sending the LoRa packet
  LoRa.endPacket();
  counter++;
}

void onReceive(int packetSize)
{
  // If there is no incoming message, return
  if (packetSize == 0)
    return;

  // Read the incoming message length, sender ID, and message ID
  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();

  char incoming[buffer_size] = {};
  // Read byte by byte, and store the byte in the buffer
  for (int m=0; m<buffer_size; m++){
    
      incoming[m] += LoRa.read();
      
  }
  
  // Error handling
  if (incomingLength != buffer_size)
  {
    Serial.println("error, message length does not match");
    return;
  }

  if (recipient != localAddress && recipient != 0xFF)
  {
    Serial.println("This message is not for me");
    return;
  }

  // Print the incoming message information to the serial monitor for troubleshooting
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length " + String(incomingLength));
  for(int i=0; i<buffer_size;i++){
    Serial.print(incoming[i]);
  }
  Serial.println("; RSSI" + String(LoRa.packetRssi()));

  Serial.println();

  for(int j=0; j<buffer_size; j++){
    LoRa_received_buffer[j] = incoming[j];
  }

  // Parse the received message
  recParsing(LoRa_received_buffer);
  Serial.println(lat_away);
  Serial.println(lon_away);
  Serial.println(dis_away);
  
}

void ludac_lora_loop() {
  
  if(GPS_serial.available()){
    

    // Convert GPS lat, lon, and distance to the corresponding char arrays
    char lat[15] = {0};
    sprintf(lat, "%f", fake_gps_lat); //change this to GPS.latitude in practise
    
    char lon[15] = {0};
    sprintf(lon, "%f", fake_gps_lon); //change this to GPS.longtitude in practise

    // call the relaDistance function to calculate the relative distance
    float dist = relaDistance(fake_gps_lat, fake_gps_lon, 53.534940, -113.541585);
    
    char dis[15] = {0};
    sprintf(dis, "%f", dist);
    
    // Insert the lat, lon, and distance into the buffer array
    for(int j = 0; j<15; j++){
      LoRa_sending_buffer[j] = lat[j];
    }
    
    for(int k = 0; k<15; k++){
      LoRa_sending_buffer[k+15] = lon[k];
    }
      
    for(int z = 0; z<15; z++){
      LoRa_sending_buffer[z+30] = dis[z];
    }
    
  }

  // If the difference between current and the last send time is greater than the interval time, 
  // send the buffered data
  if (millis() - lastSendTime > interval)
  {
    sendMSG(LoRa_sending_buffer);
    
    lastSendTime = millis();
    interval = random(2000) + 1000;
  }

  // When not sending LoRa packets, listen to the incoming messages
  onReceive(LoRa.parsePacket());
}
