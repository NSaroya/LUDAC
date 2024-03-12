
#ifndef LUDAC_GPS_H
#define LUDAC_GPS_H

#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>
#include <SoftwareSerial.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

// Placeholder GPS data for LoRa testing when GPS doesn't find a fix 
extern const float fake_gps_lat;
extern const float fake_gps_lon;

#endif