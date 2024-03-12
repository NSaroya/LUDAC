// This file only contains the set-up routine and main program loop.
// Function definitions go in ludac_functions.cpp, everything else goes in header files.
// Keep this file as easy to read as possible.

#include <ludac_functions.h> // All necessary includes go here



/* ---------------------- */
/* --- SET-UP ROUTINE --- */
/* ---------------------- */

void setup() {

    // Initialize serial monitor
    Serial.begin(9600);

    // Initialize peripherals (in this order)
    initLudacWIFI();
    //initLudacGPS();
    //initLudacLORA(); 

}



/* ------------------------- */
/* --- MAIN PROGRAM LOOP --- */
/* ------------------------- */

void loop() {

    ludac_espnow_loop();
    //ludac_lora_loop();
    
}