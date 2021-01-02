#include <Arduino.h>
#include "frequie.h"
#include "bbi2c.h"

Frequie<BBI2C, SI5351_XTAL_25MHZ, SI5351_PLL_900MHZ, SI5351_DEVICE_ADDRESS, 3> frequie(21, 22);

void setup() {
  Serial.begin(9600);
  Serial.println("Detecting device...");
  if(frequie.detect()) {
    Serial.println("Found :)");
    
    if(frequie.init()) {
      Serial.println("Device initialized successfully");

      frequie.set_clock_frequency(0, 1000000);
      frequie.set_clock_frequency(1, 2000000);
      frequie.set_clock_frequency(2, 5000000);

      frequie.enable_clock(0);
      frequie.enable_clock(1);
      frequie.enable_clock(2);
    } else {
      Serial.println("Device initialization failed :(");

      while(true) {
        // STOP
      }  
    }
  } else {
    Serial.println("Device not found :(");

    while(true) {
      // STOP
    }  
  }
}

void loop() {
  /*
    frequie.set_clock_frequency(0, 1000000);
    frequie.enable_clock(0);
  */
}