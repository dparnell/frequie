#include <Arduino.h>
#include "frequie.h"

#include "bbi2c.h"
Frequie<BBI2C<21, 22, 1>, SI5351_XTAL_25MHZ, SI5351_PLL_900MHZ, SI5351_DEVICE_ADDRESS, 3> frequie;

/*
#include "wirei2c.h"
Frequie<WireI2C<Wire, 21, 22, 100000>, SI5351_XTAL_25MHZ, SI5351_PLL_900MHZ, SI5351_DEVICE_ADDRESS, 3> frequie;
*/

void setup() {
  Serial.begin(9600);

  Serial.println("BOOTED");

  frequie.scan_i2c_bus(Serial);

  Serial.println("Detecting device...");
  while(!frequie.detect()) {
    delay(1000);
    frequie.scan_i2c_bus(Serial);
  }

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
/*
frequie.write_register(183, 146); 
frequie.write_register(16, 128); 
frequie.write_register(17, 128); 
frequie.write_register(18, 128); 
frequie.write_register(177, 160); 
frequie.write_register(16, 128); 
frequie.write_register(17, 128); 
frequie.write_register(18, 111); 
frequie.write_register(34, 255); 
frequie.write_register(35, 255); 
frequie.write_register(36, 0); 
frequie.write_register(37, 9); 
frequie.write_register(38, 51); 
frequie.write_register(39, 243); 
frequie.write_register(40, 50); 
frequie.write_register(41, 179); 
frequie.write_register(58, 0); 
frequie.write_register(59, 1); 
frequie.write_register(60, 0); 
frequie.write_register(61, 138); 
frequie.write_register(62, 0); 
frequie.write_register(63, 0); 
frequie.write_register(64, 0); 
frequie.write_register(65, 0); 
frequie.write_register(177, 128); 
*/
}

void loop() {
  /*
    frequie.set_clock_frequency(0, 1000000);
    frequie.enable_clock(0);
  */
}