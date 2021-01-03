#include <Arduino.h>
#include <frequie.h>

#include <bbi2c.h>
Frequie<BBI2C<21, 22, 1>, SI5351_XTAL_25MHZ, SI5351_PLL_900MHZ, SI5351_DEVICE_ADDRESS, 3> frequie;

/*
#include <wirei2c.h>
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
}

void loop() {
  /*
    frequie.set_clock_frequency(0, 1000000);
    frequie.enable_clock(0);
  */
}