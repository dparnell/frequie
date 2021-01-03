frequie
=======

This code allows the Si5351 chip to be driven easily on Arduino based devices.
Unlike other Arduino code for the Si5351 chip the PLL is set once at initialization time and the clocks are individually configured to run based on PLLA.
PLLB is not used, however since the code to write to registers is exposed it is possible to use this code to configure the device in any way required.

The values produced for P1, P2 and P3 for a given clock have been verified to be the same as produced by the official Clock Builder Pro software from Silicon Labs for a given set of frequencies.

Usage
=====

```c++
#include <frequie.h>
#include <wirei2c.h>

Frequie<WireI2C<Wire, 21, 22, 100000>, SI5351_XTAL_25MHZ, SI5351_PLL_900MHZ, SI5351_DEVICE_ADDRESS, 3> frequie;

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
    // do nothing
}
```

