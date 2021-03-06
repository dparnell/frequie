frequie
=======

This code allows the Si5351 chip to be driven easily on Arduino based devices.
Unlike other Arduino code for the Si5351 chip the PLL is set once at initialization time and the clocks are individually configured to run based on PLLA.
PLLB is not used, however since the code to write to registers is exposed it is possible to use this code to configure the device in any way required.

The values produced for P1, P2 and P3 for a given clock have been verified to be the same as produced by the official Clock Builder Pro software from Silicon Labs for a given set of frequencies.

Usage
=====

Frequie uses C++ templates to do all the configuration work at compile time. This means it should take up less code space and less RAM than other libraries.
Below is an example of the usage of this library.

The library ships with two I2C interfaces. The first uses Wire while the second bit bangs I2C. The bit bang version may be more appropriate for smaller devices that do not need the full Wire feature set.

```c++
#include <frequie.h>
#include <wirei2c.h>

/**
 * Create an instance of Frequie using Wire to talk to the Si5351 chip with SDA on pin 21, SCL on pin 22 and a transfer rate of 100kHz,
 * crystal reference of 25MHz, PLLA frequency of 900 MHz, using the standard Si5351 device address and finally having only 3 clock generators.
 */
Frequie<WireI2C<Wire, 21, 22, 100000>, SI5351_XTAL_25MHZ, SI5351_PLL_900MHZ, SI5351_DEVICE_ADDRESS, 3> frequie;

void setup() {
  Serial.begin(9600);

  Serial.println("BOOTED");

  // scan the I2C and display a map of the connected devices
  frequie.scan_i2c_bus(Serial);

  Serial.println("Detecting device...");
  while(!frequie.detect()) {
    // no Si5351 device was found at the configured address so loop around until one appears
    delay(1000);
    frequie.scan_i2c_bus(Serial);
  }

  Serial.println("Found :)");
  // initialize the Si5351 chip
  if(frequie.init()) {
    Serial.println("Device initialized successfully");

    // set up some frequencies
    frequie.set_clock_frequency(0, 1000000);
    frequie.set_clock_frequency(1, 2000000);
    frequie.set_clock_frequency(2, 5000000);

    // turn on the clocks
    frequie.enable_clock(0);
    frequie.enable_clock(1);
    frequie.enable_clock(2);

    // profit
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

license
=======

```
Copyright 2021 Daniel Parnell me@danielparnell.com

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
   in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```