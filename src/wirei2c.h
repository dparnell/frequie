#ifndef _WIREI2C_
#define _WIREI2C_

/*

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

*/

#include <stdint.h>
#include <Wire.h>


template<TwoWire& WIRE, uint8_t SDA_PIN, uint8_t SCL_PIN, uint32_t FREQUENCY> class WireI2C
{
public:
    WireI2C()
    {

    };

    void begin() {
        WIRE.begin(SDA_PIN, SCL_PIN, FREQUENCY);
    };

    void end() {

    };

    void beginTransmission(uint8_t i2c_bus_addr) {
        WIRE.beginTransmission(i2c_bus_addr);
    };

    bool endTransmission() {
        return WIRE.endTransmission();
    };

    bool write(uint8_t data) {
        return WIRE.write(data);
    };

    uint8_t read(uint8_t i2c_bus_addr, bool stop) {
        uint8_t result;
        WIRE.readTransmission(i2c_bus_addr, &result, 1, stop);

        return result;
    };
};

#endif