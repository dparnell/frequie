#ifndef _BBI2C_
#define _BBI2C_

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
#include <Arduino.h>

/**
 * The BBI2C template builds a bit banged I2C interface for use in smaller systems
 * and those where the existing I2C interface is not appropriate.
 *
 * @param SDA_PIN the pin to use for data
 * @param SCL_PIN the pin to use for clock pulses
 * @param CLOCK_DELAY the aproximate number of miliseconds wide the clock pulses should be (typically 1)
 */
template<uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t CLOCK_DELAY>class BBI2C
{
public:
    BBI2C() {};

    /**
     * Start the I2C interface
     */
    void begin() {
        pinMode(SDA_PIN, OUTPUT);
        pinMode(SCL_PIN, OUTPUT);
        digitalWrite(SDA_PIN, HIGH);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(10);
    };

    /**
     * Shut down the I2C interface
     */
    void end() {
        pinMode(SDA_PIN, INPUT);
        pinMode(SCL_PIN, INPUT);
    };

    /**
     * Start a write operation on the I2C bus
     *
     * @param i2c_bus_addr the device address to communicate with
     */
    void beginTransmission(uint8_t i2c_bus_addr) {
        address_ack = _begin(i2c_bus_addr, false);
    };

    /**
     * Finish a I2C transaction
     *
     * @return the value of the I2C ack bit from the device (LOW for write and HIGH for read)
     */
    bool endTransmission() {
        // send i2c stop condition
        delayMicroseconds(CLOCK_DELAY);
        digitalWrite(SDA_PIN, LOW);
        delayMicroseconds(CLOCK_DELAY);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(CLOCK_DELAY);
        digitalWrite(SDA_PIN, HIGH);

        return address_ack;
    };

    /**
     * Send the given byte out over the I2C interface
     *
     * @return true if the remote device acknowleged the write
     */
    bool write(uint8_t data) {
        for(int i=0; i<8; i++) {
            if(data & 0x80) {
                digitalWrite(SDA_PIN, HIGH);
            } else {
                digitalWrite(SDA_PIN, LOW);
            }
            delayMicroseconds(CLOCK_DELAY);
            digitalWrite(SCL_PIN, HIGH);
            delayMicroseconds(CLOCK_DELAY);
            digitalWrite(SCL_PIN, LOW);
            delayMicroseconds(CLOCK_DELAY);
            digitalWrite(SDA_PIN, HIGH);

            data = data << 1;
        }

        // now read the ACK bit from the bus
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(CLOCK_DELAY);
        pinMode(SDA_PIN, INPUT);
        uint8_t ack = digitalRead(SDA_PIN);
        pinMode(SDA_PIN, OUTPUT);
        digitalWrite(SDA_PIN, LOW);
        digitalWrite(SCL_PIN, LOW);

        return ack == HIGH;
    };

    /**
     * Read a byte from the I2C bus
     *
     * @param i2c_bus_addr the address of the device to read from
     * @param stop true if we only want to read a single byte
     * @return the value read from the I2C bus
     */
    uint8_t read(uint8_t i2c_bus_addr, bool stop) {
        _begin(i2c_bus_addr, true);

        return _read(stop);
    };

private:
    bool _begin(uint8_t i2c_bus_address, bool read) {
        // i2c start condition
        digitalWrite(SDA_PIN, LOW);
        delayMicroseconds(CLOCK_DELAY);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(CLOCK_DELAY);

        uint8_t b = i2c_bus_address << 1;
        if(read) {
            b = b | 1;
        }

        return write(b);
    };

    uint8_t _read(boolean isLast) {
        uint8_t value = 0;
        pinMode(SDA_PIN, INPUT);
        for(int i=0; i<8; i++) {
            delayMicroseconds(CLOCK_DELAY);
            digitalWrite(SCL_PIN, HIGH);
            value = value << 1;
            if(digitalRead(SDA_PIN)) {
                value = value | 1;
            }
            digitalWrite(SCL_PIN, LOW);
        }
        pinMode(SDA_PIN, OUTPUT);

        if(isLast) {
            digitalWrite(SDA_PIN, HIGH);
        } else {
            digitalWrite(SDA_PIN, LOW);
        }

        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(CLOCK_DELAY);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(CLOCK_DELAY);

        return value;
    };

private:
    bool address_ack;
};

#endif