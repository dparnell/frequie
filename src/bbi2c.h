#ifndef _BBI2C_
#define _BBI2C_

#include <stdint.h>
#include <Arduino.h>

#define SPEED 1

#define SDA_HIGH(pin) digitalWrite(pin, HIGH)
#define SDA_LOW(pin) digitalWrite(pin, LOW)
#define SCL_HIGH(pin) digitalWrite(pin, HIGH)
#define SCL_LOW(pin) digitalWrite(pin, LOW)

class BBI2C
{
public:
    BBI2C(uint8_t sda_pin, uint8_t scl_pin) :
        sda(sda_pin), scl(scl_pin)
    {

    };

	void begin() {
        pinMode(sda, OUTPUT);
        pinMode(scl, OUTPUT);
        digitalWrite(sda, HIGH);
        digitalWrite(scl, HIGH); 
        delayMicroseconds(10);       
    };

	void end() {
        pinMode(sda, INPUT);
        pinMode(scl, INPUT);
    };

	bool beginTransmission(uint8_t i2c_bus_addr) {
        return _begin(i2c_bus_addr, false);
    };

	void endTransmission() {
        // send i2c stop condition
        delayMicroseconds(SPEED);
        SDA_LOW(sda);
        delayMicroseconds(SPEED);
        SCL_HIGH(scl);
        delayMicroseconds(SPEED);
        SDA_HIGH(sda);
    };

	bool write(uint8_t data) {        
        for(int i=0; i<8; i++) {
            if(data & 0x80) {
                SDA_HIGH(sda);
            } else {
                SDA_LOW(sda);
            }
            delayMicroseconds(SPEED);
            SCL_HIGH(scl);
            delayMicroseconds(SPEED);
            SCL_LOW(scl);
            delayMicroseconds(SPEED);
            SDA_LOW(sda);

            data = data << 1;
        }

        // now for the ACK bit
        SCL_HIGH(scl);
        delayMicroseconds(SPEED);
        pinMode(sda, INPUT);
        uint8_t ack = digitalRead(sda);
        pinMode(sda, OUTPUT);
        SCL_LOW(scl);

        return ack == 1;
    };

	uint8_t read(uint8_t i2c_bus_addr, bool stop) {
        _begin(i2c_bus_addr, true);
        
        return _read(true);
    };

private:
    uint8_t sda;
    uint8_t scl;

    bool _begin(uint8_t i2c_bus_address, bool read) {
        // i2c start condition
        SDA_LOW(sda);
        delayMicroseconds(SPEED);
        SCL_LOW(scl);
        delayMicroseconds(SPEED);

        uint8_t b = i2c_bus_address << 1;
        if(read) {
            b = b | 1;
        }

        return write(b);
    };

    uint8_t _read(boolean isLast) {
        uint8_t value = 0;
        pinMode(sda, INPUT);
        for(int i=0; i<8; i++) {
            delayMicroseconds(SPEED);
            SCL_HIGH(scl);
            value = value << 1;
            if(digitalRead(sda)) {
                value = value | 1;
            }
            SCL_LOW(scl);
        }
        pinMode(sda, OUTPUT);

        if(isLast) {
            SDA_HIGH(sda);
        } else {
            SDA_LOW(sda);
        }

        SCL_HIGH(scl);
        delayMicroseconds(SPEED);
        SCL_LOW(scl);
        delayMicroseconds(SPEED);

        return value;
    };
};

#endif