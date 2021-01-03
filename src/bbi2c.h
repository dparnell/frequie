#ifndef _BBI2C_
#define _BBI2C_

#include <stdint.h>
#include <Arduino.h>

template<uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t SPEED>class BBI2C
{
public:
    BBI2C() {};

	void begin() {
        pinMode(SDA_PIN, OUTPUT);
        pinMode(SCL_PIN, OUTPUT);
        digitalWrite(SDA_PIN, HIGH);
        digitalWrite(SCL_PIN, HIGH); 
        delayMicroseconds(10);       
    };

	void end() {
        pinMode(SDA_PIN, INPUT);
        pinMode(SCL_PIN, INPUT);
    };

	void beginTransmission(uint8_t i2c_bus_addr) {
        address_ack = _begin(i2c_bus_addr, false);
    };

	bool endTransmission() {
        // send i2c stop condition
        delayMicroseconds(SPEED);
        digitalWrite(SDA_PIN, LOW);
        delayMicroseconds(SPEED);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(SPEED);
        digitalWrite(SDA_PIN, HIGH);

        return address_ack;
    };

	bool write(uint8_t data) {        
        for(int i=0; i<8; i++) {
            if(data & 0x80) {
                digitalWrite(SDA_PIN, HIGH);
            } else {
                digitalWrite(SDA_PIN, LOW);
            }
            delayMicroseconds(SPEED);
            digitalWrite(SCL_PIN, HIGH);
            delayMicroseconds(SPEED);
            digitalWrite(SCL_PIN, LOW);
            delayMicroseconds(SPEED);
            digitalWrite(SDA_PIN, HIGH);

            data = data << 1;
        }

        // now read the ACK bit from the bus
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(SPEED);
        pinMode(SDA_PIN, INPUT);
        uint8_t ack = digitalRead(SDA_PIN);
        pinMode(SDA_PIN, OUTPUT);
        digitalWrite(SDA_PIN, LOW);
        digitalWrite(SCL_PIN, LOW);

        return ack == HIGH;
    };

	uint8_t read(uint8_t i2c_bus_addr, bool stop) {
        _begin(i2c_bus_addr, true);
        
        return _read(true);
    };

private:
    bool _begin(uint8_t i2c_bus_address, bool read) {
        // i2c start condition
        digitalWrite(SDA_PIN, LOW);
        delayMicroseconds(SPEED);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(SPEED);

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
            delayMicroseconds(SPEED);
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
        delayMicroseconds(SPEED);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(SPEED);

        return value;
    };

private:
    bool address_ack;
};

#endif