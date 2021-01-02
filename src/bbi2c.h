#ifndef _BBI2C_
#define _BBI2C_

#include <stdint.h>
#include <Arduino.h>

#define SDA_HIGH() digitalWrite(SDA_PIN, HIGH)
#define SDA_LOW() digitalWrite(SDA_PIN, LOW)
#define SCL_HIGH() digitalWrite(SCL_PIN, HIGH)
#define SCL_LOW() digitalWrite(SCL_PIN, LOW)

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
        SDA_LOW();
        delayMicroseconds(SPEED);
        SCL_HIGH();
        delayMicroseconds(SPEED);
        SDA_HIGH();

        return address_ack;
    };

	bool write(uint8_t data) {        
        for(int i=0; i<8; i++) {
            if(data & 0x80) {
                SDA_HIGH();
            } else {
                SDA_LOW();
            }
            delayMicroseconds(SPEED);
            SCL_HIGH();
            delayMicroseconds(SPEED);
            SCL_LOW();
            delayMicroseconds(SPEED);
            SDA_HIGH();

            data = data << 1;
        }

        // now read the ACK bit from the bus
        SCL_HIGH();
        delayMicroseconds(SPEED);
        pinMode(SDA_PIN, INPUT);
        uint8_t ack = digitalRead(SDA_PIN);
        pinMode(SDA_PIN, OUTPUT);
        SDA_LOW();
        SCL_LOW();

        return ack == HIGH;
    };

	uint8_t read(uint8_t i2c_bus_addr, bool stop) {
        _begin(i2c_bus_addr, true);
        
        return _read(true);
    };

private:
    bool _begin(uint8_t i2c_bus_address, bool read) {
        // i2c start condition
        SDA_LOW();
        delayMicroseconds(SPEED);
        SCL_LOW();
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
            SCL_HIGH();
            value = value << 1;
            if(digitalRead(SDA_PIN)) {
                value = value | 1;
            }
            SCL_LOW();
        }
        pinMode(SDA_PIN, OUTPUT);

        if(isLast) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }

        SCL_HIGH();
        delayMicroseconds(SPEED);
        SCL_LOW();
        delayMicroseconds(SPEED);

        return value;
    };

private:
    bool address_ack;
};

#endif