#ifndef _WIREI2C_
#define _WIREI2C_

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