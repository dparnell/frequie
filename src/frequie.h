#ifndef _FREQUIE_
#define _FREQUIE_

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

#define SI5351_XTAL_25MHZ 25000000
#define SI5351_XTAL_27MHZ 27000000

#define SI5351_PLL_900MHZ 900000000
#define SI5351_PLL_600MHZ 600000000

#define SI5351_DEVICE_ADDRESS 0x60

template<typename TI2C, uint64_t XTAL_FREQUENCY, uint64_t PLL_FREQUENCY, uint8_t DEVICE_ADDRESS, uint8_t CLOCK_COUNT> class Frequie
{
private:
    int32_t calibration_offset;
public:
    Frequie(int32_t correction = 0)
    {
        i2c.begin();
    }

    /**
     *  Given the crystal error in parts per billion set the calibration offset
     *  If this is called after init it is necessary to call update_pll_divider to actually apply the correction factor
     * 
     *  @param correction the crystal error
     */
    void set_correction(int32_t correction) {
        calibration_offset = (int32_t)((((((int64_t)correction) << 31) / 1000000000LL) * XTAL_FREQUENCY) >> 31);        
    }

    /**
     *  Fetch the calibration offset
     * 
     *  @return the calibration offset
     */
    int32_t get_correction() {
        return calibration_offset;
    }

    /**
     * Determine if the Si5351 is present
     *
     * @return true if device is detected on the I2C bus
     */
    bool detect() {
        i2c.beginTransmission(DEVICE_ADDRESS);
        return !i2c.endTransmission();
    }

    /**
     * Scan the I2C bus looking for connected devices
     *
     * @param out the stream to write the scan results to
     */
    void scan_i2c_bus(Stream &out) {
        out.println("Scanning...");
        out.print("  0123456789ABCDEF");
        for(int i=0; i<128; i++) {
            i2c.beginTransmission(i);
            bool flag = i2c.endTransmission();

            if(i % 16 == 0) {
                out.println();
                out.print(i >> 4, 16);
                out.print(" ");
            }
            out.print(flag ? "." : "+");
        }
        out.println();
    }

    /**
     * Initialize the Si5351 chip
     *
     * @return true if the device could be successfully initialized
     */
    bool init() {
        uint32_t plla_p1;
        uint32_t plla_p2;
        uint32_t plla_p3;
        bool integer_div;

        if(calc_divider_params(PLL_FREQUENCY, XTAL_FREQUENCY + calibration_offset, &plla_p1, &plla_p2, &plla_p3, NULL, &integer_div)) {
            i2c.begin();
            write_register(183, 0x92);  // 8pf caps

            write_register(16, 0x80); // disable clock 0
            write_register(17, 0x80); // disable clock 1
            write_register(18, 0x80); // disable clock 2
            if(CLOCK_COUNT > 3) {
                write_register(19, 0x80); // disable clock 3
                if(CLOCK_COUNT > 4) {
                    write_register(20, 0x80); // disable clock 4
                    if(CLOCK_COUNT > 5) {
                        write_register(21, 0x80); // disable clock 5
                        if(CLOCK_COUNT > 6) {
                            write_register(22, 0x80); // disable clock 6
                            if(CLOCK_COUNT > 7) {
                                write_register(23, 0x80); // disable clock 7
                            }
                        }
                    }
                }
            }

            // write the values for the PLLA dividers
            write_divider_registers(26, plla_p1, plla_p2, plla_p3, 0);

            return !write_register(177, 0xA0); // reset the PLLs
        }

        return false;
    }

    /**
     * Update the PLL dividers to take an updated calibration factor into account
    */
    void update_pll_divider() {
        uint32_t plla_p1;
        uint32_t plla_p2;
        uint32_t plla_p3;
        bool integer_div;

        if(calc_divider_params(PLL_FREQUENCY, XTAL_FREQUENCY + calibration_offset, &plla_p1, &plla_p2, &plla_p3, NULL, &integer_div)) {
            // write the values for the PLLA dividers
            write_divider_registers(26, plla_p1, plla_p2, plla_p3, 0);

            write_register(177, 0xA0); // reset the PLLs
        }

    }

    /**
     * Configure the given clock generator to produce the given frequency
     *
     * @param clock the zero based clock generator
     * @param frequency the desired output frequency in Hz
     */
    bool set_clock_frequency(uint8_t clock, uint64_t frequency) {
        uint32_t p1;
        uint32_t p2;
        uint32_t p3;
        uint8_t r;
        bool is_integer;

        if(clock < CLOCK_COUNT) {
            if(calc_divider_params(PLL_FREQUENCY, frequency, &p1, &p2, &p3, &r, &is_integer)) {
                uint8_t base = 42 + clock * 8;

                write_divider_registers(base, p1, p2, p3, r);

                return true;
            }
        }

        return false;
    }

    /**
     * Enable the given clock generator
     *
     * @param clock the zero based clock generator to enable
     */
    void enable_clock(uint8_t clock) {
        if(clock < CLOCK_COUNT) {
            write_register(16 + clock, 0x0C);
        }
    }

    /**
     * Disable the given clock generator
     *
     * @param clock the zero based clock generator to disable
     */
    void disable_clock(uint8_t clock) {
        if(clock < CLOCK_COUNT) {
            write_register(16 + clock, 0x80);
        }
    }

    /**
     * write the given value to a device register
     *
     * @param reg the device register to write to
     * @param value the value to write to the register
     */
    bool write_register(uint8_t reg, uint8_t value) {
        i2c.beginTransmission(DEVICE_ADDRESS);
        i2c.write(reg);
        i2c.write(value);
        return i2c.endTransmission();
    }

    /**
     * write the given values to the divider registers at the given base address
     * @param base the base address of the divider
     * @param p1 the p1 value
     * @param p2 the p2 value
     * @param p3 the p3 value
     * @param r the r value (only valid for clocks)
     */
    void write_divider_registers(uint8_t base, uint32_t p1, uint32_t p2, uint32_t p3, uint8_t r) {
        uint8_t r_value = r << 4;
        if(r == 4) {
            r_value = r_value | 0b1100; // it is a divide by 4, so the datasheet says we need to set this as well
        }

        write_register(base + 0, (p3 >> 8) & 0xff); // P3[15:8]
        write_register(base + 1, p3 & 0xff); // P3[7:0]
        write_register(base + 2, r_value | ((p1 >> 16) & 0x03)); // P1[17:16]
        write_register(base + 3, (p1 >> 8) & 0xff); // P1[15:8]
        write_register(base + 4, p1 & 0xff); // P1[7:0]
        write_register(base + 5, ((p3 >> 12) & 0xf0) | ((p2 >> 16) & 0x0f)); // P3[19:16], P2[19:16]
        write_register(base + 6, (p2 >> 8) & 0xff); // P2[15:8]
        write_register(base + 7, p2 & 0xff); // P2[7:0]
    }

    /**
     *  Calculate the register values for the given divider
     *
     * @param f1 the top frequency
     * @param f2 the bottom frequency
     * @param p1 divider register value output
     * @param p2 divider register value output
     * @param p3 divider register value output
     * @param r null if there we are calculating the register values for a PLL
     * @param is_integer set to true if
     *
     * @return true if the requested frequency is possible and the outputs have been updated with the calculated values
     */
    bool calc_divider_params(uint64_t f1, uint64_t f2, uint32_t *p1, uint32_t *p2, uint32_t *p3, uint8_t *r, bool *is_integer)
    {
        uint64_t freq = f2;
        double nTmp = (double)f1 / (double)freq;
        uint8_t rTmp = 0;

        if(!r) {
            // we are calculating the divider for the PLL so there is no extra dividers

            if(nTmp < 15 || nTmp > 90) {
                // this value is outside the allowed range
                return false;
            }
        } else {
            // we are calculating the divider for a clock
            while((nTmp < 8 || nTmp > 2048) && rTmp <= 7) {
                rTmp = rTmp + 1;
                freq = freq * 2;

                nTmp = (double)f1 / (double)freq;
            }

            if(rTmp > 7) {
                // we can not do the requested frequency
                return false;
            }

            *r = rTmp;
        }

        // now calculate a, b and c
        uint32_t a = (uint32_t)nTmp;
        uint32_t b;
        uint32_t c;

        uint64_t rem = f1 % freq;
        if(rem == 0) {
            if(is_integer) {
                // it is an even integer
                *is_integer = (a & 1) == 0;
            }

            b = 0;
            c = 1;
        } else {
            if(is_integer) {
                *is_integer = false;
            }

            // we have a fractional divider, work out the divider values by calculating the
            // greatest common divisor for the values that will go into b and c
            uint64_t tmpA = rem;
            uint64_t tmpB = freq;
            while(tmpB != 0) {
                uint64_t nextB = tmpA % tmpB;
                tmpA = tmpB;
                tmpB = nextB;
            }

            b = (uint32_t)(rem / tmpA);
            c = (uint32_t)(freq / tmpA);
        }

        // now calculate the register values p1, p2 and p3
        uint32_t fraction_term = (uint32_t)(128.0 * (double)b / (double)c);

        *p1 = 128 * a + fraction_term - 512;
        *p2 = 128 * b - c * fraction_term;
        *p3 = c;

        return true;
    }

    TI2C i2c;
};

#endif