/* SerialFlash Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/PaulStoffregen/SerialFlash
 * Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy.
 * Please support PJRC's efforts to develop open source software by purchasing
 * Teensy or other genuine PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SerialFlash_h_
#define SerialFlash_h_

#include <Arduino.h>
#include <api/BlockDevice.h>
#include <SPI.h>
#include "util/SerialFlash_directwrite.h"

class SerialFlashChip : public arduino::BlockDevice
{
public:
	SerialFlashChip(SPIClass& device = SPI, uint8_t pin = 6) : SPIPORT(device), pin(pin) {}
	bool begin(SPIClass& device, uint8_t pin = 6);
	bool begin(uint8_t pin = 6);
	uint32_t capacity(const uint8_t *id);
	uint32_t blockSize() const;
	void sleep();
	void wakeup();
	void readID(uint8_t *buf);
	void readSerialNumber(uint8_t *buf);
	void read(uint32_t addr, void *buf, uint32_t len);
	bool ready();
	void wait();
	void write(uint32_t addr, const void *buf, uint32_t len);
	void eraseAll();
	void eraseBlock(uint32_t addr);

	virtual int init() {
		return (begin(pin) != true);
	}
    virtual int deinit() {
    	return 0;
    }

    virtual int read(void *buffer, bd_addr_t addr, bd_size_t size) {
    	read(addr, buffer, size);
    	return size;
    }

    virtual int program(const void *buffer, bd_addr_t addr, bd_size_t size) {
    	write(addr, buffer, size);
    	return size;
    }

    virtual int erase(bd_addr_t addr, bd_size_t size)
    {
        for (uint32_t i = 0; i < size / blockSize(); i++) {
        	eraseBlock(addr + i * blockSize());
        }
        return 0;
    }

    virtual int trim(bd_addr_t addr, bd_size_t size)
    {
        return 0;
    }

    virtual bd_size_t get_read_size() const {
    	return blockSize();
    }

    virtual bd_size_t get_program_size() const {
    	return blockSize();
    }

    virtual bd_size_t size() const {
		return _size;
    };

    virtual const char *get_type() const {
    	return "SerialFlashChip";
    };

private:
	uint16_t dirindex = 0; // current position for readdir()
	uint8_t flags = 0;	// chip features
	uint8_t busy = 0;	// 0 = ready
				// 1 = suspendable program operation
				// 2 = suspendable erase operation
				// 3 = busy for realz!!
	uint32_t _size;
	SPIClass& SPIPORT;
	volatile IO_REG_TYPE *cspin_basereg;
	IO_REG_TYPE cspin_bitmask;
	uint8_t pin;
};

extern SerialFlashChip SerialFlash;


#endif
