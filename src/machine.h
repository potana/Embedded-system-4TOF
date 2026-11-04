/* The parts of MicroPython's `machine` module that 4ToF.py uses, ported to
 * the pico-sdk with the same pin handling and bit timing:
 *
 *   Pin        - rp2 port machine_pin.c
 *   SoftSPI    - drivers/bus/softspi.c (polarity 0, phase 0, MSB first)
 *   SoftI2C    - extmod/machine_i2c.c (default freq 400 kHz, timeout 50 ms)
 *
 * MicroPython is MIT licensed (see README.md).  I2C methods raise OSError
 * (pyexc.h) exactly where MicroPython would.
 */
#ifndef MACHINE_H
#define MACHINE_H

#include <stddef.h>
#include <stdint.h>
#include "pico/stdlib.h"

/* Pin(n, Pin.OUT) */
void pin_init_out(uint pin);
/* Pin(n, Pin.IN, Pin.PULL_UP) */
void pin_init_in_pullup(uint pin);

typedef struct {
    uint sck, mosi, miso;
    uint32_t delay_half;        /* us */
} soft_spi_t;

/* SoftSPI(baudrate=..., polarity=0, phase=0, sck=, mosi=, miso=) */
void soft_spi_init(soft_spi_t *spi, uint32_t baudrate, uint sck, uint mosi, uint miso);
/* spi.write(buf) */
void soft_spi_write(const soft_spi_t *spi, const uint8_t *buf, size_t len);

typedef struct {
    uint scl, sda;
    uint32_t us_delay;          /* half clock period */
    uint32_t us_timeout;        /* clock stretching limit */
} soft_i2c_t;

/* SoftI2C(sda=Pin(sda), scl=Pin(scl)) */
void soft_i2c_init(soft_i2c_t *i2c, uint sda, uint scl);

/* i2c.readfrom_mem(addr, memaddr, len, addrsize=addrsize) */
void i2c_readfrom_mem(const soft_i2c_t *i2c, uint8_t addr, uint32_t memaddr,
                      uint8_t *buf, size_t len, int addrsize);
/* i2c.writeto_mem(addr, memaddr, buf, addrsize=addrsize) */
void i2c_writeto_mem(const soft_i2c_t *i2c, uint8_t addr, uint32_t memaddr,
                     const uint8_t *buf, size_t len, int addrsize);

#endif
