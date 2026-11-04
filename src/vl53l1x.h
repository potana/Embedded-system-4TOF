/* C port of the MicroPython vl53l1x.py used by 4ToF.py.
 *
 * Like the Python: the constructor resets the sensor and writes ST's default
 * configuration (registers 0x2D..0x87, whose last byte starts ranging), and
 * read() only reads the result registers - it does not start a new
 * measurement or clear the interrupt. */
#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdint.h>
#include "machine.h"

typedef struct {
    const soft_i2c_t *i2c;
    uint8_t address;
} vl53l1x_t;

/* VL53L1X(i2c) */
void vl53l1x_new(vl53l1x_t *d, const soft_i2c_t *i2c);
/* read(): final crosstalk corrected range of SD0, in mm */
int vl53l1x_read(vl53l1x_t *d);

#endif
