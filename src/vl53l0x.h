/* C port of the MicroPython vl53l0x.py used by 4ToF.py (the parts it runs:
 * the constructor and ping()).  Errors are raised as in Python (pyexc.h):
 * I2C failures as OSError, a missing sensor as RuntimeError, and the module's
 * own TimeoutError (a RuntimeError subclass, so safe_read() does not catch
 * it) as RuntimeError "TimeoutError". */
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>
#include "machine.h"

typedef struct {
    const soft_i2c_t *i2c;
    uint8_t address;
    uint8_t stop_variable;
    bool started;
    uint32_t measurement_timing_budget_us;
} vl53l0x_t;

/* VL53L0X(i2c) */
void vl53l0x_new(vl53l0x_t *d, const soft_i2c_t *i2c);
/* Long range mode, up to about 2 m (not part of vl53l0x.py). */
void vl53l0x_set_long_range(vl53l0x_t *d);
/* ping(): start(), read(), stop(); returns the range register value (mm) */
int vl53l0x_ping(vl53l0x_t *d);

#endif
