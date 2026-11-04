#include "vl53l1x.h"
#include "pyexc.h"

/* Bytes correspond to registers 0x2d-0x87. See the VL53L1X datasheet or
 * https://github.com/drakxtwo/vl53l1x_pico for the per-register comments. */
static const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02,
    0x0a, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
    0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x40
};

static void writeReg(vl53l1x_t *d, uint16_t reg, uint8_t value)
{
    i2c_writeto_mem(d->i2c, d->address, reg, &value, 1, 16);
}

static void writeReg16Bit(vl53l1x_t *d, uint16_t reg, int value)
{
    uint8_t b[2] = { (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF) };
    i2c_writeto_mem(d->i2c, d->address, reg, b, 2, 16);
}

static int readReg16Bit(vl53l1x_t *d, uint16_t reg)
{
    uint8_t b[2];
    i2c_readfrom_mem(d->i2c, d->address, reg, b, 2, 16);
    return (b[0] << 8) + b[1];
}

static void reset(vl53l1x_t *d)
{
    writeReg(d, 0x0000, 0x00);
    sleep_ms(100);                      /* machine.lightsleep(100) */
    writeReg(d, 0x0000, 0x01);
}

void vl53l1x_new(vl53l1x_t *d, const soft_i2c_t *i2c)
{
    d->i2c = i2c;
    d->address = 0x29;
    reset(d);
    sleep_ms(1);                        /* machine.lightsleep(1) */
    if (readReg16Bit(d, 0x010F) != 0xEACC) {
        raise_runtime("Failed to find expected ID register values. Check wiring!");
    }
    i2c_writeto_mem(d->i2c, d->address, 0x2D, VL51L1X_DEFAULT_CONFIGURATION,
                    sizeof VL51L1X_DEFAULT_CONFIGURATION, 16);
    /* The real API triggers this change in VL53L1_init_and_start_range() once
     * a measurement is started; assumes MM1 and MM2 are disabled. */
    writeReg16Bit(d, 0x001E, readReg16Bit(d, 0x0022) * 4);
    sleep_ms(200);                      /* machine.lightsleep(200) */
}

int vl53l1x_read(vl53l1x_t *d)
{
    /* RESULT__RANGE_STATUS block */
    uint8_t data[17];
    i2c_readfrom_mem(d->i2c, d->address, 0x0089, data, 17, 16);
    return (data[13] << 8) + data[14];
}
