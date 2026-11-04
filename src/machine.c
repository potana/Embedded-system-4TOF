#include "machine.h"
#include "pyexc.h"
#include "hardware/gpio.h"

/* ---- Pin ------------------------------------------------------------------ */

void pin_init_out(uint pin)
{
    gpio_init(pin);
    gpio_disable_pulls(pin);     /* rp2 Pin() sets "no pull" unless asked */
    gpio_set_dir(pin, GPIO_OUT);
}

void pin_init_in_pullup(uint pin)
{
    gpio_init(pin);
    gpio_set_pulls(pin, true, false);
}

/* ---- SoftSPI (drivers/bus/softspi.c) -------------------------------------- */

void soft_spi_init(soft_spi_t *spi, uint32_t baudrate, uint sck, uint mosi, uint miso)
{
    spi->sck = sck;
    spi->mosi = mosi;
    spi->miso = miso;
    /* baudrate_to_delay_half(): round up so the real rate is not above the
     * requested one; 100000 -> 5 us */
    spi->delay_half = 500000 / baudrate;
    if (500000 % baudrate != 0) spi->delay_half += 1;

    /* MP_SPI_IOCTL_INIT: SCK = polarity (0), SCK and MOSI outputs, MISO input.
     * Pin(n) without arguments leaves the pad pulls as they are. */
    gpio_set_function(sck, GPIO_FUNC_SIO);
    gpio_put(sck, 0);
    gpio_set_dir(sck, GPIO_OUT);
    gpio_set_function(mosi, GPIO_FUNC_SIO);
    gpio_set_dir(mosi, GPIO_OUT);
    gpio_set_function(miso, GPIO_FUNC_SIO);
    gpio_set_dir(miso, GPIO_IN);
}

void soft_spi_write(const soft_spi_t *spi, const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t data_out = buf[i];
        for (int j = 0; j < 8; j++, data_out <<= 1) {
            gpio_put(spi->mosi, (data_out >> 7) & 1);
            busy_wait_us_32(spi->delay_half);
            gpio_put(spi->sck, 1);
            (void)gpio_get(spi->miso);       /* MicroPython samples MISO here */
            busy_wait_us_32(spi->delay_half);
            gpio_put(spi->sck, 0);
        }
    }
}

/* ---- SoftI2C (extmod/machine_i2c.c) --------------------------------------- */

/* open drain: a line is driven low or released (input); output value is 0 */
static void od_low(uint pin)  { gpio_set_dir(pin, GPIO_OUT); }
static void od_high(uint pin) { gpio_set_dir(pin, GPIO_IN); }

static void i2c_delay(const soft_i2c_t *s) { busy_wait_us_32(s->us_delay); }

static void scl_low(const soft_i2c_t *s) { od_low(s->scl); }

static int scl_release(const soft_i2c_t *s)
{
    uint32_t count = s->us_timeout;
    od_high(s->scl);
    i2c_delay(s);
    /* clock stretching: wait for SCL to rise, with timeout */
    for (; gpio_get(s->scl) == 0 && count; --count) {
        busy_wait_us_32(1);
    }
    if (count == 0) return -PY_ETIMEDOUT;
    return 0;
}

static void sda_low(const soft_i2c_t *s)     { od_low(s->sda); }
static void sda_release(const soft_i2c_t *s) { od_high(s->sda); }
static int  sda_read(const soft_i2c_t *s)    { return gpio_get(s->sda); }

static int i2c_start(const soft_i2c_t *s)
{
    sda_release(s);
    i2c_delay(s);
    int ret = scl_release(s);
    if (ret != 0) return ret;
    sda_low(s);
    i2c_delay(s);
    return 0;
}

static int i2c_stop(const soft_i2c_t *s)
{
    i2c_delay(s);
    sda_low(s);
    i2c_delay(s);
    int ret = scl_release(s);
    sda_release(s);
    i2c_delay(s);
    return ret;
}

void soft_i2c_init(soft_i2c_t *s, uint sda, uint scl)
{
    s->sda = sda;
    s->scl = scl;
    s->us_delay = 500000 / 400000;        /* freq = 400 kHz -> 1 us */
    if (s->us_delay == 0) s->us_delay = 1;
    s->us_timeout = 50000;                /* timeout = 50000 us */

    /* mp_hal_pin_open_drain(): released, output value 0, pulls unchanged */
    const uint pins[2] = { scl, sda };
    for (int i = 0; i < 2; i++) {
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_put(pins[i], 0);
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
    }
    i2c_stop(s);                          /* ignore error */
}

/* 0 = ack, 1 = nack, < 0 = -errno */
static int write_byte(const soft_i2c_t *s, uint8_t val)
{
    i2c_delay(s);
    scl_low(s);

    for (int i = 7; i >= 0; i--) {
        if ((val >> i) & 1) sda_release(s);
        else                sda_low(s);
        i2c_delay(s);
        int ret = scl_release(s);
        if (ret != 0) {
            sda_release(s);
            return ret;
        }
        scl_low(s);
    }

    sda_release(s);
    i2c_delay(s);
    int ret = scl_release(s);
    if (ret != 0) return ret;

    int ack = sda_read(s);
    i2c_delay(s);
    scl_low(s);
    return ack;
}

/* 0 = ok, < 0 = -errno */
static int read_byte(const soft_i2c_t *s, uint8_t *val, int nack)
{
    i2c_delay(s);
    scl_low(s);
    i2c_delay(s);

    uint8_t data = 0;
    for (int i = 7; i >= 0; i--) {
        int ret = scl_release(s);
        if (ret != 0) return ret;
        data = (uint8_t)((data << 1) | sda_read(s));
        scl_low(s);
        i2c_delay(s);
    }
    *val = data;

    /* send ack/nack bit */
    if (!nack) sda_low(s);
    i2c_delay(s);
    int ret = scl_release(s);
    if (ret != 0) {
        sda_release(s);
        return ret;
    }
    scl_low(s);
    sda_release(s);
    return 0;
}

typedef struct { size_t len; uint8_t *buf; } i2c_buf_t;

#define FLAG_READ 0x01
#define FLAG_STOP 0x02

/* mp_machine_soft_i2c_transfer(): >= 0 = number of acks (writes) or 0
 * (reads), < 0 = -errno */
static int transfer(const soft_i2c_t *s, uint8_t addr, size_t n, i2c_buf_t *bufs, unsigned flags)
{
    int ret = i2c_start(s);
    if (ret != 0) return ret;

    ret = write_byte(s, (uint8_t)((addr << 1) | (flags & FLAG_READ)));
    if (ret < 0) {
        return ret;
    } else if (ret != 0) {
        /* nack received, release the bus cleanly */
        i2c_stop(s);
        return -PY_ENODEV;
    }

    int transfer_ret = 0;
    for (; n--; ++bufs) {
        size_t len = bufs->len;
        uint8_t *buf = bufs->buf;
        if (flags & FLAG_READ) {
            while (len--) {
                ret = read_byte(s, buf++, (n | len) == 0);
                if (ret != 0) return ret;
            }
        } else {
            while (len--) {
                ret = write_byte(s, *buf++);
                if (ret < 0) {
                    return ret;
                } else if (ret != 0) {
                    /* nack received, stop sending */
                    n = 0;
                    break;
                }
                ++transfer_ret;
            }
        }
    }

    if (flags & FLAG_STOP) {
        ret = i2c_stop(s);
        if (ret != 0) return ret;
    }
    return transfer_ret;
}

static size_t fill_memaddr_buf(uint8_t *buf, uint32_t memaddr, int addrsize)
{
    size_t n = 0;
    for (int16_t i = (int16_t)(addrsize - 8); i >= 0; i -= 8) {
        buf[n++] = (uint8_t)(memaddr >> i);
    }
    return n;
}

void i2c_readfrom_mem(const soft_i2c_t *s, uint8_t addr, uint32_t memaddr,
                      uint8_t *buf, size_t len, int addrsize)
{
    uint8_t memaddr_buf[4];
    i2c_buf_t wr = { fill_memaddr_buf(memaddr_buf, memaddr, addrsize), memaddr_buf };
    int ret = transfer(s, addr, 1, &wr, 0);          /* no STOP: repeated start */
    if (ret != (int)wr.len) {
        /* must generate STOP */
        i2c_buf_t none = { 0, NULL };
        transfer(s, addr, 1, &none, FLAG_STOP);
    } else {
        i2c_buf_t rd = { len, buf };
        ret = transfer(s, addr, 1, &rd, FLAG_READ | FLAG_STOP);
    }
    if (ret < 0) raise_oserror(-ret);
}

void i2c_writeto_mem(const soft_i2c_t *s, uint8_t addr, uint32_t memaddr,
                     const uint8_t *buf, size_t len, int addrsize)
{
    uint8_t memaddr_buf[4];
    i2c_buf_t bufs[2] = {
        { fill_memaddr_buf(memaddr_buf, memaddr, addrsize), memaddr_buf },
        { len, (uint8_t *)buf },
    };
    int ret = transfer(s, addr, 2, bufs, FLAG_STOP);
    if (ret < 0) raise_oserror(-ret);
}
