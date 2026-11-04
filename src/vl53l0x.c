#include "vl53l0x.h"
#include "pyexc.h"

#define IO_TIMEOUT               1000
#define SYSRANGE_START           0x00
#define EXTSUP_HV                0x89
#define MSRC_CONFIG              0x60
#define FINAL_RATE_RTN_LIMIT     0x44
#define SYSTEM_SEQUENCE          0x01
#define SPAD_REF_START           0x4f
#define SPAD_ENABLES             0xb0
#define REF_EN_START_SELECT      0xb6
#define SPAD_NUM_REQUESTED       0x4e
#define INTERRUPT_GPIO           0x0a
#define INTERRUPT_CLEAR          0x0b
#define GPIO_MUX_ACTIVE_HIGH     0x84
#define RESULT_INTERRUPT_STATUS  0x13
#define RESULT_RANGE_STATUS      0x14

/* ---- register helpers (_register / _registers / _flag / _config) -------- */

static uint8_t rd8(vl53l0x_t *d, uint8_t reg)
{
    uint8_t v;
    i2c_readfrom_mem(d->i2c, d->address, reg, &v, 1, 8);
    return v;
}

static void wr8(vl53l0x_t *d, uint8_t reg, uint8_t v)
{
    i2c_writeto_mem(d->i2c, d->address, reg, &v, 1, 8);
}

/* struct='>H' */
static uint16_t rd16(vl53l0x_t *d, uint8_t reg)
{
    uint8_t b[2];
    i2c_readfrom_mem(d->i2c, d->address, reg, b, 2, 8);
    return (uint16_t)((b[0] << 8) | b[1]);
}

static void wr16(vl53l0x_t *d, uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    i2c_writeto_mem(d->i2c, d->address, reg, b, 2, 8);
}

static void flag(vl53l0x_t *d, uint8_t reg, int bit, bool value)
{
    uint8_t data = rd8(d, reg);
    uint8_t mask = (uint8_t)(1u << bit);
    if (value) data |= mask;
    else       data &= (uint8_t)~mask;
    wr8(d, reg, data);
}

typedef struct { uint8_t reg, val; } regval_t;

static void config(vl53l0x_t *d, const regval_t *list, size_t n)
{
    for (size_t i = 0; i < n; i++) wr8(d, list[i].reg, list[i].val);
}

#define CONFIG(d, ...) do { const regval_t l_[] = { __VA_ARGS__ }; \
                            config((d), l_, sizeof l_ / sizeof l_[0]); } while (0)

/* ---- init ---------------------------------------------------------------- */

static void spad_info(vl53l0x_t *d, int *count, bool *is_aperture)
{
    CONFIG(d, {0x80, 0x01}, {0xff, 0x01}, {0x00, 0x00}, {0xff, 0x06});
    flag(d, 0x83, 3, true);
    CONFIG(d, {0xff, 0x07}, {0x81, 0x01}, {0x80, 0x01}, {0x94, 0x6b}, {0x83, 0x00});
    int timeout;
    for (timeout = 0; timeout < IO_TIMEOUT; timeout++) {
        if (rd8(d, 0x83)) break;
        sleep_ms(1);
    }
    if (timeout == IO_TIMEOUT) raise_runtime("TimeoutError");
    CONFIG(d, {0x83, 0x01});
    uint8_t value = rd8(d, 0x92);
    CONFIG(d, {0x81, 0x00}, {0xff, 0x06});
    flag(d, 0x83, 3, false);
    CONFIG(d, {0xff, 0x01}, {0x00, 0x01}, {0xff, 0x00}, {0x80, 0x00});
    *count = value & 0x7f;
    *is_aperture = (value & 0x80) != 0;
}

static void calibrate(vl53l0x_t *d, uint8_t vhv_init_byte)
{
    wr8(d, SYSRANGE_START, (uint8_t)(0x01 | vhv_init_byte));
    volatile int timeout;
    for (timeout = 0; timeout < IO_TIMEOUT; timeout++) {
        exc_frame_t f;
        volatile bool done = false;
        exc_push(&f);
        if (setjmp(f.buf) == 0) {
            uint8_t inter_status = rd8(d, RESULT_INTERRUPT_STATUS);
            if (inter_status & 0x07) done = true;
            exc_pop(&f);
        } else {
            if (f.type != EXC_OSERROR) exc_reraise(&f);
            sleep_ms((uint32_t)timeout);         /* except OSError: */
        }
        if (done) break;
        sleep_ms(1);
    }
    if (timeout == IO_TIMEOUT) raise_runtime("TimeoutError: Is XShut pulled up");
    wr8(d, INTERRUPT_CLEAR, 0x01);
    wr8(d, SYSRANGE_START, 0x00);
}

static void init(vl53l0x_t *d, bool power2v8)
{
    /* validate in the same way the adafruit code does */
    if (rd8(d, 0xC0) != 0xEE || rd8(d, 0xC1) != 0xAA || rd8(d, 0xC2) != 0x10) {
        raise_runtime("Failed to find expected ID register values. Check wiring!");
    }

    flag(d, EXTSUP_HV, 0, power2v8);

    /* I2C standard mode */
    CONFIG(d, {0x88, 0x00}, {0x80, 0x01}, {0xff, 0x01}, {0x00, 0x00});
    d->stop_variable = rd8(d, 0x91);
    CONFIG(d, {0x00, 0x01}, {0xff, 0x00}, {0x80, 0x00});

    /* disable signal_rate_msrc and signal_rate_pre_range limit checks */
    flag(d, MSRC_CONFIG, 1, true);
    flag(d, MSRC_CONFIG, 4, true);

    /* rate_limit: int(0.1 * (1 << 7)) = 12 */
    wr16(d, FINAL_RATE_RTN_LIMIT, 12);

    wr8(d, SYSTEM_SEQUENCE, 0xff);

    int spad_count;
    bool is_aperture;
    spad_info(d, &spad_count, &is_aperture);
    uint8_t spad_map[6];
    i2c_readfrom_mem(d->i2c, d->address, SPAD_ENABLES, spad_map, 6, 8);

    /* set reference spads */
    CONFIG(d, {0xff, 0x01}, {SPAD_REF_START, 0x00}, {SPAD_NUM_REQUESTED, 0x2c},
              {0xff, 0x00}, {REF_EN_START_SELECT, 0xb4});

    /* Same loop as the Python, including its bit index (i >> 2, not i % 8);
     * bits above 7 do not change a byte. */
    int spads_enabled = 0;
    for (int i = 0; i < 48; i++) {
        uint32_t mask = 1u << (i >> 2);
        if ((i < 12 && is_aperture) || spads_enabled >= spad_count) {
            spad_map[i / 8] &= (uint8_t)~mask;
        } else if (spad_map[i / 8] & mask) {
            spads_enabled += 1;
        }
    }

    i2c_writeto_mem(d->i2c, d->address, SPAD_ENABLES, spad_map, 6, 8);

    CONFIG(d,
        {0xff, 0x01}, {0x00, 0x00},

        {0xff, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},

        {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00},

        {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20},

        {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03},
        {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06},
        {0x51, 0x00}, {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
        {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},

        {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF}, {0x4A, 0x00},

        {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21},

        {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26},
        {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40},

        {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},

        {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04},

        {0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
        {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00},
        {0x77, 0x00},

        {0xFF, 0x01}, {0x0D, 0x01},

        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},

        {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    );

    wr8(d, INTERRUPT_GPIO, 0x04);
    flag(d, GPIO_MUX_ACTIVE_HIGH, 4, false);
    wr8(d, INTERRUPT_CLEAR, 0x01);

    wr8(d, SYSTEM_SEQUENCE, 0x01);
    calibrate(d, 0x40);
    wr8(d, SYSTEM_SEQUENCE, 0x02);
    calibrate(d, 0x00);

    wr8(d, SYSTEM_SEQUENCE, 0xe8);
}

void vl53l0x_new(vl53l0x_t *d, const soft_i2c_t *i2c)
{
    d->i2c = i2c;
    d->address = 0x29;
    sleep_ms(100);                      /* give the I2C time to init */
    init(d, true);
    d->started = false;
    d->measurement_timing_budget_us = 0;
    /* set_measurement_timing_budget(0) returns False straight away
     * (0 < 20000 us minimum), so it does nothing. */
}

/* ---- long range mode (not in vl53l0x.py) ---------------------------------
 *
 * Reaches about 2 m instead of about 1.2 m: signal rate limit 0.1 MCPS (init
 * above already sets it) and VCSEL pulse periods of 18 / 14 PCLKs.  The timing
 * helpers and the period change follow the Pololu VL53L0X library, which is
 * based on ST's API. */

#define PRE_RANGE_VALID_PHASE_LOW     0x56
#define PRE_RANGE_VALID_PHASE_HIGH    0x57
#define PRE_RANGE_VCSEL_PERIOD        0x50
#define PRE_RANGE_TIMEOUT_MACROP_HI   0x51
#define MSRC_TIMEOUT_MACROP           0x46
#define FINAL_RANGE_VALID_PHASE_LOW   0x47
#define FINAL_RANGE_VALID_PHASE_HIGH  0x48
#define FINAL_RANGE_VCSEL_PERIOD      0x70
#define FINAL_RANGE_TIMEOUT_MACROP_HI 0x71
#define GLOBAL_VCSEL_WIDTH            0x32
#define ALGO_PHASECAL_TIMEOUT         0x30
#define ALGO_PHASECAL_LIM             0x30   /* on register page 1 */

#define LONG_RANGE_BUDGET_US 33000

#define decode_vcsel_period(reg_val)   (((reg_val) + 1) << 1)
#define encode_vcsel_period(pclks)     (((pclks) >> 1) - 1)
#define calc_macro_period(vcsel_pclks) ((((uint32_t)2304 * (vcsel_pclks) * 1655) + 500) / 1000)

static uint32_t timeout_mclks_to_us(uint16_t mclks, uint8_t vcsel_pclks)
{
    uint32_t macro_ns = calc_macro_period(vcsel_pclks);
    return (((uint32_t)mclks * macro_ns) + 500) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t us, uint8_t vcsel_pclks)
{
    uint32_t macro_ns = calc_macro_period(vcsel_pclks);
    return ((us * 1000) + (macro_ns / 2)) / macro_ns;
}

static uint16_t decode_timeout(uint16_t reg_val)
{
    /* format: (LSByte * 2^MSByte) + 1 */
    return (uint16_t)(((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1);
}

static uint16_t encode_timeout(uint32_t mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    if (mclks > 0) {
        ls_byte = mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFF));
    }
    return 0;
}

typedef struct { bool tcc, msrc, dss, pre_range, final_range; } step_enables_t;

typedef struct {
    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} step_timeouts_t;

static void get_step_enables(vl53l0x_t *d, step_enables_t *e)
{
    uint8_t cfg = rd8(d, SYSTEM_SEQUENCE);
    e->tcc         = (cfg >> 4) & 1;
    e->dss         = (cfg >> 3) & 1;
    e->msrc        = (cfg >> 2) & 1;
    e->pre_range   = (cfg >> 6) & 1;
    e->final_range = (cfg >> 7) & 1;
}

static void get_step_timeouts(vl53l0x_t *d, const step_enables_t *e, step_timeouts_t *t)
{
    t->pre_range_vcsel_pclks = decode_vcsel_period(rd8(d, PRE_RANGE_VCSEL_PERIOD));
    uint16_t msrc_mclks = (uint16_t)(rd8(d, MSRC_TIMEOUT_MACROP) + 1);
    t->msrc_dss_tcc_us = timeout_mclks_to_us(msrc_mclks, (uint8_t)t->pre_range_vcsel_pclks);

    t->pre_range_mclks = decode_timeout(rd16(d, PRE_RANGE_TIMEOUT_MACROP_HI));
    t->pre_range_us = timeout_mclks_to_us(t->pre_range_mclks, (uint8_t)t->pre_range_vcsel_pclks);

    t->final_range_vcsel_pclks = decode_vcsel_period(rd8(d, FINAL_RANGE_VCSEL_PERIOD));
    t->final_range_mclks = decode_timeout(rd16(d, FINAL_RANGE_TIMEOUT_MACROP_HI));
    if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
    t->final_range_us = timeout_mclks_to_us(t->final_range_mclks, (uint8_t)t->final_range_vcsel_pclks);
}

static void set_timing_budget_us(vl53l0x_t *d, uint32_t budget_us)
{
    const uint32_t StartOverhead = 1910, EndOverhead = 960, MsrcOverhead = 660,
                   TccOverhead = 590, DssOverhead = 690, PreRangeOverhead = 660,
                   FinalRangeOverhead = 550;
    step_enables_t e;
    step_timeouts_t t;
    uint32_t used = StartOverhead + EndOverhead;

    get_step_enables(d, &e);
    get_step_timeouts(d, &e, &t);

    if (e.tcc) used += t.msrc_dss_tcc_us + TccOverhead;
    if (e.dss) used += 2 * (t.msrc_dss_tcc_us + DssOverhead);
    else if (e.msrc) used += t.msrc_dss_tcc_us + MsrcOverhead;
    if (e.pre_range) used += t.pre_range_us + PreRangeOverhead;

    if (e.final_range) {
        used += FinalRangeOverhead;
        if (used > budget_us) return;             /* budget too small: keep the old one */
        uint32_t final_mclks = timeout_us_to_mclks(budget_us - used, (uint8_t)t.final_range_vcsel_pclks);
        if (e.pre_range) final_mclks += t.pre_range_mclks;
        wr16(d, FINAL_RANGE_TIMEOUT_MACROP_HI, encode_timeout(final_mclks));
    }
}

void vl53l0x_set_long_range(vl53l0x_t *d)
{
    const uint8_t pre_pclks = 18, final_pclks = 14;
    step_enables_t e;
    step_timeouts_t t;

    get_step_enables(d, &e);
    get_step_timeouts(d, &e, &t);

    CONFIG(d, {PRE_RANGE_VALID_PHASE_HIGH, 0x50}, {PRE_RANGE_VALID_PHASE_LOW, 0x08},
              {PRE_RANGE_VCSEL_PERIOD, (uint8_t)encode_vcsel_period(pre_pclks)});
    uint32_t pre_mclks = timeout_us_to_mclks(t.pre_range_us, pre_pclks);
    wr16(d, PRE_RANGE_TIMEOUT_MACROP_HI, encode_timeout(pre_mclks));
    uint32_t msrc_mclks = timeout_us_to_mclks(t.msrc_dss_tcc_us, pre_pclks);
    wr8(d, MSRC_TIMEOUT_MACROP, (uint8_t)((msrc_mclks > 256) ? 255 : (msrc_mclks - 1)));

    get_step_timeouts(d, &e, &t);                 /* pre range has changed */
    CONFIG(d, {FINAL_RANGE_VALID_PHASE_HIGH, 0x48}, {FINAL_RANGE_VALID_PHASE_LOW, 0x08},
              {GLOBAL_VCSEL_WIDTH, 0x03}, {ALGO_PHASECAL_TIMEOUT, 0x07},
              {0xFF, 0x01}, {ALGO_PHASECAL_LIM, 0x20}, {0xFF, 0x00},
              {FINAL_RANGE_VCSEL_PERIOD, (uint8_t)encode_vcsel_period(final_pclks)});
    uint32_t final_mclks = timeout_us_to_mclks(t.final_range_us, final_pclks);
    if (e.pre_range) final_mclks += t.pre_range_mclks;
    wr16(d, FINAL_RANGE_TIMEOUT_MACROP_HI, encode_timeout(final_mclks));

    set_timing_budget_us(d, LONG_RANGE_BUDGET_US);
    d->measurement_timing_budget_us = LONG_RANGE_BUDGET_US;

    uint8_t sequence_config = rd8(d, SYSTEM_SEQUENCE);
    wr8(d, SYSTEM_SEQUENCE, 0x02);
    calibrate(d, 0x00);
    wr8(d, SYSTEM_SEQUENCE, sequence_config);
}

/* ---- ranging ------------------------------------------------------------- */

static void start(vl53l0x_t *d)
{
    CONFIG(d, {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0x91, d->stop_variable},
              {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00});
    wr8(d, SYSRANGE_START, 0x02);       /* period = 0: back-to-back mode */
    d->started = true;
}

static void stop(vl53l0x_t *d)
{
    wr8(d, SYSRANGE_START, 0x01);
    CONFIG(d, {0xFF, 0x01}, {0x00, 0x00}, {0x91, d->stop_variable}, {0x00, 0x01},
              {0xFF, 0x00});
    d->started = false;
}

static int read(vl53l0x_t *d)
{
    if (!d->started) {
        CONFIG(d, {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0x91, d->stop_variable},
                  {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}, {SYSRANGE_START, 0x01});
        int timeout;
        for (timeout = 0; timeout < IO_TIMEOUT; timeout++) {
            if (!(rd8(d, SYSRANGE_START) & 0x01)) break;
            sleep_ms(1);
        }
        if (timeout == IO_TIMEOUT) raise_runtime("TimeoutError");
    }
    int timeout;
    for (timeout = 0; timeout < IO_TIMEOUT; timeout++) {
        if (rd8(d, RESULT_INTERRUPT_STATUS) & 0x07) break;
        sleep_ms(1);
    }
    if (timeout == IO_TIMEOUT) raise_runtime("TimeoutError");
    int value = rd16(d, RESULT_RANGE_STATUS + 10);
    wr8(d, INTERRUPT_CLEAR, 0x01);
    return value;
}

int vl53l0x_ping(vl53l0x_t *d)
{
    start(d);
    int distance = read(d);
    stop(d);
    return distance;
}
