/* Sensor benchmark: same start-up as main.c, then repeats these on USB serial:
 *
 *   VL53L0X (S1, S3): time per ping(), spread of the readings
 *   VL53L1X (S2, S4): time per read(), time between new measurements (from
 *                     the sensor's stream counter), spread of the readings
 *   LCD:              time to refresh the panel and draw one full update
 *
 * Keep the sensors and what they point at still: the spread is the noise at
 * that distance, not the accuracy.  Accuracy needs a measured target.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "machine.h"
#include "pyexc.h"
#include "st7735.h"
#include "sysfont.h"
#include "vl53l0x.h"
#include "vl53l1x.h"

#define SPI_BAUD     500000  /* as main.c */
#define L0X_SAMPLES  200
#define L1X_SAMPLES  100
#define L1X_LIMIT_US 20000000u
#define NO_TARGET_MM 2000    /* readings at or above this count as nothing */

static soft_spi_t spi;
static tft_t tft;
static soft_i2c_t i2c_1, i2c_2, i2c_3, i2c_4;
static vl53l0x_t sensor_1, sensor_3;
static vl53l1x_t sensor_2, sensor_4;

typedef struct {
    int n, none;
    double sum, sumsq;
    int min, max;
} spread_t;

static void spread_add(spread_t *s, int v)
{
    if (v >= NO_TARGET_MM) { s->none++; return; }
    if (s->n == 0 || v < s->min) s->min = v;
    if (s->n == 0 || v > s->max) s->max = v;
    s->n++;
    s->sum += v;
    s->sumsq += (double)v * v;
}

static void spread_print(const spread_t *s)
{
    if (s->n == 0) { printf("  no target in %d readings\n", s->none); return; }
    double mean = s->sum / s->n;
    double var = s->sumsq / s->n - mean * mean;
    printf("  readings=%d nothing=%d mean=%.1f sd=%.1f min=%d max=%d mm\n",
           s->n, s->none, mean, sqrt(var > 0 ? var : 0), s->min, s->max);
}

static void bench_l0x(const char *name, vl53l0x_t *d)
{
    spread_t s = {0};
    uint32_t tmin = UINT32_MAX, tmax = 0;
    uint64_t tsum = 0;
    for (int i = 0; i < L0X_SAMPLES; i++) {
        uint32_t t0 = time_us_32();
        int v = vl53l0x_ping(d);
        uint32_t t = time_us_32() - t0;
        if (t < tmin) tmin = t;
        if (t > tmax) tmax = t;
        tsum += t;
        spread_add(&s, v);
    }
    printf("%s VL53L0X ping: avg=%.1f min=%.1f max=%.1f ms\n", name,
           (double)tsum / L0X_SAMPLES / 1000.0, tmin / 1000.0, tmax / 1000.0);
    spread_print(&s);
}

static void bench_l1x(const char *name, vl53l1x_t *d)
{
    spread_t s = {0};
    uint8_t b[17], prev[17];
    int ok_status = 0, gaps = 0;
    uint64_t gap_sum = 0;
    uint32_t gap_max = 0;

    uint32_t t0 = time_us_32();
    (void)vl53l1x_read(d);
    uint32_t read_us = time_us_32() - t0;

    /* The stream count does not move here, so a new measurement is seen as
     * any change in the result block (signal rates differ every time). */
    i2c_readfrom_mem(d->i2c, d->address, 0x0089, prev, 17, 16);
    uint32_t last_t = 0, start = time_us_32();
    while (s.n + s.none < L1X_SAMPLES && time_us_32() - start < L1X_LIMIT_US) {
        i2c_readfrom_mem(d->i2c, d->address, 0x0089, b, 17, 16);
        if (memcmp(b, prev, sizeof b) != 0) {
            uint32_t now = time_us_32();
            if (last_t) {
                uint32_t gap = now - last_t;
                gaps++;
                gap_sum += gap;
                if (gap > gap_max) gap_max = gap;
            }
            last_t = now;
            memcpy(prev, b, sizeof b);
            if ((b[0] & 0x1F) == 9) ok_status++;
            spread_add(&s, (b[13] << 8) + b[14]);
        }
        sleep_ms(2);
    }
    printf("%s VL53L1X read()=%.1f ms; new measurement every avg=%.1f max=%.1f ms; status ok=%d\n",
           name, read_us / 1000.0, gaps ? (double)gap_sum / gaps / 1000.0 : 0.0,
           gap_max / 1000.0, ok_status);
    spread_print(&s);
}

/* The real program pings once every 5 s.  A ping that returns far faster than
 * a measurement takes cannot have measured anything new. */
static void bench_idle_ping(void)
{
    for (int i = 0; i < 4; i++) {
        sleep_ms(5000);
        uint32_t t0 = time_us_32();
        int a = vl53l0x_ping(&sensor_1);
        uint32_t t1 = time_us_32();
        int b = vl53l0x_ping(&sensor_3);
        uint32_t t2 = time_us_32();
        int a2 = vl53l0x_ping(&sensor_1);
        uint32_t t3 = time_us_32();
        printf("after 5 s idle: S1 %d mm in %.1f ms (next ping %d mm in %.1f ms), S3 %d mm in %.1f ms\n",
               a, (t1 - t0) / 1000.0, a2, (t3 - t2) / 1000.0, b, (t2 - t1) / 1000.0);
    }
}

static void bench_lcd(void)
{
    uint32_t t0 = time_us_32();
    tft_refresh(&tft);
    uint32_t refresh = time_us_32() - t0;

    t0 = time_us_32();
    for (int i = 0; i < 4; i++) tft_fillrect(&tft, 20 + i * 20, 40, 4, 4, TFT_WHITE);
    for (int i = 0; i < 4; i++) {
        int y = 124 + i * 9;
        tft_fillrect(&tft, 14, y, 36, 8, TFT_BLACK);
        tft_text(&tft, 14, y, "1234mm", TFT_WHITE, &sysfont, 1, true);
    }
    uint32_t draw = time_us_32() - t0;
    printf("LCD refresh=%.1f ms, 4 dots + 4 read-outs=%.1f ms\n", refresh / 1000.0, draw / 1000.0);

    t0 = time_us_32();
    tft_fill(&tft, TFT_BLACK);
    printf("LCD full clear=%.1f ms\n", (time_us_32() - t0) / 1000.0);
}

int main(void)
{
    stdio_init_all();

    soft_spi_init(&spi, SPI_BAUD, 9, 8, 28);
    tft_new(&tft, &spi, 3, 2);
    static const int TABCOLORS[3] = { TFT_GREENTAB, TFT_BLUETAB, TFT_GREENTAB };
    for (int i = 0; i < 3; i++) {
        tft_init_7735(&tft, TABCOLORS[i]);
        sleep_ms(300);
    }
    tft_fill(&tft, TFT_BLACK);

    soft_i2c_init(&i2c_1, 0, 1);
    soft_i2c_init(&i2c_2, 4, 5);
    soft_i2c_init(&i2c_3, 6, 7);
    soft_i2c_init(&i2c_4, 26, 27);

    vl53l0x_new(&sensor_1, &i2c_1);
    vl53l1x_new(&sensor_2, &i2c_2);
    vl53l0x_new(&sensor_3, &i2c_3);
    vl53l1x_new(&sensor_4, &i2c_4);
    vl53l0x_set_long_range(&sensor_1);
    vl53l0x_set_long_range(&sensor_3);

    for (int round = 1; ; round++) {
        printf("--- round %d ---\n", round);
        bench_l0x("S1", &sensor_1);
        bench_l1x("S2", &sensor_2);
        bench_l0x("S3", &sensor_3);
        bench_l1x("S4", &sensor_4);
        bench_lcd();
        bench_idle_ping();
    }
}
