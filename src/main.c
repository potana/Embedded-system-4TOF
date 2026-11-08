/* 4-sensor ToF scanner, based on 4ToF.py: a scan every SCAN_PERIOD_MS or on a
 * GP20 press, plotted as dots coloured by distance (depth_color.c: red = near,
 * yellow, green, cyan, blue = far, over 0..MAX_VALID_MM).
 *
 * The MicroPython libraries it imports are ported in the other files of this
 * folder: machine.c (Pin, SoftSPI, SoftI2C), st7735.c (ST7735.py),
 * sysfont.c (sysfont.py), vl53l0x.c (vl53l0x.py), vl53l1x.c (vl53l1x.py).
 * print() goes to the USB serial port, like the Thonny shell.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "depth_color.h"
#include "lidar_map.h"
#include "machine.h"
#include "pyexc.h"
#include "st7735.h"
#include "sysfont.h"
#include "vl53l0x.h"
#include "vl53l1x.h"

#define NONE (-1)            /* Python None (readings are never negative) */

/* ---- Display (confirmed) ---- */
#define PIN_SCK  9
#define PIN_MOSI 8
#define PIN_DC   3
#define PIN_RST  2
/* Display clock.  4ToF.py used 100 kHz, which takes 3.3 s to clear the screen;
 * 500 kHz takes under 1 s.  Go back to 100000 if the display misbehaves. */
#define SPI_BAUD 500000

static soft_spi_t spi;
static tft_t tft;

/* Point cloud area: x 0..126, y 0..READOUT_Y0-3 (above the read-out strip).
 * The centre sits in the middle of that area and SCALE is worked out from
 * the maximum range, so a reading just under MAX_VALID_MM lands on the edge
 * of the area in every direction instead of falling off the screen. */
#define DOT_SIZE 4                                   /* px, was 2 */
#define CX 64
#define CY ((READOUT_Y0 - 2) / 2)                    /* 61 */
#define CLOUD_RADIUS_PX (CY - DOT_SIZE / 2)          /* 59 */

/* ---- 4 TOF sensors ---- */
static soft_i2c_t i2c_1, i2c_2, i2c_3, i2c_4;
static vl53l0x_t sensor_1, sensor_3;
static vl53l1x_t sensor_2, sensor_4;

#define BUTTON_PIN 20

#define MAX_VALID_MM 2000    /* maximum range plotted; sets the scale too */
#define SCALE        ((float)CLOUD_RADIUS_PX / MAX_VALID_MM)   /* px per mm */
#define STEP_DEG     LIDAR_STEP_DEG
#define SCAN_PERIOD_MS 5000  /* automatic scan; GP20 also scans straight away */
static int rotation_deg;
static lidar_map_t lidar;

/* ---- Distance read-out: four text lines in a strip at the bottom ---- */
/* The strip is kept free of plotted dots (see plot()). */
#define READOUT_Y0 124       /* first text line; lines are 9 px apart (124..159) */
#define LINE_H     9
#define VALUE_X    14        /* numbers start here; the "S1".."S4" labels sit at x=0 */
#define VALUE_W    36        /* 6 characters x 6 px: "1234mm" */

static void draw_labels(void)
{
    for (int i = 0; i < 4; i++) {
        char s[4];
        snprintf(s, sizeof s, "S%d", i + 1);
        tft_text(&tft, 0, READOUT_Y0 + i * LINE_H, s, TFT_WHITE, &sysfont, 1, true);
    }
}

static char last_text[4][12];
static bool last_set[4];      /* last_text[i] is not None */

/* Show one sensor's distance. Redraws only when the text changed,
 * because the software SPI is slow. */
static void show_distance(int i, int dist_mm)
{
    char s[12];
    if (dist_mm == NONE)               snprintf(s, sizeof s, " err");
    else if (dist_mm >= MAX_VALID_MM)  snprintf(s, sizeof s, "%4d", MAX_VALID_MM);  /* nothing detected */
    else                               snprintf(s, sizeof s, "%4d", dist_mm);
    if (last_set[i] && strcmp(s, last_text[i]) == 0) return;
    strcpy(last_text[i], s);
    last_set[i] = true;
    int y = READOUT_Y0 + i * LINE_H;
    tft_fillrect(&tft, VALUE_X, y, VALUE_W, 8, TFT_BLACK);
    char t[16];
    snprintf(t, sizeof t, "%s%s", s, dist_mm != NONE ? "mm" : "");
    tft_text(&tft, VALUE_X, y, t, TFT_WHITE, &sysfont, 1, true);
}

/* safe_read(fn): fn() or None if it raised OSError.  Any other exception
 * goes on and ends the program, as in Python. */
typedef enum { READ_S1, READ_S2, READ_S3, READ_S4 } reader_t;

static int call_reader(reader_t r)
{
    switch (r) {
    case READ_S1: return vl53l0x_ping(&sensor_1);
    case READ_S2: return vl53l1x_read(&sensor_2);
    case READ_S3: return vl53l0x_ping(&sensor_3);
    default:      return vl53l1x_read(&sensor_4);
    }
}

static int safe_read(reader_t r)
{
    exc_frame_t f;
    exc_push(&f);
    if (setjmp(f.buf) == 0) {
        int v = call_reader(r);
        exc_pop(&f);
        return v;
    }
    if (f.type != EXC_OSERROR) exc_reraise(&f);
    return NONE;
}

static void plot(int dist_mm, int deg)
{
    float r = (float)deg * (3.14159265f / 180.0f);       /* math.radians() */
    int x = CX + (int)(dist_mm * SCALE * cosf(r));
    int y = CY - (int)(dist_mm * SCALE * sinf(r));
    /* solid DOT_SIZE x DOT_SIZE square centred on the point */
    int x0 = x - DOT_SIZE / 2, y0 = y - DOT_SIZE / 2;
    if (0 <= x0 && x0 + DOT_SIZE <= 127 && 0 <= y0 && y0 + DOT_SIZE <= READOUT_Y0 - 2) {
        tft_fillrect(&tft, x0, y0, DOT_SIZE, DOT_SIZE, depth_color565(dist_mm, MAX_VALID_MM));
    }
}

static void render_lidar(void)
{
    /* Rebuild only the plot region; keep the four live text read-outs. */
    tft_fillrect(&tft, 0, 0, 128, READOUT_Y0 - 1, TFT_BLACK);
    tft_rect(&tft, CX - 9, CY, 19, 1, TFT_WHITE);
    tft_rect(&tft, CX, CY - 9, 1, 19, TFT_WHITE);
    for (int bin = 0; bin < LIDAR_BINS; ++bin) {
        const lidar_point_t *point = &lidar.point[bin];
        if (point->valid) plot(point->distance_mm, bin * LIDAR_STEP_DEG);
    }
}

/* Host mode: the PC (tools/scan3d.py) sends 'S' once the gimbal has stopped and
 * gets "S d1 d2 d3 d4\n" back.  Distances are raw mm (8190 = nothing detected),
 * -1 = I2C error.  The first 'S' switches off the timed/button scans, which
 * would otherwise take readings while the gimbal is moving. */
static bool host_mode;

static void host_scan(void)
{
    int d[4];
    for (int i = 0; i < 4; i++) d[i] = safe_read((reader_t)i);
    printf("S %d %d %d %d\n", d[0], d[1], d[2], d[3]);
}

static void fmt_value(char *buf, size_t n, int v)
{
    if (v == NONE) snprintf(buf, n, "None");
    else           snprintf(buf, n, "%d", v);
}

int main(void)
{
    stdio_init_all();                    /* print() -> USB serial */

    soft_spi_init(&spi, SPI_BAUD, PIN_SCK, PIN_MOSI, 28);
    tft_new(&tft, &spi, PIN_DC, PIN_RST);

    /* ---- Full 3-step cycle before settling ---- */
    static const int TABCOLORS[3] = { TFT_GREENTAB, TFT_BLUETAB, TFT_GREENTAB };
    for (int i = 0; i < 3; i++) {
        tft_init_7735(&tft, TABCOLORS[i]);
        sleep_ms(300);
    }

    tft_fill(&tft, TFT_BLACK);
    sleep_ms(200);

    tft_rect(&tft, CX - 9, CY, 19, 1, TFT_WHITE);
    tft_rect(&tft, CX, CY - 9, 1, 19, TFT_WHITE);

    soft_i2c_init(&i2c_1, 0, 1);         /* Grove 1 */
    soft_i2c_init(&i2c_2, 4, 5);         /* VL53L1X */
    soft_i2c_init(&i2c_3, 6, 7);         /* Grove 5 */
    soft_i2c_init(&i2c_4, 26, 27);       /* VL53L1X */

    vl53l0x_new(&sensor_1, &i2c_1);
    vl53l1x_new(&sensor_2, &i2c_2);
    vl53l0x_new(&sensor_3, &i2c_3);
    vl53l1x_new(&sensor_4, &i2c_4);

    vl53l0x_set_long_range(&sensor_1);
    vl53l0x_set_long_range(&sensor_3);

    pin_init_in_pullup(BUTTON_PIN);

    draw_labels();

    lidar_map_clear(&lidar);
    absolute_time_t next_scan = get_absolute_time();
    bool button_was_down = false;
    for (;;) {
        const bool button_down = gpio_get(BUTTON_PIN) == 0;
        const bool button_pressed = button_down && !button_was_down;
        button_was_down = button_down;

        if (getchar_timeout_us(0) == 'S') {
            host_mode = true;
            host_scan();
        }

        if (!host_mode && (time_reached(next_scan) || button_pressed)) {
            next_scan = make_timeout_time_ms(SCAN_PERIOD_MS);

            /* read each sensor once, then plot it and show its distance */
            int d1 = safe_read(READ_S1);
            int d2 = safe_read(READ_S2);
            int d3 = safe_read(READ_S3);
            int d4 = safe_read(READ_S4);

            /* Without this the panel can lose its settings after a press.
             * Resetting it before each draw worked but flashed the screen
             * white, so only the settings are sent again.  If the screen
             * stops updating, go back to tft_init_7735(&tft, TFT_GREENTAB). */
            tft_refresh(&tft);

            const int distances[4] = {d1, d2, d3, d4};
            for (uint8_t sensor = 0; sensor < 4; ++sensor) {
                const int distance = distances[sensor];
                const bool valid = distance != NONE && distance > 0 &&
                                   distance < MAX_VALID_MM;
                lidar_map_update(&lidar, sensor, rotation_deg, distance, valid);
            }
            render_lidar();

            show_distance(0, d1);
            show_distance(1, d2);
            show_distance(2, d3);
            show_distance(3, d4);
            char a[12], b[12], c[12], e[12];
            fmt_value(a, sizeof a, d1);
            fmt_value(b, sizeof b, d2);
            fmt_value(c, sizeof c, d3);
            fmt_value(e, sizeof e, d4);
            printf("clockwise=%d deg points=%d/72  S1=%s S2=%s S3=%s S4=%s mm\n",
                   rotation_deg, lidar_map_valid_count(&lidar), a, b, c, e);

            rotation_deg = (rotation_deg + STEP_DEG) % 360;
            sleep_ms(300);
        }
        sleep_ms(5);
    }
}
