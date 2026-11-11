/* 4-sensor ToF scanner, based on 4ToF.py.  The four sensors sit on a ring 90
 * degrees apart; a 2-axis gimbal (yaw + pitch) moves them over the scene and
 * every pose is stored in a 3D map (lidar_map.c, 1 degree cells) that is shown
 * on the LCD as a rotatable 3D point cloud (lidar_view.c), coloured by distance
 * (depth_color.c: red = near, yellow, green, cyan, blue = far, 0..MAX_VALID_MM).
 *
 * Two ways to run it:
 *
 *  - Gimbal / host mode (the real one).  The PC (tools/scan3d.py) moves the
 *    gimbal and, once it has stopped, sends a line to this board:
 *        P <yaw> <pitch>   read all four sensors at that pose (degrees, the
 *                          gimbal's physical angles), store + draw them, reply
 *                          "S d1 d2 d3 d4" (raw mm, 8190 = nothing, -1 = I2C error)
 *        S                 read and reply only, nothing stored
 *        C                 clear the map and the picture
 *        V <az> <el>       set the camera (degrees) and redraw everything
 *        R                 redraw the whole picture from the map
 *    C, V and R answer "OK".  The first command switches off the timed scans
 *    below, which would read while the gimbal is moving.  In host mode GP20
 *    turns the camera 30 degrees.  The scanned range is decided by the PC
 *    (a full 360 x 90 sweep, or any yaw/pitch window).
 *
 *  - Standalone (no PC): a reading every SCAN_PERIOD_MS or on a GP20 press, at
 *    a yaw that advances SCAN_STEP_DEG each time (turn the head by hand).
 *
 * The MicroPython libraries 4ToF.py imports are ported in the other files of
 * this folder: machine.c (Pin, SoftSPI, SoftI2C), st7735.c (ST7735.py),
 * sysfont.c (sysfont.py), vl53l0x.c (vl53l0x.py), vl53l1x.c (vl53l1x.py).
 * print() goes to the USB serial port, like the Thonny shell.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"

#include "depth_color.h"
#include "lidar_map.h"
#include "lidar_view.h"
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

/* ---- 4 TOF sensors ---- */
static soft_i2c_t i2c_1, i2c_2, i2c_3, i2c_4;
static vl53l0x_t sensor_1, sensor_3;
static vl53l1x_t sensor_2, sensor_4;

#define BUTTON_PIN 20

#define MAX_VALID_MM   2000  /* maximum range plotted; sets the scale and colours */
#define SCAN_PERIOD_MS 5000  /* standalone scan; GP20 also scans straight away */
#define SCAN_STEP_DEG  5     /* standalone: yaw advance per scan (by hand) */
#define VIEW_AZ_DEG    210.0f   /* default camera */
#define VIEW_EL_DEG    25.0f
#define VIEW_TURN_DEG  30.0f    /* GP20 in host mode */
#define READOUT_PERIOD_MS 250   /* host mode: text is slow, redraw at most this often */

static int rotation_deg;      /* standalone yaw */
static lidar_map_t  lidar;    /* 129 600 bytes at 1 degree */
static lidar_view_t view;     /* framebuffer + depth buffer, 47 KB */

/* ---- Distance read-out: four text lines in a strip at the bottom ---- */
/* The 3D view stops above the strip (LIDAR_VIEW_H = 122 rows). */
#define READOUT_Y0 124       /* first text line; lines are 9 px apart (124..159) */
#define LINE_H     9
#define VALUE_X    14        /* numbers start here; the "S1".."S4" labels sit at x=0 */
#define VALUE_W    36        /* 6 characters x 6 px: "1234mm" */
#define INFO_X     70        /* yaw / pitch / point count, right of the distances */
#define INFO_W     54

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
static char info_last[3][12];

/* Forget what the read-out shows so the next update redraws it. */
static void readout_invalidate(void)
{
    memset(last_set, 0, sizeof last_set);
    memset(info_last, 0, sizeof info_last);
}

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

/* Same idea for the three info lines (0 = yaw, 1 = pitch, 2 = points). */
static void show_info(int line, const char *text)
{
    if (strcmp(text, info_last[line]) == 0) return;
    snprintf(info_last[line], sizeof info_last[line], "%s", text);
    int y = READOUT_Y0 + line * LINE_H;
    tft_fillrect(&tft, INFO_X, y, INFO_W, 8, TFT_BLACK);
    tft_text(&tft, INFO_X, y, text, TFT_WHITE, &sysfont, 1, true);
}

static void show_readout(const int d[4], float yaw, float pitch)
{
    for (int i = 0; i < 4; i++) show_distance(i, d[i]);
    char s[12];
    snprintf(s, sizeof s, "Y%4d", (int)(yaw + (yaw < 0 ? -0.5f : 0.5f)));
    show_info(0, s);
    snprintf(s, sizeof s, "P%4d", (int)(pitch + (pitch < 0 ? -0.5f : 0.5f)));
    show_info(1, s);
    snprintf(s, sizeof s, "N%5d", lidar_map_valid_count(&lidar));
    show_info(2, s);
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

static void read_all(int d[4])
{
    for (int i = 0; i < 4; i++) d[i] = safe_read((reader_t)i);
}

/* ---- 3D view on the LCD ---- */
static void blit_view_rect(const lidar_rect_t *r)
{
    tft_blit(&tft, r->x, r->y, r->w, r->h, &view.color[r->y][r->x], LIDAR_VIEW_W);
}

/* Send the whole framebuffer (~0.7 s over the software SPI). */
static void blit_view(void)
{
    const lidar_rect_t all = { 0, 0, LIDAR_VIEW_W, LIDAR_VIEW_H };
    blit_view_rect(&all);
}

static void render_all(void)
{
    lidar_view_render(&view, &lidar);
    blit_view();
}

/* Store the four readings taken at one pose and draw the new points.  Only the
 * pixels of the new dots are sent; an invalid reading clears its map cell but
 * its old dot stays on screen until the next full redraw (R, V or GP20). */
static void record_pose(const int d[4], float yaw, float pitch)
{
    /* Without this the panel can lose its settings now and then.  Resetting it
     * worked but flashed the screen white, so only the settings are sent again.
     * If the screen stops updating, go back to tft_init_7735(&tft, TFT_GREENTAB). */
    tft_refresh(&tft);

    for (uint8_t sensor = 0; sensor < 4; ++sensor) {
        const bool valid = d[sensor] != NONE && d[sensor] > 0 && d[sensor] < MAX_VALID_MM;
        const int cell = lidar_map_update(&lidar, sensor, yaw, pitch, d[sensor], valid);
        lidar_rect_t r;
        if (valid && lidar_view_plot(&view, cell, d[sensor], &r)) blit_view_rect(&r);
    }
}

/* ---- Display recovery ----
 * The panel can't be read back, so a glitch from handling (loose wire, supply
 * dip) that resets it, or corrupts its settings, goes unnoticed and leaves it
 * white; the incremental dot drawing never repaints it.  The framebuffer holds
 * the whole picture, so reset the panel and send everything again.  Run on a
 * timer while idle and on every GP20 press. */
#define RECOVER_PERIOD_MS 20000
#define RECOVER_IDLE_MS   3000    /* not within this long of a pose, or the gimbal waits */
static int  last_d[4] = { NONE, NONE, NONE, NONE };
static float last_yaw, last_pitch;
static absolute_time_t next_recover, last_pose_time;

static void recover_display(void)
{
    tft_init_7735(&tft, TFT_GREENTAB);            /* includes the hardware reset */
    tft_fill(&tft, TFT_BLACK);
    blit_view();
    draw_labels();
    readout_invalidate();
    show_readout(last_d, last_yaw, last_pitch);
    next_recover = make_timeout_time_ms(RECOVER_PERIOD_MS);
}

/* ---- Host mode ---- */
static bool host_mode;
static absolute_time_t next_readout;

static void reply_readings(const int d[4])
{
    printf("S %d %d %d %d\n", d[0], d[1], d[2], d[3]);
}

/* Parse "<float> <float>"; false if either is missing. */
static bool parse_two(const char *s, float *a, float *b)
{
    char *e1, *e2;
    *a = strtof(s, &e1);
    if (e1 == s) return false;
    *b = strtof(e1, &e2);
    return e2 != e1;
}

static void handle_command(const char *line)
{
    while (*line == ' ') line++;
    const char cmd = *line;
    if (cmd == '\0') return;
    const char *args = line + 1;
    int d[4];
    float a, b;

    switch (cmd) {
    case 'S':
        host_mode = true;
        read_all(d);
        reply_readings(d);
        break;
    case 'P':
        if (!parse_two(args, &a, &b)) { printf("E bad P\n"); break; }
        host_mode = true;
        read_all(d);
        reply_readings(d);                 /* answer first: the gimbal can move on */
        record_pose(d, a, b);
        memcpy(last_d, d, sizeof last_d);
        last_yaw = a;  last_pitch = b;
        last_pose_time = get_absolute_time();
        if (time_reached(next_readout)) {
            next_readout = make_timeout_time_ms(READOUT_PERIOD_MS);
            show_readout(d, a, b);
        }
        break;
    case 'C':
        host_mode = true;
        lidar_map_clear(&lidar);
        lidar_view_clear(&view);
        blit_view();
        readout_invalidate();
        printf("OK\n");
        break;
    case 'V':
        if (!parse_two(args, &a, &b)) { printf("E bad V\n"); break; }
        host_mode = true;
        lidar_view_set_angles(&view, a, b);
        render_all();
        printf("OK\n");
        break;
    case 'R':
        host_mode = true;
        render_all();
        printf("OK\n");
        break;
    default:
        printf("E unknown '%c'\n", cmd);
        break;
    }
}

/* Collect serial input into lines without blocking. */
static void poll_serial(void)
{
    static char buf[48];
    static size_t len;
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            buf[len] = '\0';
            if (len) handle_command(buf);
            len = 0;
        } else if (len < sizeof buf - 1) {
            buf[len++] = (char)c;
        }
    }
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

    lidar_map_clear(&lidar);
    lidar_view_init(&view, VIEW_AZ_DEG, VIEW_EL_DEG, MAX_VALID_MM);
    blit_view();                         /* empty view with the axes */

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

    absolute_time_t next_scan = get_absolute_time();
    next_readout = get_absolute_time();
    next_recover = make_timeout_time_ms(RECOVER_PERIOD_MS);
    last_pose_time = get_absolute_time();
    float view_az = VIEW_AZ_DEG;
    bool button_was_down = false;
    for (;;) {
        const bool button_down = gpio_get(BUTTON_PIN) == 0;
        const bool button_pressed = button_down && !button_was_down;
        button_was_down = button_down;

        poll_serial();

        if (host_mode) {
            if (button_pressed) {                    /* orbit the camera, repair the panel */
                view_az += VIEW_TURN_DEG;
                if (view_az >= 360.0f) view_az -= 360.0f;
                lidar_view_set_angles(&view, view_az, VIEW_EL_DEG);
                lidar_view_render(&view, &lidar);
                recover_display();                   /* also sends the new picture */
            }
        } else if (time_reached(next_scan) || button_pressed) {
            next_scan = make_timeout_time_ms(SCAN_PERIOD_MS);

            /* read each sensor once, then plot it and show its distance */
            int d[4];
            read_all(d);
            record_pose(d, (float)rotation_deg, 0.0f);
            show_readout(d, (float)rotation_deg, 0.0f);

            char a[12], b[12], c[12], e[12];
            fmt_value(a, sizeof a, d[0]);
            fmt_value(b, sizeof b, d[1]);
            fmt_value(c, sizeof c, d[2]);
            fmt_value(e, sizeof e, d[3]);
            printf("yaw=%d deg points=%d  S1=%s S2=%s S3=%s S4=%s mm\n",
                   rotation_deg, lidar_map_valid_count(&lidar), a, b, c, e);

            memcpy(last_d, d, sizeof last_d);
            last_yaw = (float)rotation_deg;  last_pitch = 0.0f;
            last_pose_time = get_absolute_time();
            rotation_deg = (rotation_deg + SCAN_STEP_DEG) % 360;
            sleep_ms(300);
        }

        /* periodic repair, only while no scan is in progress */
        if (time_reached(next_recover) &&
            absolute_time_diff_us(last_pose_time, get_absolute_time()) > RECOVER_IDLE_MS * 1000LL)
            recover_display();
        sleep_ms(5);
    }
}
