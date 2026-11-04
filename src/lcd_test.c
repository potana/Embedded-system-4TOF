/* Display test: finds out what makes the LCD stop updating.
 *
 * Same start-up as main.c, then a counter and a colour square are redrawn
 * in a loop while one suspect at a time runs: 3 ticks each for P0..P5, then
 * 100 ticks of P6, long enough to catch a fault that only happens now and
 * then.  The step showing when the counter freezes is the cause:
 *
 *   P0 idle    drawing only
 *   P1 print   + printf on USB serial
 *   P2 S1      + VL53L0X S1 ping every tick
 *   P3 S2      + VL53L1X S2 read every tick
 *   P4 S3      + VL53L0X S3 ping every tick
 *   P5 S4      + VL53L1X S4 read every tick
 *   P6 all     + all four sensors every tick
 *   P7 press   + all four sensors after each GP20 press (stays here)
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/structs/watchdog.h"

#include "depth_color.h"
#include "machine.h"
#include "pyexc.h"
#include "st7735.h"
#include "sysfont.h"
#include "vl53l0x.h"
#include "vl53l1x.h"

#define BUTTON_PIN      20
#define TICKS_PER_PHASE 3        /* P0..P5 */
#define P6_TICKS        100
#define LAST_PHASE      7

static soft_spi_t spi;
static tft_t tft;
static soft_i2c_t i2c_1, i2c_2, i2c_3, i2c_4;
static vl53l0x_t sensor_1, sensor_3;
static vl53l1x_t sensor_2, sensor_4;

static const char *const PHASE_NAME[LAST_PHASE + 1] = {
    "idle", "print", "S1", "S2", "S3", "S4", "all", "press"
};

/* What caused the last reset, from the chip's own records.  POWER after a
 * reboot nobody asked for means the supply dipped; RUN means the RUN pin was
 * pulled low; WATCHDOG means a software reset. */
static const char *reset_cause(void)
{
    uint32_t r = vreg_and_chip_reset_hw->chip_reset;
    if (watchdog_hw->reason & (WATCHDOG_REASON_TIMER_BITS | WATCHDOG_REASON_FORCE_BITS)) return "WATCHDOG";
    if (r & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS)         return "RUN";
    if (r & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) return "DEBUG";
    if (r & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS)         return "POWER";
    return "other";
}

/* True if GP20 went low since the last call.  The GPIO block latches the
 * edge in hardware, so a press during the (slow) drawing is not missed. */
static bool button_pressed(void)
{
    uint32_t bit = (uint32_t)GPIO_IRQ_EDGE_FALL << (4 * (BUTTON_PIN % 8));
    bool hit = (iobank0_hw->intr[BUTTON_PIN / 8] & bit) != 0;
    gpio_acknowledge_irq(BUTTON_PIN, GPIO_IRQ_EDGE_FALL);
    return hit;
}

/* Read one sensor (1..4); an I2C error is only reported, as in safe_read(). */
static int read_sensor(int n)
{
    exc_frame_t f;
    exc_push(&f);
    if (setjmp(f.buf) == 0) {
        int v;
        switch (n) {
        case 1:  v = vl53l0x_ping(&sensor_1); break;
        case 2:  v = vl53l1x_read(&sensor_2); break;
        case 3:  v = vl53l0x_ping(&sensor_3); break;
        default: v = vl53l1x_read(&sensor_4); break;
        }
        exc_pop(&f);
        return v;
    }
    if (f.type != EXC_OSERROR) exc_reraise(&f);
    return -1;
}

/* The full start-up sequence from 4ToF.py. */
static void lcd_start(void)
{
    static const int TABCOLORS[3] = { TFT_GREENTAB, TFT_BLUETAB, TFT_GREENTAB };
    for (int i = 0; i < 3; i++) {
        tft_init_7735(&tft, TABCOLORS[i]);
        sleep_ms(300);
    }
    tft_fill(&tft, TFT_BLACK);
    sleep_ms(200);
}

int main(void)
{
    const char *boot = reset_cause();
    stdio_init_all();

    soft_spi_init(&spi, 100000, 9, 8, 28);
    tft_new(&tft, &spi, 3, 2);
    lcd_start();

    soft_i2c_init(&i2c_1, 0, 1);
    soft_i2c_init(&i2c_2, 4, 5);
    soft_i2c_init(&i2c_3, 6, 7);
    soft_i2c_init(&i2c_4, 26, 27);

    vl53l0x_new(&sensor_1, &i2c_1);
    vl53l1x_new(&sensor_2, &i2c_2);
    vl53l0x_new(&sensor_3, &i2c_3);
    vl53l1x_new(&sensor_4, &i2c_4);

    pin_init_in_pullup(BUTTON_PIN);

    char bootline[24];
    snprintf(bootline, sizeof bootline, "boot %s", boot);
    tft_text(&tft, 10, 18, bootline, TFT_WHITE, &sysfont, 1, true);

    int presses = 0;
    for (int tick = 0; ; tick++) {
        int phase = tick / TICKS_PER_PHASE;
        if (phase >= 6) phase = (tick < 6 * TICKS_PER_PHASE + P6_TICKS) ? 6 : LAST_PHASE;

        int d[5] = { 0, -2, -2, -2, -2 };           /* -2 = not read */
        if (phase >= 2 && phase <= 5) {
            d[phase - 1] = read_sensor(phase - 1);
        }
        bool pressed = (phase == 7) && button_pressed();
        if (phase == 6 || pressed) {
            for (int n = 1; n <= 4; n++) d[n] = read_sensor(n);
            if (pressed) presses++;
        }

        char line[24];
        snprintf(line, sizeof line, "P%d %s %d", phase, PHASE_NAME[phase], tick);
        tft_fillrect(&tft, 10, 30, 110, 8, TFT_BLACK);
        tft_text(&tft, 10, 30, line, TFT_WHITE, &sysfont, 1, true);
        snprintf(line, sizeof line, "presses %d", presses);
        tft_fillrect(&tft, 10, 42, 110, 8, TFT_BLACK);
        tft_text(&tft, 10, 42, line, TFT_WHITE, &sysfont, 1, true);
        tft_fillrect(&tft, 40, 60, 40, 40, depth_color565((tick % 8) * 250, 2000));

        if (phase >= 1) {
            printf("P%d %s tick=%d presses=%d S1=%d S2=%d S3=%d S4=%d boot=%s\n",
                   phase, PHASE_NAME[phase], tick, presses, d[1], d[2], d[3], d[4], boot);
        }
        /* A press leaves the panel uninitialised: 3 s later, try to bring it
         * back.  If this works the panel only lost its state, not its power. */
        if (pressed) {
            sleep_ms(3000);
            printf("restarting LCD\n");
            lcd_start();
            tft_text(&tft, 10, 18, bootline, TFT_WHITE, &sysfont, 1, true);
        }
        sleep_ms(500);
    }
}
