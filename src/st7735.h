/* C port of ST7735.py from st7735-pico-micropython (1.44" / 0.96" additions
 * to Guy Carver's 1.8" driver).  Only the parts 4ToF.py uses are ported, with
 * the same command bytes, window values and delays, so the panel sees the
 * same traffic as under MicroPython. */
#ifndef ST7735_H
#define ST7735_H

#include <stdbool.h>
#include <stdint.h>
#include "machine.h"
#include "sysfont.h"

/* tabcolor values */
#define TFT_GREENTAB        0x0
#define TFT_REDTAB          0x1
#define TFT_BLACKTAB        0x2
#define TFT_GREENTAB2       0x3
#define TFT_GREENTAB3       0x4
#define TFT_GREENTAB128x128 0x5
#define TFT_GREENTAB80x160  0x6
#define TFT_REDTAB80x160    0x7
#define TFT_BLUETAB         0xB

/* TFTColor(aR, aG, aB): 16 bit rgb 565 */
#define TFT_COLOR(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define TFT_BLACK 0
#define TFT_WHITE TFT_COLOR(0xFF, 0xFF, 0xFF)

typedef struct {
    const soft_spi_t *spi;
    uint dc, reset;
    bool use_reset;
    int tabcolor;
    int size[2];
    int rotate;
    int offset[2];
    bool rgb;
    uint8_t colorData[2];
    uint8_t windowLocData[4];
    uint8_t buf[64];            /* colorData * 32 */
} tft_t;

/* TFT(spi, aDC, aReset) */
void tft_new(tft_t *t, const soft_spi_t *spi, uint dc, uint reset);
void tft_init_7735(tft_t *t, int tabcolor);
/* Send the green tab settings again without resetting the panel, so the
 * picture stays on screen.  Not part of ST7735.py. */
void tft_refresh(tft_t *t);

void tft_pixel(tft_t *t, int x, int y, uint16_t color);
/* text(aPos, aString, aColor, aFont, aSize, nowrap) */
void tft_text(tft_t *t, int x, int y, const char *s, uint16_t color,
              const sysfont_t *font, int size, bool nowrap);
void tft_hline(tft_t *t, int x, int y, int len, uint16_t color);
void tft_vline(tft_t *t, int x, int y, int len, uint16_t color);
void tft_rect(tft_t *t, int x, int y, int w, int h, uint16_t color);
void tft_fillrect(tft_t *t, int x, int y, int w, int h, uint16_t color);
void tft_fill(tft_t *t, uint16_t color);

#endif
