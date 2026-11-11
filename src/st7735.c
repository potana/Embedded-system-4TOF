#include "st7735.h"
#include "hardware/gpio.h"

/* TFTRotations and TFTRGB are bits to set on MADCTL */
static const uint8_t TFTRotations[4] = { 0x00, 0x60, 0xC0, 0xA0 };
#define TFTBGR 0x08
#define TFTRGB 0x00

#define SWRESET 0x01
#define SLPOUT  0x11
#define NORON   0x13
#define INVOFF  0x20
#define INVON   0x21
#define DISPON  0x29
#define CASET   0x2A
#define RASET   0x2B
#define RAMWR   0x2C
#define COLMOD  0x3A
#define MADCTL  0x36
#define FRMCTR1 0xB1
#define FRMCTR2 0xB2
#define FRMCTR3 0xB3
#define INVCTR  0xB4
#define DISSET5 0xB6
#define PWCTR1  0xC0
#define PWCTR2  0xC1
#define PWCTR3  0xC2
#define PWCTR4  0xC3
#define PWCTR5  0xC4
#define VMCTR1  0xC5
#define PWCTR6  0xFC
#define GMCTRP1 0xE0
#define GMCTRN1 0xE1

static int clamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);   /* max(aMin, min(aMax, aValue)) */
}

/* Python // and % (floor division, result has the divisor's sign) */
static int py_div(int a, int b) { int q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
static int py_mod(int a, int b) { int m = a % b; if (m != 0 && ((m < 0) != (b < 0))) m += b; return m; }

static void writecommand(tft_t *t, uint8_t c)
{
    gpio_put(t->dc, 0);
    soft_spi_write(t->spi, &c, 1);
}

static void writedata(tft_t *t, const uint8_t *d, size_t n)
{
    gpio_put(t->dc, 1);
    soft_spi_write(t->spi, d, n);
}

static void writedata1(tft_t *t, uint8_t v)
{
    writedata(t, &v, 1);
}

void tft_new(tft_t *t, const soft_spi_t *spi, uint dc, uint reset)
{
    t->tabcolor = 0;
    t->size[0] = 128; t->size[1] = 128;
    t->rotate = 2;                      /* vertical with top toward pins */
    t->offset[0] = 2; t->offset[1] = 3;
    t->rgb = false;
    t->dc = dc;
    pin_init_out(dc);
    t->use_reset = true;
    t->reset = reset;
    pin_init_out(reset);
    t->spi = spi;
    t->colorData[0] = t->colorData[1] = 0;
    for (int i = 0; i < 4; i++) t->windowLocData[i] = 0;
}

static void setMADCTL(tft_t *t)
{
    writecommand(t, MADCTL);
    writedata1(t, (uint8_t)(TFTRotations[t->rotate] | (t->rgb ? TFTRGB : TFTBGR)));
}

static void rotation(tft_t *t, int aRot)
{
    int rotchange = 0;
    if (0 <= aRot && aRot < 4) {
        rotchange = t->rotate ^ aRot;
        t->rotate = aRot;
    }
    /* switching between vertical and horizontal swaps size and offset */
    if (rotchange & 1) {
        int s = t->size[0];   t->size[0] = t->size[1];     t->size[1] = s;
        int o = t->offset[0]; t->offset[0] = t->offset[1]; t->offset[1] = o;
    }
    if (t->tabcolor == TFT_GREENTAB128x128) {
        static const int off[4][2] = { {2, 1}, {1, 2}, {2, 3}, {3, 2} };
        t->offset[0] = off[aRot][0];
        t->offset[1] = off[aRot][1];
    }
    setMADCTL(t);
}

static void setwindowpoint(tft_t *t, int px, int py)
{
    int x = t->offset[0] + px;
    int y = t->offset[1] + py;
    writecommand(t, CASET);
    t->windowLocData[0] = (uint8_t)t->offset[0];
    t->windowLocData[1] = (uint8_t)x;
    t->windowLocData[2] = (uint8_t)t->offset[0];
    t->windowLocData[3] = (uint8_t)x;
    writedata(t, t->windowLocData, 4);

    writecommand(t, RASET);
    t->windowLocData[0] = (uint8_t)t->offset[1];
    t->windowLocData[1] = (uint8_t)y;
    t->windowLocData[2] = (uint8_t)t->offset[1];
    t->windowLocData[3] = (uint8_t)y;
    writedata(t, t->windowLocData, 4);
    writecommand(t, RAMWR);
}

static void setwindowloc(tft_t *t, int x0, int y0, int x1, int y1)
{
    writecommand(t, CASET);
    t->windowLocData[0] = (uint8_t)t->offset[0];
    t->windowLocData[1] = (uint8_t)(t->offset[0] + x0);
    t->windowLocData[2] = (uint8_t)t->offset[0];
    t->windowLocData[3] = (uint8_t)(t->offset[0] + x1);
    writedata(t, t->windowLocData, 4);

    writecommand(t, RASET);
    t->windowLocData[0] = (uint8_t)t->offset[1];
    t->windowLocData[1] = (uint8_t)(t->offset[1] + y0);
    t->windowLocData[2] = (uint8_t)t->offset[1];
    t->windowLocData[3] = (uint8_t)(t->offset[1] + y1);
    writedata(t, t->windowLocData, 4);
    writecommand(t, RAMWR);
}

static void setColor(tft_t *t, uint16_t c)
{
    t->colorData[0] = (uint8_t)(c >> 8);
    t->colorData[1] = (uint8_t)c;
    for (int i = 0; i < 64; i += 2) {
        t->buf[i] = t->colorData[0];
        t->buf[i + 1] = t->colorData[1];
    }
}

/* Send the current colour aPixels times. */
static void draw(tft_t *t, int aPixels)
{
    gpio_put(t->dc, 1);
    for (int i = 0; i < py_div(aPixels, 32); i++) soft_spi_write(t->spi, t->buf, 64);
    int rest = py_mod(aPixels, 32);
    if (rest > 0) soft_spi_write(t->spi, t->buf, (size_t)rest * 2);
}

static void pushcolor(tft_t *t, uint16_t c)
{
    t->colorData[0] = (uint8_t)(c >> 8);
    t->colorData[1] = (uint8_t)c;
    writedata(t, t->colorData, 2);
}

void tft_pixel(tft_t *t, int x, int y, uint16_t color)
{
    if (0 <= x && x < t->size[0] && 0 <= y && y < t->size[1]) {
        setwindowpoint(t, x, y);
        pushcolor(t, color);
    }
}

static void tft_char(tft_t *t, int x, int y, unsigned char ch, uint16_t color,
                     const sysfont_t *font, int sw, int sh)
{
    int ci = ch;
    if (font->start <= ci && ci <= font->end) {
        int fontw = font->width;
        int fonth = font->height;
        size_t first = (size_t)(ci - font->start) * (size_t)fontw;
        /* charA = Data[ci:ci + fontw]; a slice past the end is shorter */
        size_t last = first + (size_t)fontw;
        if (first > font->data_len) first = font->data_len;
        if (last > font->data_len) last = font->data_len;

        int px = x;
        if (sw <= 1 && sh <= 1) {
            for (size_t k = first; k < last; k++) {
                unsigned c = font->data[k];
                int py = y;
                for (int r = 0; r < fonth; r++) {
                    if (c & 0x01) tft_pixel(t, px, py, color);
                    py += 1;
                    c >>= 1;
                }
                px += 1;
            }
        } else {
            for (size_t k = first; k < last; k++) {
                unsigned c = font->data[k];
                int py = y;
                for (int r = 0; r < fonth; r++) {
                    if (c & 0x01) tft_fillrect(t, px, py, sw, sh, color);
                    py += sh;
                    c >>= 1;
                }
                px += sw;
            }
        }
    }
}

void tft_text(tft_t *t, int x, int y, const char *s, uint16_t color,
              const sysfont_t *font, int size, bool nowrap)
{
    if (font == NULL) return;
    int sw = size, sh = size;
    int px = x, py = y;
    int width = sw * font->width + 1;
    for (; *s; s++) {
        tft_char(t, px, py, (unsigned char)*s, color, font, sw, sh);
        px += width;
        /* > rather than >= lets the blank right edge print off screen */
        if (px + width > t->size[0]) {
            if (nowrap) break;
            py += font->height * sh + 1;
            px = x;
        }
    }
}

void tft_vline(tft_t *t, int x, int y, int len, uint16_t color)
{
    int sx = clamp(x, 0, t->size[0]), sy = clamp(y, 0, t->size[1]);
    int ex = sx, ey = clamp(sy + len, 0, t->size[1]);
    if (ey < sy) { int tmp = sy; sy = ey; ey = tmp; }
    setwindowloc(t, sx, sy, ex, ey);
    setColor(t, color);
    draw(t, len);
}

void tft_hline(tft_t *t, int x, int y, int len, uint16_t color)
{
    int sx = clamp(x, 0, t->size[0]), sy = clamp(y, 0, t->size[1]);
    int ex = clamp(sx + len, 0, t->size[0]), ey = sy;
    if (ex < sx) { int tmp = sx; sx = ex; ex = tmp; }
    setwindowloc(t, sx, sy, ex, ey);
    setColor(t, color);
    draw(t, len);
}

void tft_rect(tft_t *t, int x, int y, int w, int h, uint16_t color)
{
    tft_hline(t, x, y, w, color);
    tft_hline(t, x, y + h - 1, w, color);
    tft_vline(t, x, y, h, color);
    tft_vline(t, x + w - 1, y, h, color);
}

void tft_fillrect(tft_t *t, int x, int y, int w, int h, uint16_t color)
{
    int sx = clamp(x, 0, t->size[0]), sy = clamp(y, 0, t->size[1]);
    int ex = clamp(sx + w - 1, 0, t->size[0]), ey = clamp(sy + h - 1, 0, t->size[1]);

    /* same corner swapping as the Python */
    if (ex < sx) { int tmp = ex; ex = sx; sx = tmp; }
    if (ey < sy) { int tmp = ey; ey = sy; sy = tmp; }

    setwindowloc(t, sx, sy, ex, ey);
    int numPixels = (ex - sx + 1) * (ey - sy + 1);
    setColor(t, color);
    draw(t, numPixels);
}

void tft_fill(tft_t *t, uint16_t color)
{
    tft_fillrect(t, 0, 0, t->size[0], t->size[1], color);
}

void tft_blit(tft_t *t, int x, int y, int w, int h, const uint16_t *pix, int stride)
{
    if (w <= 0 || h <= 0 || x < 0 || y < 0 || x + w > t->size[0] || y + h > t->size[1]) return;
    setwindowloc(t, x, y, x + w - 1, y + h - 1);
    gpio_put(t->dc, 1);
    for (int r = 0; r < h; r++) {
        const uint16_t *row = pix + (size_t)r * (size_t)stride;
        for (int c = 0; c < w;) {
            int n = w - c;
            if (n > 32) n = 32;                       /* buf holds 32 pixels */
            for (int i = 0; i < n; i++) {
                t->buf[2 * i]     = (uint8_t)(row[c + i] >> 8);
                t->buf[2 * i + 1] = (uint8_t)row[c + i];
            }
            soft_spi_write(t->spi, t->buf, (size_t)n * 2);
            c += n;
        }
    }
}

static void tft_reset(tft_t *t)
{
    gpio_put(t->dc, 0);
    if (t->use_reset) {
        gpio_put(t->reset, 1);
        sleep_us(500);
        gpio_put(t->reset, 0);
        sleep_us(500);
        gpio_put(t->reset, 1);
    }
    sleep_us(500);
}

static const uint8_t dataGMCTRP[16] = { 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29,
                                        0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10 };
static const uint8_t dataGMCTRN[16] = { 0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d, 0x2e,
                                        0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10 };

/* tft_refresh(): repeat the settings without the resets */
static bool skip_reset;

void tft_refresh(tft_t *t)
{
    skip_reset = true;
    tft_init_7735(t, t->tabcolor);
    skip_reset = false;
}

void tft_init_7735(tft_t *t, int tabcolor)
{
    t->tabcolor = tabcolor;
    if (t->tabcolor == TFT_BLUETAB) {
        /* Initialize blue tab version. */
        t->size[0] = 128; t->size[1] = 160;
        t->offset[0] = 2; t->offset[1] = 1;
        t->rgb = true;

        tft_reset(t);
        writecommand(t, SWRESET);                /* software reset */
        sleep_us(50);
        writecommand(t, SLPOUT);                 /* out of sleep mode */
        sleep_us(500);

        writecommand(t, COLMOD);                 /* 16 bit color */
        writedata1(t, 0x05);
        sleep_us(10);

        { const uint8_t d[3] = { 0x00, 0x06, 0x03 };   /* fastest refresh, 6 lines front, 3 back */
          writecommand(t, FRMCTR1); writedata(t, d, 3); }
        sleep_us(10);

        writecommand(t, MADCTL);                 /* row/col address, bottom to top refresh */
        writedata1(t, 0x08);

        { const uint8_t d[2] = { 0x15, 0x02 }; writecommand(t, DISSET5); writedata(t, d, 2); }
        writecommand(t, INVCTR);                 /* line inversion */
        writedata1(t, 0x00);

        { const uint8_t d[2] = { 0x02, 0x70 }; writecommand(t, PWCTR1); writedata(t, d, 2); }  /* GVDD = 4.7V, 1.0uA */
        sleep_us(10);
        writecommand(t, PWCTR2);                 /* VGH = 14.7V, VGL = -7.35V */
        writedata1(t, 0x05);
        { const uint8_t d[2] = { 0x01, 0x02 }; writecommand(t, PWCTR3); writedata(t, d, 2); }  /* opamp current small, boost frequency */
        { const uint8_t d[2] = { 0x3C, 0x38 }; writecommand(t, VMCTR1); writedata(t, d, 2); }  /* VCOMH = 4V, VCOML = -1.1V */
        sleep_us(10);
        { const uint8_t d[2] = { 0x11, 0x15 }; writecommand(t, PWCTR6); writedata(t, d, 2); }

        writecommand(t, GMCTRP1);
        writedata(t, dataGMCTRP, 16);
        writecommand(t, GMCTRN1);
        writedata(t, dataGMCTRN, 16);
        sleep_us(10);

        writecommand(t, CASET);                  /* column address set */
        t->windowLocData[0] = 0x00;
        t->windowLocData[1] = (uint8_t)t->offset[0];
        t->windowLocData[2] = 0x00;
        t->windowLocData[3] = (uint8_t)(t->size[0] + t->offset[0]);
        writedata(t, t->windowLocData, 4);

        writecommand(t, RASET);                  /* row address set */
        t->windowLocData[1] = (uint8_t)t->offset[1];
        t->windowLocData[3] = (uint8_t)(t->size[1] + t->offset[1]);
        writedata(t, t->windowLocData, 4);

        writecommand(t, NORON);                  /* normal display on */
        sleep_us(10);
        writecommand(t, RAMWR);
        sleep_us(500);
        writecommand(t, DISPON);
        sleep_us(500);
    } else {
        /* Initialize a green tab version. */
        if (!skip_reset) {
            tft_reset(t);
            writecommand(t, SWRESET);            /* software reset */
            sleep_us(150);
        }
        writecommand(t, SLPOUT);                 /* out of sleep mode */
        sleep_us(255);

        uint8_t data3[3] = { 0x01, 0x2C, 0x2D };
        writecommand(t, FRMCTR1);                /* frame rate control */
        writedata(t, data3, 3);
        writecommand(t, FRMCTR2);
        writedata(t, data3, 3);
        { const uint8_t d[6] = { 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d };
          writecommand(t, FRMCTR3); writedata(t, d, 6); }
        sleep_us(10);

        writecommand(t, INVCTR);                 /* display inversion control */
        writedata1(t, 0x07);
        writecommand(t, PWCTR1);
        data3[0] = 0xA2; data3[1] = 0x02; data3[2] = 0x84;
        writedata(t, data3, 3);
        writecommand(t, PWCTR2);
        writedata1(t, 0xC5);
        { const uint8_t d[2] = { 0x0A, 0x00 }; writecommand(t, PWCTR3); writedata(t, d, 2); }
        { const uint8_t d[2] = { 0x8A, 0x2A }; writecommand(t, PWCTR4); writedata(t, d, 2); }
        { const uint8_t d[2] = { 0x8A, 0xEE }; writecommand(t, PWCTR5); writedata(t, d, 2); }
        writecommand(t, VMCTR1);
        writedata1(t, 0x0E);
        writecommand(t, INVOFF);

        if (t->tabcolor == TFT_GREENTAB) { t->offset[0] = 2; t->offset[1] = 1; }
        if (t->tabcolor == TFT_REDTAB)   { t->offset[0] = 0; t->offset[1] = 0; }
        if (t->tabcolor == TFT_BLACKTAB) {
            t->offset[0] = 0; t->offset[1] = 0;
            t->rgb = false;
        } else if (t->tabcolor == TFT_GREENTAB2) {
            t->offset[0] = 2; t->offset[1] = 1;
            t->rgb = false;
        } else if (t->tabcolor == TFT_GREENTAB3) {
            t->offset[0] = 2; t->offset[1] = 3;
        } else if (t->tabcolor == TFT_GREENTAB128x128) {
            t->size[0] = 128; t->size[1] = 128;
            t->offset[0] = 2; t->offset[1] = 1;
            t->rgb = false;
        } else if (t->tabcolor == TFT_GREENTAB80x160) {
            t->size[0] = 80; t->size[1] = 160;
            t->offset[0] = 26; t->offset[1] = 1;
            t->rgb = false;
            writecommand(t, INVON);
        } else if (t->tabcolor == TFT_REDTAB80x160) {
            t->size[0] = 80; t->size[1] = 160;
            t->offset[0] = 24; t->offset[1] = 0;
        }

        /* rotate to the same orientation with the pins at the top */
        if (t->tabcolor == TFT_GREENTAB80x160 || t->tabcolor == TFT_REDTAB80x160) rotation(t, 1);
        else                                                                       rotation(t, 2);

        /* set the color mapping of RGB or BGR */
        setMADCTL(t);

        writecommand(t, COLMOD);
        writedata1(t, 0x05);

        writecommand(t, CASET);                  /* column address set */
        t->windowLocData[0] = 0x00;
        t->windowLocData[1] = (uint8_t)t->offset[0];
        t->windowLocData[2] = 0x00;
        t->windowLocData[3] = (uint8_t)(t->size[0] + t->offset[0]);
        writedata(t, t->windowLocData, 4);

        writecommand(t, RASET);                  /* row address set */
        t->windowLocData[1] = (uint8_t)t->offset[1];
        t->windowLocData[3] = (uint8_t)(t->size[1] + t->offset[1]);
        writedata(t, t->windowLocData, 4);

        writecommand(t, GMCTRP1);
        writedata(t, dataGMCTRP, 16);
        writecommand(t, GMCTRN1);
        writedata(t, dataGMCTRN, 16);

        writecommand(t, NORON);                  /* normal display on */
        sleep_us(10);
        writecommand(t, DISPON);
        sleep_us(100);
    }
}
