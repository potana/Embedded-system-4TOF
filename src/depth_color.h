/* Distance -> colour for the point cloud and the depth picture. */
#ifndef DEPTH_COLOR_H
#define DEPTH_COLOR_H

#include <stdint.h>

/* RGB565 helpers */
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_YELLOW  0xFFE0
#define COLOR_GREY    0x4208   /* "no valid reading" */

/* Near = red, then yellow, green, cyan, blue = far.
 * dist_mm <= 0 gives red, dist_mm >= max_mm gives blue. */
uint16_t depth_color565(int dist_mm, int max_mm);

#endif
