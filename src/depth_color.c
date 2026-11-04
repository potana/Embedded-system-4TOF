#include "depth_color.h"

uint16_t depth_color565(int dist_mm, int max_mm)
{
    if (max_mm <= 0) return 0x001F;
    int t = dist_mm;
    if (t < 0) t = 0;
    if (t > max_mm) t = max_mm;
    /* position along the gradient, 0..4096 -> four segments of 1024 */
    int pos = (int)(((long)t * 4096) / max_mm);
    int seg = pos >> 10;
    int f = pos & 1023;
    int up = (f * 255) / 1023;
    int down = 255 - up;
    uint8_t r, g, b;
    switch (seg) {
    case 0:  r = 255;  g = (uint8_t)up;   b = 0;           break; /* red -> yellow  */
    case 1:  r = (uint8_t)down; g = 255;  b = 0;           break; /* yellow -> green */
    case 2:  r = 0;    g = 255;  b = (uint8_t)up;          break; /* green -> cyan  */
    case 3:  r = 0;    g = (uint8_t)down; b = 255;         break; /* cyan -> blue   */
    default: r = 0;    g = 0;    b = 255;                  break; /* at max: blue   */
    }
    return rgb565(r, g, b);
}
