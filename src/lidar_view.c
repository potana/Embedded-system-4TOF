#include "lidar_view.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "depth_color.h"

#define CX (LIDAR_VIEW_W / 2)
#define CY (LIDAR_VIEW_H / 2)
#define PI_F 3.14159265f

/* cos/sin of every bin angle; yaw and elevation bins are the same size. */
static float cos_t[LIDAR_YAW_BINS], sin_t[LIDAR_YAW_BINS];
static bool  tables_ready;

void lidar_view_set_angles(lidar_view_t *v, float az_deg, float el_deg)
{
    v->az_deg = az_deg;
    v->el_deg = el_deg;
    const float az = az_deg * (PI_F / 180.0f), el = el_deg * (PI_F / 180.0f);
    const float sa = sinf(az), ca = cosf(az), se = sinf(el), ce = cosf(el);

    /* camera sits at `toward` (times a large distance) looking at the origin */
    v->toward[0] = ce * ca;  v->toward[1] = ce * sa;  v->toward[2] = se;
    v->right[0]  = -sa;      v->right[1]  = ca;       v->right[2]  = 0.0f;
    v->up[0]     = -ca * se; v->up[1]     = -sa * se; v->up[2]     = ce;
}

void lidar_view_init(lidar_view_t *v, float az_deg, float el_deg, int max_mm)
{
    if (!tables_ready) {
        for (int i = 0; i < LIDAR_YAW_BINS; ++i) {
            const float a = (float)(i * LIDAR_STEP_DEG) * (PI_F / 180.0f);
            cos_t[i] = cosf(a);
            sin_t[i] = sinf(a);
        }
        tables_ready = true;
    }
    v->max_mm = max_mm;
    v->cutaway = true;
    /* the whole sphere of radius max + sensor offset fits in the view */
    v->px_per_mm = (float)(CY - LIDAR_DOT_PX - 1) / (float)(max_mm + LIDAR_SENSOR_RADIUS_MM);
    lidar_view_set_angles(v, az_deg, el_deg);
    lidar_view_clear(v);
}

static int round_i(float x) { return (int)floorf(x + 0.5f); }

/* Screen position and camera-ward depth (mm) of a point given as a direction
 * and a length from the scanner. */
static void project(const lidar_view_t *v, float dx, float dy, float dz, float len,
                    int *sx, int *sy, float *toward_mm)
{
    *sx = CX + round_i(len * v->px_per_mm *
                       (v->right[0] * dx + v->right[1] * dy + v->right[2] * dz));
    *sy = CY - round_i(len * v->px_per_mm *
                       (v->up[0] * dx + v->up[1] * dy + v->up[2] * dz));
    *toward_mm = len * (v->toward[0] * dx + v->toward[1] * dy + v->toward[2] * dz);
}

static void put_pixel(lidar_view_t *v, int x, int y, uint16_t c)
{
    if (x < 0 || x >= LIDAR_VIEW_W || y < 0 || y >= LIDAR_VIEW_H) return;
    v->color[y][x] = c;
    v->depth[y][x] = 0;
}

static void draw_axis(lidar_view_t *v, float dx, float dy, float dz)
{
    int x1, y1;
    float unused;
    project(v, dx, dy, dz, 450.0f, &x1, &y1, &unused);
    /* integer DDA from the origin pixel to the tip */
    int n = abs(x1 - CX) > abs(y1 - CY) ? abs(x1 - CX) : abs(y1 - CY);
    for (int i = 0; i <= n; ++i)
        put_pixel(v, CX + (x1 - CX) * i / (n ? n : 1), CY + (y1 - CY) * i / (n ? n : 1),
                  COLOR_WHITE);
}

void lidar_view_clear(lidar_view_t *v)
{
    memset(v->color, 0, sizeof v->color);
    memset(v->depth, 0, sizeof v->depth);
    draw_axis(v, 1.0f, 0.0f, 0.0f);
    draw_axis(v, 0.0f, 1.0f, 0.0f);
    draw_axis(v, 0.0f, 0.0f, 1.0f);
}

bool lidar_view_plot(lidar_view_t *v, int cell, int distance_mm, lidar_rect_t *dirty)
{
    if (cell < 0 || cell >= LIDAR_CELLS || distance_mm <= 0) return false;
    const int yb = cell / LIDAR_ELEV_BINS, eb = cell % LIDAR_ELEV_BINS;
    const float len = (float)(distance_mm + LIDAR_SENSOR_RADIUS_MM);

    int sx, sy;
    float t;
    project(v, cos_t[eb] * cos_t[yb], cos_t[eb] * sin_t[yb], sin_t[eb], len, &sx, &sy, &t);
    /* camera side of the scanner; 1 mm of slack keeps points on the cut plane */
    if (v->cutaway && t > 1.0f) return false;

    /* depth 1 (nearest the camera) .. 255 (farthest) over +-(max + radius) */
    const float span = (float)(v->max_mm + LIDAR_SENSOR_RADIUS_MM);
    int d = (int)((span - t) * (254.0f / (2.0f * span))) + 1;
    if (d < 1)   d = 1;
    if (d > 255) d = 255;

    const uint16_t color = depth_color565(distance_mm, v->max_mm);
    const int x0 = sx - LIDAR_DOT_PX / 2, y0 = sy - LIDAR_DOT_PX / 2;
    int minx = LIDAR_VIEW_W, miny = LIDAR_VIEW_H, maxx = -1, maxy = -1;
    for (int y = y0; y < y0 + LIDAR_DOT_PX; ++y) {
        for (int x = x0; x < x0 + LIDAR_DOT_PX; ++x) {
            if ((unsigned)x >= LIDAR_VIEW_W || (unsigned)y >= LIDAR_VIEW_H) continue;
            const int i = y * LIDAR_VIEW_W + x;
            uint8_t *depth = &v->depth[0][0];
            uint16_t *pixel = &v->color[0][0];
            if (depth[i] != 0 && depth[i] <= d) continue;   /* hidden */
            depth[i] = (uint8_t)d;
            pixel[i] = color;
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
        }
    }
    if (maxx < 0) return false;
    if (dirty != NULL) {
        dirty->x = minx;  dirty->y = miny;
        dirty->w = maxx - minx + 1;  dirty->h = maxy - miny + 1;
    }
    return true;
}

void lidar_view_render(lidar_view_t *v, const lidar_map_t *map)
{
    lidar_view_clear(v);
    for (int cell = 0; cell < LIDAR_CELLS; ++cell) {
        if (map->cell[cell] != 0)
            lidar_view_plot(v, cell, map->cell[cell] * LIDAR_UNIT_MM, NULL);
    }
}
