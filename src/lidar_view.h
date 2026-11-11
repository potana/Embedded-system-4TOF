/* 3D point-cloud view of a lidar_map_t, drawn into an RGB565 framebuffer.
 *
 * Orthographic projection with an orbiting camera (azimuth + elevation), a
 * per-pixel depth buffer (nearest point wins) and a "cutaway": points on the
 * camera's side of the scanner are dropped, so for a 360 degree room scan you
 * look at the far walls from outside, like a doll's house, instead of at the
 * near wall hiding everything else.  Dots are coloured by sensor range
 * (depth_color.c).  World frame as tools/pointcloud.py: +X forward at yaw 0,
 * +Y left, +Z up.
 *
 * The framebuffer mirrors the screen region it is blitted to, so a new point
 * only needs its few changed pixels sent over the (slow) software SPI.
 * Pure C with no hardware access, so it is unit-tested on the PC.
 */
#ifndef LIDAR_VIEW_HEADER_H      /* not LIDAR_VIEW_H: that is the view height */
#define LIDAR_VIEW_HEADER_H

#include <stdbool.h>
#include <stdint.h>
#include "lidar_map.h"

#define LIDAR_VIEW_W 128
#define LIDAR_VIEW_H 122            /* rows above the text read-out strip */
#define LIDAR_DOT_PX 2              /* each point is drawn DOT x DOT pixels */
#define LIDAR_SENSOR_RADIUS_MM 25   /* sensor offset from the axis, as pointcloud.py */

typedef struct {
    uint16_t color[LIDAR_VIEW_H][LIDAR_VIEW_W];
    uint8_t  depth[LIDAR_VIEW_H][LIDAR_VIEW_W];   /* 0 = empty, 1 = nearest .. 255 */
    float    az_deg, el_deg;
    float    right[3], up[3], toward[3];          /* camera basis in the world frame */
    float    px_per_mm;
    int      max_mm;                              /* range that maps to the far colour */
    bool     cutaway;
} lidar_view_t;

typedef struct { int x, y, w, h; } lidar_rect_t;

/* Build the lookup tables and point the camera.  az_deg is the direction the
 * camera sits in (0 = on the +X axis looking back at the scanner). */
void lidar_view_init(lidar_view_t *v, float az_deg, float el_deg, int max_mm);
void lidar_view_set_angles(lidar_view_t *v, float az_deg, float el_deg);

/* Black framebuffer with the +X, +Y and +Z axes drawn from the scanner. */
void lidar_view_clear(lidar_view_t *v);

/* Plot one map cell at the given distance.  Returns true when any pixel changed
 * and sets *dirty to the rectangle that changed (clipped to the view). */
bool lidar_view_plot(lidar_view_t *v, int cell, int distance_mm, lidar_rect_t *dirty);

/* Clear, then plot every occupied cell of the map. */
void lidar_view_render(lidar_view_t *v, const lidar_map_t *map);

#endif
