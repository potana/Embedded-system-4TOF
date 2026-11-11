/* Host test:
 *   gcc -Wall -Wextra -Isrc tests/test_lidar_view.c src/lidar_view.c src/lidar_map.c \
 *       src/depth_color.c -lm
 */
#include "lidar_view.h"

#include <assert.h>
#include "depth_color.h"

#define CELL(yaw_deg, elev_deg) \
    (((yaw_deg) / LIDAR_STEP_DEG) * LIDAR_ELEV_BINS + (elev_deg) / LIDAR_STEP_DEG)

static lidar_view_t v;
static lidar_map_t map;

static int lit_pixels(void)
{
    int n = 0;
    for (int y = 0; y < LIDAR_VIEW_H; ++y)
        for (int x = 0; x < LIDAR_VIEW_W; ++x) n += v.depth[y][x] != 0;
    return n;
}

int main(void)
{
    /* Camera on -X looking along +X, level: right = -Y, up = +Z. */
    lidar_view_init(&v, 180.0f, 0.0f, 2000);
    lidar_rect_t r;

    /* straight ahead lands in the middle of the view */
    assert(lidar_view_plot(&v, CELL(0, 0), 1000, &r));
    assert(r.x <= LIDAR_VIEW_W / 2 && r.x + r.w > LIDAR_VIEW_W / 2 - 1);
    assert(r.y <= LIDAR_VIEW_H / 2 && r.y + r.h > LIDAR_VIEW_H / 2 - 1);

    /* +Y (yaw 90) is on the left of the screen, up (elevation 90) is higher */
    assert(lidar_view_plot(&v, CELL(90, 0), 1000, &r));
    assert(r.x + r.w <= LIDAR_VIEW_W / 2);
    assert(lidar_view_plot(&v, CELL(0, 90), 1000, &r));
    assert(r.y + r.h <= LIDAR_VIEW_H / 2);
    /* -Y (yaw 270) is on the right */
    assert(lidar_view_plot(&v, CELL(270, 0), 1000, &r));
    assert(r.x >= LIDAR_VIEW_W / 2);

    /* behind the scanner = camera side: cut away */
    assert(!lidar_view_plot(&v, CELL(180, 0), 1000, &r));
    v.cutaway = false;
    assert(lidar_view_plot(&v, CELL(180, 0), 1000, &r));
    v.cutaway = true;

    /* Occlusion.  Camera on -X: points along +X at 500 and 1500 mm share one
     * pixel and the 500 mm one is nearer the camera, so it wins in either
     * drawing order and the farther one is rejected. */
    lidar_view_init(&v, 180.0f, 0.0f, 2000);
    const int mid_y = LIDAR_VIEW_H / 2, mid_x = LIDAR_VIEW_W / 2;
    assert(lidar_view_plot(&v, CELL(0, 0), 1500, &r));
    assert(v.color[mid_y][mid_x] == depth_color565(1500, 2000));
    assert(lidar_view_plot(&v, CELL(0, 0), 500, &r));       /* nearer: overwrites */
    assert(v.color[mid_y][mid_x] == depth_color565(500, 2000));
    assert(!lidar_view_plot(&v, CELL(0, 0), 1500, &r));     /* farther: hidden */
    assert(v.color[mid_y][mid_x] == depth_color565(500, 2000));
    lidar_view_clear(&v);
    assert(lidar_view_plot(&v, CELL(0, 0), 500, &r));
    assert(!lidar_view_plot(&v, CELL(0, 0), 1500, &r));     /* other order, same result */
    assert(v.color[mid_y][mid_x] == depth_color565(500, 2000));

    /* render() draws every occupied cell and is repeatable */
    lidar_map_clear(&map);
    for (int yaw = 0; yaw < 360; yaw += 2)
        for (int el = 0; el < 360; el += 2)
            lidar_map_update(&map, 0, (float)yaw, (float)el, 1000, true);
    lidar_view_init(&v, 210.0f, 25.0f, 2000);
    lidar_view_render(&v, &map);
    const int first = lit_pixels();
    assert(first > 500);                                    /* a visible shell */
    lidar_view_render(&v, &map);
    assert(lit_pixels() == first);

    /* an empty map shows only the three axes */
    lidar_map_clear(&map);
    lidar_view_render(&v, &map);
    assert(lit_pixels() == 0);                              /* axes have depth 0 */
    assert(v.color[LIDAR_VIEW_H / 2][LIDAR_VIEW_W / 2] == 0xFFFF);
    return 0;
}
