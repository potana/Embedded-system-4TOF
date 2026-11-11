/* Host test:  gcc -Wall -Wextra -Isrc tests/test_lidar_map.c src/lidar_map.c -lm */
#include "lidar_map.h"

#include <assert.h>

#define CELL(yaw_bin, elev_bin) ((yaw_bin) * LIDAR_ELEV_BINS + (elev_bin))
#define PER_DEG (1.0f / LIDAR_STEP_DEG)

int main(void)
{
    static lidar_map_t map;
    lidar_map_clear(&map);
    assert(lidar_map_valid_count(&map) == 0);

    /* finer than 5 degrees, and the map really has that many cells */
    assert(LIDAR_STEP_DEG < 5);
    assert(LIDAR_CELLS == (360 / LIDAR_STEP_DEG) * (360 / LIDAR_STEP_DEG));

    /* Pose yaw 0, pitch 0: S1 looks forward (elevation 0), S2 up (90),
     * S3 back (180), S4 down (270). */
    assert(lidar_map_update(&map, 0, 0, 0, 1000, true) == CELL(0, 0));
    assert(lidar_map_update(&map, 1, 0, 0,  900, true) == CELL(0, 90 * PER_DEG));
    assert(lidar_map_update(&map, 2, 0, 0,  800, true) == CELL(0, 180 * PER_DEG));
    assert(lidar_map_update(&map, 3, 0, 0,  700, true) == CELL(0, 270 * PER_DEG));
    assert(lidar_map_valid_count(&map) == 4);
    assert(lidar_map_distance_mm(&map, CELL(0, 0)) == 1000);   /* 125 * 8 */

    /* Pitching the ring by 90 puts S1 where S2 started: it takes the cell. */
    assert(lidar_map_update(&map, 0, 0, 90, 650, true) == CELL(0, 90 * PER_DEG));
    assert(lidar_map_distance_mm(&map, CELL(0, 90 * PER_DEG)) == 648);   /* 81 * 8 */

    /* Yaw picks the other axis, and wraps (-90 == 270, 450 == 90). */
    assert(lidar_map_update(&map, 0, -90, 0, 500, true) == CELL(270 * PER_DEG, 0));
    assert(lidar_map_update(&map, 0, 450, 0, 500, true) == CELL(90 * PER_DEG, 0));

    /* Sub-5-degree poses land in different cells. */
    const int a = lidar_map_update(&map, 0, 10.0f, 0, 400, true);
    const int b = lidar_map_update(&map, 0, 11.0f, 0, 400, true);
    assert(a != b);
    assert(lidar_map_update(&map, 0, 10.2f, 0, 400, true) == a);   /* same cell */

    /* An invalid newer sample clears the stale obstacle. */
    const int before = lidar_map_valid_count(&map);
    assert(lidar_map_update(&map, 0, 10.0f, 0, 0, false) == a);
    assert(lidar_map_distance_mm(&map, a) == 0);
    assert(lidar_map_valid_count(&map) == before - 1);

    /* Tiny and huge distances stay representable and non-empty. */
    const int c = lidar_map_update(&map, 0, 200.0f, 0, 1, true);
    assert(lidar_map_distance_mm(&map, c) > 0);
    const int e = lidar_map_update(&map, 0, 201.0f, 0, 5000, true);
    assert(lidar_map_distance_mm(&map, e) == 255 * LIDAR_UNIT_MM);

    assert(lidar_map_update(&map, 4, 0, 0, 100, true) == -1);
    assert(lidar_map_update(NULL, 0, 0, 0, 100, true) == -1);
    return 0;
}
