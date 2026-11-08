#include "lidar_map.h"

#include <assert.h>

int main(void)
{
    lidar_map_t map;
    lidar_map_clear(&map);
    assert(lidar_map_valid_count(&map) == 0);

    /* S1 starts at 0 degrees. */
    assert(lidar_map_update(&map, 0, 0, 1000, true) == 0);
    assert(map.point[0].distance_mm == 1000);

    /* S2 initially owns 90 degrees. */
    assert(lidar_map_update(&map, 1, 0, 900, true) == 18);
    assert(map.point[18].sensor_id == 1);

    /* After 90 degrees clockwise, S1 reaches 90 and replaces S2. */
    assert(lidar_map_update(&map, 0, 90, 700, true) == 18);
    assert(map.point[18].sensor_id == 0);
    assert(map.point[18].distance_mm == 700);

    /* An invalid newer sample clears the stale obstacle. */
    assert(lidar_map_update(&map, 0, 90, 0, false) == 18);
    assert(!map.point[18].valid);
    assert(lidar_map_valid_count(&map) == 1); /* S1's original 0-degree point */

    assert(lidar_map_update(&map, 4, 0, 100, true) == -1);
    return 0;
}
