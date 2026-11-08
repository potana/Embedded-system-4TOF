#include "lidar_map.h"

#include <string.h>

static int normalize_deg(int deg)
{
    deg %= 360;
    return deg < 0 ? deg + 360 : deg;
}

void lidar_map_clear(lidar_map_t *map)
{
    if (map != NULL) memset(map, 0, sizeof(*map));
}

int lidar_map_update(lidar_map_t *map, uint8_t sensor_id,
                     int rotation_deg, int distance_mm, bool valid)
{
    static const int sensor_bearing_deg[4] = {0, 90, 180, 270};
    if (map == NULL || sensor_id >= 4) return -1;

    /*
     * Project convention: clockwise head rotation increases the displayed
     * bearing. S1 therefore moves 0, 5, ... 90 degrees and then replaces the
     * original 90-degree sample supplied by S2.
     */
    const int bearing = normalize_deg(sensor_bearing_deg[sensor_id] + rotation_deg);
    const int bin = (bearing + LIDAR_STEP_DEG / 2) / LIDAR_STEP_DEG % LIDAR_BINS;
    lidar_point_t *point = &map->point[bin];
    point->sensor_id = sensor_id;
    point->valid = valid;
    point->distance_mm = valid ? (uint16_t)distance_mm : 0;
    return bin;
}

int lidar_map_valid_count(const lidar_map_t *map)
{
    if (map == NULL) return 0;
    int count = 0;
    for (int i = 0; i < LIDAR_BINS; ++i) count += map->point[i].valid ? 1 : 0;
    return count;
}
