#include "lidar_map.h"

#include <math.h>
#include <string.h>

/* Angle of each sensor round the ring at pitch 0 (matches pointcloud.py). */
static const int sensor_mount_deg[4] = {0, 90, 180, 270};

static int bin_of(float deg, int bins)
{
    int b = (int)floorf(deg / (float)LIDAR_STEP_DEG + 0.5f) % bins;
    return b < 0 ? b + bins : b;
}

void lidar_map_clear(lidar_map_t *map)
{
    if (map != NULL) memset(map, 0, sizeof(*map));
}

int lidar_map_update(lidar_map_t *map, uint8_t sensor_id,
                     float yaw_deg, float pitch_deg,
                     int distance_mm, bool valid)
{
    if (map == NULL || sensor_id >= 4) return -1;

    const int yaw_bin  = bin_of(yaw_deg, LIDAR_YAW_BINS);
    const int elev_bin = bin_of((float)sensor_mount_deg[sensor_id] + pitch_deg,
                                LIDAR_ELEV_BINS);
    const int cell = yaw_bin * LIDAR_ELEV_BINS + elev_bin;

    if (!valid) {
        map->cell[cell] = 0;
    } else {
        int units = (distance_mm + LIDAR_UNIT_MM / 2) / LIDAR_UNIT_MM;
        if (units < 1)   units = 1;      /* 0 means "empty" */
        if (units > 255) units = 255;
        map->cell[cell] = (uint8_t)units;
    }
    return cell;
}

int lidar_map_distance_mm(const lidar_map_t *map, int cell)
{
    if (map == NULL || cell < 0 || cell >= LIDAR_CELLS) return 0;
    return map->cell[cell] * LIDAR_UNIT_MM;
}

int lidar_map_valid_count(const lidar_map_t *map)
{
    if (map == NULL) return 0;
    int count = 0;
    for (int i = 0; i < LIDAR_CELLS; ++i) count += map->cell[i] != 0;
    return count;
}
