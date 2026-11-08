#ifndef LIDAR_MAP_H
#define LIDAR_MAP_H

#include <stdbool.h>
#include <stdint.h>

#define LIDAR_STEP_DEG 5
#define LIDAR_BINS (360 / LIDAR_STEP_DEG)

typedef struct {
    uint16_t distance_mm;
    uint8_t sensor_id;
    bool valid;
} lidar_point_t;

typedef struct {
    lidar_point_t point[LIDAR_BINS];
} lidar_map_t;

void lidar_map_clear(lidar_map_t *map);

/*
 * Update the global bearing reached by one of four sensors mounted 90 degrees
 * apart. rotation_deg is the clockwise rotation of the complete sensor head;
 * increasing rotation increases the global bearing in this project's display
 * convention.
 * A new invalid result deliberately clears the older point at that bearing.
 */
int lidar_map_update(lidar_map_t *map, uint8_t sensor_id,
                     int rotation_deg, int distance_mm, bool valid);

int lidar_map_valid_count(const lidar_map_t *map);

#endif
