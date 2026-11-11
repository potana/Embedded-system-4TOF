#ifndef LIDAR_MAP_H
#define LIDAR_MAP_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 3D map of everything the four sensors have seen, as a sphere of cells indexed
 * by (yaw, elevation).  It uses the same geometry as tools/pointcloud.py:
 *
 *   - the four sensors sit on a ring 90 degrees apart (mount angles 0, 90, 180,
 *     270: forward, up, back, down at pitch 0);
 *   - pitch tilts the ring, so a sensor looks along elevation = mount + pitch
 *     (0 = horizon, 90 = straight up), and one 90 degree pitch sweep covers
 *     every elevation once;
 *   - yaw spins the whole head about the vertical axis.
 *
 * LIDAR_STEP_DEG is the size of a cell in both angles.  It must divide 360.
 * One cell is one byte (a distance in LIDAR_UNIT_MM steps, 0 = nothing known),
 * so the map costs (360 / step)^2 bytes: 129 600 at 1 degree, 32 400 at 2.
 * 1 degree is already finer than the 25-27 degree beam of the sensors; going
 * lower would need a wider cell type and does not add information.
 */
#define LIDAR_STEP_DEG   1
#define LIDAR_YAW_BINS   (360 / LIDAR_STEP_DEG)
#define LIDAR_ELEV_BINS  (360 / LIDAR_STEP_DEG)
#define LIDAR_CELLS      (LIDAR_YAW_BINS * LIDAR_ELEV_BINS)

#define LIDAR_UNIT_MM    8       /* one stored step; 255 steps = 2040 mm */

_Static_assert(360 % LIDAR_STEP_DEG == 0, "LIDAR_STEP_DEG must divide 360");
_Static_assert(LIDAR_YAW_BINS == LIDAR_ELEV_BINS, "views share one trig table");

typedef struct {
    uint8_t cell[LIDAR_CELLS];   /* cell[yaw_bin * LIDAR_ELEV_BINS + elev_bin] */
} lidar_map_t;

void lidar_map_clear(lidar_map_t *map);

/*
 * Record one reading.  yaw_deg / pitch_deg are the gimbal's physical angles at
 * the moment of the reading (any value; they are wrapped).  Returns the cell
 * index, or -1 for a bad sensor_id.  A new invalid result deliberately clears
 * the older point in that cell.  Valid distances are quantised to
 * LIDAR_UNIT_MM and never stored as 0.
 */
int lidar_map_update(lidar_map_t *map, uint8_t sensor_id,
                     float yaw_deg, float pitch_deg,
                     int distance_mm, bool valid);

/* Stored distance of a cell in mm, 0 if it is empty. */
int lidar_map_distance_mm(const lidar_map_t *map, int cell);

int lidar_map_valid_count(const lidar_map_t *map);

#endif
