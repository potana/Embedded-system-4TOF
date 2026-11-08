"""Geometry and filtering for the 2-axis gimbal + 4 ToF sensor scanner.

Pure standard library, so it runs (and is tested) without numpy.

Coordinate frame (right-handed, origin = where the yaw and pitch axes meet):
    +X  forward at yaw 0
    +Y  left
    +Z  up
Yaw spins everything about +Z (counter-clockwise seen from above is positive).
Pitch tilts the sensor ring about its own axis; positive pitch raises the beam
that points forward at pitch 0.

The 4 sensors sit on a ring whose axis is the pitch axis, 90 degrees apart:
at pitch 0 sensor 0 looks forward, sensor 1 up, sensor 2 back, sensor 3 down.
One 90 degree pitch sweep therefore covers the full 360 degrees of elevation.
Each sensor is displaced from the origin by `radius_mm` outwards along its beam
and `axial_mm` along the pitch axis; the measured range starts at the sensor.
"""
from __future__ import annotations

import json
import math
from collections import defaultdict
from dataclasses import dataclass, field, asdict
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float, float]


@dataclass
class SensorMount:
    mount_deg: float          # angle round the ring at pitch 0 (0 = forward, 90 = up)
    radius_mm: float = 0.0    # distance from the pitch axis, along the beam
    axial_mm: float = 0.0     # offset along the pitch axis (+Y at yaw 0, pitch 0)


def _default_sensors() -> List[SensorMount]:
    # Placeholder displacement: measure your head and replace these.
    return [SensorMount(90.0 * i, radius_mm=25.0) for i in range(4)]


@dataclass
class Geometry:
    sensors: List[SensorMount] = field(default_factory=_default_sensors)
    # Pitch axis position relative to the origin, in the yaw-0 frame (mm).
    pitch_axis_offset_mm: Point = (0.0, 0.0, 0.0)
    # Map motor encoder degrees to physical angles:  angle = sign * (raw - zero)
    yaw_sign: float = 1.0
    yaw_zero_deg: float = 0.0
    pitch_sign: float = 1.0
    pitch_zero_deg: float = 0.0

    @staticmethod
    def load(path: str) -> "Geometry":
        with open(path) as f:
            d = json.load(f)
        d["sensors"] = [SensorMount(**s) for s in d.get("sensors", [])] or _default_sensors()
        if "pitch_axis_offset_mm" in d:
            d["pitch_axis_offset_mm"] = tuple(d["pitch_axis_offset_mm"])
        return Geometry(**d)

    def dump(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)


def sensor_ray(geo: Geometry, sensor: int, yaw_deg: float, pitch_deg: float
               ) -> Tuple[Point, Point]:
    """World-frame (origin of the beam, unit direction) for one sensor pose."""
    s = geo.sensors[sensor]
    yaw = math.radians(geo.yaw_sign * (yaw_deg - geo.yaw_zero_deg))
    pitch = math.radians(geo.pitch_sign * (pitch_deg - geo.pitch_zero_deg))
    ring = math.radians(s.mount_deg) + pitch          # beam angle in the X-Z plane
    ux, uz = math.cos(ring), math.sin(ring)           # beam direction, yaw 0
    ox, oy, oz = geo.pitch_axis_offset_mm
    px = ox + s.radius_mm * ux                        # sensor position, yaw 0
    py = oy + s.axial_mm
    pz = oz + s.radius_mm * uz
    cy, sy = math.cos(yaw), math.sin(yaw)

    def rot(x: float, y: float, z: float) -> Point:
        return (x * cy - y * sy, x * sy + y * cy, z)

    return rot(px, py, pz), rot(ux, 0.0, uz)


def to_point(geo: Geometry, sensor: int, yaw_deg: float, pitch_deg: float,
             distance_mm: float) -> Point:
    """The measured 3D point, relative to the motor origin."""
    (ox, oy, oz), (dx, dy, dz) = sensor_ray(geo, sensor, yaw_deg, pitch_deg)
    return (ox + distance_mm * dx, oy + distance_mm * dy, oz + distance_mm * dz)


@dataclass
class Sample:
    yaw_deg: float
    pitch_deg: float
    sensor: int
    distance_mm: float


def samples_to_points(geo: Geometry, samples: Iterable[Sample], min_mm: float = 1.0,
                      max_mm: float = 2000.0) -> List[Tuple[Point, float]]:
    """Convert samples to (point, distance) pairs, dropping out-of-range readings."""
    out = []
    for s in samples:
        if not (min_mm <= s.distance_mm < max_mm):
            continue
        out.append((to_point(geo, s.sensor, s.yaw_deg, s.pitch_deg, s.distance_mm),
                    s.distance_mm))
    return out


def remove_outliers(points: Sequence[Point], ranges: Optional[Sequence[float]] = None,
                    angular_step_deg: float = 10.0, spacing_factor: float = 2.0,
                    min_radius_mm: float = 30.0, min_neighbours: int = 2
                    ) -> List[bool]:
    """Radius outlier removal; returns a keep-mask.

    A point is kept when at least `min_neighbours` other points lie within its
    search radius.  Samples are taken on an angular grid, so neighbours on a
    surface are ~ range * step apart; the radius therefore grows with range:
        radius = max(min_radius_mm, spacing_factor * range * step_rad)
    `ranges` is the sensor distance of each point (defaults to distance from the
    origin).  Points with no neighbours (isolated returns, e.g. a reflection or
    a spurious reading) are dropped.
    """
    n = len(points)
    if n == 0:
        return []
    if ranges is None:
        ranges = [math.sqrt(x * x + y * y + z * z) for x, y, z in points]
    step = math.radians(angular_step_deg)
    radii = [max(min_radius_mm, spacing_factor * r * step) for r in ranges]
    cell = max(radii)

    grid = defaultdict(list)
    for i, (x, y, z) in enumerate(points):
        grid[(int(x // cell), int(y // cell), int(z // cell))].append(i)

    keep = [False] * n
    for i, (x, y, z) in enumerate(points):
        r2 = radii[i] ** 2
        cx, cy, cz = int(x // cell), int(y // cell), int(z // cell)
        found = 0
        for gx in (cx - 1, cx, cx + 1):
            for gy in (cy - 1, cy, cy + 1):
                for gz in (cz - 1, cz, cz + 1):
                    for j in grid.get((gx, gy, gz), ()):
                        if j == i:
                            continue
                        px, py, pz = points[j]
                        if (px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2 <= r2:
                            found += 1
                            if found >= min_neighbours:
                                break
                    if found >= min_neighbours:
                        break
                if found >= min_neighbours:
                    break
            if found >= min_neighbours:
                break
        keep[i] = found >= min_neighbours
    return keep


def process(geo: Geometry, samples: Iterable[Sample], max_mm: float = 2000.0,
            **filter_kwargs) -> Tuple[List[Tuple[Point, float]], int]:
    """samples -> filtered (point, distance) list, plus how many were removed."""
    pts = samples_to_points(geo, samples, max_mm=max_mm)
    keep = remove_outliers([p for p, _ in pts], [d for _, d in pts], **filter_kwargs)
    kept = [pd for pd, k in zip(pts, keep) if k]
    return kept, len(pts) - len(kept)


def write_ply(path: str, pts: Sequence[Tuple[Point, float]]) -> None:
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex %d\n" % len(pts))
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property float range_mm\nend_header\n")
        for (x, y, z), d in pts:
            f.write("%.2f %.2f %.2f %.1f\n" % (x, y, z, d))
