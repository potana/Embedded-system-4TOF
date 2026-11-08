"""Ray-cast a synthetic room so the pipeline can run without the gimbal."""
from __future__ import annotations

import math
import random
from typing import Iterator, List, Optional

from pointcloud import Geometry, Sample, sensor_ray

# Room is an axis-aligned box [lo, hi] around the motor, plus one sphere.
ROOM_LO = (-1200.0, -900.0, -700.0)
ROOM_HI = (1300.0, 1000.0, 800.0)
SPHERE_C = (700.0, -200.0, -300.0)
SPHERE_R = 250.0


def cast(origin, d, max_mm: float) -> Optional[float]:
    """Distance along unit ray d from origin to the nearest surface, or None."""
    best = None
    # inside of the box: nearest exit along each axis
    for a in range(3):
        if abs(d[a]) < 1e-9:
            continue
        t = ((ROOM_HI[a] if d[a] > 0 else ROOM_LO[a]) - origin[a]) / d[a]
        best = t if best is None else min(best, t)
    # sphere
    oc = [origin[i] - SPHERE_C[i] for i in range(3)]
    b = sum(oc[i] * d[i] for i in range(3))
    c = sum(v * v for v in oc) - SPHERE_R ** 2
    disc = b * b - c
    if disc >= 0:
        t = -b - math.sqrt(disc)
        if t > 0 and (best is None or t < best):
            best = t
    return best if best is not None and best < max_mm else None


def scan(geo: Geometry, yaw_step: float = 10.0, pitch_step: float = 10.0,
         noise_mm: float = 4.0, outlier_rate: float = 0.01, max_mm: float = 2000.0,
         seed: int = 1) -> Iterator[Sample]:
    """Yield the samples a real scan would produce (nothing detected -> 8190)."""
    rng = random.Random(seed)
    yaw = 0.0
    while yaw < 360.0:
        pitch = 0.0
        while pitch < 90.0:
            for s in range(len(geo.sensors)):
                o, d = sensor_ray(geo, s, yaw, pitch)
                t = cast(o, d, max_mm)
                if t is None:
                    t = 8190.0
                else:
                    t += rng.gauss(0.0, noise_mm)
                    if rng.random() < outlier_rate:      # spurious isolated return
                        t = rng.uniform(100.0, max_mm - 1)
                yield Sample(yaw, pitch, s, t)
            pitch += pitch_step
        yaw += yaw_step
