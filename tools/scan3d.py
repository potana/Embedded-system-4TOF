#!/usr/bin/env python3
"""3D scanner host tool.

    scan3d.py sim                       demo with a simulated room (no hardware)
    scan3d.py scan --gimbal COM5 --tof COM7
    scan3d.py view samples.csv          re-process / re-plot a saved scan

`scan` drives the gimbal Pico (serial protocol of the gimbal firmware:
"M <motor> <deg> <sec>" -> "D <motor> <deg>", "?" -> "A <yaw> <pitch>") and asks
the sensor Pico for a reading with "S" -> "S d1 d2 d3 d4".  Needs pyserial.
Plotting needs matplotlib.  Everything is saved as raw samples first
(<out>.csv), then filtered and exported (<out>.ply).
"""
from __future__ import annotations

import argparse
import csv
import sys
import time

import pointcloud as pc
import simulate


# ---------------------------------------------------------------- hardware ---
class Gimbal:
    def __init__(self, port: str, yaw_motor: int, pitch_motor: int, speed_dps: float):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.yaw_m, self.pitch_m, self.speed = yaw_motor, pitch_motor, speed_dps
        self.ser.reset_input_buffer()
        self.angles()                      # raises if the gimbal is not answering

    def _line(self, deadline: float):
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                yield line

    def angles(self, timeout: float = 15.0):
        """Live (yaw, pitch) from the encoders.  Waits out the start-up calibration."""
        end = time.time() + timeout
        while time.time() < end:
            self.ser.write(b"?\n")
            for line in self._line(time.time() + 1.0):
                f = line.split()
                if len(f) == 3 and f[0] == "A":
                    a = {1: float(f[1]), 2: float(f[2])}
                    return a[self.yaw_m], a[self.pitch_m]
        raise TimeoutError("gimbal did not answer '?'")

    def move(self, yaw: float, pitch: float, cur) -> tuple:
        """Move both axes and block until both report done; returns measured angles."""
        dur = max(0.3, max(abs(yaw - cur[0]), abs(pitch - cur[1])) / self.speed)
        for m, target in ((self.yaw_m, yaw), (self.pitch_m, pitch)):
            self.ser.write(("M %d %.3f %.3f\n" % (m, target, dur)).encode())
        pending = {self.yaw_m, self.pitch_m}
        for line in self._line(time.time() + dur + 10.0):
            f = line.split()
            if len(f) == 3 and f[0] == "D" and int(f[1]) in pending:
                pending.discard(int(f[1]))
                if not pending:
                    break
        else:
            raise TimeoutError("gimbal never reported the move done")
        time.sleep(0.15)                              # let the head stop ringing
        return self.angles()


class Tof:
    def __init__(self, port: str):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=2.0)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def read(self):
        """Four distances in mm; None where the sensor had an I2C error."""
        self.ser.write(b"S\n")
        end = time.time() + 5.0
        while time.time() < end:
            f = self.ser.readline().decode(errors="ignore").split()
            if len(f) == 5 and f[0] == "S":
                return [None if int(v) < 0 else float(v) for v in f[1:]]
        raise TimeoutError("sensor board did not answer 'S'")


def frange(lo: float, hi: float, step: float):
    v = lo
    while v < hi - 1e-9:
        yield v
        v += step


def live_scan(args):
    gimbal = Gimbal(args.gimbal, args.yaw_motor, args.pitch_motor, args.speed)
    tof = Tof(args.tof)
    samples = []
    cur = gimbal.angles()
    yaws = list(frange(args.yaw_min, args.yaw_max, args.yaw_step))
    pitches = list(frange(args.pitch_min, args.pitch_max, args.pitch_step))
    total = len(yaws) * len(pitches)
    i = 0
    for yi, yaw in enumerate(yaws):
        # boustrophedon: alternate pitch direction so the head never flies back
        for pitch in (pitches if yi % 2 == 0 else reversed(pitches)):
            cur = gimbal.move(yaw, pitch, cur)
            i += 1
            for sensor, d in enumerate(tof.read()):
                if d is not None:
                    # record the *measured* angles, not the commanded ones
                    samples.append(pc.Sample(cur[0], cur[1], sensor, d))
            print("pose %d/%d yaw=%.1f pitch=%.1f" % (i, total, *cur), flush=True)
    gimbal.move(0.0, 0.0, cur)
    return samples


# ------------------------------------------------------------------ output ---
def save_csv(path, samples):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["yaw_deg", "pitch_deg", "sensor", "distance_mm"])
        for s in samples:
            w.writerow(["%.3f" % s.yaw_deg, "%.3f" % s.pitch_deg, s.sensor,
                        "%.1f" % s.distance_mm])


def load_csv(path):
    with open(path, newline="") as f:
        return [pc.Sample(float(r["yaw_deg"]), float(r["pitch_deg"]),
                          int(r["sensor"]), float(r["distance_mm"]))
                for r in csv.DictReader(f)]


def plot(pts, max_mm, title, out_png=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection="3d")
    xs, ys, zs = ([p[0][i] for p in pts] for i in range(3))
    sc = ax.scatter(xs, ys, zs, c=[p[1] for p in pts], cmap="turbo_r", s=6,
                    vmin=0, vmax=max_mm)
    ax.scatter([0], [0], [0], c="k", marker="x", s=80, label="motor (origin)")
    for vec, col in (((1, 0, 0), "r"), ((0, 1, 0), "g"), ((0, 0, 1), "b")):
        ax.plot([0, 200 * vec[0]], [0, 200 * vec[1]], [0, 200 * vec[2]], col, lw=2)
    if pts:                                       # equal axes so shapes aren't distorted
        r = max(max(abs(v) for v in c) for c in (xs, ys, zs))
        ax.set_xlim(-r, r), ax.set_ylim(-r, r), ax.set_zlim(-r, r)
        ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X forward (mm)"), ax.set_ylabel("Y left (mm)"), ax.set_zlabel("Z up (mm)")
    fig.colorbar(sc, label="sensor range (mm)", shrink=0.6)
    ax.set_title(title)
    ax.legend(loc="upper left")
    if out_png:
        fig.savefig(out_png, dpi=130)
    if show:
        plt.show()


def finish(samples, args):
    geo = pc.Geometry.load(args.geometry) if args.geometry else pc.Geometry()
    kept, removed = pc.process(
        geo, samples, max_mm=args.max_range,
        angular_step_deg=args.filter_step, spacing_factor=args.spacing_factor,
        min_radius_mm=args.min_radius, min_neighbours=args.min_neighbours)
    msg = "%d samples -> %d points kept, %d outliers removed" % (
        len(samples), len(kept), removed)
    print(msg)
    pc.write_ply(args.out + ".ply", kept)
    plot(kept, args.max_range, msg, args.out + ".png", show=not args.no_show)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["sim", "scan", "view"])
    ap.add_argument("file", nargs="?", help="samples.csv for 'view'")
    ap.add_argument("--out", default="scan", help="output basename (.csv .ply .png)")
    ap.add_argument("--geometry", help="JSON sensor/gimbal geometry (see pointcloud.Geometry)")
    ap.add_argument("--max-range", type=float, default=2000.0)
    ap.add_argument("--no-show", action="store_true", help="save the PNG, don't open a window")
    g = ap.add_argument_group("scan pattern")
    g.add_argument("--yaw-min", type=float, default=0.0)
    g.add_argument("--yaw-max", type=float, default=360.0)
    g.add_argument("--yaw-step", type=float, default=10.0)
    g.add_argument("--pitch-min", type=float, default=0.0)
    g.add_argument("--pitch-max", type=float, default=90.0,
                   help="4 sensors 90 deg apart: 0..90 covers every elevation once")
    g.add_argument("--pitch-step", type=float, default=10.0)
    g = ap.add_argument_group("hardware")
    g.add_argument("--gimbal", help="serial port of the gimbal Pico")
    g.add_argument("--tof", help="serial port of the sensor Pico")
    g.add_argument("--yaw-motor", type=int, default=1)
    g.add_argument("--pitch-motor", type=int, default=2)
    g.add_argument("--speed", type=float, default=60.0, help="deg/s used to time moves")
    g = ap.add_argument_group("outlier filter")
    g.add_argument("--spacing-factor", type=float, default=2.0,
                   help="neighbour radius = this x range x angular step")
    g.add_argument("--min-radius", type=float, default=30.0)
    g.add_argument("--min-neighbours", type=int, default=2)
    args = ap.parse_args(argv)
    args.filter_step = max(args.yaw_step, args.pitch_step)

    if args.mode == "sim":
        geo = pc.Geometry.load(args.geometry) if args.geometry else pc.Geometry()
        samples = [s for s in simulate.scan(geo, args.yaw_step, args.pitch_step)]
    elif args.mode == "scan":
        if not (args.gimbal and args.tof):
            ap.error("scan needs --gimbal and --tof")
        samples = live_scan(args)
    else:
        if not args.file:
            ap.error("view needs a samples.csv")
        samples = load_csv(args.file)
    if args.mode != "view":
        save_csv(args.out + ".csv", samples)
    finish(samples, args)


if __name__ == "__main__":
    sys.exit(main())
