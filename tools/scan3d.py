#!/usr/bin/env python3
"""3D scanner host tool.

    scan3d.py sim                       demo with a simulated room (no hardware)
    scan3d.py scan --gimbal COM5 --tof COM7
    scan3d.py view samples.csv          re-process / re-plot a saved scan

`scan` drives the gimbal Pico (serial protocol of the gimbal firmware:
"M <motor> <deg> <sec>" -> "D <motor> <deg>", "?" -> "A <yaw> <pitch>") and asks
the sensor Pico for a reading once the gimbal has stopped with
"P <yaw> <pitch>" -> "S d1 d2 d3 d4" (the sensor Pico also stores the pose in its
3D map and draws it on its LCD; "C", "V <az> <el>" and "R" clear, aim the LCD
camera and redraw).  Needs pyserial.
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

    def _readings(self, what: str):
        end = time.time() + 5.0
        while time.time() < end:
            f = self.ser.readline().decode(errors="ignore").split()
            if len(f) == 5 and f[0] == "S":
                return [None if int(v) < 0 else float(v) for v in f[1:]]
        raise TimeoutError("sensor board did not answer '%s'" % what)

    def read(self):
        """Four distances in mm; None where the sensor had an I2C error.
        Nothing is stored or drawn on the board."""
        self.ser.write(b"S\n")
        return self._readings("S")

    def capture(self, yaw: float, pitch: float):
        """The 'capture' signal: read the four sensors at this pose.  The board
        stores the pose in its 3D map and draws the new points on its LCD."""
        self.ser.write(("P %.3f %.3f\n" % (yaw, pitch)).encode())
        return self._readings("P")

    def command(self, line: str, timeout: float = 15.0):
        """C (clear), V <az> <el> (camera), R (redraw): the board answers OK."""
        self.ser.write((line + "\n").encode())
        end = time.time() + timeout
        while time.time() < end:
            if self.ser.readline().decode(errors="ignore").split()[:1] == ["OK"]:
                return
        raise TimeoutError("sensor board did not answer '%s'" % line)


def frange(lo: float, hi: float, step: float):
    v = lo
    while v < hi - 1e-9:
        yield v
        v += step


def camera_azimuth(geo, args):
    """Where the LCD camera sits.  A partial scan is viewed from behind the
    scanner, looking along the middle of the scanned window (plus a little
    sideways so it reads as 3D); a full 360 uses the board's default view."""
    if args.view_az is not None:
        return args.view_az
    if args.yaw_max - args.yaw_min >= 360.0:
        return 210.0
    mid = geo.yaw_sign * ((args.yaw_min + args.yaw_max) / 2.0 - geo.yaw_zero_deg)
    return (mid + 180.0 + 25.0) % 360.0


def live_scan(args, geo):
    gimbal = Gimbal(args.gimbal, args.yaw_motor, args.pitch_motor, args.speed)
    tof = Tof(args.tof)
    samples = []
    cur = gimbal.angles()
    yaws = list(frange(args.yaw_min, args.yaw_max, args.yaw_step))
    pitches = list(frange(args.pitch_min, args.pitch_max, args.pitch_step))
    total = len(yaws) * len(pitches)
    # the board's map and LCD view want physical angles, like pointcloud.Geometry
    phys_yaw = lambda raw: geo.yaw_sign * (raw - geo.yaw_zero_deg)
    phys_pitch = lambda raw: geo.pitch_sign * (raw - geo.pitch_zero_deg)
    tof.command("C")
    tof.command("V %.1f %.1f" % (camera_azimuth(geo, args), args.view_el))
    i = 0
    for yi, yaw in enumerate(yaws):
        # boustrophedon: alternate pitch direction so the head never flies back
        for pitch in (pitches if yi % 2 == 0 else reversed(pitches)):
            cur = gimbal.move(yaw, pitch, cur)
            i += 1
            # gimbal has stopped: signal the sensor board to capture this pose
            for sensor, d in enumerate(tof.capture(phys_yaw(cur[0]), phys_pitch(cur[1]))):
                if d is not None:
                    # record the *measured* angles, not the commanded ones
                    samples.append(pc.Sample(cur[0], cur[1], sensor, d))
            print("pose %d/%d yaw=%.1f pitch=%.1f" % (i, total, *cur), flush=True)
    tof.command("R")                                  # redraw the finished picture
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
    g = ap.add_argument_group(
        "scan pattern",
        "Full sphere: the defaults.  A window, e.g. 30..60 deg of yaw and 20..50 "
        "of pitch: --yaw-min 30 --yaw-max 60 --pitch-min 20 --pitch-max 50.  Steps "
        "below 5 deg give a dense map (the board keeps 1 deg cells) but many poses: "
        "360x90 at 2 deg is 8100 poses, at roughly a second each.")
    g.add_argument("--yaw-min", type=float, default=0.0)
    g.add_argument("--yaw-max", type=float, default=360.0)
    g.add_argument("--yaw-step", type=float, default=2.0)
    g.add_argument("--pitch-min", type=float, default=0.0)
    g.add_argument("--pitch-max", type=float, default=90.0,
                   help="4 sensors 90 deg apart: 0..90 covers every elevation once")
    g.add_argument("--pitch-step", type=float, default=2.0)
    g = ap.add_argument_group("sensor board LCD")
    g.add_argument("--view-az", type=float, default=None,
                   help="camera azimuth in deg (default: faces the scanned window)")
    g.add_argument("--view-el", type=float, default=25.0, help="camera elevation in deg")
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
        geo = pc.Geometry.load(args.geometry) if args.geometry else pc.Geometry()
        samples = live_scan(args, geo)
    else:
        if not args.file:
            ap.error("view needs a samples.csv")
        samples = load_csv(args.file)
    if args.mode != "view":
        save_csv(args.out + ".csv", samples)
    finish(samples, args)


if __name__ == "__main__":
    sys.exit(main())
