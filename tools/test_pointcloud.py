import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import pointcloud as pc
import simulate


def near(a, b, tol=1e-6):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def geo(radius=0.0, axial=0.0, **kw):
    return pc.Geometry(sensors=[pc.SensorMount(90.0 * i, radius, axial) for i in range(4)], **kw)


class GeometryTests(unittest.TestCase):
    def test_pose_zero_directions(self):
        g = geo()
        # forward, up, back, down
        self.assertTrue(near(pc.to_point(g, 0, 0, 0, 1000), (1000, 0, 0)))
        self.assertTrue(near(pc.to_point(g, 1, 0, 0, 1000), (0, 0, 1000)))
        self.assertTrue(near(pc.to_point(g, 2, 0, 0, 1000), (-1000, 0, 0)))
        self.assertTrue(near(pc.to_point(g, 3, 0, 0, 1000), (0, 0, -1000)))

    def test_yaw_is_counter_clockwise_about_z(self):
        self.assertTrue(near(pc.to_point(geo(), 0, 90, 0, 500), (0, 500, 0)))

    def test_positive_pitch_raises_forward_beam(self):
        self.assertTrue(near(pc.to_point(geo(), 0, 0, 30, 1000),
                             (1000 * math.cos(math.radians(30)), 0,
                              1000 * math.sin(math.radians(30)))))

    def test_pitch_90_makes_sensor1_look_back(self):
        self.assertTrue(near(pc.to_point(geo(), 1, 0, 90, 1000), (-1000, 0, 0)))

    def test_sensor_displacement_radial(self):
        # sensor sits 25 mm out along its beam: 1000 mm reading is 1025 mm from origin
        self.assertTrue(near(pc.to_point(geo(radius=25), 0, 0, 0, 1000), (1025, 0, 0)))
        # and the offset rotates with yaw and pitch
        self.assertTrue(near(pc.to_point(geo(radius=25), 0, 90, 0, 1000), (0, 1025, 0)))
        self.assertTrue(near(pc.to_point(geo(radius=25), 0, 0, 90, 1000), (0, 0, 1025)))

    def test_sensor_displacement_axial(self):
        # 40 mm along the pitch axis (+Y at yaw 0): shifts the point sideways,
        # and yaw swings that offset round the origin
        self.assertTrue(near(pc.to_point(geo(axial=40), 0, 0, 0, 1000), (1000, 40, 0)))
        self.assertTrue(near(pc.to_point(geo(axial=40), 0, 90, 0, 1000), (-40, 1000, 0)))

    def test_pitch_axis_offset_and_signs(self):
        g = geo(pitch_axis_offset_mm=(0, 0, 100), yaw_sign=-1.0, pitch_sign=-1.0,
                yaw_zero_deg=10.0)
        # yaw raw 100, zero 10, sign -1 -> -90 deg (clockwise): beam points to -Y
        self.assertTrue(near(pc.to_point(g, 0, 100, 0, 500), (0, -500, 100)))
        # pitch raw 90 with sign -1 -> -90: forward beam points down
        self.assertTrue(near(pc.to_point(g, 0, 10, 90, 500), (0, 0, -400)))


class FilterTests(unittest.TestCase):
    def cluster(self, cx, n=5, gap=20.0):
        return [(cx + i * gap, 0.0, 0.0) for i in range(n)]

    def test_isolated_point_removed_cluster_kept(self):
        pts = self.cluster(0) + [(5000.0, 5000.0, 5000.0)]
        keep = pc.remove_outliers(pts, [500.0] * len(pts), angular_step_deg=3.0)
        self.assertEqual(keep, [True] * 5 + [False])

    def test_pair_is_not_enough_for_two_neighbours(self):
        keep = pc.remove_outliers([(0, 0, 0), (10, 0, 0)], [500.0, 500.0], angular_step_deg=3.0)
        self.assertEqual(keep, [False, False])
        keep = pc.remove_outliers([(0, 0, 0), (10, 0, 0)], [500.0, 500.0],
                                  angular_step_deg=3.0, min_neighbours=1)
        self.assertEqual(keep, [True, True])

    def test_radius_grows_with_range(self):
        # 300 mm apart: neighbours for a far surface on a coarse grid, not for a near one
        pts = [(i * 300.0, 0.0, 0.0) for i in range(5)]
        near_keep = pc.remove_outliers(pts, [200.0] * 5, angular_step_deg=10.0)
        far_keep = pc.remove_outliers(pts, [1800.0] * 5, angular_step_deg=10.0)
        self.assertEqual(near_keep, [False] * 5)
        self.assertEqual(far_keep, [True] * 5)

    def test_empty(self):
        self.assertEqual(pc.remove_outliers([]), [])

    def test_out_of_range_dropped_before_filtering(self):
        samples = [pc.Sample(0, 0, 0, 8190), pc.Sample(0, 0, 0, 0), pc.Sample(0, 0, 0, 1999)]
        self.assertEqual(len(pc.samples_to_points(geo(), samples)), 1)


class EndToEnd(unittest.TestCase):
    def test_simulated_room_is_reconstructed(self):
        g = pc.Geometry()                                   # 25 mm sensor displacement
        samples = list(simulate.scan(g, 10.0, 10.0, noise_mm=3.0, outlier_rate=0.02, seed=3))
        kept, removed = pc.process(g, samples, angular_step_deg=10.0)

        def on_surface(p):
            x, y, z = p
            wall = min(abs(x - simulate.ROOM_LO[0]), abs(x - simulate.ROOM_HI[0]),
                       abs(y - simulate.ROOM_LO[1]), abs(y - simulate.ROOM_HI[1]),
                       abs(z - simulate.ROOM_LO[2]), abs(z - simulate.ROOM_HI[2]))
            sph = abs(math.dist(p, simulate.SPHERE_C) - simulate.SPHERE_R)
            return min(wall, sph) < 25.0

        good = sum(on_surface(p) for p, _ in kept)
        self.assertGreater(len(kept), 400)
        self.assertGreater(good / len(kept), 0.98)          # outliers gone
        self.assertGreater(removed, 0)

    def test_ignoring_displacement_degrades_reconstruction(self):
        truth = pc.Geometry()
        samples = list(simulate.scan(truth, 10.0, 10.0, noise_mm=0.0, outlier_rate=0.0))
        naive = pc.Geometry(sensors=[pc.SensorMount(90.0 * i) for i in range(4)])

        def wall_err(g):
            err = []
            for p, _ in pc.samples_to_points(g, samples):
                x, y, z = p
                err.append(min(abs(x - simulate.ROOM_LO[0]), abs(x - simulate.ROOM_HI[0]),
                               abs(y - simulate.ROOM_LO[1]), abs(y - simulate.ROOM_HI[1]),
                               abs(z - simulate.ROOM_LO[2]), abs(z - simulate.ROOM_HI[2]),
                               abs(math.dist(p, simulate.SPHERE_C) - simulate.SPHERE_R)))
            return sum(err) / len(err)

        self.assertLess(wall_err(truth), 1e-6)
        self.assertGreater(wall_err(naive), 5.0)


if __name__ == "__main__":
    unittest.main()
