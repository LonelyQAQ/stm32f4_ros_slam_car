import math
import unittest

from scan_geometry import project_scan


class ProjectScanTests(unittest.TestCase):
    def test_laser_mount_yaw_is_applied_once(self):
        points = project_scan(
            ranges=[1.0],
            angle_min=0.0,
            angle_increment=math.pi / 180.0,
            range_min=0.05,
            range_max=12.0,
            map_laser=(2.0, 3.0, math.pi),
        )

        self.assertAlmostEqual(points[0], 1.0, places=6)
        self.assertAlmostEqual(points[1], 3.0, places=6)

    def test_invalid_ranges_are_omitted(self):
        points = project_scan(
            ranges=[0.01, float('inf'), 2.0],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.05,
            range_max=12.0,
            map_laser=(0.0, 0.0, 0.0),
        )

        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0], 2.0 * math.cos(0.2))
        self.assertAlmostEqual(points[1], 2.0 * math.sin(0.2))


if __name__ == '__main__':
    unittest.main()
