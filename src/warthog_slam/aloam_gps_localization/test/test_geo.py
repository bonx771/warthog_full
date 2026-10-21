#!/usr/bin/env python3

import math
import unittest

from aloam_gps_localization.geo import (
    fit_se2_robust,
    latlon_to_utm,
    normalize_angle,
    trajectory_span,
    transform_xy,
)


class GeoTest(unittest.TestCase):
    def test_utm_central_meridian_at_equator(self):
        easting, northing, zone, hemisphere = latlon_to_utm(0.0, 3.0)
        self.assertEqual(zone, 31)
        self.assertEqual(hemisphere, "N")
        self.assertAlmostEqual(easting, 500000.0, places=3)
        self.assertAlmostEqual(northing, 0.0, places=3)

    def test_se2_fit_rejects_one_outlier(self):
        yaw = math.radians(32.0)
        source = [(0.0, 0.0), (4.0, 0.0), (8.0, 2.0), (12.0, 5.0), (15.0, 9.0)]
        samples = []
        for easting, northing in source:
            map_x, map_y = transform_xy(easting, northing, 0.0, 0.0, 7.0, -3.0, yaw)
            samples.append((easting, northing, map_x, map_y, 0.0, 0.0))
        samples.append((6.0, 7.0, 40.0, -25.0, 0.0, 0.0))

        result = fit_se2_robust(samples, absolute_outlier_m=0.5)
        self.assertEqual(len(result["inliers"]), len(source))
        self.assertAlmostEqual(normalize_angle(result["utm_to_map_yaw"] - yaw), 0.0, places=8)
        self.assertLess(result["rms_error_m"], 1e-8)

    def test_trajectory_span(self):
        samples = [(0.0, 0.0, 0.0, 0.0), (3.0, 4.0, 0.0, 0.0)]
        self.assertAlmostEqual(trajectory_span(samples), 5.0)


if __name__ == "__main__":
    unittest.main()

