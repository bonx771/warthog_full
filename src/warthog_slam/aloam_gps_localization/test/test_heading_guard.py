#!/usr/bin/env python3

import math
import unittest

from aloam_gps_localization.heading_guard import (
    path_heading_error,
    rate_limit,
    turn_speed,
)


class HeadingGuardTest(unittest.TestCase):
    def test_path_behind_requires_half_turn(self):
        error = path_heading_error(
            [(0.0, 0.0), (-0.5, 0.0), (-1.0, 0.0)],
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            lookahead_distance=0.8,
        )
        self.assertAlmostEqual(abs(error), math.pi)

    def test_lookahead_uses_forward_path_direction(self):
        error = path_heading_error(
            [(0.0, 0.0), (0.0, 0.5), (0.0, 1.0)],
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            lookahead_distance=0.8,
        )
        self.assertAlmostEqual(error, math.pi / 2.0)

    def test_turn_speed_respects_limits_and_direction(self):
        self.assertAlmostEqual(turn_speed(math.pi, 0.4, 0.65, 1.4), 0.65)
        self.assertAlmostEqual(turn_speed(-0.1, 0.4, 0.65, 1.4), -0.4)

    def test_rate_limit(self):
        self.assertAlmostEqual(rate_limit(0.0, 0.65, 0.15), 0.15)
        self.assertAlmostEqual(rate_limit(0.4, -0.65, 0.15), 0.25)


if __name__ == "__main__":
    unittest.main()
