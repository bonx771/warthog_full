#!/usr/bin/env python3

import math
import unittest

from aloam_gps_localization.terrain_map import (
    GroundSurface,
    PolylineCorridor,
    clear_corridor_cells,
    dilate_cells,
    ground_anchor_from_pose,
    median_smooth_anchor_heights,
    thin_ordered_anchors,
)


class TerrainMapTest(unittest.TestCase):
    def test_ground_anchor_uses_pose_orientation(self):
        half_angle = math.radians(10.0) / 2.0
        quaternion = (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))
        anchor = ground_anchor_from_pose((4.0, 2.0, 3.0), quaternion, 1.23)
        self.assertAlmostEqual(anchor[0], 4.0 - 1.23 * math.sin(math.radians(10.0)), places=8)
        self.assertAlmostEqual(anchor[1], 2.0, places=8)
        self.assertAlmostEqual(anchor[2], 3.0 - 1.23 * math.cos(math.radians(10.0)), places=8)

    def test_ground_surface_follows_height_change(self):
        anchors = [(float(x), 0.0, 0.1 * x) for x in range(0, 21)]
        surface = GroundSurface(anchors, cell_size=2.0, neighbor_count=6, max_distance=10.0)
        self.assertAlmostEqual(surface.height(4.0, 1.0), 0.4, delta=0.08)
        self.assertAlmostEqual(surface.height(16.0, -1.0), 1.6, delta=0.08)
        self.assertIsNone(surface.height(100.0, 0.0))

    def test_thinning_preserves_path_and_latest_stationary_pose(self):
        anchors = [
            (0.0, 0.0, 0.0),
            (0.1, 0.0, 0.1),
            (0.2, 0.0, 0.2),
            (0.3, 0.0, 0.3),
            (1.0, 0.0, 0.4),
            (1.0, 0.0, 0.5),
        ]
        thinned = thin_ordered_anchors(anchors, 0.25)
        self.assertEqual(
            thinned,
            [(0.0, 0.0, 0.0), (0.3, 0.0, 0.3), (1.0, 0.0, 0.5)],
        )

    def test_median_smoothing_rejects_height_spike(self):
        anchors = [(0.0, 0.0, 0.0), (1.0, 0.0, 10.0), (2.0, 0.0, 0.2)]
        smoothed = median_smooth_anchor_heights(anchors, 3)
        self.assertAlmostEqual(smoothed[1][2], 0.2)

    def test_dilation_makes_occupied_cells_thicker(self):
        expanded = dilate_cells({(5, 5)}, width=11, height=11, radius_m=0.10, resolution=0.05)
        self.assertIn((5, 5), expanded)
        self.assertIn((5, 7), expanded)
        self.assertNotIn((5, 8), expanded)
        self.assertGreater(len(expanded), 1)

    def test_polyline_corridor_uses_distance_to_segments(self):
        corridor = PolylineCorridor([(0.0, 0.0), (10.0, 0.0)], half_width=2.0)
        self.assertTrue(corridor.contains(5.0, 1.99))
        self.assertTrue(corridor.contains(-1.0, 0.0))
        self.assertFalse(corridor.contains(5.0, 2.01))
        self.assertFalse(corridor.contains(-2.01, 0.0))

    def test_corridor_is_cleared_after_cell_dilation(self):
        corridor = PolylineCorridor([(0.0, 0.0), (10.0, 0.0)], half_width=1.0)
        occupied = {(0, 5), (1, 5), (2, 5), (3, 5)}
        cleared = clear_corridor_cells(
            occupied,
            x_min=0.0,
            y_min=-2.0,
            resolution=1.0,
            corridor=corridor,
        )
        self.assertEqual(cleared, {(0, 5), (3, 5)})


if __name__ == "__main__":
    unittest.main()
