#!/usr/bin/env python3
import math
import os
from collections import defaultdict

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class CloudTo2DMap:
    def __init__(self):
        self.topic = rospy.get_param('~topic', '/helios_cloud_registered')
        self.output_prefix = rospy.get_param('~output_prefix', 'helios_map')
        self.resolution = float(rospy.get_param('~resolution', 0.05))
        self.z_min = float(rospy.get_param('~z_min', -0.2))
        self.z_max = float(rospy.get_param('~z_max', 1.5))
        self.padding = float(rospy.get_param('~padding', 1.0))
        self.min_points_per_cell = int(rospy.get_param('~min_points_per_cell', 1))
        self.negate = int(rospy.get_param('~negate', 0))
        self.occupied_thresh = float(rospy.get_param('~occupied_thresh', 0.65))
        self.free_thresh = float(rospy.get_param('~free_thresh', 0.196))
        self.once = bool(rospy.get_param('~once', True))

        self.saved = False
        self.sub = rospy.Subscriber(self.topic, PointCloud2, self.callback, queue_size=1)
        rospy.loginfo('Waiting for PointCloud2 on %s', self.topic)

    def callback(self, msg: PointCloud2):
        if self.saved and self.once:
            return

        points_xy = []
        occupied_counts = defaultdict(int)

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            if z < self.z_min or z > self.z_max:
                continue
            points_xy.append((x, y))

        if not points_xy:
            rospy.logwarn('No points remained after z filter: z in [%.3f, %.3f]', self.z_min, self.z_max)
            return

        xs = [p[0] for p in points_xy]
        ys = [p[1] for p in points_xy]
        x_min = min(xs) - self.padding
        x_max = max(xs) + self.padding
        y_min = min(ys) - self.padding
        y_max = max(ys) + self.padding

        width = int(math.ceil((x_max - x_min) / self.resolution)) + 1
        height = int(math.ceil((y_max - y_min) / self.resolution)) + 1

        for x, y in points_xy:
            col = int((x - x_min) / self.resolution)
            row = int((y - y_min) / self.resolution)
            if 0 <= col < width and 0 <= row < height:
                occupied_counts[(row, col)] += 1

        # PGM convention used by ROS map_server:
        #   0   = occupied (black)
        #   254 = free (white)
        #   205 = unknown (gray)
        grid = [[254 for _ in range(width)] for _ in range(height)]

        for (row, col), count in occupied_counts.items():
            if count >= self.min_points_per_cell:
                # invert Y so map has standard top-left image origin
                img_row = height - 1 - row
                grid[img_row][col] = 0

        pgm_path = self.output_prefix + '.pgm'
        yaml_path = self.output_prefix + '.yaml'

        with open(pgm_path, 'wb') as f:
            header = f'P5\n{width} {height}\n255\n'.encode('ascii')
            f.write(header)
            for row in grid:
                f.write(bytearray(row))

        yaml_text = (
            f'image: {os.path.basename(pgm_path)}\n'
            f'resolution: {self.resolution}\n'
            f'origin: [{x_min}, {y_min}, 0.0]\n'
            f'negate: {self.negate}\n'
            f'occupied_thresh: {self.occupied_thresh}\n'
            f'free_thresh: {self.free_thresh}\n'
        )
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_text)

        rospy.loginfo('Saved 2D map from %s', self.topic)
        rospy.loginfo('  PGM : %s', os.path.abspath(pgm_path))
        rospy.loginfo('  YAML: %s', os.path.abspath(yaml_path))
        rospy.loginfo('  size: %d x %d, resolution=%.3f', width, height, self.resolution)
        rospy.loginfo('  bounds: x=[%.3f, %.3f], y=[%.3f, %.3f]', x_min, x_max, y_min, y_max)

        self.saved = True
        if self.once:
            rospy.signal_shutdown('Map saved once.')


if __name__ == '__main__':
    rospy.init_node('cloud_to_2d_map')
    CloudTo2DMap()
    rospy.spin()
