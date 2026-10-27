#!/usr/bin/env python3

import math
import os
import threading
from collections import defaultdict

import rospy
from nav_msgs.msg import Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

from aloam_gps_localization.terrain_map import (
    GroundSurface,
    PolylineCorridor,
    clear_corridor_cells,
    dilate_cells,
    ground_anchor_from_pose,
    median_smooth_anchor_heights,
    thin_ordered_anchors,
)


def _float_param(name, default):
    return float(rospy.get_param(name, default))


def _normalized_frame(frame_id):
    return (frame_id or "").lstrip("/")


class TerrainRelativeMapSaver:
    """Project an A-LOAM feature map using height above the local trajectory ground."""

    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/laser_cloud_map")
        self.trajectory_topic = rospy.get_param("~trajectory_topic", "/aft_mapped_path")
        self.output_prefix = os.path.expanduser(rospy.get_param("~output_prefix", "aloam_map_rtk"))

        self.lidar_height = _float_param("~lidar_height", 1.23)
        self.min_height = _float_param("~min_height_above_ground", 0.20)
        self.max_height = _float_param("~max_height_above_ground", 1.50)
        self.resolution = _float_param("~resolution", 0.05)
        self.padding = _float_param("~padding", 1.0)
        self.min_points_per_cell = int(rospy.get_param("~min_points_per_cell", 1))
        self.occupied_radius = _float_param("~occupied_radius", 0.10)

        self.ground_cell_size = _float_param("~ground_cell_size", 2.0)
        self.ground_neighbors = int(rospy.get_param("~ground_neighbors", 8))
        self.max_ground_distance = _float_param("~max_ground_distance", 30.0)
        self.ground_query_resolution = _float_param("~ground_query_resolution", 0.50)
        self.trajectory_sample_spacing = _float_param("~trajectory_sample_spacing", 0.25)
        self.trajectory_smoothing_window = int(
            rospy.get_param("~trajectory_smoothing_window", 5)
        )
        # Zero keeps the original saver behavior. A positive value removes all
        # points inside this XY distance from the driven trajectory, regardless
        # of point height.
        self.road_half_width = _float_param("~road_half_width", 0.0)

        self.negate = int(rospy.get_param("~negate", 0))
        self.occupied_thresh = _float_param("~occupied_thresh", 0.65)
        self.free_thresh = _float_param("~free_thresh", 0.196)

        self._validate_parameters()
        self.lock = threading.RLock()
        self.latest_cloud = None
        self.latest_path = None
        self.processing = False
        self.saved = False

        # Keep both latest messages: either topic may arrive first after launch.
        rospy.Subscriber(self.trajectory_topic, Path, self._path_callback, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=1)
        rospy.loginfo(
            "Terrain-relative map saver waiting for cloud=%s and trajectory=%s",
            self.cloud_topic,
            self.trajectory_topic,
        )
        rospy.loginfo(
            "Height above local ground: %.2f..%.2f m, LiDAR height=%.2f m, occupied radius=%.2f m",
            self.min_height,
            self.max_height,
            self.lidar_height,
            self.occupied_radius,
        )
        if self.road_half_width > 0.0:
            rospy.loginfo(
                "Road removal enabled: clearing %.2f m on each side of trajectory",
                self.road_half_width,
            )

    def _validate_parameters(self):
        if self.lidar_height <= 0.0:
            raise ValueError("lidar_height must be positive")
        if self.min_height < 0.0 or self.max_height <= self.min_height:
            raise ValueError("invalid height-above-ground interval")
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if self.padding < self.occupied_radius:
            raise ValueError("padding must be at least occupied_radius")
        if self.min_points_per_cell < 1:
            raise ValueError("min_points_per_cell must be at least one")
        if self.occupied_radius < 0.0:
            raise ValueError("occupied_radius cannot be negative")
        if self.ground_cell_size <= 0.0 or self.ground_query_resolution <= 0.0:
            raise ValueError("ground grid resolutions must be positive")
        if self.ground_neighbors < 1 or self.max_ground_distance <= 0.0:
            raise ValueError("invalid ground interpolation parameters")
        if not math.isfinite(self.road_half_width) or self.road_half_width < 0.0:
            raise ValueError("road_half_width must be finite and cannot be negative")

    def _path_callback(self, msg):
        if not msg.poses:
            return
        with self.lock:
            if self.saved:
                return
            self.latest_path = msg
        self._try_process()

    def _cloud_callback(self, msg):
        with self.lock:
            if self.saved:
                return
            self.latest_cloud = msg
        self._try_process()

    def _try_process(self):
        with self.lock:
            if self.saved or self.processing or self.latest_cloud is None or self.latest_path is None:
                return
            cloud = self.latest_cloud
            path = self.latest_path
            self.processing = True

        try:
            self._save(cloud, path)
        except (OSError, ValueError) as exc:
            rospy.logerr("Terrain-relative map was not saved: %s", exc)
            with self.lock:
                self.processing = False
            return

        with self.lock:
            self.saved = True
            self.processing = False
        rospy.signal_shutdown("Terrain-relative 2-D map saved once.")

    def _ground_anchors(self, path):
        anchors = []
        for stamped_pose in path.poses:
            pose = stamped_pose.pose
            position = pose.position
            orientation = pose.orientation
            values = (
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            if not all(math.isfinite(value) for value in values):
                continue
            try:
                anchor = ground_anchor_from_pose(
                    (position.x, position.y, position.z),
                    (orientation.x, orientation.y, orientation.z, orientation.w),
                    self.lidar_height,
                )
            except ValueError:
                continue
            anchors.append(anchor)

        anchors = thin_ordered_anchors(anchors, self.trajectory_sample_spacing)
        anchors = median_smooth_anchor_heights(anchors, self.trajectory_smoothing_window)
        if not anchors:
            raise ValueError("trajectory contains no valid LiDAR poses")
        return anchors

    def _save(self, cloud, path):
        cloud_frame = _normalized_frame(cloud.header.frame_id)
        path_frame = _normalized_frame(path.header.frame_id)
        if path_frame and cloud_frame and path_frame != cloud_frame:
            raise ValueError(
                "cloud frame '%s' differs from trajectory frame '%s'"
                % (cloud.header.frame_id, path.header.frame_id)
            )

        anchors = self._ground_anchors(path)
        ground_surface = GroundSurface(
            anchors,
            cell_size=self.ground_cell_size,
            neighbor_count=self.ground_neighbors,
            max_distance=self.max_ground_distance,
        )
        rospy.loginfo("Built local ground surface from %d trajectory anchors", len(anchors))
        road_corridor = None
        if self.road_half_width > 0.0:
            road_corridor = PolylineCorridor(anchors, self.road_half_width)

        kept_points = []
        ground_cache = {}
        stats = defaultdict(int)
        for point in point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = point
            stats["input"] += 1
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                stats["invalid"] += 1
                continue
            if road_corridor is not None and road_corridor.contains(x, y):
                stats["road_corridor"] += 1
                continue

            cache_key = (
                int(math.floor(x / self.ground_query_resolution)),
                int(math.floor(y / self.ground_query_resolution)),
            )
            if cache_key not in ground_cache:
                query_x = (cache_key[0] + 0.5) * self.ground_query_resolution
                query_y = (cache_key[1] + 0.5) * self.ground_query_resolution
                ground_cache[cache_key] = ground_surface.height(query_x, query_y)
            ground_z = ground_cache[cache_key]
            if ground_z is None:
                stats["no_ground"] += 1
                continue

            height_above_ground = z - ground_z
            if height_above_ground < self.min_height:
                stats["below"] += 1
                continue
            if height_above_ground > self.max_height:
                stats["above"] += 1
                continue
            kept_points.append((x, y))
            stats["kept"] += 1

        if not kept_points:
            raise ValueError(
                "no cloud points remain after road-corridor and %.2f..%.2f m height filtering"
                % (self.min_height, self.max_height)
            )

        x_min = min(point[0] for point in kept_points) - self.padding
        x_max = max(point[0] for point in kept_points) + self.padding
        y_min = min(point[1] for point in kept_points) - self.padding
        y_max = max(point[1] for point in kept_points) + self.padding
        width = int(math.ceil((x_max - x_min) / self.resolution)) + 1
        height = int(math.ceil((y_max - y_min) / self.resolution)) + 1

        occupied_counts = defaultdict(int)
        for x, y in kept_points:
            col = int(math.floor((x - x_min) / self.resolution))
            row = int(math.floor((y - y_min) / self.resolution))
            if 0 <= col < width and 0 <= row < height:
                occupied_counts[(row, col)] += 1
        occupied = {
            cell for cell, count in occupied_counts.items() if count >= self.min_points_per_cell
        }
        occupied = dilate_cells(
            occupied,
            width,
            height,
            self.occupied_radius,
            self.resolution,
        )
        if road_corridor is not None:
            occupied_before_clear = len(occupied)
            occupied = clear_corridor_cells(
                occupied,
                x_min,
                y_min,
                self.resolution,
                road_corridor,
            )
            stats["road_cells_after_dilation"] = occupied_before_clear - len(occupied)

        pixels = bytearray([254]) * (width * height)
        for row, col in occupied:
            image_row = height - 1 - row
            pixels[image_row * width + col] = 0

        directory = os.path.dirname(self.output_prefix)
        if directory:
            os.makedirs(directory, exist_ok=True)
        pgm_path = self.output_prefix + ".pgm"
        yaml_path = self.output_prefix + ".yaml"
        pgm_temporary = pgm_path + ".tmp"
        yaml_temporary = yaml_path + ".tmp"

        with open(pgm_temporary, "wb") as stream:
            stream.write(("P5\n%d %d\n255\n" % (width, height)).encode("ascii"))
            stream.write(pixels)
        yaml_text = (
            "image: %s\n"
            "resolution: %s\n"
            "origin: [%s, %s, 0.0]\n"
            "negate: %d\n"
            "occupied_thresh: %s\n"
            "free_thresh: %s\n"
            % (
                os.path.basename(pgm_path),
                self.resolution,
                x_min,
                y_min,
                self.negate,
                self.occupied_thresh,
                self.free_thresh,
            )
        )
        with open(yaml_temporary, "w", encoding="utf-8") as stream:
            stream.write(yaml_text)
        os.replace(pgm_temporary, pgm_path)
        os.replace(yaml_temporary, yaml_path)

        rospy.loginfo(
            "Point filter: input=%d kept=%d road_corridor=%d below=%d above=%d no_local_ground=%d",
            stats["input"],
            stats["kept"],
            stats["road_corridor"],
            stats["below"],
            stats["above"],
            stats["no_ground"],
        )
        if road_corridor is not None:
            rospy.loginfo(
                "Cleared %d occupied cells that dilation placed back inside road corridor",
                stats["road_cells_after_dilation"],
            )
        rospy.loginfo(
            "Saved terrain-relative map: %s and %s (%dx%d, %.3f m/cell, %d occupied cells)",
            pgm_path,
            yaml_path,
            width,
            height,
            self.resolution,
            len(occupied),
        )


if __name__ == "__main__":
    rospy.init_node("save_terrain_relative_2d_map")
    try:
        TerrainRelativeMapSaver()
    except ValueError as error:
        rospy.logfatal("Invalid terrain-relative map configuration: %s", error)
        raise SystemExit(2)
    rospy.spin()
