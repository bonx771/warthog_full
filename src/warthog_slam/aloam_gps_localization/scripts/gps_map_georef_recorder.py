#!/usr/bin/env python3

import json
import math
import os
import threading

import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger, TriggerResponse

from aloam_gps_localization.geo import (
    antenna_to_base,
    circular_mean,
    circular_rms,
    finite,
    fit_se2_robust,
    latlon_to_utm,
    normalize_angle,
    sensor_to_base,
    trajectory_span,
    yaw_from_quaternion,
)


def _float_param(name, default):
    return float(rospy.get_param(name, default))


def _stamp_or_now(header):
    if header.stamp != rospy.Time(0):
        return header.stamp
    return rospy.Time.now()


class GpsMapGeorefRecorder:
    """Collect synchronized A-LOAM/RTK samples and fit a 2-D rigid transform."""

    def __init__(self):
        self.lock = threading.RLock()

        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")
        self.heading_topic = rospy.get_param("~heading_topic", "/gps/heading")
        self.aloam_odom_topic = rospy.get_param("~aloam_odom_topic", "/aft_mapped_to_init")
        self.output_file = os.path.expanduser(rospy.get_param("~output_file"))
        self.map_name = rospy.get_param("~map_name", "aloam_map")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.aloam_frame = rospy.get_param("~aloam_frame", "camera_init")

        self.max_gps_variance = _float_param("~max_gps_variance_m2", 0.04)
        self.max_heading_variance = _float_param(
            "~max_heading_variance_rad2", math.radians(5.0) ** 2
        )
        self.sync_slop = _float_param("~sync_slop_sec", 0.20)
        self.max_sensor_age = _float_param("~max_sensor_age_sec", 1.0)
        self.min_sample_distance = _float_param("~min_sample_distance_m", 0.40)
        self.min_samples = max(3, int(rospy.get_param("~min_samples", 20)))
        self.min_trajectory_span = _float_param("~min_trajectory_span_m", 8.0)
        self.max_fit_rms = _float_param("~max_fit_rms_m", 0.75)
        self.outlier_threshold = _float_param("~outlier_threshold_m", 0.75)
        self.max_samples = max(self.min_samples, int(rospy.get_param("~max_samples", 5000)))

        # All offsets follow REP-103: +x forward, +y left, yaw counter-clockwise.
        self.gps_antenna_x = _float_param("~gps_antenna_x", 0.0)
        self.gps_antenna_y = _float_param("~gps_antenna_y", 0.0)
        self.lidar_x = _float_param("~lidar_x", 0.40)
        self.lidar_y = _float_param("~lidar_y", 0.0)
        self.lidar_yaw = _float_param("~lidar_yaw", 0.0)
        self.heading_yaw_offset = _float_param("~heading_yaw_offset", 0.0)
        self.estimate_heading_yaw_offset = bool(
            rospy.get_param("~estimate_heading_yaw_offset", True)
        )

        self.auto_save = bool(rospy.get_param("~auto_save", True))
        self.auto_save_period = max(1.0, _float_param("~auto_save_period_sec", 5.0))
        self.auto_save_on_shutdown = bool(rospy.get_param("~auto_save_on_shutdown", True))

        self.latest_gps = None
        self.latest_heading = None
        self.samples = []
        self.utm_zone = None
        self.utm_hemisphere = None
        self.last_saved_count = 0
        self.last_result = None

        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb, queue_size=30)
        rospy.Subscriber(self.heading_topic, Imu, self._heading_cb, queue_size=30)
        rospy.Subscriber(self.aloam_odom_topic, Odometry, self._odom_cb, queue_size=100)

        rospy.Service("~save", Trigger, self._save_service)
        rospy.Service("~reset", Trigger, self._reset_service)
        rospy.Service("~status", Trigger, self._status_service)
        if self.auto_save:
            rospy.Timer(rospy.Duration(self.auto_save_period), self._auto_save_timer)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "GPS map georef recorder ready: GPS=%s heading=%s A-LOAM=%s output=%s",
            self.gps_topic,
            self.heading_topic,
            self.aloam_odom_topic,
            self.output_file,
        )
        rospy.loginfo(
            "Only fixes with variance <= %.4f m^2 and heading variance <= %.6f rad^2 are accepted.",
            self.max_gps_variance,
            self.max_heading_variance,
        )

    def _gps_cb(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not finite(msg.latitude) or not finite(msg.longitude):
            return
        if self._gps_variance(msg) > self.max_gps_variance:
            return
        with self.lock:
            self.latest_gps = msg

    def _heading_cb(self, msg):
        covariance = msg.orientation_covariance
        if covariance and covariance[0] == -1.0:
            return
        yaw_variance = covariance[8] if len(covariance) >= 9 else float("inf")
        if not finite(yaw_variance) or yaw_variance < 0.0:
            return
        if yaw_variance > self.max_heading_variance:
            return
        try:
            yaw = yaw_from_quaternion(msg.orientation)
        except ValueError:
            return
        with self.lock:
            self.latest_heading = (msg, yaw, yaw_variance)

    def _odom_cb(self, msg):
        try:
            lidar_yaw_map = yaw_from_quaternion(msg.pose.pose.orientation)
        except ValueError:
            return

        with self.lock:
            gps = self.latest_gps
            heading_item = self.latest_heading
        if gps is None or heading_item is None:
            rospy.logwarn_throttle(5.0, "Waiting for RTK Fixed-quality /gps/fix and valid /gps/heading")
            return

        heading_msg, heading_yaw_raw, _heading_variance = heading_item
        odom_stamp = _stamp_or_now(msg.header)
        gps_stamp = _stamp_or_now(gps.header)
        heading_stamp = _stamp_or_now(heading_msg.header)
        now = rospy.Time.now()
        if abs((odom_stamp - gps_stamp).to_sec()) > self.sync_slop:
            return
        if abs((odom_stamp - heading_stamp).to_sec()) > self.sync_slop:
            return
        if self.max_sensor_age > 0.0 and (now - gps_stamp).to_sec() > self.max_sensor_age:
            return

        try:
            easting, northing, zone, hemisphere = latlon_to_utm(gps.latitude, gps.longitude)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "Invalid GPS coordinate: %s", exc)
            return

        with self.lock:
            if self.utm_zone is None:
                self.utm_zone = zone
                self.utm_hemisphere = hemisphere
            elif zone != self.utm_zone or hemisphere != self.utm_hemisphere:
                rospy.logerr_throttle(
                    5.0,
                    "GPS crossed UTM zone/hemisphere: expected %d%s, got %d%s",
                    self.utm_zone,
                    self.utm_hemisphere,
                    zone,
                    hemisphere,
                )
                return

        base_yaw_enu = normalize_angle(heading_yaw_raw + self.heading_yaw_offset)
        base_easting, base_northing = antenna_to_base(
            easting,
            northing,
            base_yaw_enu,
            self.gps_antenna_x,
            self.gps_antenna_y,
        )

        base_yaw_map = normalize_angle(lidar_yaw_map - self.lidar_yaw)
        base_map_x, base_map_y = sensor_to_base(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            base_yaw_map,
            self.lidar_x,
            self.lidar_y,
        )

        sample = (
            base_easting,
            base_northing,
            base_map_x,
            base_map_y,
            heading_yaw_raw,
            base_yaw_map,
            self._gps_variance(gps),
        )

        with self.lock:
            if self.samples:
                previous = self.samples[-1]
                source_move = math.hypot(sample[0] - previous[0], sample[1] - previous[1])
                target_move = math.hypot(sample[2] - previous[2], sample[3] - previous[3])
                if max(source_move, target_move) < self.min_sample_distance:
                    return
            self.samples.append(sample)
            if len(self.samples) > self.max_samples:
                self.samples = self.samples[::2]
            count = len(self.samples)
            span = trajectory_span(self.samples)

        rospy.loginfo_throttle(
            2.0,
            "Georef samples=%d/%d trajectory_span=%.2f/%.2f m",
            count,
            self.min_samples,
            span,
            self.min_trajectory_span,
        )

    @staticmethod
    def _gps_variance(msg):
        covariance = msg.position_covariance
        if len(covariance) < 5:
            return float("inf")
        cov_x = float(covariance[0]) if finite(covariance[0]) else float("inf")
        cov_y = float(covariance[4]) if finite(covariance[4]) else float("inf")
        if cov_x < 0.0 or cov_y < 0.0:
            return float("inf")
        return max(cov_x, cov_y)

    def _calculate(self):
        with self.lock:
            samples = list(self.samples)
            zone = self.utm_zone
            hemisphere = self.utm_hemisphere

        if len(samples) < self.min_samples:
            raise ValueError("need at least %d samples; currently %d" % (self.min_samples, len(samples)))
        span = trajectory_span(samples)
        if span < self.min_trajectory_span:
            raise ValueError(
                "trajectory span %.2f m is below required %.2f m"
                % (span, self.min_trajectory_span)
            )

        fit = fit_se2_robust(samples, self.outlier_threshold)
        inliers = fit.pop("inliers")
        if len(inliers) < self.min_samples:
            raise ValueError(
                "only %d inliers remain after outlier rejection (need %d)"
                % (len(inliers), self.min_samples)
            )
        if fit["rms_error_m"] > self.max_fit_rms:
            raise ValueError(
                "fit RMS %.3f m is above allowed %.3f m"
                % (fit["rms_error_m"], self.max_fit_rms)
            )

        heading_offsets = [
            normalize_angle(sample[5] - sample[4] - fit["utm_to_map_yaw"])
            for sample in inliers
        ]
        estimated_heading_offset = circular_mean(heading_offsets)
        heading_rms = circular_rms(heading_offsets, estimated_heading_offset)
        saved_heading_offset = (
            estimated_heading_offset
            if self.estimate_heading_yaw_offset
            else self.heading_yaw_offset
        )

        result = {
            "version": 1,
            "map_name": self.map_name,
            "map_frame": self.map_frame,
            "aloam_frame": self.aloam_frame,
            "utm_zone": int(zone),
            "utm_hemisphere": hemisphere,
            "utm_origin_easting": float(fit["utm_origin_easting"]),
            "utm_origin_northing": float(fit["utm_origin_northing"]),
            "map_origin_x": float(fit["map_origin_x"]),
            "map_origin_y": float(fit["map_origin_y"]),
            "utm_to_map_yaw": float(fit["utm_to_map_yaw"]),
            "heading_yaw_offset": float(saved_heading_offset),
            "gps_antenna_x": float(self.gps_antenna_x),
            "gps_antenna_y": float(self.gps_antenna_y),
            "lidar_x": float(self.lidar_x),
            "lidar_y": float(self.lidar_y),
            "lidar_yaw": float(self.lidar_yaw),
            "fit": {
                "sample_count": len(samples),
                "inlier_count": len(inliers),
                "trajectory_span_m": float(span),
                "rms_error_m": float(fit["rms_error_m"]),
                "median_error_m": float(fit["median_error_m"]),
                "max_error_m": float(fit["max_error_m"]),
                "heading_rms_rad": float(heading_rms),
            },
        }
        return result

    def _write(self, result):
        directory = os.path.dirname(self.output_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.output_file + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(result, stream, default_flow_style=False, sort_keys=False)
        os.replace(temporary, self.output_file)
        with self.lock:
            self.last_saved_count = result["fit"]["sample_count"]
            self.last_result = result

    def _save(self, log_failure=True):
        try:
            result = self._calculate()
            self._write(result)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            if log_failure:
                rospy.logwarn("Georef not saved: %s", exc)
            return False, str(exc)

        message = (
            "saved %s: inliers=%d/%d span=%.2fm RMS=%.3fm yaw=%.6frad"
            % (
                self.output_file,
                result["fit"]["inlier_count"],
                result["fit"]["sample_count"],
                result["fit"]["trajectory_span_m"],
                result["fit"]["rms_error_m"],
                result["utm_to_map_yaw"],
            )
        )
        rospy.loginfo(message)
        return True, message

    def _save_service(self, _request):
        ok, message = self._save(log_failure=True)
        return TriggerResponse(success=ok, message=message)

    def _reset_service(self, _request):
        with self.lock:
            self.samples = []
            self.utm_zone = None
            self.utm_hemisphere = None
            self.last_saved_count = 0
            self.last_result = None
        return TriggerResponse(success=True, message="cleared all georeference samples")

    def _status_service(self, _request):
        with self.lock:
            samples = list(self.samples)
            saved_count = self.last_saved_count
            zone = self.utm_zone
            hemisphere = self.utm_hemisphere
        payload = {
            "samples": len(samples),
            "required_samples": self.min_samples,
            "trajectory_span_m": trajectory_span(samples),
            "required_span_m": self.min_trajectory_span,
            "saved_samples": saved_count,
            "utm_zone": "%s%s" % (zone if zone is not None else "", hemisphere or ""),
            "output_file": self.output_file,
        }
        return TriggerResponse(success=True, message=json.dumps(payload, sort_keys=True))

    def _auto_save_timer(self, _event):
        with self.lock:
            count = len(self.samples)
            saved_count = self.last_saved_count
        if count <= saved_count:
            return
        self._save(log_failure=False)

    def _on_shutdown(self):
        if self.auto_save_on_shutdown:
            self._save(log_failure=False)


if __name__ == "__main__":
    rospy.init_node("gps_map_georef_recorder")
    GpsMapGeorefRecorder()
    rospy.spin()

