#!/usr/bin/env python3

import json
import math
import os
import threading

import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger, TriggerResponse

from aloam_gps_localization.geo import (
    antenna_to_base,
    finite,
    latlon_to_utm,
    normalize_angle,
    quaternion_from_yaw,
    transform_xy,
    yaw_from_quaternion,
)


def _float_param(name, default):
    return float(rospy.get_param(name, default))


def _stamp_or_now(header):
    if header.stamp != rospy.Time(0):
        return header.stamp
    return rospy.Time.now()


class GpsAmclInitializer:
    """Publish one AMCL initial pose from RTK position, heading, and map georeference."""

    REQUIRED_KEYS = (
        "utm_zone",
        "utm_hemisphere",
        "utm_origin_easting",
        "utm_origin_northing",
        "map_origin_x",
        "map_origin_y",
        "utm_to_map_yaw",
    )

    def __init__(self):
        self.lock = threading.RLock()
        self.georef_file = os.path.expanduser(rospy.get_param("~georef_file"))
        self.expected_map_name = rospy.get_param("~map_name", "")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")
        self.heading_topic = rospy.get_param("~heading_topic", "/gps/heading")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.initialpose_topic = rospy.get_param("~initialpose_topic", "/initialpose")
        self.candidate_topic = rospy.get_param(
            "~candidate_topic", "/gps_amcl_initialpose_candidate"
        )

        self.max_gps_variance = _float_param("~max_gps_variance_m2", 0.04)
        self.max_heading_variance = _float_param(
            "~max_heading_variance_rad2", math.radians(5.0) ** 2
        )
        self.max_sensor_age = _float_param("~max_sensor_age_sec", 1.0)
        self.max_georef_rms = _float_param("~max_georef_rms_m", 1.0)
        self.initial_xy_variance = _float_param("~initial_xy_variance_m2", 0.04)
        self.initial_yaw_variance = _float_param(
            "~initial_yaw_variance_rad2", math.radians(5.0) ** 2
        )
        self.publish_count = max(1, int(rospy.get_param("~publish_count", 5)))
        self.publish_interval = max(0.02, _float_param("~publish_interval_sec", 0.20))
        self.map_settle_time = max(0.0, _float_param("~map_settle_time_sec", 1.0))
        self.auto_publish = bool(rospy.get_param("~auto_publish", True))
        self.require_initialpose_subscriber = bool(
            rospy.get_param("~require_initialpose_subscriber", True)
        )

        self.georef = self._load_georef()
        self.latest_gps = None
        self.latest_heading = None
        self.map_msg = None
        self.map_received_time = None
        self.published = False
        self.publishing = False
        self.last_candidate = None
        self.last_reason = "waiting for inputs"

        self.initialpose_pub = rospy.Publisher(
            self.initialpose_topic, PoseWithCovarianceStamped, queue_size=1
        )
        self.candidate_pub = rospy.Publisher(
            self.candidate_topic, PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb, queue_size=20)
        rospy.Subscriber(self.heading_topic, Imu, self._heading_cb, queue_size=20)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Service("~reinitialize", Trigger, self._reinitialize_service)
        rospy.Service("~status", Trigger, self._status_service)
        rospy.Timer(rospy.Duration(0.20), self._timer_cb)

        rospy.loginfo(
            "GPS AMCL initializer ready: georef=%s GPS=%s heading=%s initialpose=%s",
            self.georef_file,
            self.gps_topic,
            self.heading_topic,
            self.initialpose_topic,
        )

    def _load_georef(self):
        if not os.path.isfile(self.georef_file):
            raise RuntimeError("georeference file does not exist: %s" % self.georef_file)
        with open(self.georef_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise RuntimeError("georeference file must contain a YAML dictionary")
        missing = [key for key in self.REQUIRED_KEYS if key not in data]
        if missing:
            raise RuntimeError("georeference file is missing: %s" % ", ".join(missing))
        if int(data.get("version", 0)) != 1:
            raise RuntimeError("unsupported georeference version: %r" % data.get("version"))

        for key in self.REQUIRED_KEYS:
            if key in ("utm_hemisphere",):
                continue
            if not finite(data[key]):
                raise RuntimeError("invalid georeference value for %s" % key)

        file_map_name = str(data.get("map_name", ""))
        if self.expected_map_name and file_map_name != self.expected_map_name:
            raise RuntimeError(
                "georeference map_name %r does not match requested map %r"
                % (file_map_name, self.expected_map_name)
            )
        file_map_frame = str(data.get("map_frame", self.map_frame))
        if file_map_frame != self.map_frame:
            raise RuntimeError(
                "georeference map_frame %r does not match %r"
                % (file_map_frame, self.map_frame)
            )

        fit = data.get("fit", {})
        rms = float(fit.get("rms_error_m", 0.0))
        if rms > self.max_georef_rms:
            raise RuntimeError(
                "georeference RMS %.3f m exceeds allowed %.3f m"
                % (rms, self.max_georef_rms)
            )
        hemisphere = str(data["utm_hemisphere"]).upper()
        if hemisphere not in ("N", "S"):
            raise RuntimeError("utm_hemisphere must be N or S")
        data["utm_hemisphere"] = hemisphere
        return data

    def _gps_cb(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not finite(msg.latitude) or not finite(msg.longitude):
            return
        with self.lock:
            self.latest_gps = msg

    def _heading_cb(self, msg):
        covariance = msg.orientation_covariance
        if covariance and covariance[0] == -1.0:
            return
        try:
            yaw = yaw_from_quaternion(msg.orientation)
        except ValueError:
            return
        yaw_variance = covariance[8] if len(covariance) >= 9 else float("inf")
        with self.lock:
            self.latest_heading = (msg, yaw, yaw_variance)

    def _map_cb(self, msg):
        with self.lock:
            self.map_msg = msg
            if self.map_received_time is None:
                self.map_received_time = rospy.Time.now()

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

    def _candidate(self):
        with self.lock:
            gps = self.latest_gps
            heading_item = self.latest_heading
            map_msg = self.map_msg
            map_received_time = self.map_received_time

        if map_msg is None or map_received_time is None:
            return None, "waiting for map_server"
        if (rospy.Time.now() - map_received_time).to_sec() < self.map_settle_time:
            return None, "waiting for map/AMCL to settle"
        if self.require_initialpose_subscriber and self.initialpose_pub.get_num_connections() < 1:
            return None, "waiting for AMCL /initialpose subscriber"
        if gps is None:
            return None, "waiting for /gps/fix"
        if heading_item is None:
            return None, "waiting for /gps/heading"

        heading_msg, heading_yaw_raw, heading_variance = heading_item
        gps_variance = self._gps_variance(gps)
        if gps_variance > self.max_gps_variance:
            return None, "GPS variance %.4f m^2 exceeds %.4f m^2" % (
                gps_variance,
                self.max_gps_variance,
            )
        if not finite(heading_variance) or heading_variance > self.max_heading_variance:
            return None, "heading variance %.6f rad^2 exceeds %.6f rad^2" % (
                heading_variance,
                self.max_heading_variance,
            )

        now = rospy.Time.now()
        gps_age = (now - _stamp_or_now(gps.header)).to_sec()
        heading_age = (now - _stamp_or_now(heading_msg.header)).to_sec()
        if self.max_sensor_age > 0.0 and abs(gps_age) > self.max_sensor_age:
            return None, "GPS data is stale (%.2fs)" % gps_age
        if self.max_sensor_age > 0.0 and abs(heading_age) > self.max_sensor_age:
            return None, "heading data is stale (%.2fs)" % heading_age

        natural_e, natural_n, natural_zone, hemisphere = latlon_to_utm(
            gps.latitude, gps.longitude
        )
        del natural_e, natural_n
        expected_zone = int(self.georef["utm_zone"])
        expected_hemisphere = self.georef["utm_hemisphere"]
        if natural_zone != expected_zone or hemisphere != expected_hemisphere:
            return None, "GPS UTM zone %d%s does not match map zone %d%s" % (
                natural_zone,
                hemisphere,
                expected_zone,
                expected_hemisphere,
            )
        easting, northing, _zone, _hemisphere = latlon_to_utm(
            gps.latitude, gps.longitude, forced_zone=expected_zone
        )

        heading_offset = float(self.georef.get("heading_yaw_offset", 0.0))
        base_yaw_enu = normalize_angle(heading_yaw_raw + heading_offset)
        gps_antenna_x = float(self.georef.get("gps_antenna_x", 0.0))
        gps_antenna_y = float(self.georef.get("gps_antenna_y", 0.0))
        base_easting, base_northing = antenna_to_base(
            easting,
            northing,
            base_yaw_enu,
            gps_antenna_x,
            gps_antenna_y,
        )

        map_yaw_offset = float(self.georef["utm_to_map_yaw"])
        map_x, map_y = transform_xy(
            base_easting,
            base_northing,
            float(self.georef["utm_origin_easting"]),
            float(self.georef["utm_origin_northing"]),
            float(self.georef["map_origin_x"]),
            float(self.georef["map_origin_y"]),
            map_yaw_offset,
        )
        map_yaw = normalize_angle(base_yaw_enu + map_yaw_offset)

        if not self._inside_map(map_msg, map_x, map_y):
            return None, "RTK pose x=%.3f y=%.3f lies outside the loaded map" % (map_x, map_y)

        fit = self.georef.get("fit", {})
        fit_rms = float(fit.get("rms_error_m", 0.0))
        heading_rms = float(fit.get("heading_rms_rad", 0.0))
        xy_variance = max(
            self.initial_xy_variance,
            gps_variance + fit_rms * fit_rms,
        )
        yaw_variance = max(
            self.initial_yaw_variance,
            heading_variance + heading_rms * heading_rms,
        )

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = map_x
        msg.pose.pose.position.y = map_y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(map_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = xy_variance
        msg.pose.covariance[7] = xy_variance
        msg.pose.covariance[35] = yaw_variance
        return msg, "RTK pose x=%.3f y=%.3f yaw=%.3f" % (map_x, map_y, map_yaw)

    @staticmethod
    def _inside_map(map_msg, x, y):
        origin = map_msg.info.origin
        try:
            origin_yaw = yaw_from_quaternion(origin.orientation)
        except ValueError:
            origin_yaw = 0.0
        dx = x - origin.position.x
        dy = y - origin.position.y
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        width_m = map_msg.info.width * map_msg.info.resolution
        height_m = map_msg.info.height * map_msg.info.resolution
        return 0.0 <= local_x < width_m and 0.0 <= local_y < height_m

    def _timer_cb(self, _event):
        if not self.auto_publish:
            return
        with self.lock:
            if self.published or self.publishing:
                return
        self._attempt_publish()

    def _attempt_publish(self):
        try:
            candidate, reason = self._candidate()
        except (ValueError, TypeError) as exc:
            candidate, reason = None, "failed to calculate RTK pose: %s" % exc

        with self.lock:
            self.last_reason = reason
            if candidate is None or self.publishing:
                return False, reason
            self.last_candidate = candidate
            self.publishing = True

        self.candidate_pub.publish(candidate)
        thread = threading.Thread(target=self._publish_worker, args=(candidate, reason))
        thread.daemon = True
        thread.start()
        return True, reason

    def _publish_worker(self, candidate, reason):
        try:
            for _index in range(self.publish_count):
                candidate.header.stamp = rospy.Time.now()
                self.initialpose_pub.publish(candidate)
                rospy.sleep(self.publish_interval)
            with self.lock:
                self.published = True
                self.last_reason = reason
            rospy.loginfo("AMCL initialized from %s", reason)
        finally:
            with self.lock:
                self.publishing = False

    def _reinitialize_service(self, _request):
        with self.lock:
            if self.publishing:
                return TriggerResponse(success=False, message="initial pose is already being published")
            self.published = False
        ok, message = self._attempt_publish()
        return TriggerResponse(success=ok, message=message)

    def _status_service(self, _request):
        with self.lock:
            gps = self.latest_gps
            heading = self.latest_heading
            map_ready = self.map_msg is not None
            published = self.published
            publishing = self.publishing
            reason = self.last_reason
            candidate = self.last_candidate
        payload = {
            "map_ready": map_ready,
            "gps_received": gps is not None,
            "heading_received": heading is not None,
            "published": published,
            "publishing": publishing,
            "reason": reason,
            "georef_file": self.georef_file,
        }
        if candidate is not None:
            payload["candidate_x"] = candidate.pose.pose.position.x
            payload["candidate_y"] = candidate.pose.pose.position.y
        return TriggerResponse(success=True, message=json.dumps(payload, sort_keys=True))


if __name__ == "__main__":
    rospy.init_node("gps_amcl_initializer")
    try:
        GpsAmclInitializer()
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as exc:
        rospy.logfatal("Cannot start GPS AMCL initializer: %s", exc)
        raise
    rospy.spin()

