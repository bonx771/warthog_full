#!/usr/bin/env python3

import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger, TriggerResponse


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _float_param(name, default=float("nan")):
    value = rospy.get_param(name, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _latlon_to_utm(latitude, longitude):
    """Convert WGS84 latitude/longitude to UTM easting/northing."""
    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)

    zone = int((longitude + 180.0) / 6.0) + 1
    lon_origin = math.radians((zone - 1) * 6 - 180 + 3)

    a = 6378137.0
    f = 1.0 / 298.257223563
    k0 = 0.9996
    e_sq = f * (2.0 - f)
    e_prime_sq = e_sq / (1.0 - e_sq)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    tan_lat = math.tan(lat_rad)

    n = a / math.sqrt(1.0 - e_sq * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = e_prime_sq * cos_lat * cos_lat
    aa = cos_lat * (lon_rad - lon_origin)

    m = a * (
        (1.0 - e_sq / 4.0 - 3.0 * e_sq ** 2 / 64.0 - 5.0 * e_sq ** 3 / 256.0) * lat_rad
        - (3.0 * e_sq / 8.0 + 3.0 * e_sq ** 2 / 32.0 + 45.0 * e_sq ** 3 / 1024.0) * math.sin(2.0 * lat_rad)
        + (15.0 * e_sq ** 2 / 256.0 + 45.0 * e_sq ** 3 / 1024.0) * math.sin(4.0 * lat_rad)
        - (35.0 * e_sq ** 3 / 3072.0) * math.sin(6.0 * lat_rad)
    )

    easting = k0 * n * (
        aa
        + (1.0 - t + c) * aa ** 3 / 6.0
        + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * e_prime_sq) * aa ** 5 / 120.0
    ) + 500000.0

    northing = k0 * (
        m
        + n * tan_lat * (
            aa * aa / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c * c) * aa ** 4 / 24.0
            + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * e_prime_sq) * aa ** 6 / 720.0
        )
    )

    hemisphere = "N"
    if latitude < 0.0:
        northing += 10000000.0
        hemisphere = "S"

    return easting, northing, zone, hemisphere


class GpsInitialposeManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")
        self.heading_topic = rospy.get_param("~heading_topic", "/imu/data")
        self.amcl_pose_topic = rospy.get_param("~amcl_pose_topic", "/amcl_pose")
        self.initialpose_topic = rospy.get_param("~initialpose_topic", "/initialpose")

        self.use_gps_initialpose = rospy.get_param("~use_gps_initialpose", True)
        self.use_saved_pose_fallback = rospy.get_param("~use_saved_pose_fallback", True)
        self.auto_publish_on_start = rospy.get_param("~auto_publish_on_start", True)
        self.gps_wait_timeout = _float_param("~gps_wait_timeout_sec", 8.0)
        self.auto_publish_timeout = _float_param("~auto_publish_timeout_sec", 30.0)
        self.publish_count = int(rospy.get_param("~initialpose_publish_count", 5))
        self.publish_interval = _float_param("~initialpose_publish_interval_sec", 0.2)
        self.save_period = _float_param("~save_period_sec", 1.0)

        self.max_gps_covariance = _float_param("~max_gps_covariance", 1.0)
        self.xy_covariance = _float_param("~initial_xy_covariance", 0.25)
        self.yaw_covariance = _float_param("~initial_yaw_covariance", 0.0685)
        self.heading_timeout = _float_param("~heading_timeout_sec", 2.0)

        self.gps_antenna_x = _float_param("~gps_antenna_x", float("nan"))
        self.gps_antenna_y = _float_param("~gps_antenna_y", float("nan"))
        self.heading_yaw_offset = _float_param("~heading_yaw_offset", float("nan"))

        self.utm_to_map_yaw = _float_param("~utm_to_map_yaw", float("nan"))
        self.map_origin_x = _float_param("~map_origin_x", float("nan"))
        self.map_origin_y = _float_param("~map_origin_y", float("nan"))
        self.utm_origin_easting = _float_param("~utm_origin_easting", float("nan"))
        self.utm_origin_northing = _float_param("~utm_origin_northing", float("nan"))
        self.gps_origin_lat = _float_param("~gps_origin_lat", float("nan"))
        self.gps_origin_lon = _float_param("~gps_origin_lon", float("nan"))

        default_pose_file = os.path.expanduser("~/.ros/gps_pkg/last_amcl_pose.json")
        default_alignment_file = os.path.expanduser("~/.ros/gps_pkg/gps_map_alignment.json")
        self.pose_file = os.path.expanduser(rospy.get_param("~pose_file", default_pose_file))
        self.alignment_file = os.path.expanduser(rospy.get_param("~alignment_file", default_alignment_file))
        self.max_saved_pose_age = _float_param("~max_saved_pose_age_sec", 0.0)

        self.last_gps = None
        self.last_heading = None
        self.last_heading_time = None
        self.last_amcl_pose = None
        self.last_save_time = rospy.Time(0)
        self.start_time = rospy.Time.now()
        self.auto_publish_done = False
        self.auto_publish_expired = False

        self._load_alignment()
        self._resolve_gps_origin()
        self._apply_default_offsets()
        self.saved_pose = self._load_saved_pose()

        self.initialpose_pub = rospy.Publisher(
            self.initialpose_topic,
            PoseWithCovarianceStamped,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb, queue_size=10)
        rospy.Subscriber(self.heading_topic, Imu, self._heading_cb, queue_size=10)
        rospy.Subscriber(self.amcl_pose_topic, PoseWithCovarianceStamped, self._amcl_pose_cb, queue_size=10)

        rospy.Service("~publish_initialpose", Trigger, self._publish_initialpose_srv)
        rospy.Service("~publish_saved_pose", Trigger, self._publish_saved_pose_srv)
        rospy.Service("~save_alignment_from_current_pose", Trigger, self._save_alignment_srv)

        rospy.Timer(rospy.Duration(0.2), self._timer_cb)

        rospy.loginfo("GPS initialpose manager started. gps=%s heading=%s amcl=%s initialpose=%s",
                      self.gps_topic, self.heading_topic, self.amcl_pose_topic, self.initialpose_topic)
        if not self._gps_alignment_ready():
            rospy.logwarn(
                "GPS->map alignment is not configured. Set gps_origin_lat/lon or utm_origin_easting/northing, "
                "plus map_origin_x/y and utm_to_map_yaw. Saved AMCL pose fallback can still work."
            )

    def _resolve_gps_origin(self):
        if _finite(self.utm_origin_easting) and _finite(self.utm_origin_northing):
            return
        if _finite(self.gps_origin_lat) and _finite(self.gps_origin_lon):
            easting, northing, zone, hemi = _latlon_to_utm(self.gps_origin_lat, self.gps_origin_lon)
            self.utm_origin_easting = easting
            self.utm_origin_northing = northing
            rospy.loginfo("GPS origin converted to UTM: e=%.3f n=%.3f zone=%d%s",
                          easting, northing, zone, hemi)

    def _apply_default_offsets(self):
        if not _finite(self.gps_antenna_x):
            self.gps_antenna_x = 0.0
        if not _finite(self.gps_antenna_y):
            self.gps_antenna_y = 0.0
        if not _finite(self.heading_yaw_offset):
            self.heading_yaw_offset = 0.0

    def _load_alignment(self):
        if not os.path.exists(self.alignment_file):
            return
        try:
            with open(self.alignment_file, "r") as f:
                data = json.load(f)
        except (OSError, ValueError) as exc:
            rospy.logwarn("Failed to load GPS->map alignment from %s: %s", self.alignment_file, exc)
            return

        fields = (
            "utm_origin_easting",
            "utm_origin_northing",
            "map_origin_x",
            "map_origin_y",
            "utm_to_map_yaw",
            "heading_yaw_offset",
            "gps_antenna_x",
            "gps_antenna_y",
        )
        for field in fields:
            if field not in data or not _finite(data[field]):
                continue
            if field == "utm_origin_easting" and not _finite(self.utm_origin_easting):
                self.utm_origin_easting = float(data[field])
            elif field == "utm_origin_northing" and not _finite(self.utm_origin_northing):
                self.utm_origin_northing = float(data[field])
            elif field == "map_origin_x" and not _finite(self.map_origin_x):
                self.map_origin_x = float(data[field])
            elif field == "map_origin_y" and not _finite(self.map_origin_y):
                self.map_origin_y = float(data[field])
            elif field == "utm_to_map_yaw" and not _finite(self.utm_to_map_yaw):
                self.utm_to_map_yaw = float(data[field])
            elif field == "heading_yaw_offset" and not _finite(self.heading_yaw_offset):
                self.heading_yaw_offset = float(data[field])
            elif field == "gps_antenna_x" and not _finite(self.gps_antenna_x):
                self.gps_antenna_x = float(data[field])
            elif field == "gps_antenna_y" and not _finite(self.gps_antenna_y):
                self.gps_antenna_y = float(data[field])

        rospy.loginfo("Loaded GPS->map alignment from %s", self.alignment_file)

    def _gps_alignment_ready(self):
        return (
            _finite(self.utm_origin_easting)
            and _finite(self.utm_origin_northing)
            and _finite(self.map_origin_x)
            and _finite(self.map_origin_y)
            and _finite(self.utm_to_map_yaw)
        )

    def _gps_cb(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not _finite(msg.latitude) or not _finite(msg.longitude):
            return
        with self.lock:
            self.last_gps = msg

    def _heading_cb(self, msg):
        if msg.orientation_covariance and msg.orientation_covariance[0] == -1.0:
            return
        yaw = _yaw_from_quaternion(msg.orientation)
        with self.lock:
            self.last_heading = yaw
            self.last_heading_time = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

    def _amcl_pose_cb(self, msg):
        with self.lock:
            self.last_amcl_pose = msg
        now = rospy.Time.now()
        if (now - self.last_save_time).to_sec() >= self.save_period:
            self._save_pose(msg)
            self.last_save_time = now

    def _timer_cb(self, _event):
        if not self.auto_publish_on_start or self.auto_publish_done or self.auto_publish_expired:
            return

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        ok, message = self._publish_best_initialpose(allow_saved=elapsed >= self.gps_wait_timeout)
        if ok:
            self.auto_publish_done = True
            rospy.loginfo("Auto initialpose published: %s", message)
            return

        if elapsed >= self.auto_publish_timeout:
            self.auto_publish_expired = True
            rospy.logwarn("Auto initialpose timed out after %.1fs: %s", elapsed, message)

    def _publish_initialpose_srv(self, _request):
        ok, message = self._publish_best_initialpose(allow_saved=True)
        return TriggerResponse(success=ok, message=message)

    def _publish_saved_pose_srv(self, _request):
        ok, message = self._publish_saved_pose()
        return TriggerResponse(success=ok, message=message)

    def _save_alignment_srv(self, _request):
        ok, message = self._save_alignment_from_current_pose()
        return TriggerResponse(success=ok, message=message)

    def _publish_best_initialpose(self, allow_saved):
        if self.use_gps_initialpose:
            ok, message = self._publish_gps_pose()
            if ok:
                return ok, message
            gps_error = message
        else:
            gps_error = "GPS initialpose disabled"

        if allow_saved and self.use_saved_pose_fallback:
            ok, message = self._publish_saved_pose()
            if ok:
                return ok, message
            return False, "%s; saved pose unavailable: %s" % (gps_error, message)

        return False, gps_error

    def _publish_gps_pose(self):
        if not self._gps_alignment_ready():
            return False, "GPS->map alignment is not configured"

        with self.lock:
            gps = self.last_gps
            heading = self.last_heading
            heading_time = self.last_heading_time

        if gps is None:
            return False, "waiting for valid GPS fix"

        gps_cov = self._gps_xy_covariance(gps)
        if gps_cov > self.max_gps_covariance:
            return False, "GPS covariance %.3f is above max %.3f" % (gps_cov, self.max_gps_covariance)

        if heading is None or heading_time is None:
            return False, "waiting for heading IMU"

        heading_age = abs((rospy.Time.now() - heading_time).to_sec())
        if heading_age > self.heading_timeout:
            return False, "heading is too old: %.2fs" % heading_age

        easting, northing, zone, hemi = _latlon_to_utm(gps.latitude, gps.longitude)
        dx = easting - self.utm_origin_easting
        dy = northing - self.utm_origin_northing
        c = math.cos(self.utm_to_map_yaw)
        s = math.sin(self.utm_to_map_yaw)

        antenna_x = self.map_origin_x + c * dx - s * dy
        antenna_y = self.map_origin_y + s * dx + c * dy
        yaw = _normalize_angle(heading + self.utm_to_map_yaw + self.heading_yaw_offset)

        base_x = antenna_x - (
            math.cos(yaw) * self.gps_antenna_x - math.sin(yaw) * self.gps_antenna_y
        )
        base_y = antenna_y - (
            math.sin(yaw) * self.gps_antenna_x + math.cos(yaw) * self.gps_antenna_y
        )

        pose = self._make_pose(base_x, base_y, yaw, max(gps_cov, self.xy_covariance), self.yaw_covariance)
        self._publish_pose_repeated(pose)
        return True, "GPS initialpose x=%.3f y=%.3f yaw=%.3f UTM=%d%s" % (
            base_x, base_y, yaw, zone, hemi
        )

    def _publish_saved_pose(self):
        saved = self.saved_pose
        if saved is None:
            saved = self._load_saved_pose()
            self.saved_pose = saved
        if saved is None:
            return False, "no saved AMCL pose file"

        stamp = saved.get("stamp", 0.0)
        if self.max_saved_pose_age > 0.0 and stamp > 0.0:
            age = rospy.Time.now().to_sec() - stamp
            if age > self.max_saved_pose_age:
                return False, "saved pose is too old: %.1fs" % age

        pose = self._make_pose(
            saved["x"],
            saved["y"],
            saved["yaw"],
            saved.get("xy_covariance", self.xy_covariance),
            saved.get("yaw_covariance", self.yaw_covariance),
        )
        self._publish_pose_repeated(pose)
        return True, "saved initialpose x=%.3f y=%.3f yaw=%.3f" % (
            saved["x"], saved["y"], saved["yaw"]
        )

    def _make_pose(self, x, y, yaw, xy_covariance, yaw_covariance):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = xy_covariance
        msg.pose.covariance[7] = xy_covariance
        msg.pose.covariance[35] = yaw_covariance
        return msg

    def _publish_pose_repeated(self, pose):
        count = max(1, self.publish_count)
        for _ in range(count):
            pose.header.stamp = rospy.Time.now()
            self.initialpose_pub.publish(pose)
            rospy.sleep(self.publish_interval)

    def _gps_xy_covariance(self, msg):
        cov = msg.position_covariance
        cov_x = cov[0] if len(cov) > 0 and _finite(cov[0]) else self.xy_covariance
        cov_y = cov[4] if len(cov) > 4 and _finite(cov[4]) else self.xy_covariance
        return max(cov_x, cov_y)

    def _save_pose(self, msg):
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        cov = msg.pose.covariance
        data = {
            "frame_id": msg.header.frame_id or self.map_frame,
            "stamp": rospy.Time.now().to_sec(),
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw,
            "xy_covariance": max(cov[0], cov[7]) if len(cov) >= 8 else self.xy_covariance,
            "yaw_covariance": cov[35] if len(cov) >= 36 else self.yaw_covariance,
        }
        try:
            directory = os.path.dirname(self.pose_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            tmp_file = self.pose_file + ".tmp"
            with open(tmp_file, "w") as f:
                json.dump(data, f, indent=2, sort_keys=True)
            os.replace(tmp_file, self.pose_file)
            self.saved_pose = data
        except OSError as exc:
            rospy.logwarn_throttle(5.0, "Failed to save AMCL pose to %s: %s", self.pose_file, exc)

    def _save_alignment_from_current_pose(self):
        with self.lock:
            gps = self.last_gps
            heading = self.last_heading
            heading_time = self.last_heading_time
            amcl_pose = self.last_amcl_pose

        if gps is None:
            return False, "no valid GPS fix available"
        if heading is None or heading_time is None:
            return False, "no valid heading available"
        if amcl_pose is None:
            return False, "no AMCL pose available"

        heading_age = abs((rospy.Time.now() - heading_time).to_sec())
        if heading_age > self.heading_timeout:
            return False, "heading is too old: %.2fs" % heading_age

        easting, northing, zone, hemi = _latlon_to_utm(gps.latitude, gps.longitude)
        base_x = amcl_pose.pose.pose.position.x
        base_y = amcl_pose.pose.pose.position.y
        yaw_map = _yaw_from_quaternion(amcl_pose.pose.pose.orientation)
        antenna_x = base_x + (
            math.cos(yaw_map) * self.gps_antenna_x - math.sin(yaw_map) * self.gps_antenna_y
        )
        antenna_y = base_y + (
            math.sin(yaw_map) * self.gps_antenna_x + math.cos(yaw_map) * self.gps_antenna_y
        )

        self.utm_origin_easting = easting
        self.utm_origin_northing = northing
        self.map_origin_x = antenna_x
        self.map_origin_y = antenna_y
        self.utm_to_map_yaw = _normalize_angle(yaw_map - heading - self.heading_yaw_offset)

        data = {
            "stamp": rospy.Time.now().to_sec(),
            "utm_origin_easting": self.utm_origin_easting,
            "utm_origin_northing": self.utm_origin_northing,
            "utm_zone": "%d%s" % (zone, hemi),
            "map_origin_x": self.map_origin_x,
            "map_origin_y": self.map_origin_y,
            "utm_to_map_yaw": self.utm_to_map_yaw,
            "heading_yaw_offset": self.heading_yaw_offset,
            "gps_antenna_x": self.gps_antenna_x,
            "gps_antenna_y": self.gps_antenna_y,
        }

        try:
            directory = os.path.dirname(self.alignment_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            tmp_file = self.alignment_file + ".tmp"
            with open(tmp_file, "w") as f:
                json.dump(data, f, indent=2, sort_keys=True)
            os.replace(tmp_file, self.alignment_file)
        except OSError as exc:
            return False, "failed to save alignment to %s: %s" % (self.alignment_file, exc)

        return True, "saved GPS->map alignment using UTM %d%s, map x=%.3f y=%.3f yaw=%.3f" % (
            zone, hemi, self.map_origin_x, self.map_origin_y, self.utm_to_map_yaw
        )

    def _load_saved_pose(self):
        if not os.path.exists(self.pose_file):
            return None
        try:
            with open(self.pose_file, "r") as f:
                data = json.load(f)
            for key in ("x", "y", "yaw"):
                if key not in data or not _finite(data[key]):
                    rospy.logwarn("Ignoring saved pose with invalid key %s in %s", key, self.pose_file)
                    return None
            return data
        except (OSError, ValueError) as exc:
            rospy.logwarn("Failed to load saved AMCL pose from %s: %s", self.pose_file, exc)
            return None


if __name__ == "__main__":
    rospy.init_node("gps_initialpose_manager")
    GpsInitialposeManager()
    rospy.spin()
