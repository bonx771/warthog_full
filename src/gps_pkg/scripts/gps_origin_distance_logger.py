#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6371000.0


def _is_valid_fix(msg):
    return (
        msg.status.status != NavSatStatus.STATUS_NO_FIX
        and math.isfinite(msg.latitude)
        and math.isfinite(msg.longitude)
    )


def _haversine_distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    sin_dlat = math.sin(dlat / 2.0)
    sin_dlon = math.sin(dlon / 2.0)
    a = sin_dlat * sin_dlat + math.cos(lat1) * math.cos(lat2) * sin_dlon * sin_dlon
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


class GpsOriginDistanceLogger:
    def __init__(self):
        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")
        self.min_position_change_deg = float(
            rospy.get_param("~min_position_change_deg", 1e-10)
        )

        self.origin_fix = None
        self.last_logged_fix = None
        self.fix_count = 0

        self.subscriber = rospy.Subscriber(
            self.gps_topic, NavSatFix, self._callback, queue_size=10
        )

        rospy.loginfo(
            "Logging GPS distance from first valid fix on topic %s",
            self.gps_topic,
        )

    def _callback(self, msg):
        if not _is_valid_fix(msg):
            rospy.logwarn_throttle(5.0, "Skipping GPS sample without valid fix.")
            return

        current_fix = (msg.latitude, msg.longitude, msg.altitude)

        if self.origin_fix is None:
            self.origin_fix = current_fix
            self.last_logged_fix = current_fix
            self.fix_count = 1
            rospy.loginfo(
                "GPS origin fix set: lat=%.8f, lon=%.8f, alt=%.3f",
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )
            return

        if self.last_logged_fix is not None:
            lat_changed = abs(current_fix[0] - self.last_logged_fix[0])
            lon_changed = abs(current_fix[1] - self.last_logged_fix[1])
            alt_changed = abs(current_fix[2] - self.last_logged_fix[2])
            if (
                lat_changed < self.min_position_change_deg
                and lon_changed < self.min_position_change_deg
                and alt_changed < 1e-6
            ):
                return

        self.fix_count += 1
        self.last_logged_fix = current_fix

        distance_m = _haversine_distance_m(
            self.origin_fix[0],
            self.origin_fix[1],
            msg.latitude,
            msg.longitude,
        )

        rospy.loginfo(
            "GPS fix #%d distance from origin: %.3f m | lat=%.8f, lon=%.8f, alt=%.3f",
            self.fix_count,
            distance_m,
            msg.latitude,
            msg.longitude,
            msg.altitude,
        )


if __name__ == "__main__":
    rospy.init_node("gps_origin_distance_logger")
    GpsOriginDistanceLogger()
    rospy.spin()
