#!/usr/bin/env python3

import math
import threading

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

from aloam_gps_localization.heading_guard import (
    path_heading_error,
    rate_limit,
    turn_speed,
)


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class SharpTurnGuard:
    """Stop forward motion and align with the path before a sharp turn."""

    def __init__(self):
        self.input_topic = rospy.get_param("~input_cmd_vel_topic", "/cmd_vel_trajectory_raw")
        self.output_topic = rospy.get_param("~output_cmd_vel_topic", "/cmd_vel")
        self.plan_topic = rospy.get_param("~plan_topic", "/move_base/NavfnROS/plan")
        self.status_topic = rospy.get_param("~status_topic", "/move_base/status")
        self.base_frame = rospy.get_param("~base_frame", "base_link").lstrip("/")

        self.enter_angle = math.radians(float(rospy.get_param("~enter_angle_deg", 85.0)))
        self.exit_angle = math.radians(float(rospy.get_param("~exit_angle_deg", 25.0)))
        self.lookahead_distance = float(rospy.get_param("~lookahead_distance", 1.0))
        self.minimum_speed = float(rospy.get_param("~minimum_angular_speed", 0.40))
        self.maximum_speed = float(rospy.get_param("~maximum_angular_speed", 0.65))
        self.angular_acceleration = float(
            rospy.get_param("~angular_acceleration_limit", 1.50)
        )
        self.turn_gain = float(rospy.get_param("~turn_gain", 1.40))
        self.nominal_control_frequency = float(
            rospy.get_param("~nominal_control_frequency", 10.0)
        )
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.05))

        self._validate_parameters()

        self._lock = threading.Lock()
        self._path_points = []
        self._path_frame = ""
        self._goal_active = False
        self._turning = False
        self._last_angular = 0.0
        self._last_publish_time = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._publisher = rospy.Publisher(self.output_topic, Twist, queue_size=1)

        rospy.Subscriber(self.plan_topic, Path, self._plan_callback, queue_size=1)
        rospy.Subscriber(
            self.status_topic, GoalStatusArray, self._status_callback, queue_size=5
        )
        rospy.Subscriber(self.input_topic, Twist, self._cmd_vel_callback, queue_size=1)

        rospy.loginfo(
            "Sharp-turn guard: %.1f deg enter, %.1f deg exit, %.2f rad/s max, "
            "%.2f rad/s^2 acceleration (%s -> %s)",
            math.degrees(self.enter_angle),
            math.degrees(self.exit_angle),
            self.maximum_speed,
            self.angular_acceleration,
            self.input_topic,
            self.output_topic,
        )

    def _validate_parameters(self):
        if not 0.0 < self.enter_angle <= math.pi:
            raise ValueError("enter_angle_deg must be in (0, 180]")
        if not 0.0 <= self.exit_angle < self.enter_angle:
            raise ValueError("exit_angle_deg must be smaller than enter_angle_deg")
        if self.lookahead_distance <= 0.0:
            raise ValueError("lookahead_distance must be positive")
        if not 0.0 < self.minimum_speed <= self.maximum_speed:
            raise ValueError(
                "angular speeds must satisfy 0 < minimum_angular_speed <= maximum_angular_speed"
            )
        if self.angular_acceleration <= 0.0:
            raise ValueError("angular_acceleration_limit must be positive")
        if self.turn_gain <= 0.0:
            raise ValueError("turn_gain must be positive")
        if self.nominal_control_frequency <= 0.0:
            raise ValueError("nominal_control_frequency must be positive")

    def _plan_callback(self, message):
        frame = message.header.frame_id.lstrip("/")
        if not frame and message.poses:
            frame = message.poses[0].header.frame_id.lstrip("/")
        points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
        ]
        with self._lock:
            self._path_frame = frame
            self._path_points = points
            # A non-empty global plan is emitted for an active navigation goal.
            # This avoids passing through one control tick before the action
            # status topic catches up with a newly accepted goal.
            if points:
                self._goal_active = True

    def _status_callback(self, message):
        goal_active = any(item.status == GoalStatus.ACTIVE for item in message.status_list)
        publish_stop = False
        with self._lock:
            if self._goal_active and not goal_active and self._turning:
                self._turning = False
                self._last_angular = 0.0
                self._last_publish_time = rospy.get_time()
                publish_stop = True
            self._goal_active = goal_active
        if publish_stop:
            self._publisher.publish(Twist())

    def _heading_error(self):
        with self._lock:
            frame = self._path_frame
            points = list(self._path_points)

        if not frame or not points:
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(2.0, "Sharp-turn guard TF unavailable: %s", error)
            return None

        translation = transform.transform.translation
        robot_yaw = _yaw_from_quaternion(transform.transform.rotation)
        return path_heading_error(
            points,
            translation.x,
            translation.y,
            robot_yaw,
            self.lookahead_distance,
        )

    def _publish_passthrough(self, message, now):
        with self._lock:
            self._last_angular = message.angular.z
            self._last_publish_time = now
        self._publisher.publish(message)

    def _cmd_vel_callback(self, message):
        now = rospy.get_time()
        with self._lock:
            goal_active = self._goal_active
            turning = self._turning

        if not goal_active:
            with self._lock:
                self._turning = False
            self._publish_passthrough(message, now)
            return

        heading_error = self._heading_error()
        if heading_error is None:
            if turning:
                stop = Twist()
                with self._lock:
                    self._last_angular = 0.0
                    self._last_publish_time = now
                self._publisher.publish(stop)
            else:
                self._publish_passthrough(message, now)
            return

        absolute_error = abs(heading_error)
        if not turning and absolute_error >= self.enter_angle:
            turning = True
            rospy.loginfo(
                "Sharp turn %.1f deg: stopping translation and rotating in place",
                math.degrees(heading_error),
            )
        elif turning and absolute_error <= self.exit_angle:
            turning = False
            rospy.loginfo(
                "Path aligned to %.1f deg: returning control to TrajectoryPlannerROS",
                math.degrees(heading_error),
            )

        with self._lock:
            self._turning = turning

        if not turning:
            self._publish_passthrough(message, now)
            return

        desired_angular = turn_speed(
            heading_error,
            self.minimum_speed,
            self.maximum_speed,
            self.turn_gain,
        )
        with self._lock:
            if self._last_publish_time is None:
                delta_time = 1.0 / self.nominal_control_frequency
            else:
                delta_time = now - self._last_publish_time
                if delta_time <= 0.0 or delta_time > 0.5:
                    delta_time = 1.0 / self.nominal_control_frequency
            angular = rate_limit(
                self._last_angular,
                desired_angular,
                self.angular_acceleration * delta_time,
            )
            self._last_angular = angular
            self._last_publish_time = now

        command = Twist()
        command.angular.z = angular
        self._publisher.publish(command)


def main():
    rospy.init_node("sharp_turn_guard")
    SharpTurnGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
