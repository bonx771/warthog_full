#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class GapFollowProMax:

    def __init__(self):

        rospy.init_node("gap_follow_pro_max")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # ===== SPEED =====
        self.max_speed = 1.8
        self.min_speed = 0.3

        # ===== GAP =====
        self.bubble_radius = 35
        self.min_gap_size = 60

        # ===== SAFETY =====
        self.stop_dist = 0.6
        self.go_dist = 1.2

        # ===== STATE =====
        self.in_danger = False

        # ===== ESCAPE =====
        self.escape_mode = False
        self.escape_start = None
        self.escape_time = rospy.Duration(1.5)

        # ===== TURN =====
        self.turn_mode = False
        self.turn_start = None
        self.turn_time = rospy.Duration(2.0)

        # ===== MEMORY =====
        self.last_turn_dir = 1

        # ===== STUCK DETECT =====
        self.history = []
        self.stuck_time = rospy.Duration(2.5)
        self.stuck_start = None

        # ===== SMOOTH =====
        self.prev_steering = 0


    # ===============================
    def preprocess(self, ranges):

        proc = np.array(ranges)

        proc[np.isinf(proc)] = 10
        proc[np.isnan(proc)] = 10

        kernel = np.ones(5) / 5
        proc = np.convolve(proc, kernel, mode='same')

        return proc


    # ===============================
    def find_max_gap(self, free_space):

        max_len = 0
        max_start = 0
        max_end = 0

        start = None

        for i in range(len(free_space)):

            if free_space[i] > 0:
                if start is None:
                    start = i
            else:
                if start is not None:
                    length = i - start
                    if length > max_len:
                        max_len = length
                        max_start = start
                        max_end = i
                    start = None

        if start is not None:
            length = len(free_space) - start
            if length > max_len:
                max_start = start
                max_end = len(free_space)

        return max_start, max_end


    # ===============================
    def detect_stuck(self, front):

        self.history.append(front)

        if len(self.history) > 20:
            self.history.pop(0)

        if len(self.history) < 20:
            return False

        movement = max(self.history) - min(self.history)

        if movement < 0.05:
            if self.stuck_start is None:
                self.stuck_start = rospy.Time.now()
            elif rospy.Time.now() - self.stuck_start > self.stuck_time:
                return True
        else:
            self.stuck_start = None

        return False


    # ===============================
    def scan_callback(self, scan):

        ranges = self.preprocess(scan.ranges)
        twist = Twist()

        center = len(ranges) // 2
        front = np.min(ranges[center - 25:center + 25])

        # DEBUG
        rospy.logwarn_throttle(1.0,
            f"front={front:.2f}, danger={self.in_danger}, escape={self.escape_mode}, turn={self.turn_mode}")

        # ====================================
        # STUCK → FORCE BREAK
        # ====================================
        if self.detect_stuck(front):

            rospy.logwarn("STUCK → BREAK LOOP")

            self.turn_mode = True
            self.turn_start = rospy.Time.now()

            self.last_turn_dir *= -1
            self.history = []

            return

        # ====================================
        # TURN MODE
        # ====================================
        if self.turn_mode:

            twist.linear.x = 0.0
            twist.angular.z = 1.5 * self.last_turn_dir

            if rospy.Time.now() - self.turn_start > self.turn_time:
                self.turn_mode = False

            self.cmd_pub.publish(twist)
            return

        # ====================================
        # ESCAPE MODE
        # ====================================
        if self.escape_mode:

            twist.linear.x = -0.8
            twist.angular.z = np.random.uniform(-2.0, 2.0)

            if rospy.Time.now() - self.escape_start > self.escape_time:
                self.escape_mode = False

            self.cmd_pub.publish(twist)
            return

        # ====================================
        # HYSTERESIS
        # ====================================
        if front < self.stop_dist:
            self.in_danger = True
        elif front > self.go_dist:
            self.in_danger = False

        # ====================================
        # DANGER → ESCAPE
        # ====================================
        if self.in_danger and not self.escape_mode:

            self.escape_mode = True
            self.escape_start = rospy.Time.now()
            return

        # ====================================
        # GAP FOLLOW
        # ====================================
        closest = np.argmin(ranges)

        start = max(0, closest - self.bubble_radius)
        end = min(len(ranges), closest + self.bubble_radius)

        ranges[start:end] = 0

        gap_start, gap_end = self.find_max_gap(ranges)
        gap_size = gap_end - gap_start

        if gap_size < self.min_gap_size:

            self.turn_mode = True
            self.turn_start = rospy.Time.now()

            twist.angular.z = 1.5 * self.last_turn_dir
            self.cmd_pub.publish(twist)
            return

        best = (gap_start + gap_end) // 2
        angle = scan.angle_min + best * scan.angle_increment

        # ====================================
        # SMART BIAS (thoát bẫy)
        # ====================================
        bias = 0.15 * self.last_turn_dir

        steering = angle * 1.8 + bias

        # smoothing
        steering = 0.7 * self.prev_steering + 0.3 * steering
        steering = np.clip(steering, -1.5, 1.5)

        self.prev_steering = steering

        twist.angular.z = steering

        # ====================================
        # SPEED CONTROL PRO
        # ====================================
        angle_abs = abs(angle)

        if angle_abs < 0.1:
            speed = self.max_speed
        elif angle_abs < 0.3:
            speed = 1.2
        else:
            speed = self.min_speed

        # scale theo khoảng trống
        speed *= max(min(front / 2.5, 1.0), 0.25)

        twist.linear.x = speed

        self.cmd_pub.publish(twist)


if __name__ == "__main__":

    GapFollowProMax()
    rospy.spin()