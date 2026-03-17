#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowGapPro:

    def __init__(self):

        rospy.init_node("follow_gap_pro")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # ===== SPEED =====
        self.max_speed = 1.5
        self.min_speed = 0.3

        # ===== GAP =====
        self.bubble_radius = 30
        self.min_gap_size = 80

        # ===== SAFETY (hysteresis) =====
        self.stop_dist = 0.7
        self.go_dist = 1.0

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


    # ===============================
    # Lidar preprocess
    # ===============================
    def preprocess(self, ranges):

        proc = np.array(ranges)

        proc[np.isinf(proc)] = 10
        proc[np.isnan(proc)] = 0

        # smoothing
        kernel = np.ones(5) / 5
        proc = np.convolve(proc, kernel, mode='same')

        return proc


    # ===============================
    # Find max gap
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
    # Main callback
    # ===============================
    def scan_callback(self, scan):

        ranges = self.preprocess(scan.ranges)
        twist = Twist()

        center = len(ranges) // 2
        front = np.min(ranges[center - 20:center + 20])

        # ====================================
        # TURN MODE (escape hard)
        # ====================================
        if self.turn_mode:

            twist.linear.x = 0
            twist.angular.z = 1.2

            if rospy.Time.now() - self.turn_start > self.turn_time:
                self.turn_mode = False

            self.cmd_pub.publish(twist)
            return

        # ====================================
        # ESCAPE MODE
        # ====================================
        if self.escape_mode:

            twist.linear.x = -0.5
            twist.angular.z = np.random.choice([-1.0, 1.0])

            if rospy.Time.now() - self.escape_start > self.escape_time:
                self.escape_mode = False

            self.cmd_pub.publish(twist)
            return

        # ====================================
        # HYSTERESIS LOGIC
        # ====================================
        if front < self.stop_dist:
            self.in_danger = True
        elif front > self.go_dist:
            self.in_danger = False

        # ====================================
        # DANGER → ESCAPE
        # ====================================
        if self.in_danger:

            self.escape_mode = True
            self.escape_start = rospy.Time.now()

            self.cmd_pub.publish(Twist())
            return

        # ====================================
        # FOLLOW GAP
        # ====================================

        # find closest obstacle
        closest = np.argmin(ranges)

        start = max(0, closest - self.bubble_radius)
        end = min(len(ranges), closest + self.bubble_radius)

        ranges[start:end] = 0

        gap_start, gap_end = self.find_max_gap(ranges)

        gap_size = gap_end - gap_start

        # nếu gap quá nhỏ → quay thoát
        if gap_size < self.min_gap_size:
            self.turn_mode = True
            self.turn_start = rospy.Time.now()
            self.cmd_pub.publish(Twist())
            return

        # chọn điểm giữa gap
        best = (gap_start + gap_end) // 2

        angle = scan.angle_min + best * scan.angle_increment

        # ====================================
        # SMOOTH STEERING
        # ====================================
        steering = angle * 1.5
        steering = np.clip(steering, -1.0, 1.0)

        twist.angular.z = steering

        # ====================================
        # SPEED CONTROL
        # ====================================
        angle_abs = abs(angle)

        if angle_abs < 0.1:
            speed = self.max_speed
        elif angle_abs < 0.3:
            speed = 1.0
        else:
            speed = self.min_speed

        # thêm giảm tốc khi gần vật
        speed *= min(front / 2.0, 1.0)

        twist.linear.x = speed

        self.cmd_pub.publish(twist)


if __name__ == "__main__":

    FollowGapPro()
    rospy.spin()