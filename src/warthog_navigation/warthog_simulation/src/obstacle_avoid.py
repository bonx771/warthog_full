#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowGap:

    def __init__(self):

        rospy.init_node("follow_gap")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # ===== SPEED =====
        self.max_speed = 1.8
        self.min_speed = 0.3

        # ===== SAFETY =====
        self.bubble_radius = 35

        # ===== BACKUP MODE =====
        self.backup_mode = False
        self.backup_start = None
        self.backup_time = rospy.Duration(1.5)

        # ===== STUCK DETECTION =====
        self.stuck_count = 0

        # ===== TURN MODE (escape) =====
        self.turn_mode = False
        self.turn_start = None
        self.turn_time = rospy.Duration(1.0)   # ~90 degree


    def preprocess_lidar(self, ranges):

        proc = np.array(ranges)

        proc[np.isinf(proc)] = 10
        proc[np.isnan(proc)] = 0

        return proc


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


    def scan_callback(self, scan):

        ranges = self.preprocess_lidar(scan.ranges)

        twist = Twist()

        front = np.min(ranges[len(ranges)//2 - 20 : len(ranges)//2 + 20])


        # ===== BACKUP MODE =====
        if self.backup_mode:

            twist.linear.x = -1.0
            twist.angular.z = 0.8

            if rospy.Time.now() - self.backup_start > self.backup_time:
                self.backup_mode = False

            self.cmd_pub.publish(twist)
            return


        # ===== ESCAPE TURN MODE =====
        if self.turn_mode:

            twist.linear.x = 0
            twist.angular.z = 1.5

            if rospy.Time.now() - self.turn_start > self.turn_time:
                self.turn_mode = False

            self.cmd_pub.publish(twist)
            return


        # ===== EMERGENCY STOP =====
        if front < 0.8:

            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_pub.publish(twist)

            self.backup_mode = True
            self.backup_start = rospy.Time.now()

            return


        # ===== FOLLOW GAP =====

        closest = np.argmin(ranges)

        start = max(0, closest - self.bubble_radius)
        end = min(len(ranges), closest + self.bubble_radius)

        ranges[start:end] = 0

        gap_start, gap_end = self.find_max_gap(ranges)

        best = int((gap_start + gap_end) / 2)

        angle = scan.angle_min + best * scan.angle_increment


        # quay nhanh hơn
        twist.angular.z = angle * 3.5


        # ===== SPEED CONTROL =====

        if abs(angle) < 0.1:
            speed = self.max_speed
        elif abs(angle) < 0.35:
            speed = 1.0
        else:
            speed = self.min_speed

        twist.linear.x = speed


        # ===== STUCK DETECTION =====
        if speed < 0.4:
            self.stuck_count += 1
        else:
            self.stuck_count = 0


        if self.stuck_count > 3:

            # kích hoạt quay 90°
            self.turn_mode = True
            self.turn_start = rospy.Time.now()
            self.stuck_count = 0


        self.cmd_pub.publish(twist)


if __name__ == "__main__":

    FollowGap()
    rospy.spin()