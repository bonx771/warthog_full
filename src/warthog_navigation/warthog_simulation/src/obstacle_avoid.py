#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoid:
    def __init__(self):
        rospy.init_node("obstacle_avoid")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.turning = False
        self.turn_time = rospy.Duration(2.0)   # thời gian quay ~ 90°
        self.start_turn = None

    def scan_callback(self, msg):

        front = min(msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10])

        twist = Twist()

        if front < 1.0 and not self.turning:
            self.turning = True
            self.start_turn = rospy.Time.now()

        if self.turning:
            twist.angular.z = -0.6   # quay phải
            if rospy.Time.now() - self.start_turn > self.turn_time:
                self.turning = False
        else:
            twist.linear.x = 0.5

        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    ObstacleAvoid()
    rospy.spin()