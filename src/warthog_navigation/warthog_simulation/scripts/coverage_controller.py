#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

FORWARD = 0
ROTATING = 1

class CoverageController:
    def __init__(self):
        rospy.init_node("coverage_controller")

        self.pub = rospy.Publisher("/cmd_vel_1", Twist, queue_size=10)
        rospy.Subscriber("/front/scan", LaserScan, self.scan_cb)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        rospy.Subscriber("/coverage_cmd", String, self.cmd_cb)

        self.active = False
        self.state = FORWARD

        self.left = 10.0
        self.front = 10.0
        self.right = 10.0
        self.yaw = 0.0
        self.target_yaw = 0.0

        # ===== THAM SỐ QUAN TRỌNG =====
        self.SAFE_DIST = 2.2          # tăng khoảng cách an toàn
        self.TURN_ANGLE = math.pi/2   # 90 độ
        self.YAW_TOL = math.radians(3)  # sai số góc
        self.TURN_SPEED = 0.3         # tốc độ quay
        self.FORWARD_SPEED = 0.25      # tốc độ đi thẳng

        self.rate = rospy.Rate(10)
        rospy.loginfo("Coverage controller ready")

    def cmd_cb(self, msg):
        self.active = (msg.data == "START")
        if not self.active:
            self.state = FORWARD
            self.pub.publish(Twist())

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

    def safe_min(self, arr):
        arr = [d for d in arr if not math.isinf(d) and not math.isnan(d)]
        return min(arr) if arr else 10.0

    def scan_cb(self, scan):
        n = len(scan.ranges)

        front = scan.ranges[int(0.45*n):int(0.55*n)]
        left  = scan.ranges[int(0.65*n):int(0.85*n)]
        right = scan.ranges[int(0.15*n):int(0.35*n)]

        self.front = self.safe_min(front)
        self.left  = self.safe_min(left)
        self.right = self.safe_min(right)

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    def run(self):
        while not rospy.is_shutdown():
            cmd = Twist()

            if not self.active:
                self.pub.publish(cmd)
                self.rate.sleep()
                continue

            # ===== ĐI THẲNG =====
            if self.state == FORWARD:
                if self.front < self.SAFE_DIST:
                    # chọn hướng quay
                    if self.left > self.right:
                        self.target_yaw = self.yaw + self.TURN_ANGLE   # quay trái
                    else:
                        self.target_yaw = self.yaw - self.TURN_ANGLE   # quay phải
                    self.state = ROTATING

                elif self.left < self.SAFE_DIST * 0.9:
                    # bánh trái sắp chạm → né phải nhẹ
                    cmd.angular.z = -0.3
                    cmd.linear.x = 0.2

                elif self.right < self.SAFE_DIST * 0.9:
                    # bánh phải sắp chạm → né trái nhẹ
                    cmd.angular.z = 0.3
                    cmd.linear.x = 0.2

                else:
                    cmd.linear.x = self.FORWARD_SPEED

            # ===== QUAY 90 ĐỘ =====
            elif self.state == ROTATING:
                error = self.angle_diff(self.target_yaw, self.yaw)
                if abs(error) > self.YAW_TOL:
                    cmd.angular.z = self.TURN_SPEED if error > 0 else -self.TURN_SPEED
                else:
                    self.state = FORWARD

            self.pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    CoverageController().run()
