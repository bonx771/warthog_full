#!/usr/bin/env python3
# RealSense D455 ALL data subscriber + auto start driver

import rospy
import subprocess
import time

from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
from cv_bridge import CvBridge


class RealSenseSubscriberNode:

    def __init__(self):

        # Init ROS node
        rospy.init_node('camera_node', anonymous=False)

        self.driver_process = subprocess.Popen([
            "roslaunch",
            "realsense2_camera",
            "rs_camera.launch"
        ])

        # đợi driver publish topic
        time.sleep(3)

        self.camera_ns = "camera"

        self.color_topic = f"/{self.camera_ns}/color/image_raw"
        self.depth_topic = f"/{self.camera_ns}/depth/image_rect_raw"
        self.ir1_topic = f"/{self.camera_ns}/infra1/image_rect_raw"
        self.ir2_topic = f"/{self.camera_ns}/infra2/image_rect_raw"

        self.bridge = CvBridge()

        # Subscribers
        rospy.Subscriber(self.color_topic, Image, self.color_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)

        rospy.Subscriber(self.ir1_topic, Image, self.ir1_callback, queue_size=1)
        rospy.Subscriber(self.ir2_topic, Image, self.ir2_callback, queue_size=1)

        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        rospy.Subscriber("/camera/gyro/sample", Imu, self.gyro_callback)
        rospy.Subscriber("/camera/accel/sample", Imu, self.accel_callback)

        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloud_callback)

    # IMAGE CALLBACKS
    def color_callback(self, msg):

        rospy.loginfo_throttle(
            1,
            f"[COLOR] {msg.width}x{msg.height} encoding={msg.encoding}"
        )

    def depth_callback(self, msg):

        rospy.loginfo_throttle(
            1,
            f"[DEPTH] {msg.width}x{msg.height} encoding={msg.encoding}"
        )

    def ir1_callback(self, msg):

        rospy.loginfo_throttle(
            1,
            f"[IR1] {msg.width}x{msg.height}"
        )

    def ir2_callback(self, msg):

        rospy.loginfo_throttle(
            1,
            f"[IR2] {msg.width}x{msg.height}"
        )

    # CAMERA INFO
    def camera_info_callback(self, msg):

        rospy.loginfo_once("Camera intrinsics received")
        rospy.loginfo_once(f"K matrix: {msg.K}")

    # IMU
    def gyro_callback(self, msg):

        rospy.loginfo_throttle(
            1,
            f"[GYRO] x={msg.angular_velocity.x:.3f} "
            f"y={msg.angular_velocity.y:.3f} "
            f"z={msg.angular_velocity.z:.3f}"
        )

    def accel_callback(self, msg):

        rospy.loginfo_throttle(
            1,
            f"[ACCEL] x={msg.linear_acceleration.x:.3f} "
            f"y={msg.linear_acceleration.y:.3f} "
            f"z={msg.linear_acceleration.z:.3f}"
        )

    # POINT CLOUD
    def pointcloud_callback(self, msg):

        rospy.loginfo_throttle(
            2,
            f"[POINTCLOUD] frame={msg.header.frame_id}"
        )

    # ===============================

    def spin(self):

        rospy.spin()

        # nếu node tắt thì kill driver
        self.driver_process.terminate()


if __name__ == "__main__":

    try:

        node = RealSenseSubscriberNode()
        node.spin()

    except rospy.ROSInterruptException:
        pass