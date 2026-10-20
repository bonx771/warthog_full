#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomToTF:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/warthog_velocity_controller/odom")
        self.parent_frame = rospy.get_param("~parent_frame", "")
        self.child_frame = rospy.get_param("~child_frame", "")

        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)

        rospy.loginfo("odom_to_tf started")
        rospy.loginfo("odom_topic: %s", self.odom_topic)

    def odom_callback(self, msg):
        parent = self.parent_frame if self.parent_frame else msg.header.frame_id
        child = self.child_frame if self.child_frame else msg.child_frame_id

        if not parent or not child:
            rospy.logwarn_throttle(2.0, "Missing parent or child frame in odometry")
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    OdomToTF()
    rospy.spin()
