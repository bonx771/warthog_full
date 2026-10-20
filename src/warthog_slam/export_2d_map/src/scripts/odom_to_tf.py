#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


def odom_callback(msg):
    parent_frame = rospy.get_param("~parent_frame", msg.header.frame_id)
    child_frame = rospy.get_param("~child_frame", msg.child_frame_id)

    if not parent_frame:
        parent_frame = msg.header.frame_id

    if not child_frame:
        child_frame = msg.child_frame_id

    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("odom_to_tf")

    odom_topic = rospy.get_param("~odom_topic", "/warthog_velocity_controller/odom")

    br = tf2_ros.TransformBroadcaster()
    received_odom = {"value": False}

    def wrapped_callback(msg):
        received_odom["value"] = True
        odom_callback(msg)

    def warn_if_no_odom(_event):
        if received_odom["value"]:
            return
        rospy.logwarn_throttle(
            5.0,
            "No odometry messages on %s, so TF odom -> base_link is not being published",
            odom_topic,
        )

    rospy.Subscriber(odom_topic, Odometry, wrapped_callback, queue_size=20)
    rospy.Timer(rospy.Duration(1.0), warn_if_no_odom)

    rospy.loginfo("odom_to_tf started")
    rospy.loginfo("Listening odom topic: %s", odom_topic)

    rospy.spin()
