#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CoverageCmdBridge:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel_1', Twist, queue_size=10)
        rospy.Subscriber('/coverage_cmd', String, self.cb)

    def cb(self, msg):
        twist = Twist()

        cmd = msg.data.lower()

        if cmd == "forward":
            twist.linear.x = 0.3
        elif cmd == "backward":
            twist.linear.x = -0.3
        elif cmd == "left":
            twist.angular.z = 0.5
        elif cmd == "right":
            twist.angular.z = -0.5
        elif cmd == "stop":
            pass
        else:
            rospy.logwarn(f"Unknown coverage_cmd: {msg.data}")
            return

        self.pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("coverage_cmd_bridge")
    CoverageCmdBridge()
    rospy.spin()
