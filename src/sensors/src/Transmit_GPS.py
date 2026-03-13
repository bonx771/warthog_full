#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import UInt8MultiArray


PORT = "/dev/ttyTHS0"
BAUD = 115200


class GPSTransmitter:

    def __init__(self):

        rospy.init_node("gps_uart_transmitter")

        self.ser = serial.Serial(PORT, BAUD, timeout=1)

        rospy.Subscriber(
            "/gps/raw",
            UInt8MultiArray,
            self.raw_callback,
            queue_size=50
        )

        rospy.loginfo("GPS UART transmitter started")

    def raw_callback(self, msg):

        try:

            frame = bytes(msg.data)

            self.ser.write(frame)

            rospy.loginfo(
                "UART TX: " + " ".join(f"{b:02X}" for b in frame)
            )

        except Exception as e:

            rospy.logerr(f"UART send error: {e}")


if __name__ == "__main__":

    node = GPSTransmitter()

    rospy.spin()