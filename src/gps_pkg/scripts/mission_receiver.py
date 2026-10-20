#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
from datetime import datetime
from std_msgs.msg import String

# ===================== CONFIG =====================
SAVE_DIR = os.path.expanduser("~/warthog_ekf/src/gps_pkg/missions_txt")
# Nếu muốn lưu thẳng trong gps_pkg thì đổi thành:
# SAVE_DIR = os.path.expanduser("~/warthog_ekf/src/gps_pkg")

# ===================== SAVE TXT =====================
def save_mission_to_txt(data):
    try:
        os.makedirs(SAVE_DIR, exist_ok=True)

        timestamp = data.get("timestamp", int(datetime.now().timestamp() * 1000))
        dt_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        filename = f"mission_{dt_str}_{timestamp}.txt"
        filepath = os.path.join(SAVE_DIR, filename)

        with open(filepath, "w", encoding="utf-8") as f:
            for wp in data.get("waypoints", []):
                # seq = wp.get("seq")
                lat = wp.get("latitude")
                lon = wp.get("longitude")
                # f.write(f"{seq},{lat},{lon}\n")
                f.write(f"{lat} {lon}\n")

        rospy.loginfo("Mission saved to: %s", filepath)

    except Exception as e:
        rospy.logerr("Failed to save mission txt: %s", str(e))

# ===================== CALLBACK =====================
def mission_callback(msg):
    try:
        data = json.loads(msg.data)

        rospy.loginfo("===== RECEIVED MISSION =====")
        rospy.loginfo("Timestamp: %s", data.get("timestamp"))
        rospy.loginfo("Total: %s", data.get("total"))

        for wp in data.get("waypoints", []):
            rospy.loginfo(
                "WP %s -> lat: %.8f, lon: %.8f",
                wp.get("seq"),
                wp.get("latitude"),
                wp.get("longitude")
            )

        # LƯU FILE TXT
        save_mission_to_txt(data)

    except Exception as e:
        rospy.logerr("Mission parse error: %s", str(e))

# ===================== MAIN =====================
def main():
    rospy.init_node("mission_receiver")
    rospy.Subscriber("/mission_waypoints", String, mission_callback)
    rospy.loginfo("Mission receiver started.")
    rospy.spin()

if __name__ == "__main__":
    main()