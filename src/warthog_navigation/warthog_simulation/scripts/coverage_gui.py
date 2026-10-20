#!/usr/bin/env python3
import sys
import rospy
import subprocess
from datetime import datetime
from std_msgs.msg import String
from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton,
    QVBoxLayout, QLabel, QMessageBox
)

class CoverageGUI(QWidget):
    def __init__(self):
        super().__init__()

        # ---------------- ROS ----------------
        rospy.init_node("coverage_gui", anonymous=True)
        self.pub = rospy.Publisher("/coverage_cmd", String, queue_size=1)

        # ---------------- GUI ----------------
        self.setWindowTitle("Giao Diện Điều Khiển Quét Bản Đồ")
        self.setGeometry(300, 300, 320, 260)

        layout = QVBoxLayout()
        self.setLayout(layout)

        title = QLabel("<h3>Điều khiển quét bản đồ</h3>")
        title.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(title)

        # -------- Buttons --------
        self.btn_start = QPushButton("▶ BẮT ĐẦU QUÉT")
        self.btn_stop = QPushButton("⏹ DỪNG QUÉT")
        self.btn_save_map = QPushButton("💾 LƯU MAP")

        for btn in [self.btn_start, self.btn_stop, self.btn_save_map]:
            btn.setFixedHeight(42)
            layout.addWidget(btn)

        # -------- Signals --------
        self.btn_start.clicked.connect(self.start_coverage)
        self.btn_stop.clicked.connect(self.stop_coverage)
        self.btn_save_map.clicked.connect(self.save_map)

    # ======================================================
    def start_coverage(self):
        self.pub.publish("START")
        print("[GUI] START published")

    def stop_coverage(self):
        self.pub.publish("STOP")
        print("[GUI] STOP published")

    # ======================================================
    def save_map(self):
        """
        Lưu map vào thư mục cố định:
        warthog_full/src/warthog_navigation/warthog_slam/maps
        """
        if not rospy.core.is_initialized():
            QMessageBox.critical(self, "Lỗi", "ROS node chưa sẵn sàng!")
            return

        # 📂 THƯ MỤC LƯU MAP (CỐ ĐỊNH)
        base_dir = "/home/cuong/warthog_full/src/warthog_navigation/warthog_slam/maps"

        # 🔹 Tạo thư mục nếu chưa tồn tại
        import os
        os.makedirs(base_dir, exist_ok=True)

        # 🕒 Tên map theo thời gian
        map_name = "map_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        map_path = os.path.join(base_dir, map_name)

        cmd = [
            "rosrun", "map_server", "map_saver",
            "-f", map_path
        ]

        try:
            print(f"[GUI] Saving map to: {map_path}")
            subprocess.Popen(cmd)

            QMessageBox.information(
                self,
                "Lưu map thành công",
                f"Map đã được lưu tại:\n{map_path}.pgm\n{map_path}.yaml"
            )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Lỗi lưu map",
                f"Không thể lưu map:\n{e}"
            )
            print(f"[ERROR] Map saving failed: {e}")


# ======================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = CoverageGUI()
    gui.show()
    sys.exit(app.exec_())
