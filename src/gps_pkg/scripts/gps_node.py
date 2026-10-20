#!/usr/bin/env python3

import rospy
import serial
from sensor_msgs.msg import NavSatFix, NavSatStatus

DEFAULT_GPS_PORT = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 9600
DEFAULT_GPS_TOPIC = "/gps/fix"

# Sai số giả định theo NMEA GGA fix quality.
# 4 = RTK fixed, 5 = RTK float, 2 = DGPS, 1 = single GPS.
UERE_BY_FIX_QUALITY = {
    4: 0.03,
    5: 0.20,
    2: 0.50,
    1: 3.00,
}
DEFAULT_UERE = 3.0

# Lưu DOP mới nhất
latest_hdop = None
latest_vdop = None
latest_fix_quality = 0


def parse_gngga(sentence):
    global latest_hdop, latest_fix_quality

    parts = sentence.split(',')

    if parts[0].endswith("GGA") and len(parts) > 9 and parts[2] and parts[4]:
        try:
            # Latitude
            lat_raw = parts[2]
            lat_dir = parts[3]
            lat_deg = float(lat_raw[:2])
            lat_min = float(lat_raw[2:])
            latitude = lat_deg + lat_min / 60.0
            if lat_dir == "S":
                latitude = -latitude

            # Longitude
            lon_raw = parts[4]
            lon_dir = parts[5]
            lon_deg = float(lon_raw[:3])
            lon_min = float(lon_raw[3:])
            longitude = lon_deg + lon_min / 60.0
            if lon_dir == "W":
                longitude = -longitude

            # Fix quality
            latest_fix_quality = int(parts[6]) if parts[6] else 0

            # HDOP
            latest_hdop = float(parts[8]) if parts[8] else None

            # Altitude
            altitude = float(parts[9]) if parts[9] else 0.0

            return latitude, longitude, altitude

        except (ValueError, IndexError):
            pass

    return None, None, None


def parse_gngsa(sentence):
    global latest_hdop, latest_vdop

    parts = sentence.split(',')

    if parts[0].endswith("GSA") and len(parts) >= 18:
        try:
            # Theo chuẩn NMEA:
            # parts[-3] = PDOP
            # parts[-2] = HDOP
            # parts[-1] = VDOP*checksum
            pdop = parts[-3] if len(parts) >= 3 else None
            hdop = parts[-2] if len(parts) >= 2 else None
            vdop_raw = parts[-1] if len(parts) >= 1 else None

            if hdop:
                latest_hdop = float(hdop)

            if vdop_raw:
                vdop = vdop_raw.split('*')[0]
                latest_vdop = float(vdop)

        except (ValueError, IndexError):
            pass


def uere_for_fix_quality(fix_quality):
    return UERE_BY_FIX_QUALITY.get(fix_quality, DEFAULT_UERE)


def ros_status_for_fix_quality(fix_quality):
    if fix_quality >= 4:
        return NavSatStatus.STATUS_GBAS_FIX
    if fix_quality > 0:
        return NavSatStatus.STATUS_FIX
    return NavSatStatus.STATUS_NO_FIX


def build_covariance(hdop, vdop, fix_quality):
    """
    Ước lượng covariance từ DOP
    sigma_h = HDOP * UERE
    sigma_v = VDOP * UERE
    covariance = sigma^2
    """
    # fallback nếu chưa có dữ liệu
    if hdop is None:
        hdop = 2.0
    if vdop is None:
        vdop = hdop * 1.5

    uere = uere_for_fix_quality(fix_quality)
    sigma_h = hdop * uere
    sigma_v = vdop * uere

    cov_xx = sigma_h ** 2
    cov_yy = sigma_h ** 2
    cov_zz = sigma_v ** 2

    return [
        cov_xx, 0.0,    0.0,
        0.0,    cov_yy, 0.0,
        0.0,    0.0,    cov_zz
    ]


def gps_publisher():
    rospy.init_node('gps_node', anonymous=True)
    gps_port = rospy.get_param("~gps_port", DEFAULT_GPS_PORT)
    baud_rate = int(rospy.get_param("~baud_rate", DEFAULT_BAUD_RATE))
    gps_topic = rospy.get_param("~gps_topic", DEFAULT_GPS_TOPIC)
    pub = rospy.Publisher(gps_topic, NavSatFix, queue_size=10)

    rate = rospy.Rate(30)  # 30Hz

    # 👉 lưu giá trị GPS gần nhất
    last_msg = None
    last_time = None

    try:
        ser = serial.Serial(gps_port, baud_rate, timeout=1)
        rospy.loginfo(f"Đang đọc GPS từ {gps_port}, publish {gps_topic}")
    except serial.SerialException as e:
        rospy.logerr(f"Không thể mở GPS: {e}")
        return

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()

            # ===== Nếu có dữ liệu mới =====
            if line:
                if line.startswith("$GNGSA") or line.startswith("$GPGSA") or line.startswith("$GLGSA"):
                    parse_gngsa(line)

                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    lat, lon, alt = parse_gngga(line)

                    if lat is not None and lon is not None:
                        msg = NavSatFix()
                        msg.header.frame_id = "gps_link"

                        msg.status.status = ros_status_for_fix_quality(latest_fix_quality)

                        msg.status.service = NavSatStatus.SERVICE_GPS

                        msg.latitude = lat
                        msg.longitude = lon
                        msg.altitude = alt

                        msg.position_covariance = build_covariance(
                            latest_hdop,
                            latest_vdop,
                            latest_fix_quality,
                        )
                        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                        # 👉 cập nhật dữ liệu mới nhất
                        last_msg = msg
                        last_time = rospy.Time.now()

            # ===== Publish 30Hz (dù có data mới hay không) =====
            if last_msg is not None:
                last_msg.header.stamp = rospy.Time.now()  # cập nhật timestamp mới
                pub.publish(last_msg)

                rospy.loginfo_throttle(
                    2,
                    f"Publishing @30Hz | lat={last_msg.latitude:.8f}, "
                    f"lon={last_msg.longitude:.8f}, fix_quality={latest_fix_quality}"
                )

            # ===== cảnh báo nếu GPS quá lâu không update =====
            if last_time is not None:
                if (rospy.Time.now() - last_time).to_sec() > 1.0:
                    rospy.logwarn_throttle(2, "GPS data too old!")

        except Exception as e:
            rospy.logwarn(f"Lỗi khi đọc GPS: {e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
