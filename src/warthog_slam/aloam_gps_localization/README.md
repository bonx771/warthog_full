# A-LOAM + RTK georeference + AMCL

Package này tạo thêm pipeline mới và không sửa các launch/source hiện có.
Driver RTK được dùng là `gps_UBX_node.py` trong workspace
`/home/cuong/warthog`, tương ứng với cấu hình STM32 trong
`warthog/src/gps_pkg/launch/navsat_transform_gps.launch`:

- `/gps/fix` (`sensor_msgs/NavSatFix`)
- `/gps/heading` (`sensor_msgs/Imu`, yaw ENU)
- STM32, 115200 baud

`warthog_full/src/gps_pkg/scripts/gps_initialpose_manager.py` không được gọi.

## 1. Build

```bash
cd /home/cuong/warthog_full
catkin_make --pkg aloam_gps_localization
source devel/setup.bash
```

## 2. Quét map và thu georeference

Đo chính xác offset theo frame `base_link` trước khi chạy:

- `gps_antenna_x/y`: tâm anten so với tâm xe, `+x` về trước, `+y` sang trái.
- `lidar_x/y`: tâm LiDAR so với tâm xe.
- `heading_offset_deg`: offset baseline/heading đã được driver RTK áp dụng.

Ví dụ:

```bash
roslaunch aloam_gps_localization aloam_rtk_mapping.launch \
  map_name:=aloam_map_rtk \
  gps_antenna_x:=0.0 gps_antenna_y:=0.0 \
  lidar_x:=0.40 lidar_y:=0.0 \
  heading_offset_deg:=0.0
```

Chờ RTK Fixed rồi lái ít nhất 8–20 m, nên có cả đoạn thẳng và đoạn quay. Node
chỉ nhận GPS có variance không quá `0.04 m^2`; với driver hiện tại ngưỡng này
loại RTK Float (floor `1.0 m^2`) và nhận RTK Fixed (floor `0.0025 m^2`).

Theo dõi và ép lưu khi kết thúc:

```bash
rosservice call /gps_map_georef_recorder/status
rosservice call /gps_map_georef_recorder/save
```

Kết quả được lưu tại:

```text
$(find warthog_slam)/maps/aloam_map_rtk_geo.yaml
```

File này chứa phép biến đổi UTM sang frame map A-LOAM, sai số fit, offset cảm
biến và offset heading. Không sửa hoặc nhét GPS vào YAML occupancy map.

## 3. Ép point cloud xuống map 2D

Khi đã quét xong và `/laser_cloud_map` đã đầy đủ:

```bash
roslaunch aloam_gps_localization save_aloam_2d_map_rtk.launch \
  map_name:=aloam_map_rtk
```

Ba file phải đi cùng nhau:

```text
aloam_map_rtk.pgm
aloam_map_rtk.yaml
aloam_map_rtk_geo.yaml
```

## 4. Navigation, tự khởi tạo AMCL từ RTK

```bash
roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch \
  map_name:=aloam_map_rtk
```

Launch mới thực hiện:

1. Chạy local EKF để phát `odom -> base_link`.
2. Chạy đúng driver RTK từ workspace `warthog`.
3. Chạy launch Helios32 + map_server + AMCL + move_base cũ.
4. Chờ map, RTK Fixed-quality và heading hợp lệ.
5. Chuyển RTK sang pose trong `map` và gửi `/initialpose` đúng một lần.
6. Sau đó AMCL tiếp tục định vị bằng `/scan` và là publisher duy nhất của
   `map -> odom`.

Nếu local EKF/IMU đã được chạy bởi bringup khác:

```bash
roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch \
  map_name:=aloam_map_rtk run_local_odom:=false
```

Nếu GPS driver đã chạy riêng:

```bash
roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch \
  map_name:=aloam_map_rtk run_gps:=false publish_gps_tf:=false
```

Kiểm tra trạng thái hoặc yêu cầu khởi tạo lại sau khi robot bị di chuyển bằng
tay:

```bash
rosservice call /gps_amcl_initializer/status
rosservice call /gps_amcl_initializer/reinitialize
```

Không chạy đồng thời `ekf_se_map`/global GPS EKF có `world_frame: map`, vì nó
sẽ tranh transform `map -> odom` với AMCL.

## Điều kiện an toàn trước khi gửi goal

- `/gps_amcl_initializer/status` báo `published: true`.
- Pose `/amcl_pose` nằm đúng vị trí và đúng hướng trên RViz.
- `rosrun tf tf_echo map base_link` ổn định.
- Không có hai publisher cho `map -> odom`.
- Chỉ gửi navigation goal sau các kiểm tra trên.
