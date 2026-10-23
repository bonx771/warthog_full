# warthog_simulator
Simulation packages for Warthog.

# Hướng dẫn A-LOAM + RTK GPS + map 2D + AMCL

Pipeline mới nằm trong package `aloam_gps_localization`. Pipeline này dùng:

- A-LOAM để quét point cloud map.
- `/gps/fix` và `/gps/heading` từ driver RTK STM32 trong workspace
  `/home/cuong/warthog`.
- File `*_geo.yaml` để lưu quan hệ giữa tọa độ GPS/UTM và tọa độ map A-LOAM.
- AMCL để định vị trên map 2D. GPS chỉ tự động gửi pose ban đầu cho AMCL;
  AMCL vẫn là node duy nhất phát `map -> odom`.

Các launch cũ không bị thay thế. Không sử dụng
`warthog_full/src/gps_pkg/scripts/gps_initialpose_manager.py`.

## 1. Build lần đầu

Chạy trong Terminal 1:

```bash
cd /home/cuong/warthog_full
catkin_make --pkg aloam_gps_localization
source devel/setup.bash
```

Mỗi terminal mới đều phải chạy:

```bash
cd /home/cuong/warthog_full
source devel/setup.bash
```

## 2. Các offset phải đo trước khi quét

Các offset dùng quy ước `base_link`:

- `+x`: phía trước robot.
- `+y`: bên trái robot.
- `gps_antenna_x/y`: tâm anten RTK so với tâm `base_link`.
- `lidar_x/y`: tâm LiDAR so với tâm `base_link`.
- `heading_offset_deg`: hiệu chỉnh hướng của baseline RTK.

Nếu anten nằm ngay trên tâm `base_link`, có thể dùng
`gps_antenna_x:=0.0 gps_antenna_y:=0.0`. Nếu không, phải thay bằng số đo thực
tế. Giá trị `heading_offset_deg` lúc quét và lúc navigation phải giống nhau.

## 3. Quét A-LOAM và ghi georeference

### Terminal 1: chạy LiDAR, A-LOAM và RTK

```bash
cd /home/cuong/warthog_full
source devel/setup.bash

roslaunch aloam_gps_localization aloam_rtk_mapping.launch \
  map_name:=aloam_map_rtk \
  gps_antenna_x:=0.0 \
  gps_antenna_y:=0.0 \
  lidar_x:=0.40 \
  lidar_y:=0.0 \
  heading_offset_deg:=0.0
```

Giữ Terminal 1 chạy. Chờ RTK Fixed rồi lái robot ít nhất 8–20 m. Nên chạy
cả đoạn thẳng và đoạn quay để việc ghép GPS với map ổn định.

### Terminal 2: kiểm tra số mẫu

```bash
cd /home/cuong/warthog_full
source devel/setup.bash

rosservice call /gps_map_georef_recorder/status
```

Gọi lại lệnh trên trong lúc quét. Chỉ lưu khi kết quả có tối thiểu:

```text
"samples": 20 hoặc lớn hơn
"trajectory_span_m": 8.0 hoặc lớn hơn
```

Nếu `samples` luôn bằng `0`, kiểm tra các topic:

```bash
rostopic hz /gps/fix
rostopic hz /gps/heading
rostopic hz /aft_mapped_to_init
```

Recorder mặc định chỉ nhận GPS có covariance đủ tốt để loại RTK Float và ưu
tiên RTK Fixed.

## 4. Lưu georeference và map 2D

Vẫn giữ Terminal 1 chạy. Tại Terminal 2, lưu georeference trước:

```bash
rosservice call /gps_map_georef_recorder/save
```

Phải nhận được:

```text
success: True
```

Sau đó, vẫn trong Terminal 2, ép `/laser_cloud_map` xuống map 2D:

```bash
roslaunch aloam_gps_localization save_aloam_2d_map_rtk.launch \
  map_name:=aloam_map_rtk
```

Chờ node báo lưu thành công và tự kết thúc. Kiểm tra ba file:

```bash
ls -lh "$(rospack find warthog_slam)/maps/aloam_map_rtk"*
```

Phải có đủ:

```text
aloam_map_rtk.pgm
aloam_map_rtk.yaml
aloam_map_rtk_geo.yaml
```

Chỉ sau khi có đủ ba file mới nhấn `Ctrl+C` ở Terminal 1 để dừng mapping.

## 5. Navigation từ vị trí bất kỳ trong map

Có thể tắt hệ thống, di chuyển robot đến vị trí khác trong vùng đã quét rồi
khởi động lại. Lúc khởi động navigation nên để robot đứng yên và chờ RTK Fixed.

### Terminal 1: chạy navigation

```bash
cd /home/cuong/warthog_full
source devel/setup.bash

roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch \
  map_name:=aloam_map_rtk \
  gps_antenna_x:=0.0 \
  gps_antenna_y:=0.0 \
  heading_offset_deg:=0.0
```

Launch này chạy LiDAR, local EKF, GPS RTK, map server, AMCL và `move_base`.
Khi map, GPS Fixed và heading đã sẵn sàng, nó chuyển GPS sang tọa độ map rồi
gửi `/initialpose` cho AMCL một lần.

### Terminal 2: kiểm tra pose GPS đã được gửi cho AMCL

```bash
cd /home/cuong/warthog_full
source devel/setup.bash

rosservice call /gps_amcl_initializer/status
```

Kết quả hợp lệ phải có:

```text
"map_ready": true
"gps_received": true
"heading_received": true
"published": true
```

Kiểm tra thêm:

```bash
rostopic echo -n 1 /amcl_pose
rosrun tf tf_echo map base_link
```

Quan sát trên RViz để chắc chắn robot nằm đúng vị trí và đúng hướng. Chỉ gửi
navigation goal sau khi trạng thái báo `published: true` và pose trên RViz đúng.

Nếu robot bị di chuyển bằng tay sau khi AMCL đã khởi tạo, để robot đứng yên,
chờ RTK Fixed rồi gọi:

```bash
rosservice call /gps_amcl_initializer/reinitialize
```

## 6. Khi GPS hoặc local EKF đã chạy từ launch khác

Nếu đã chạy driver GPS bằng `navsat_transform_gps.launch`, tránh mở driver lần
hai:

```bash
roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch \
  map_name:=aloam_map_rtk \
  run_gps:=false
```

Nếu bringup khác đã phát đúng transform `odom -> base_link`, tránh chạy local
EKF lần hai:

```bash
roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch \
  map_name:=aloam_map_rtk \
  run_local_odom:=false
```

Nếu cả GPS và local EKF đều đang chạy:

```bash
roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch \
  map_name:=aloam_map_rtk \
  run_gps:=false \
  run_local_odom:=false
```

Không chạy đồng thời global GPS EKF có `world_frame: map`, vì global EKF sẽ
tranh transform `map -> odom` với AMCL.
