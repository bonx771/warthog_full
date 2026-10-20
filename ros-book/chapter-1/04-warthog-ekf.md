# Chapter 4

## Gói warthog_ekf — Nguyên lý, Cấu trúc và Hướng dẫn sử dụng

---

### 1. Tại sao cần EKF?

Mỗi cảm biến trên xe đều có sai số và giới hạn riêng:

| Cảm biến | Cung cấp | Hạn chế |
|----------|----------|---------|
| **Encoder (odometry)** | Vận tốc vx, vy, vyaw | Drift tích lũy theo thời gian do trượt bánh |
| **IMU** | Gia tốc, vận tốc góc, hướng | Drift gyro, noise gia tốc kế |
| **GPS F9P** | Vị trí tuyệt đối lat/lon | Nhảy nhiễu từng bước (~0.5–3 m standalone), chậm (10 Hz) |

**Extended Kalman Filter (EKF)** kết hợp tất cả nguồn đo lường, cân bằng độ tin cậy của từng nguồn qua ma trận covariance, tạo ra ước lượng vị trí tốt hơn bất kỳ cảm biến đơn lẻ nào.

---

### 2. Nguyên lý EKF — Hai bước lặp

```
┌─────────────────────────────────────────────────────┐
│  PREDICT (dự đoán)                                  │
│  Dùng mô hình chuyển động (encoder + IMU):          │
│    x̂ₖ = f(x̂ₖ₋₁, uₖ)                               │
│    Pₖ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ                           │
│  → Ước lượng trạng thái + covariance tăng lên       │
├─────────────────────────────────────────────────────┤
│  UPDATE (hiệu chỉnh)                                │
│  Khi nhận được đo lường mới (GPS, IMU, odom):       │
│    Kₖ = Pₖ Hₖᵀ (Hₖ Pₖ Hₖᵀ + Rₖ)⁻¹   (Kalman gain) │
│    x̂ₖ = x̂ₖ + Kₖ(zₖ − Hₖ x̂ₖ)                      │
│    Pₖ = (I − Kₖ Hₖ) Pₖ                             │
│  → Covariance giảm, ước lượng tiệm cận thực tế     │
└─────────────────────────────────────────────────────┘
```

**Q (process noise)**: Tin vào mô hình ít → Q lớn → theo cảm biến nhiều hơn.  
**R (measurement noise)**: Trong `robot_localization`, R lấy từ trường `covariance` trong message của cảm biến.

---

### 3. Kiến trúc Dual EKF + navsat_transform

Gói dùng **hai EKF song song** — đây là pattern chuẩn của `robot_localization`:

```
imu/data ─────────────┬──────────────────────────────────────────────────────────────┐
                       │                                                              │
/warthog_velocity_     │                                                              │
clearpath/odom ────────┤                                                              │
                       ▼                                                              ▼
              ┌─────────────────┐     odometry/filtered    ┌──────────────────────────────┐
              │   ekf_odom      │ ─────────────────────── ▶│   navsat_transform            │
              │  (world = odom) │                           │                              │
              └─────────────────┘                           │  imu/data ──────────────────▶│
                      │                                     │  gps/fix ───────────────────▶│
                      │ TF: odom → base_link                └──────────────────────────────┘
                      │                                              │
                      │                                    odometry/gps (GPS trong local frame)
                      │                                              │
                      ▼                                              ▼
              ┌──────────────────────────────────────────────────────────┐
              │   ekf_map   (world = map)                                │
              │   Input: odom + IMU + odometry/gps                       │
              └──────────────────────────────────────────────────────────┘
                      │
                      │ odometry/filtered_map
                      │ TF: map → base_link
                      ▼
               Vị trí tuyệt đối trong map frame (dùng cho navigation)
```

**Tại sao tách làm hai EKF?**

- `ekf_odom` chạy ổn định liên tục, **không bị gián đoạn** khi GPS mất tín hiệu. Nó tạo ra `odometry/filtered` dùng làm nguồn vận tốc liên tục cho navigation stack.
- `ekf_map` cung cấp **vị trí tuyệt đối** trong map frame. Khi GPS bị khuất (tòa nhà, cây cối), `ekf_map` vẫn dự đoán được nhờ encoder + IMU, chỉ covariance tăng dần.
- `navsat_transform` làm cầu nối: chuyển GPS lat/lon → XY trong local frame, đồng bộ với datum.

---

### 4. Cấu trúc gói

```
src/warthog_ekf/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── ekf_odom.yaml      ← EKF odom-frame: encoder + IMU
│   ├── ekf_map.yaml       ← EKF map-frame:  encoder + IMU + GPS
│   └── navsat.yaml        ← navsat_transform: GPS fix → odom local
├── launch/
│   ├── ekf_sim.launch     ← nhánh mô phỏng Gazebo
│   └── ekf_outdoor.launch ← nhánh outdoor thật
└── scripts/
    └── sim_gps_node.py    ← GPS giả từ Gazebo ground truth
```

---

### 5. Giải thích từng file cấu hình

#### 5.1 `config/ekf_odom.yaml`

```yaml
world_frame: odom   # EKF này sống trong odom frame
```

Config array của mỗi cảm biến là vector **15 phần tử boolean**, ánh xạ vào state vector:

```
[x, y, z,  roll, pitch, yaw,  vx, vy, vz,  vroll, vpitch, vyaw,  ax, ay, az]
 0  1  2    3     4     5     6   7   8      9      10      11    12  13  14
```

**Encoder odometry** (`/warthog_velocity_clearpath/odom`):
```yaml
odom0_config: [false, false, false,   # x, y, z: không dùng vị trí tích phân
               false, false, false,   # roll, pitch, yaw: không dùng
               true,  true,  false,   # vx, vy: dùng vận tốc thẳng
               false, false, true,    # vyaw: dùng vận tốc quay
               false, false, false]   # ax, ay, az: không có
```
> Lý do không dùng vị trí: odometry tích phân sẽ drift theo thời gian. Chỉ lấy vận tốc tức thời — thông tin đáng tin hơn.

**IMU** (`imu/data`), chế độ `two_d_mode: true`:
```yaml
imu0_config: [false, false, false,   # vị trí: IMU không đo
              false, false, true,    # yaw: dùng (roll, pitch bỏ vì 2D)
              false, false, false,   # vận tốc thẳng: không dùng từ IMU
              false, false, true,    # vyaw: dùng angular velocity z
              true,  true,  false]   # ax, ay: dùng; az bỏ (dominated bởi gravity)
imu0_remove_gravitational_acceleration: true  # trừ 9.81 m/s² trước khi fuse
```

#### 5.2 `config/ekf_map.yaml`

Giống `ekf_odom.yaml` nhưng thêm:

```yaml
world_frame: map   # EKF này sống trong map frame

odom1: odometry/gps              # output của navsat_transform
odom1_config: [true,  true,  false,   # x, y: dùng vị trí tuyệt đối
               false, false, false,
               false, false, false,
               false, false, false,
               false, false, false]
```

Ma trận `process_noise_covariance` của `ekf_map` có **Q(x,x) = Q(y,y) = 1.0** (lớn hơn `ekf_odom`), cho phép GPS kéo vị trí về đúng toạ độ tuyệt đối mà không bị cản trở.

#### 5.3 `config/navsat.yaml`

```yaml
delay: 3.0         # Chờ 3 giây để GPS đạt fix ổn định trước khi tính datum
zero_altitude: true        # Bỏ qua độ cao (xe mặt đất)
broadcast_utm_transform: true    # Publish TF utm→map để xem trong RViz
publish_filtered_gps: true       # Publish gps/filtered để so sánh với GPS raw
magnetic_declination_radians: 0.0  # Điều chỉnh theo vị trí thực tế
```

**Cách navsat_transform hoạt động:**
1. Nhận GPS fix đầu tiên → lưu làm **datum** (gốc tọa độ local)
2. Mỗi GPS fix tiếp theo → chuyển lat/lon → XY offset từ datum (qua phép chiếu UTM)
3. Dùng yaw từ `imu/data` để align heading giữa GPS frame và robot frame
4. Publish `odometry/gps`: vị trí XY trong map frame, covariance từ GPS message

---

### 6. Cảm biến mô phỏng — đã có sẵn trong URDF

Clearpath đã tích hợp sẵn tất cả sensor plugin trong `warthog_description/urdf/warthog.gazebo`.  
**Không cần thêm bất kỳ node GPS giả nào.**

| Cảm biến | Plugin Gazebo | Topic publish | Tần số |
|----------|--------------|---------------|--------|
| **IMU** | `libhector_gazebo_ros_imu.so` | `imu/data` | 50 Hz |
| **GPS** | `libhector_gazebo_ros_gps.so` | `/navsat/fix` | 20 Hz |
| **Encoder** | `diff_drive_controller` (qua `gazebo_ros_control`) | `/warthog_velocity_controller/odom` | 50 Hz |

Plugin IMU đã mô phỏng cả **drift** và **Gaussian noise** thực tế:
```xml
<accelDrift>0.005 0.005 0.005</accelDrift>
<accelGaussianNoise>0.005 0.005 0.005</accelGaussianNoise>
<rateDrift>0.005 0.005 0.005</rateDrift>
<rateGaussianNoise>0.005 0.005 0.005</rateGaussianNoise>
```

Plugin GPS dùng datum từ biến môi trường (đặt trước khi roslaunch):
```bash
export GAZEBO_WORLD_LAT=21.0285   # Hà Nội — HVKTQS
export GAZEBO_WORLD_LON=105.8542
# Mặc định nếu không set: lat=49.9, lon=8.9 (Đức)
```

#### Vấn đề tên topic khác nhau giữa sim và outdoor

| | Sim (Gazebo plugin) | Outdoor (thật) |
|--|--|--|
| Encoder | `/warthog_velocity_controller/odom` | `/warthog_velocity_clearpath/odom` |
| GPS | `/navsat/fix` | `gps/fix` |
| IMU | `imu/data` | `imu/data` ✓ |

File config (`ekf_odom.yaml`, `ekf_map.yaml`) dùng tên topic outdoor làm mặc định.  
`ekf_sim.launch` điều chỉnh bằng hai cách:

```xml
<!-- Override param odom0 cho EKF nodes -->
<param name="odom0" value="/warthog_velocity_controller/odom"/>

<!-- Remap GPS topic cho navsat_transform -->
<remap from="gps/fix" to="/navsat/fix"/>
```

---

### 7. Điểm quan trọng: tắt EKF mặc định của Warthog

`warthog_control/launch/control.launch` đã có sẵn một `ekf_localization_node` không cấu hình gì. Nếu để nó chạy song song sẽ gây **xung đột TF** (`odom → base_link` bị hai node publish).

`ekf_sim.launch` tắt nó bằng:
```xml
<include file="$(find warthog_control)/launch/control.launch">
  <arg name="enable_ekf" value="false"/>
</include>
```

Với `ekf_outdoor.launch`, các node điều khiển Warthog thật phải được khởi động với `ENABLE_EKF=false`:
```bash
export ENABLE_EKF=false
roslaunch warthog_control control.launch
```

---

### 8. Build và chạy

#### Build
```bash
cd ~/cnn25ws_dev
catkin_make --pkg warthog_ekf
# hoặc build toàn bộ
catkin_make
source devel/setup.bash
```

#### Chạy nhánh mô phỏng
```bash
# Mặc định: Gazebo GUI + noise GPS 0.5 m
roslaunch warthog_ekf ekf_sim.launch

# Headless (server không có màn hình)
roslaunch warthog_ekf ekf_sim.launch gui:=false headless:=true

# GPS hoàn hảo (test không có noise)
roslaunch warthog_ekf ekf_sim.launch noise_std:=0.0

# GPS chế độ RTK fix (noise nhỏ)
roslaunch warthog_ekf ekf_sim.launch noise_std:=0.02

# World khác
roslaunch warthog_ekf ekf_sim.launch world:=$(find warthog_gazebo)/worlds/warthog_playpen.world
```

#### Chạy nhánh outdoor thật
```bash
# Đảm bảo các driver đã chạy:
#   roslaunch ublox_gps ublox_device.launch  → gps/fix
#   roslaunch warthog_bringup bringup.launch ENABLE_EKF=false

roslaunch warthog_ekf ekf_outdoor.launch

# Với độ từ thiên Hà Nội
roslaunch warthog_ekf ekf_outdoor.launch mag_decl:=-0.0175

# Set datum thủ công (điểm khảo sát đã biết tọa độ)
roslaunch warthog_ekf ekf_outdoor.launch wait_for_datum:=true
```

---

### 9. Kiểm tra sau khi chạy

#### Kiểm tra topics đang publish
```bash
rostopic list | grep -E "filtered|gps|odom"
# Phải thấy:
#   /odometry/filtered        ← ekf_odom output
#   /odometry/filtered_map    ← ekf_map output
#   /odometry/gps             ← navsat_transform output
#   /gps/fix                  ← GPS input
#   /gps/filtered             ← GPS sau filter (so sánh)
```

#### Kiểm tra TF tree
```bash
rosrun tf view_frames    # tạo file PDF cây TF
# Phải có chuỗi: map → odom → base_link
```

#### Kiểm tra tần số
```bash
rostopic hz /odometry/filtered       # phải ≈ 30 Hz
rostopic hz /odometry/filtered_map   # phải ≈ 30 Hz
rostopic hz /gps/fix                 # phải ≈ 10 Hz (GPS F9P)
```

#### Kiểm tra covariance
```bash
rostopic echo /odometry/filtered | grep -A 36 covariance
# Covariance x,y (phần tử [0] và [7]) phải nhỏ và ổn định khi xe đứng yên
```

#### Xem trong RViz

RViz tự động mở cùng lúc với Gazebo/outdoor — không cần lệnh thêm:

```bash
# Sim: RViz mở cùng Gazebo
roslaunch warthog_ekf ekf_sim.launch

# Outdoor: RViz mở cùng EKF nodes
roslaunch warthog_ekf ekf_outdoor.launch
```

Config file: `warthog_ekf/rviz/ekf.rviz`. Xem chi tiết ở **Mục 16**.

---

### 10. Hiệu chỉnh thông số

#### 10.1 Khi EKF quá tin cảm biến → output giật cục
Tăng giá trị tương ứng trong `process_noise_covariance` (Q):
```yaml
# Ví dụ: EKF quá nhạy với noise GPS → tăng Q(x,x), Q(y,y)
process_noise_covariance: [5.0, 0, ..., 0, 5.0, ...]
#                           ↑ x                ↑ y
```

#### 10.2 Khi EKF phản ứng quá chậm với GPS → vị trí lag
Giảm Q(x,x), Q(y,y) trong `ekf_map.yaml` — hoặc kiểm tra lại covariance của GPS message.

#### 10.3 Kiểm tra covariance GPS có hợp lý không
```bash
rostopic echo /gps/fix | grep -A 9 position_covariance
# F9P standalone: ~1.0–9.0 (m²)
# F9P RTK fix:    ~0.0004–0.001 (m²)
# Nếu = 0.0: driver chưa set, navsat_transform sẽ crash hoặc hoạt động sai
```

#### 10.4 Heading offset — lỗi phổ biến nhất
Nếu robot đi thẳng nhưng quỹ đạo trong map bị lệch góc:
```yaml
# navsat.yaml
yaw_offset: 1.5708   # nếu IMU báo 0 khi nhìn về North (thay vì East)
magnetic_declination_radians: -0.0175   # Hà Nội
```

#### 10.5 Delay khởi động
Nếu `ekf_map` không nhận `odometry/gps` ngay từ đầu → tăng `delay` trong `navsat.yaml`:
```yaml
delay: 5.0   # chờ 5 giây thay vì 3 giây
```

---

### 11. Topics tóm tắt

| Topic | Type | Node publish | Ý nghĩa |
|-------|------|-------------|---------|
| `imu/data` | `sensor_msgs/Imu` | IMU driver | Input: hướng + angular vel + accel |
| `/warthog_velocity_clearpath/odom` | `nav_msgs/Odometry` | warthog_control | Input: vận tốc encoder |
| `gps/fix` | `sensor_msgs/NavSatFix` | GPS F9P driver | Input: vị trí GPS tuyệt đối |
| `odometry/filtered` | `nav_msgs/Odometry` | ekf_odom | Output odom-frame (dùng cho nav stack) |
| `odometry/gps` | `nav_msgs/Odometry` | navsat_transform | GPS trong local frame |
| `odometry/filtered_map` | `nav_msgs/Odometry` | ekf_map | Output map-frame (vị trí tuyệt đối) |
| `gps/filtered` | `sensor_msgs/NavSatFix` | navsat_transform | GPS sau EKF (debug) |

---

### 12. Đánh giá chất lượng EKF ở hai tầng

#### 12.1 Ý nghĩa của hai tầng output

```
Tầng 1 — ekf_odom   →  odometry/filtered
  Trả lời: "Robot đang di chuyển NHƯ THẾ NÀO?"
  - Liên tục, ổn định, không bị gián đoạn dù GPS mất
  - Covariance tăng dần khi di chuyển lâu (drift tích lũy)
  - Dùng cho: nav stack velocity, short-term motion

Tầng 2 — ekf_map    →  odometry/filtered_map
  Trả lời: "Robot đang ở ĐÂU trên bản đồ thế giới?"
  - Bị ảnh hưởng bởi GPS nhảy, nhưng ổn định hơn GPS raw
  - Covariance giảm khi có GPS, tăng khi mất GPS
  - Dùng cho: global localization, waypoint navigation
```

#### 12.2 Quan sát covariance — chỉ số sức khỏe EKF

Covariance matrix trong `nav_msgs/Odometry` là ma trận 6×6 (row-major):

```
Index:  [ 0  1  2  3  4  5 ]    → hàng x
        [ 6  7  8  9 10 11 ]    → hàng y
        [12 13 14 15 16 17 ]    → hàng z
        [18 19 20 21 22 23 ]    → hàng roll
        [24 25 26 27 28 29 ]    → hàng pitch
        [30 31 32 33 34 35 ]    → hàng yaw

Cov(x,x)   = index  0   ← variance vị trí x (m²)
Cov(y,y)   = index  7   ← variance vị trí y (m²)
Cov(yaw,yaw)= index 35  ← variance góc quay (rad²)
```

**Giá trị tham chiếu:**

| Chỉ số | Tốt | Chấp nhận | Cần xem lại |
|--------|-----|-----------|-------------|
| Cov(x,x) tầng 1 khi đứng yên | < 0.01 | 0.01–0.1 | > 0.1 |
| Cov(x,x) tầng 2 khi có GPS F9P | < 0.5 | 0.5–2.0 | > 2.0 |
| Cov(yaw,yaw) | < 0.001 | 0.001–0.01 | > 0.01 |

```bash
# Xem covariance tầng 1 liên tục (cập nhật 1 Hz)
rostopic echo -n 1 /odometry/filtered | python3 -c "
import sys, yaml
d = yaml.safe_load(sys.stdin.read())
c = d['pose']['covariance']
print(f'Cov x={c[0]:.6f}  y={c[7]:.6f}  yaw={c[35]:.6f}')
"

# So sánh covariance tầng 1 vs tầng 2
watch -n 1 "rostopic echo -n 1 /odometry/filtered_map 2>/dev/null | \
  grep -A 37 'covariance' | head -3"
```

#### 12.3 Dùng `ekf_monitor.py` — monitor real-time

Script `scripts/ekf_monitor.py` trong gói tự động in báo cáo hai tầng mỗi giây:

```bash
# Terminal riêng, chạy song song với ekf_sim/ekf_outdoor
rosrun warthog_ekf ekf_monitor.py

# Tùy chỉnh ngưỡng
rosrun warthog_ekf ekf_monitor.py _v_threshold:=0.02 _drift_warn_m:=0.05 _report_hz:=2.0
```

Output mẫu khi bình thường:
```
────────────────────────────────────────────────────────────
[Tầng 1 — ekf_odom]  (world = odom, không có GPS)
  Vị trí :  x=  1.234 m   y= -0.056 m
  |vel|  : 0.0012 m/s  (ĐỨNG YÊN)
  Cov x  :  0.0023   Cov y:  0.0021   Cov yaw:  0.0004
  Drift khi đứng yên: 0.0031 m

[Tầng 2 — ekf_map]   (world = map, có GPS)
  Vị trí :  x=  1.198 m   y= -0.071 m
  Cov x  :  0.2450   Cov y:  0.2312
  Drift khi đứng yên: 0.0089 m

  Lệch tầng 1 vs 2: 0.043 m  (map−odom: Δx=−0.036 Δy=−0.015)

[GPS raw]
  lat=21.0285123  lon=105.8542456  cov=  0.2500 m²
────────────────────────────────────────────────────────────
```

Output mẫu khi có vấn đề:
```
  Drift khi đứng yên: 0.0821 m  !! DRIFT !!    ← màu đỏ
  Cov x  :  3.4521                              ← màu đỏ, covariance quá lớn
  Lệch tầng 1 vs 2: 8.341 m                    ← màu đỏ, GPS và odom không khớp
```

#### 12.4 Xem trong RViz — so sánh trực quan hai tầng

RViz tự động mở khi chạy launch file (xem **Mục 16**). Config được load từ:

```bash
warthog_ekf/rviz/ekf.rviz
```

**Điều cần quan sát:**
- Quỹ đạo **xanh lá** (`odometry/filtered`) phải mượt, không giật — tầng 1 không có GPS
- Quỹ đạo **xanh dương** (`odometry/filtered_map`) bám sát GPS — tầng 2 có GPS
- Đường **vàng** (`odometry/gps`) là GPS thô đã convert — thường nhảy hơn hai đường kia
- Khi GPS mất: đường xanh dương tiếp tục di chuyển theo encoder, dần tách xa đường vàng

---

### 13. Phát hiện và chẩn đoán hiện tượng trôi (drift) khi đứng yên

#### 13.1 Drift là gì và tại sao xảy ra?

Khi robot **đứng hoàn toàn yên** mà vị trí `odometry/filtered` vẫn thay đổi — đó là drift.

```
Nguyên nhân theo thứ tự phổ biến:

1. IMU gyro bias  →  vyaw ≠ 0 khi đứng yên
                  →  EKF tích phân vyaw  →  yaw drift
                  →  yaw sai → vx/vy chiếu sai hướng  →  position drift

2. IMU accel bias →  ax ≠ 0 khi đứng yên (gravity không bù đủ)
                  →  EKF tích phân ax  →  vx drift  →  position drift

3. Q quá lớn     →  EKF trust mô hình động lực quá nhiều
                  →  noise nhỏ trong cảm biến bị khuếch đại

4. Encoder noise  →  rung động của xe gây encoder tick giả
                  →  vx ≠ 0 nhỏ khi đứng yên

5. GPS jump       →  GPS nhảy vị trí → navsat_transform bơm sai vào ekf_map
                  →  chỉ ảnh hưởng tầng 2
```

#### 13.2 Quy trình phát hiện từng bước

**Bước 1 — Xác nhận xe đứng yên thật sự**
```bash
# Xem vận tốc encoder thô (chưa qua EKF)
rostopic echo /warthog_velocity_controller/odom | grep -A 3 "twist:"
# linear.x, linear.y phải < 0.005 m/s khi đứng yên
```

**Bước 2 — Tách drift IMU gyro (yaw drift thuần túy)**
```bash
# Xem angular velocity z của IMU thô — phải gần 0 khi đứng yên
rostopic echo /imu/data | grep -A 3 "angular_velocity:"
# |angular_velocity.z| < 0.01 rad/s là chấp nhận được
# Nếu > 0.05 rad/s: gyro bias cao, cần IMU calibration
```

**Bước 3 — Tách drift IMU accel**
```bash
# linear_acceleration sau khi trừ gravity phải ≈ 0
rostopic echo /imu/data | grep -A 3 "linear_acceleration:"
# Với imu0_remove_gravitational_acceleration: true, az ≈ 0
# linear_acceleration.x, .y phải < 0.1 m/s² khi đứng yên
```

**Bước 4 — Xem yaw trong odometry có thay đổi không**
```bash
rostopic echo /odometry/filtered | python3 -c "
import sys, yaml, math
for line in sys.stdin:
    try:
        d = yaml.safe_load(line)
        q = d['pose']['pose']['orientation']
        yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                         1-2*(q['y']**2+q['z']**2))
        print(f'yaw = {math.degrees(yaw):.4f} deg')
    except: pass
" 2>/dev/null | head -20
```

**Bước 5 — Phân biệt drift tầng 1 vs tầng 2**

```bash
# Chạy monitor 60 giây khi đứng yên, ghi kết quả
rosrun warthog_ekf ekf_monitor.py _report_hz:=2.0 2>&1 | tee /tmp/drift_log.txt

# Phân tích sau
grep "Drift" /tmp/drift_log.txt
```

| Kịch bản | Tầng 1 drift | Tầng 2 drift | Nguyên nhân |
|----------|-------------|-------------|-------------|
| Cả hai drift | Có | Có | IMU gyro/accel bias hoặc Q quá lớn |
| Chỉ tầng 2 | Không | Có | GPS nhảy đang kéo ekf_map |
| Tầng 1 nhỏ, tầng 2 lớn | Nhỏ | Lớn | navsat heading offset sai |
| Cả hai ổn định | Không | Không | EKF hoạt động đúng |

#### 13.3 Xử lý từng loại drift

**Drift do IMU bias (tầng 1):**
```yaml
# ekf_odom.yaml — giảm tin tưởng vào ax, ay của IMU
imu0_config: [false, false, false,
              false, false, true,
              false, false, false,
              false, false, true,
              false, false, false]   # ← tắt ax, ay nếu bias quá lớn
```

**Drift do Q quá lớn:**
```yaml
# Giảm các phần tử Q tương ứng với state đang drift
# Ví dụ yaw drift: giảm Q(yaw,yaw) = phần tử [5,5] → index 80
process_noise_covariance: [..., 0.001, ...]   # Q(yaw) từ 0.01 → 0.001
#                                ↑ index 80
```

**Drift tầng 2 do GPS nhảy:**
```yaml
# navsat.yaml — tăng delay để GPS ổn định hơn
delay: 5.0

# ekf_map.yaml — tăng R GPS (tin GPS ít hơn)
# Cách làm: tăng covariance trong message GPS
# Hoặc: giảm Q(x,x), Q(y,y) → EKF bám encoder chặt hơn, GPS kéo ít hơn
process_noise_covariance: [0.1, 0, ..., 0, 0.1, ...]   # giảm từ 1.0 → 0.1
```

---

### 14. Hiệu chỉnh tham số EKF — ảnh hưởng của từng tham số

#### 14.1 Bản đồ tham số → hành vi

```
                    ┌─────────────────────────────────────────────┐
                    │           PROCESS NOISE Q                   │
                    │                                             │
         Q nhỏ ────▶│  EKF bám mô hình chặt                      │◀──── Q lớn
         (tin mô    │  Output mượt hơn                           │      (tin cảm biến
          hình)     │  Chậm theo cảm biến                         │       nhiều hơn)
                    │  Nguy cơ: bỏ qua thay đổi thật              │      Output giật
                    └─────────────────────────────────────────────┘

                    ┌─────────────────────────────────────────────┐
                    │        MEASUREMENT NOISE R                  │
                    │  (lấy từ covariance trong message cảm biến) │
         R nhỏ ────▶│  EKF tin cảm biến nhiều                     │◀──── R lớn
         (cảm biến  │  Output bám sát measurement                 │      (cảm biến
          chính xác)│  Nguy cơ: nhạy với noise                    │       không tin cậy)
                    │                                             │      EKF ít phản ứng
                    └─────────────────────────────────────────────┘
```

#### 14.2 Từng tham số quan trọng trong `ekf_odom.yaml` / `ekf_map.yaml`

**`frequency`** — Tần số chạy vòng lặp EKF
```yaml
frequency: 30   # Hz
# Tăng → latency thấp hơn, tốn CPU hơn
# Giảm → latency cao, có thể bỏ qua measurement
# Khuyến nghị: ≥ 2× tần số cảm biến chậm nhất (GPS 10 Hz → dùng 30 Hz)
```

**`sensor_timeout`** — Thời gian chờ tối đa một cảm biến
```yaml
sensor_timeout: 0.1   # giây
# Nếu cảm biến im lặng > 0.1s: EKF tiếp tục predict (không dừng)
# Tăng nếu cảm biến chậm và hay bị timeout giả
```

**`two_d_mode`** — Chỉ ước lượng x, y, yaw
```yaml
two_d_mode: true
# true:  bỏ z, roll, pitch → ổn định hơn cho xe mặt đất
# false: cần nếu xe đi trên địa hình dốc hoặc có IMU 3D tốt
```

**`odom0_differential`** — Dùng delta thay vì giá trị tuyệt đối
```yaml
odom0_differential: false
# false (mặc định): dùng giá trị vận tốc trực tiếp
# true:  lấy hiệu giữa hai pose liên tiếp → dùng khi odom publish pose tuyệt đối
#        và bị drift (không phải vận tốc)
```

**`imu0_remove_gravitational_acceleration`**
```yaml
imu0_remove_gravitational_acceleration: true
# Bắt buộc true nếu IMU báo linear_acceleration bao gồm gravity
# Nếu IMU đã tự trừ gravity: đặt false để tránh trừ 2 lần
# Cách kiểm tra: đứng yên, đọc imu/data.linear_acceleration.z
#   ≈ +9.81 → IMU chưa trừ → đặt true
#   ≈  0.00 → IMU đã trừ   → đặt false
```

#### 14.3 Ma trận `process_noise_covariance` — tuning thực tế

Ma trận 15×15, nhưng chỉ cần chỉnh đường chéo chính (15 phần tử):

```
Index đường chéo:
  Q[0]  = Q(x,x)     ← vị trí x
  Q[7]  = Q(y,y)     ← vị trí y
  Q[35] = Q(yaw,yaw) ← góc quay   [index trong mảng 1D = hàng×15+cột]

Thực tế mảng 1D 225 phần tử:
  Q(x,x)   → phần tử 0   (hàng 0, cột 0)
  Q(y,y)   → phần tử 16  (hàng 1, cột 1)  [1×15+1=16]
  Q(z,z)   → phần tử 32  (hàng 2, cột 2)
  Q(r,r)   → phần tử 48  (hàng 3, cột 3)
  Q(p,p)   → phần tử 64  (hàng 4, cột 4)
  Q(yaw)   → phần tử 80  (hàng 5, cột 5)
  Q(vx,vx) → phần tử 96  (hàng 6, cột 6)
  Q(vy,vy) → phần tử 112 (hàng 7, cột 7)
  Q(vyaw)  → phần tử 176 (hàng 11,cột 11)
  Q(ax,ax) → phần tử 192 (hàng 12,cột 12)
  Q(ay,ay) → phần tử 208 (hàng 13,cột 13)
```

**Bảng giá trị khởi điểm và ý nghĩa:**

| Tham số | Giá trị mặc định | Hiệu ứng khi tăng | Hiệu ứng khi giảm |
|---------|-----------------|-------------------|-------------------|
| Q(x,x), Q(y,y) | 1e-3 (odom), 1.0 (map) | Vị trí theo GPS nhanh hơn | Vị trí bám encoder chặt hơn |
| Q(yaw,yaw) | 0.01 | Yaw xoay theo IMU nhanh | Yaw ổn định hơn, chậm cập nhật |
| Q(vx,vx), Q(vy,vy) | 0.5 | Vận tốc thay đổi nhanh | Vận tốc mượt hơn |
| Q(ax,ax), Q(ay,ay) | 0.3 | Gia tốc thay đổi nhanh | Gia tốc ổn định |

#### 14.4 Workflow hiệu chỉnh theo phương pháp tăng dần

```
Bước 1: Bắt đầu với Q lớn (tin cảm biến nhiều)
        → quan sát xem output có giật không

Bước 2: Giảm Q dần cho đến khi output mượt nhưng
        vẫn theo kịp chuyển động thật

Bước 3: Test drift khi đứng yên (mục 13)
        → nếu drift: xác định nguồn → điều chỉnh tham số nguồn đó

Bước 4: Test với chuyển động nhanh, chuyển hướng đột ngột
        → ekf_map có lag không? → tăng Q(x,x), Q(y,y)

Bước 5: Test mất GPS 30 giây rồi GPS về lại
        → ekf_map có nhảy không? → tăng delay trong navsat.yaml
```

**Thử nhanh thay đổi tham số mà không cần restart:**
```bash
# Xem tham số hiện tại của ekf_odom
rosparam get /ekf_odom

# Thay đổi một giá trị Q trực tiếp (không cần restart)
# Lưu ý: robot_localization đọc params lúc khởi động,
# cần restart node sau khi đổi yaml
rosnode kill /ekf_odom
roslaunch warthog_ekf ekf_sim.launch   # sẽ restart node
```

---

### 15. Ghi rosbag và phân tích hậu kỳ

#### 15.1 Ghi rosbag với script có sẵn

```bash
# Cấp quyền thực thi (một lần)
chmod +x ~/cnn25ws_dev/src/warthog_ekf/scripts/record_ekf.sh

# Ghi khi chạy sim
./record_ekf.sh sim

# Ghi khi chạy outdoor
./record_ekf.sh outdoor

# Giới hạn 3 phút
./record_ekf.sh sim 180

# File lưu tại ~/bags/ekf_sim_YYYYMMDD_HHMMSS.bag
```

Hoặc ghi thủ công nhanh:
```bash
rosbag record -O /tmp/ekf_test.bag \
  /imu/data \
  /warthog_velocity_controller/odom \
  /navsat/fix \
  /odometry/filtered \
  /odometry/filtered_map \
  /odometry/gps \
  /gps/filtered \
  /tf /tf_static
```

#### 15.2 Xem thông tin bag

```bash
rosbag info ~/bags/ekf_sim_*.bag

# Output mẫu:
# path:     ekf_sim_20260423_103045.bag
# size:     45.2 MB
# duration: 3:00s
# messages: 18450
# topics:
#   /imu/data               9000 msgs  50.0 Hz
#   /odometry/filtered      5400 msgs  30.0 Hz
#   /odometry/filtered_map  5400 msgs  30.0 Hz
#   /navsat/fix             3600 msgs  20.0 Hz
```

#### 15.3 Replay bag để chạy lại phân tích

```bash
# Terminal 1: roscore
roscore

# Terminal 2: play bag (dùng sim time)
rosbag play ~/bags/ekf_sim_*.bag --clock --pause
# Space: pause/resume, s: step 1 frame, q: quit

# Terminal 3: chạy ekf_monitor trên dữ liệu đã ghi
rosrun warthog_ekf ekf_monitor.py
```

**Lưu ý quan trọng khi replay:**
```bash
# Phải set use_sim_time = true
rosparam set /use_sim_time true
rosbag play ekf_test.bag --clock
```

#### 15.4 Phân tích offline bằng Python

```python
# phân tích bag file
import rosbag
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag('~/bags/ekf_sim_test.bag')

# Thu thập vị trí hai tầng
t_odom, x_odom, y_odom = [], [], []
t_map,  x_map,  y_map  = [], [], []
t_gps,  x_gps,  y_gps  = [], [], []
cov_x_odom = []
cov_x_map  = []

for topic, msg, t in bag.read_messages(topics=[
        '/odometry/filtered',
        '/odometry/filtered_map',
        '/odometry/gps']):
    ts = t.to_sec()
    if topic == '/odometry/filtered':
        t_odom.append(ts)
        x_odom.append(msg.pose.pose.position.x)
        y_odom.append(msg.pose.pose.position.y)
        cov_x_odom.append(msg.pose.covariance[0])
    elif topic == '/odometry/filtered_map':
        t_map.append(ts)
        x_map.append(msg.pose.pose.position.x)
        y_map.append(msg.pose.pose.position.y)
        cov_x_map.append(msg.pose.covariance[0])
    elif topic == '/odometry/gps':
        t_gps.append(ts)
        x_gps.append(msg.pose.pose.position.x)
        y_gps.append(msg.pose.pose.position.y)

bag.close()

# Plot quỹ đạo
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

ax1 = axes[0]
ax1.plot(x_odom, y_odom, 'b-',  label='ekf_odom (tầng 1)', linewidth=1.5)
ax1.plot(x_map,  y_map,  'r-',  label='ekf_map (tầng 2)',  linewidth=1.5)
ax1.plot(x_gps,  y_gps,  'g.',  label='GPS raw (odometry/gps)', markersize=3)
ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
ax1.set_title('Quỹ đạo'); ax1.legend(); ax1.grid(True); ax1.axis('equal')

# Plot covariance theo thời gian
ax2 = axes[1]
t0 = min(t_odom[0], t_map[0])
ax2.plot([t - t0 for t in t_odom], cov_x_odom, 'b-', label='Cov(x) tầng 1')
ax2.plot([t - t0 for t in t_map],  cov_x_map,  'r-', label='Cov(x) tầng 2')
ax2.set_xlabel('Thời gian (s)'); ax2.set_ylabel('Variance x (m²)')
ax2.set_title('Covariance theo thời gian'); ax2.legend(); ax2.grid(True)

plt.tight_layout()
plt.savefig('/tmp/ekf_analysis.png', dpi=150)
plt.show()
print("Đã lưu: /tmp/ekf_analysis.png")
```

Lưu đoạn code trên vào `scripts/analyze_bag.py`:
```bash
python3 ~/cnn25ws_dev/src/warthog_ekf/scripts/analyze_bag.py
```

#### 15.5 Các chỉ số cần ghi lại sau mỗi lần test

```bash
# In tóm tắt nhanh từ bag
rosbag play ekf_test.bag --topics /odometry/filtered_map --clock -r 10 &

rostopic echo /odometry/filtered_map | python3 -c "
import sys, yaml, math, statistics
xs, ys = [], []
for line in sys.stdin:
    try:
        d = yaml.safe_load(line)
        xs.append(d['pose']['pose']['position']['x'])
        ys.append(d['pose']['pose']['position']['y'])
    except: pass
    if len(xs) > 1000: break
print(f'X range: {min(xs):.3f} → {max(xs):.3f} m')
print(f'Y range: {min(ys):.3f} → {max(ys):.3f} m')
print(f'X std: {statistics.stdev(xs):.4f} m')
"
```

| Chỉ số ghi lại | Lý do |
|----------------|-------|
| Drift tầng 1 khi đứng yên 60s | Baseline chất lượng IMU + encoder |
| Drift tầng 2 khi đứng yên 60s | GPS stability |
| Covariance x,y tầng 1 trung bình | EKF confidence |
| Lệch max giữa tầng 1 vs tầng 2 | GPS-IMU alignment |
| Thời gian GPS lock ban đầu | navsat delay setting |

---

### 16. Visualize với RViz

#### 16.1 Tích hợp sẵn trong launch file

RViz tự động mở cùng lúc với hệ thống — không cần terminal riêng:

```bash
# Mô phỏng: Gazebo + EKF + RViz mở cùng lúc
roslaunch warthog_ekf ekf_sim.launch

# Thực tế ngoài trời: EKF + RViz mở cùng lúc
roslaunch warthog_ekf ekf_outdoor.launch
```

Config RViz nằm tại: `src/warthog_ekf/rviz/ekf.rviz`

Nếu muốn mở RViz riêng (không qua launch file):
```bash
rviz -d $(rospack find warthog_ekf)/rviz/ekf.rviz
```

#### 16.2 Các display trong `ekf.rviz`

| Display | Topic | Màu | Ý nghĩa |
|---------|-------|-----|---------|
| **RobotModel** | `robot_description` | — | Mô hình 3D xe Warthog |
| **TF** | — | — | Chỉ hiện `map`, `odom`, `base_link` |
| **Path** | `/odometry/filtered` | Xanh lá | Quỹ đạo EKF odom (tầng 1: encoder + IMU) |
| **Odometry** | `/odometry/filtered` | Xanh lá | Mũi tên hướng và vận tốc tức thời |
| **Path** | `/odometry/filtered_map` | Xanh dương | Quỹ đạo EKF map (tầng 2: encoder + IMU + GPS) |
| **Path** | `/odometry/gps` | Vàng | GPS thô đã chuyển sang local frame |
| **Grid** | — | Xám | Lưới nền 50×50 ô |

**Fixed Frame:** `map` — tất cả display hiển thị trong cùng một frame tuyệt đối.  
**Camera:** theo sát `base_link`, nhìn từ góc 55° xuống, cách xe 20 m.

#### 16.3 Giải thích trực quan

```
Trong RViz nhìn từ trên xuống:

          ╔════════════════════════════════╗
          ║  • • • • GPS raw (vàng)        ║  ← nhảy, rộng
          ║   ~~~~~~~~~~~                  ║
          ║  ────────────────              ║  ← EKF map (xanh dương): mượt + GPS
          ║  ──────────────                ║  ← EKF odom (xanh lá): mượt, không GPS
          ║                                ║
          ║       [Warthog model]          ║
          ╚════════════════════════════════╝

Khi xe đứng yên:
  - Xanh lá và xanh dương: gần như trùng nhau, không di chuyển (EKF tốt)
  - Vàng: có thể dao động nhẹ (GPS noise ~0.5 m)

Khi xe di chuyển:
  - Xanh lá: mượt, bám theo encoder + IMU
  - Xanh dương: bám gần GPS, có thể hơi giật khi GPS update
  - Vàng: nhảy rời rạc mỗi 50 ms (20 Hz GPS)

Khi GPS mất tín hiệu:
  - Xanh lá: tiếp tục bình thường (không dùng GPS)
  - Xanh dương: tiếp tục bằng encoder, dần tách xa vàng
  - Vàng: dừng update
```

#### 16.4 Cây TF cần thấy trong RViz

Bật **TF display** → chọn tab **Tree** để kiểm tra:

```
map
 └── odom               ← do ekf_map publish
      └── base_link     ← do ekf_odom publish
           ├── imu_link
           ├── front_laser
           └── ... (các link khác của Warthog)
```

Nếu thiếu `map → odom`: `ekf_map` chưa nhận được `odometry/gps` từ `navsat_transform`.  
Nếu thiếu `odom → base_link`: `ekf_odom` chưa khởi động hoặc chưa nhận sensor data.

#### 16.5 Thêm display thủ công trong RViz

Nếu muốn thêm display không có trong config mặc định:

```
Trong RViz:
  1. Click nút "Add" (góc dưới trái panel Displays)
  2. Chọn loại display phù hợp:

  ┌─────────────────────────────────────────────────────┐
  │ Loại display  │ Dùng để xem                         │
  ├─────────────────────────────────────────────────────┤
  │ Path          │ Quỹ đạo di chuyển theo thời gian     │
  │ Odometry      │ Mũi tên vị trí + hướng hiện tại      │
  │ TF            │ Cây transform giữa các frame         │
  │ RobotModel    │ Mô hình 3D xe                        │
  │ NavSatFix     │ Điểm GPS trực tiếp trên bản đồ       │
  │ LaserScan     │ Dữ liệu LiDAR                        │
  │ MarkerArray   │ Marker tuỳ chỉnh từ node ROS         │
  └─────────────────────────────────────────────────────┘

  3. Nhập topic tương ứng vào ô "Topic"
  4. Chỉnh màu, kích thước trong panel bên dưới
```

**Lưu config sau khi chỉnh:**
```bash
# Trong RViz: File → Save Config
# hoặc Ctrl+S → tự động ghi vào ekf.rviz
```

#### 16.6 Troubleshoot RViz

**RViz mở nhưng không thấy robot model:**
```bash
# Kiểm tra robot_description đã được publish chưa
rosparam get /robot_description | head -5
# Nếu trống: description.launch chưa chạy
```

**Display báo "No transform from [odom] to [map]":**
```bash
# ekf_map chưa publish TF map→odom
# Kiểm tra ekf_map có nhận odometry/gps không
rostopic hz /odometry/gps
# Nếu = 0: navsat_transform chưa publish → chờ GPS fix
```

**Path không vẽ đường:**
```bash
# Fixed Frame phải là 'map' (góc trên của panel Displays)
# Topic phải đúng: /odometry/filtered (có dấu /)
rostopic echo /odometry/filtered | head -20   # kiểm tra topic có data không
```

**RViz không tự mở khi chạy launch file:**
```bash
# Kiểm tra rviz đã được cài
which rviz   # phải ra /opt/ros/noetic/bin/rviz
# Nếu không có: sudo apt install ros-noetic-rviz
```

---

### 17. Giải thích chi tiết `ekf_monitor.py`

Script monitor chất lượng EKF real-time tại `scripts/ekf_monitor.py`. Chạy song song với hệ thống, không can thiệp vào EKF.

#### 17.1 Hằng số và hàm tiện ích ở module level

```python
RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
```

Mã ANSI escape để tô màu terminal. `RESET` bắt buộc phải có sau mỗi đoạn màu, nếu không toàn bộ terminal sẽ bị nhuốm màu.

```python
EARTH_RADIUS = 6378137.0   # bán kính Trái Đất (m) — hệ WGS84
```

```python
def latlon_dist(lat1, lon1, lat2, lon2):
    dy = math.radians(lat2 - lat1) * EARTH_RADIUS
    dx = math.radians(lon2 - lon1) * EARTH_RADIUS * math.cos(math.radians((lat1+lat2)/2))
    return math.sqrt(dx*dx + dy*dy)
```

Chuyển độ chênh lệch lat/lon → khoảng cách mét. Dùng phép xấp xỉ phẳng (flat-earth) — hợp lệ khi hai điểm cách nhau < vài km. `cos(lat_trung_bình)` hiệu chỉnh độ thu hẹp kinh tuyến khi ra xa xích đạo: ở Hà Nội (lat≈21°), 1° kinh độ ≈ 103 km thay vì 111 km.

---

#### 17.2 `__init__` — Khởi tạo node

**Bước 1 — Đọc tham số:**

```python
rospy.init_node("ekf_monitor")
self.v_threshold = rospy.get_param("~v_threshold", 0.02)  # m/s
self.report_hz   = rospy.get_param("~report_hz",   1.0)   # Hz
self.drift_warn  = rospy.get_param("~drift_warn_m", 0.05) # m
```

Dấu `~` nghĩa là **private parameter** — tên đầy đủ là `/ekf_monitor/v_threshold`. Tham số private tránh xung đột tên với các node khác. Giá trị mặc định nằm sau dấu phẩy — node vẫn chạy được nếu không truyền tham số nào.

**Bước 2 — Khai báo biến trạng thái:**

```python
self.odom_pos   = None   # (x, y) vị trí tầng 1
self.odom_vel   = 0.0    # |v| m/s từ twist
self.odom_cov_x = None   # variance x từ covariance matrix
self.odom_cov_y = None
self.odom_cov_yaw = None

self.static_ref_odom   = None  # vị trí lúc bắt đầu đứng yên
self.static_drift_odom = 0.0   # drift tích lũy khi đứng yên
```

Tương tự cho tầng 2 (`map_*`) và GPS (`gps_lat`, `gps_lon`, `gps_cov`). Tất cả khởi tạo `None` — dùng `None` thay vì `0` để phân biệt "chưa nhận data" với "nhận data = 0".

**Bước 3 — Tự detect topic GPS:**

```python
gps_topic = rospy.get_param("~gps_topic", "")
if not gps_topic:
    published = [t for t, _ in rospy.get_published_topics()]
    gps_topic = "/navsat/fix" if "/navsat/fix" in published else "gps/fix"
```

`rospy.get_published_topics()` trả về list các `(topic_name, topic_type)`. List comprehension `[t for t, _ in ...]` bỏ phần type, chỉ lấy tên topic. Sau đó kiểm tra `/navsat/fix` (topic của Gazebo plugin) có đang được publish không — nếu có thì đang ở chế độ sim, ngược lại dùng `gps/fix` (outdoor F9P).

**Bước 4 — Subscribe và tạo timer:**

```python
rospy.Subscriber("odometry/filtered",     Odometry,  self._cb_odom)
rospy.Subscriber("odometry/filtered_map", Odometry,  self._cb_map)
rospy.Subscriber(gps_topic,               NavSatFix, self._cb_gps)

rospy.Timer(rospy.Duration(1.0 / self.report_hz), self._report)
rospy.spin()
```

`rospy.Timer` gọi `_report` đúng theo tần số `report_hz` (mặc định 1 Hz). `rospy.spin()` block thread chính, giữ node chạy và xử lý callbacks trong vòng lặp sự kiện của ROS.

---

#### 17.3 `_cb_odom` — Callback tầng 1

```python
def _cb_odom(self, msg):
    p = msg.pose.pose.position
    t = msg.twist.twist.linear
    self.odom_pos = (p.x, p.y)
    self.odom_vel = math.sqrt(t.x**2 + t.y**2)

    c = msg.pose.covariance      # list 36 phần tử (ma trận 6×6 row-major)
    self.odom_cov_x   = c[0]    # hàng 0, cột 0 → var(x)
    self.odom_cov_y   = c[7]    # hàng 1, cột 1 → var(y)   [1×6+1=7]
    self.odom_cov_yaw = c[35]   # hàng 5, cột 5 → var(yaw) [5×6+5=35]

    self._update_drift("odom")
```

`nav_msgs/Odometry` có cấu trúc lồng nhau: `pose.pose.position` (không phải `pose.position`). Lớp ngoài `pose` là `PoseWithCovariance`, lớp trong `pose.pose` mới là `Pose` thực sự.

Covariance là ma trận 6×6 biểu diễn độ bất định của `[x, y, z, roll, pitch, yaw]`. Chỉ cần 3 phần tử đường chéo tương ứng với x, y, yaw — đó là `c[0]`, `c[7]`, `c[35]`.

`odom_vel` tính bằng **norm vận tốc thẳng**: `|v| = √(vx² + vy²)`. Không dùng `vz` vì xe di chuyển 2D.

---

#### 17.4 `_cb_map` và `_cb_gps`

```python
def _cb_map(self, msg):
    p = msg.pose.pose.position
    self.map_pos   = (p.x, p.y)
    c = msg.pose.covariance
    self.map_cov_x = c[0]
    self.map_cov_y = c[7]
    self._update_drift("map")
```

Đơn giản hơn `_cb_odom`: không cần velocity (dùng chung `odom_vel` để phán đoán "đứng yên").

```python
def _cb_gps(self, msg):
    self.gps_lat = msg.latitude
    self.gps_lon = msg.longitude
    self.gps_cov = msg.position_covariance[0]  # variance x ≈ variance y
```

`sensor_msgs/NavSatFix` có trường `position_covariance` là ma trận 3×3 (9 phần tử) theo thứ tự `[EE, EN, EU, NE, NN, NU, UE, UN, UU]`. Phần tử `[0]` là variance theo hướng East (≈ variance x trong local frame). Với GPS đẳng hướng thì `cov[0] ≈ cov[4]` (variance North ≈ variance East).

---

#### 17.5 `_update_drift` — Logic phát hiện drift

```python
def _update_drift(self, tier):
    if self.odom_vel is None:
        return

    if tier == "odom" and self.odom_pos:
        if self.odom_vel < self.v_threshold:       # xe đứng yên?
            if self.static_ref_odom is None:
                self.static_ref_odom = self.odom_pos   # lưu mốc
            else:
                dx = self.odom_pos[0] - self.static_ref_odom[0]
                dy = self.odom_pos[1] - self.static_ref_odom[1]
                self.static_drift_odom = math.sqrt(dx**2 + dy**2)
        else:                                      # xe đang chạy
            self.static_ref_odom  = None           # reset mốc
            self.static_drift_odom = 0.0
```

**State machine 2 trạng thái:**

```
RUNNING (odom_vel ≥ threshold):
    static_ref = None, drift = 0

STATIC (odom_vel < threshold):
    Lần đầu vào: lưu vị trí hiện tại làm mốc
    Các lần sau:  tính khoảng cách từ mốc = drift
```

Mỗi lần callback được gọi (30 Hz), hàm này chạy và cập nhật `static_drift`. Nếu EKF tốt, drift sẽ gần 0 dù đứng lâu. Nếu IMU có bias, drift tăng dần theo thời gian → cảnh báo.

---

#### 17.6 `_fmt_cov` và `_fmt_drift` — Format màu

```python
def _fmt_cov(self, val):
    if val is None:
        return "  N/A  "
    color = GREEN if val < 0.1 else (YELLOW if val < 1.0 else RED)
    return f"{color}{val:7.4f}{RESET}"
```

Dấu `:7.4f` nghĩa là: tổng 7 ký tự, 4 chữ số sau dấu thập phân → các số thẳng hàng khi in ra terminal.

```python
def _fmt_drift(self, val, threshold):
    color = GREEN if val < threshold * 0.5 else (YELLOW if val < threshold else RED)
    warn  = f"  {RED}!! DRIFT !!{RESET}" if val >= threshold else ""
    return f"{color}{val:.4f}{RESET} m{warn}"
```

Ngưỡng chia làm 3 vùng: tốt (< 50% threshold), chú ý (50–100%), nguy hiểm (> 100% → in thêm `!! DRIFT !!`).

---

#### 17.7 `_report` — In báo cáo theo timer

```python
def _report(self, _event):
    sep = "─" * 60
    lines = [...]          # build list các dòng text
    print("\n".join(lines))
```

Tham số `_event` là `rospy.TimerEvent` chứa thông tin thời điểm gọi — không dùng nên đặt tên `_event` (convention Python: `_` nghĩa là "biến không dùng").

Dùng `lines` list rồi `"\n".join()` thay vì nhiều lệnh `print()` riêng lẻ — đảm bảo toàn bộ báo cáo xuất hiện **cùng lúc** (một lần write), tránh bị xen lẫn output từ node khác.

```python
rospy.loginfo_throttle(1, "\n" + sep)
```

`loginfo_throttle(interval, msg)` chỉ log tối đa 1 lần mỗi `interval` giây — tránh spam ROS log với separator line.

**So sánh hai tầng:**
```python
dx = self.map_pos[0] - self.odom_pos[0]
dy = self.map_pos[1] - self.odom_pos[1]
d  = math.sqrt(dx**2 + dy**2)
```

Tầng 1 (odom) và tầng 2 (map) về lý thuyết không cần khớp hoàn toàn vì chúng ở hai frame khác nhau (`odom` vs `map`). Nhưng nếu lệch > 5m nghĩa là GPS đang kéo ekf_map ra xa so với ekf_odom — thường do heading offset hoặc datum sai.

---

#### 17.8 Cách đọc output nhanh

```
Cov x = 0.002  → EKF rất tự tin (xanh)
Cov x = 0.5    → bình thường khi có GPS (vàng)
Cov x = 3.5    → GPS mất hoặc IMU bias lớn (đỏ)

Drift = 0.003 m         → đứng yên hoàn toàn (xanh)
Drift = 0.082 m !!DRIFT!! → IMU đang tích lũy sai số (đỏ)

Lệch 1 vs 2 = 0.04 m   → GPS và odom khớp (xanh)
Lệch 1 vs 2 = 8.3 m    → GPS lệch hướng hoặc datum sai (đỏ)
```

**Tham số tuỳ chỉnh:**
```bash
rosrun warthog_ekf ekf_monitor.py \
  _v_threshold:=0.01 \    # nhạy hơn với "đứng yên" (mặc định 0.02)
  _drift_warn_m:=0.03 \   # cảnh báo sớm hơn   (mặc định 0.05)
  _report_hz:=2.0 \       # báo cáo 2 Hz        (mặc định 1.0)
  _gps_topic:=/navsat/fix # ghi đè detect tự động
```
