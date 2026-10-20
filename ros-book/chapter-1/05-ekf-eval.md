# Chapter 5

## Gói warthog_ekf_eval — Đánh giá EKF tầng 1 bằng Loop Closure Test

---

### 1. Mục tiêu và phương pháp

Sau khi cấu hình EKF tầng 1 (fuse IMU + encoder, không GPS), câu hỏi đặt ra là: **EKF có đủ tin cậy để dẫn đường không?**

Gói `warthog_ekf_eval` đánh giá bằng **Loop Closure Test hình vuông**:

```
Ghi vị trí xuất phát A
       ↓
Điều khiển xe chạy hình vuông (4 cạnh + 4 góc quay 90°)
       ↓
Xe quay về A → EKF báo cách A bao xa?
       ↓
Loop Closure Error = sai số tích lũy của IMU + encoder
```

**Tại sao dùng Loop Closure?**
- Không cần thiết bị đo ngoài (GPS, motion capture)
- Phản ánh đúng khả năng dẫn đường thực tế: sai số tích lũy theo khoảng cách
- Áp dụng được cả sim lẫn ngoài trời

**Hai luồng đánh giá:**

| Luồng | Môi trường | Ground Truth | Chỉ số thêm |
|---|---|---|---|
| **Sim** | Gazebo | `/gazebo/model_states` | GT LCE, RMSE EKF vs GT |
| **Outdoor** | Thực tế | Không có | Chỉ LCE + Heading Error |

---

### 2. Cấu trúc gói

```
src/warthog_ekf_eval/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── sim_params.yaml       ← tham số cố định cho sim
│   └── outdoor_params.yaml   ← tham số cố định cho outdoor
├── launch/
│   ├── eval_sim.launch       ← chạy đánh giá trong Gazebo
│   └── eval_outdoor.launch   ← chạy đánh giá ngoài trời
└── scripts/
    ├── square_eval.py        ← node điều khiển + tính kết quả
    └── plot_results.py       ← vẽ đồ thị từ file kết quả
```

---

### 3. Cách launch

#### 3.1 Mô phỏng (Sim)

Yêu cầu: `ekf_sim.launch` đang chạy ở terminal khác.

```bash
# Terminal 1 — khởi động Gazebo + EKF (môi trường không vật cản)
roslaunch warthog_ekf ekf_sim.launch world:=/usr/share/gazebo-11/worlds/empty.world

# Terminal 2 — sau khi Gazebo load xong, chạy đánh giá
roslaunch warthog_ekf_eval eval_sim.launch

# Tuỳ chỉnh tham số
roslaunch warthog_ekf_eval eval_sim.launch side_length:=8.0 linear_vel:=0.5 angular_vel:=0.25
```

#### 3.2 Ngoài trời (Outdoor)

Yêu cầu: `ekf_outdoor.launch` đang chạy, IMU và encoder đang publish.

```bash
# Terminal 1 — khởi động EKF (chỉ tầng 1: IMU + encoder)
roslaunch warthog_ekf ekf_outdoor.launch

# Terminal 2 — chạy đánh giá
roslaunch warthog_ekf_eval eval_outdoor.launch

# Tuỳ chỉnh
roslaunch warthog_ekf_eval eval_outdoor.launch side_length:=10.0 linear_vel:=0.5
```

#### 3.3 Tham số launch

Các tham số đặt trực tiếp trong launch file, không cần sửa YAML:

| Tham số | Mặc định sim | Mặc định outdoor | Ý nghĩa |
|---|---|---|---|
| `side_length` | 5.0 m | 5.0 m | Độ dài mỗi cạnh hình vuông |
| `linear_vel` | 0.4 m/s | 0.5 m/s | Vận tốc chạy thẳng |
| `angular_vel` | 0.3 rad/s | 0.25 rad/s | Vận tốc quay 90° |
| `output_dir` | `/tmp/ekf_eval` | `/tmp/ekf_eval` | Thư mục lưu kết quả |

> **Lưu ý:** `/tmp/` bị xóa khi reboot. Dùng `output_dir:=/home/cuong/ekf_results` để giữ kết quả lâu dài.

#### 3.4 Xem đồ thị kết quả

```bash
# Tự tìm file YAML mới nhất trong output_dir
rosrun warthog_ekf_eval plot_results.py

# Chỉ định file cụ thể
rosrun warthog_ekf_eval plot_results.py /tmp/ekf_eval/result_sim_1234567.yaml
```

---

### 4. Cấu hình — config YAML

File YAML chỉ chứa tham số **cố định** (topic, mode). Tham số điều chỉnh đặt trong launch.

#### `config/sim_params.yaml`

```yaml
turn_direction: 1       # 1 = quay trái, -1 = quay phải
pause_after_side: 1.0   # dừng sau mỗi cạnh trước khi quay (s)
pause_after_turn: 0.5   # dừng sau mỗi lần quay xong (s)

mode: "sim"
gt_topic: "/gazebo/model_states"   # topic ground truth Gazebo
gt_model_name: "warthog"           # tên model trong Gazebo
cmd_vel_topic: "/cmd_vel"
ekf_odom_topic: "/odometry/filtered"
```

#### `config/outdoor_params.yaml`

```yaml
turn_direction: 1
pause_after_side: 2.0   # tăng hơn sim để EKF ổn định hơn ngoài trời
pause_after_turn: 1.5

mode: "outdoor"
gt_topic: ""            # không có ground truth
gt_model_name: ""
cmd_vel_topic: "/cmd_vel"
ekf_odom_topic: "/odometry/filtered"
```

---

### 5. Giải thích code `square_eval.py`

#### 5.1 State Machine

Node điều khiển xe bằng state machine 6 trạng thái:

```
WAIT_EKF → PAUSE(3s) → DRIVE → TURN → DRIVE → TURN → ... (×4) → DONE
```

```python
class State(Enum):
    WAIT_EKF = auto()   # chờ nhận odometry/filtered lần đầu
    PAUSE    = auto()   # dừng có thời hạn (đầu, giữa cạnh-góc)
    DRIVE    = auto()   # chạy thẳng đến đủ side_length
    TURN     = auto()   # quay 90° đến đủ góc
    DONE     = auto()   # xong, tính kết quả
```

Trình tự hoàn chỉnh: **4 cạnh + 4 lần quay** — lần quay cuối đưa xe về đúng hướng ban đầu để heading error phản ánh thực sự sai số EKF.

```
Drive→Turn(90°)→Drive→Turn(90°)→Drive→Turn(90°)→Drive→Turn(90°)→DONE
  1      1        2      2        3      3         4      4
```

#### 5.2 Khởi tạo `__init__`

```python
self.side_length  = rospy.get_param("~side_length",      5.0)
self.linear_vel   = rospy.get_param("~linear_vel",       0.4)
self.angular_vel  = rospy.get_param("~angular_vel",      0.3)
self.turn_dir     = rospy.get_param("~turn_direction",   1)
self.pause_side   = rospy.get_param("~pause_after_side", 1.0)
self.pause_turn   = rospy.get_param("~pause_after_turn", 0.5)
self.mode         = rospy.get_param("~mode",             "sim")
```

Dấu `~` = private parameter — tên đầy đủ là `/square_eval/side_length`. Subscribe `odometry/filtered` (EKF tầng 1) và `/gazebo/model_states` (nếu sim).

Vòng điều khiển **20 Hz**:

```python
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    self._loop()
    if self.state == State.DONE:
        break
    rate.sleep()
```

#### 5.3 Callback EKF — `_cb_ekf`

```python
def _cb_ekf(self, msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    self.ekf_pos = (p.x, p.y)
    self.ekf_yaw = yaw
    self.ekf_path.append((p.x, p.y, rospy.get_time()))
```

Đọc từ `nav_msgs/Odometry`. Lưu ý cấu trúc lồng: `msg.pose.pose.position` (không phải `msg.pose.position`). Quaternion chuyển sang euler lấy `yaw` (góc quay quanh trục Z). Toàn bộ path được lưu vào `ekf_path` để vẽ đồ thị sau.

#### 5.4 Callback Ground Truth — `_cb_gt`

```python
def _cb_gt(self, msg):
    idx = msg.name.index(self.gt_model_name)   # tìm "warthog" trong danh sách
    p = msg.pose[idx].position
    self.gt_pos = (p.x, p.y)
    self.gt_path.append((p.x, p.y, rospy.get_time()))
```

`/gazebo/model_states` chứa tất cả model trong scene. Phải tìm đúng tên `"warthog"` để lấy pose của robot.

#### 5.5 Điều khiển khoảng cách — `_dist_from_seg`

```python
def _dist_from_seg(self):
    dx = self.ekf_pos[0] - self.seg_start_pos[0]
    dy = self.ekf_pos[1] - self.seg_start_pos[1]
    return math.sqrt(dx*dx + dy*dy)
```

Khoảng cách **Euclidean thẳng** từ đầu cạnh (`seg_start_pos`) đến vị trí hiện tại. Xe dừng khi `dist >= side_length`. Đây là khoảng cách theo EKF — không phải odometer thực.

```
seg_start ────────────── ekf_pos (hiện tại)
    ●  ←── 2.004 m ──→  ●
          (Euclidean)
```

#### 5.6 Điều khiển góc quay — `_yaw_turned`

```python
def _yaw_turned(self):
    diff = self.ekf_yaw - self.seg_start_yaw
    diff = (diff + math.pi) % (2*math.pi) - math.pi   # normalize [-π, π]
    return abs(diff)
```

Góc đã quay kể từ đầu lượt quay (`seg_start_yaw`). Normalize về `[-π, π]` để xử lý đúng khi yaw vượt ranh giới ±180°. Xe dừng khi:

```python
target = math.pi / 2 - 0.04   # 87.7° — dừng trước 90° một chút
if turned >= target:
    self._stop()
```

Dung sai `0.04 rad ≈ 2.3°` tránh overshooting do vòng lặp 20 Hz.

#### 5.7 Vòng lặp chính — `_loop`

**WAIT_EKF:** Chờ `ekf_pos` không phải None → EKF đang publish. Khi nhận được, dừng 3 giây (PAUSE) để EKF ổn định trước khi chạy.

**PAUSE:** Đợi hết thời gian. Nếu là lần dừng đầu tiên (`start_pos is None`) → ghi vị trí xuất phát, bắt đầu DRIVE. Nếu là pause giữa cạnh và góc → chuyển TURN.

**DRIVE:** Publish `linear.x = linear_vel` cho đến khi `_dist_from_seg() >= side_length`. Khi đủ, dừng, ghi `side_done += 1`, chuyển TURN (kể cả sau cạnh thứ 4).

**TURN:** Publish `angular.z = angular_vel × turn_dir`. Khi `_yaw_turned() >= 87.7°`, dừng, ghi `turn_done += 1`. Nếu đủ 4 lần quay → DONE, tính kết quả. Nếu chưa đủ → ghi mốc mới, chuyển DRIVE.

#### 5.8 Tính kết quả — `_compute_and_report`

**Loop Closure Error (LCE):**
```python
dx  = self.ekf_pos[0] - self.start_pos[0]
dy  = self.ekf_pos[1] - self.start_pos[1]
lce = math.sqrt(dx*dx + dy*dy)
lce_pct = lce / (side_length * 4) * 100
```

Khoảng cách EKF từ điểm kết thúc về điểm xuất phát. Nếu EKF không drift → xe về đúng chỗ → LCE ≈ 0.

**Heading Error:**
```python
dyaw = self.ekf_yaw - self.start_yaw
dyaw = (dyaw + math.pi) % (2*math.pi) - math.pi
```

Sau 4 lần quay 90° = 360° → heading_error ≈ 0°. Nếu lớn → EKF gyro drift.

**GT Loop Closure Error (chỉ sim):**
```python
dx_gt = self.gt_pos[0] - self.gt_start_pos[0]
dy_gt = self.gt_pos[1] - self.gt_start_pos[1]
gt_lce = math.sqrt(dx_gt*dx_gt + dy_gt*dy_gt)
```

Sai số vị trí **thật** (Gazebo biết chính xác). So sánh với EKF LCE để thấy EKF có đang "lạc quan quá mức" không.

**RMSE (chỉ sim):** Nội suy GT theo timestamp EKF, tính sai số vị trí trung bình suốt hành trình:
```python
RMSE = √( Σ |EKF_pos - GT_pos|² / N )
```

**Tiêu chí đánh giá:**
```python
if lce_pct < 1.0:   grade = "XUẤT SẮC"  # đủ dẫn đường chính xác cao
elif lce_pct < 2.0: grade = "TỐT"        # đủ dẫn đường thông thường
elif lce_pct < 4.0: grade = "CHẤP NHẬN" # cần kết hợp GPS
elif lce_pct < 6.0: grade = "YẾU"        # cần hiệu chỉnh
else:               grade = "KÉM"        # không đủ dẫn đường
```

**Lưu file:**
- `result_{mode}_{timestamp}.yaml` — chỉ số kết quả
- `ekf_path_{mode}_{timestamp}.csv` — tọa độ EKF theo thời gian
- `gt_path_{mode}_{timestamp}.csv` — tọa độ GT theo thời gian (chỉ sim)

#### 5.9 Tính RMSE — `_compute_rmse`

```python
for ex, ey, et in self.ekf_path:
    # Nội suy GT tại thời điểm et
    alpha = (et - gt_times[i]) / (gt_times[i+1] - gt_times[i])
    gx = gt_xs[i] + alpha * (gt_xs[i+1] - gt_xs[i])
    gy = gt_ys[i] + alpha * (gt_ys[i+1] - gt_ys[i])
    errors.append((ex-gx)**2 + (ey-gy)**2)
```

EKF publish 30 Hz, GT publish 20 Hz → không cùng timestamp. Nội suy tuyến tính GT về đúng timestamp EKF để so sánh công bằng.

---

### 6. Giải thích code `plot_results.py`

#### 6.1 Load dữ liệu

```python
result  = load_yaml(yaml_path)              # chỉ số kết quả
ekf_csv = find_companion_csv(yaml_path, "ekf_path")  # tìm CSV cùng timestamp
gt_csv  = find_companion_csv(yaml_path, "gt_path")   # chỉ sim
```

`find_companion_csv` tìm file CSV cùng thư mục và cùng timestamp với YAML:
```
result_sim_1234567.yaml  →  ekf_path_sim_1234567.csv
```

#### 6.2 Normalize tọa độ

```python
# EKF normalize theo vị trí xuất phát từ YAML
ekf_x0 = result["start_pos"]["x"]
ekf_xs_n = ekf_xs - ekf_x0

# GT normalize theo gt_start_pos (thời điểm bắt đầu chạy, không phải gt_xs[0])
gt_x0, gt_y0 = gt_start["x"], gt_start["y"]
gt_xs_n = gt_xs - gt_x0
```

**Tại sao cần normalize:** EKF ở frame `odom` và GT ở frame Gazebo `world` — hai hệ tọa độ khác nhau, không thể vẽ trực tiếp. Sau normalize, cả hai bắt đầu từ `(0, 0)`, so sánh được hình dạng quỹ đạo.

**Tại sao dùng `gt_start_pos` thay vì `gt_xs[0]`:** GT ghi từ lúc node khởi động — bao gồm cả đoạn robot rơi từ `z=1.0` xuống đất (12 giây) trước khi bắt đầu chạy. `gt_start_pos` là GT đúng tại thời điểm xe bắt đầu chạy hình vuông.

#### 6.3 Hình vuông lý tưởng

```python
def ideal_square(start_x, start_y, start_yaw, side):
    pts = [(start_x, start_y)]
    yaw = start_yaw
    for _ in range(4):
        x = pts[-1][0] + side * math.cos(yaw)
        y = pts[-1][1] + side * math.sin(yaw)
        pts.append((x, y))
        yaw += math.pi / 2   # mỗi cạnh quay thêm 90°
```

Tạo hình vuông lý tưởng từ `(0,0)` với hướng ước lượng từ 5 điểm đầu của EKF path. Dùng để so sánh EKF đi có đúng hình vuông không.

#### 6.4 Đồ thị đầu ra

**Panel trái — Quỹ đạo di chuyển:**

| Ký hiệu | Màu | Ý nghĩa |
|---|---|---|
| Đường đứt nét | Xanh dương | Hình vuông lý tưởng |
| Đường liền | Đỏ | Ground Truth Gazebo (chỉ sim) |
| Đường liền | Xanh lá | Quỹ đạo EKF thực tế |
| Điểm ● | Vàng | Điểm xuất phát `(0, 0)` |
| Điểm ✕ | Cam | Điểm kết thúc EKF |
| Mũi tên ↔ | Đen | Loop Closure Error |

**Panel phải — Bảng tóm tắt chỉ số:** Chế độ, kích thước hình vuông, LCE, LCE%, Heading Error, GT LCE (sim), RMSE (sim), đánh giá màu theo grade.

---

### 7. Kết quả mẫu và cách đọc

#### 7.1 Output terminal

```
══════════════════════════════════════════════════════════════
  KẾT QUẢ ĐÁNH GIÁ EKF — Loop Closure Test (hình vuông)
══════════════════════════════════════════════════════════════
  Chế độ          : SIM
  Kích thước      : 5.0 m/cạnh × 4 = 20 m tổng
  Vận tốc         : 0.4 m/s thẳng  |  0.3 rad/s quay

  Vị trí xuất phát (EKF) : (+0.0002,  +0.0000)
  Vị trí kết thúc (EKF)  : (-0.2420,  +0.2648)

  Loop Closure Error      : 0.3687 m
  Loop Closure Error %    : 1.84%  (tổng 20 m)
  Heading Error           : 4.19°

  ── So sánh với Gazebo Ground Truth ──
  GT Loop Closure Error   : 2.3698 m
  RMSE (EKF vs GT)        : 1.5061 m

  Đánh giá: TỐT  —  Đủ điều kiện dẫn đường thông thường
══════════════════════════════════════════════════════════════
```

#### 7.2 Ý nghĩa từng chỉ số

**Loop Closure Error (LCE):**
Khoảng cách thẳng từ điểm kết thúc đến điểm xuất phát theo EKF. Phản ánh sai số tích lũy của IMU + encoder.

**LCE %:**
LCE tính theo phần trăm tổng đường đi. Dùng để so sánh công bằng giữa các hình vuông kích thước khác nhau.

| LCE % | Đánh giá | Kết luận |
|---|---|---|
| < 1% | XUẤT SẮC | Đủ dẫn đường chính xác cao |
| 1–2% | TỐT | Đủ dẫn đường thông thường |
| 2–4% | CHẤP NHẬN | Nên kết hợp GPS hiệu chỉnh |
| 4–6% | YẾU | Cần hiệu chỉnh IMU/encoder |
| > 6% | KÉM | Không đủ dẫn đường |

**Heading Error:**
Sai số góc hướng sau 4 lần quay 90° (tổng 360°). Lý tưởng ≈ 0°. Nếu lớn → gyro bias — EKF tích lũy sai số góc theo thời gian.

| Heading Error | Ý nghĩa |
|---|---|
| < 5° | Tốt |
| 5–15° | Chấp nhận |
| > 15° | Gyro bias lớn, cần hiệu chỉnh IMU |

**GT Loop Closure Error (chỉ sim):**
Sai số vị trí **thật** từ Gazebo. Nếu GT LCE >> EKF LCE → EKF đang tự tin quá mức, ước lượng sai nhưng không biết mình sai.

**RMSE (chỉ sim):**
Sai số vị trí trung bình giữa EKF và Gazebo GT suốt hành trình. RMSE lớn → EKF drift nhiều so với thực tế.

#### 7.3 Cảnh báo quan trọng khi đọc kết quả sim

```
EKF LCE  = 0.37 m (1.84%) → Đánh giá "TỐT" dựa trên EKF tự đánh giá
GT  LCE  = 2.37 m (11.8%) → Thực tế: KÉM
```

Nếu GT LCE lớn hơn EKF LCE nhiều lần, EKF **đang drift nhưng không nhận ra**. Đây là dấu hiệu cần hiệu chỉnh lại `ekf_odom.yaml`.

---

### 8. Tham số nhiễu cảm biến trong mô phỏng

#### IMU (`warthog_description/urdf/warthog.gazebo`)

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `accelDrift` | 0.005 m/s² | Bias tích lũy accelerometer |
| `accelGaussianNoise` | 0.005 m/s² | Nhiễu ngẫu nhiên accelerometer |
| `rateDrift` | 0.005 rad/s | Bias tích lũy gyro → gây heading drift |
| `rateGaussianNoise` | 0.005 rad/s | Nhiễu ngẫu nhiên gyro |
| `headingDrift` | 0.005 rad | Drift góc heading |

`rateDrift = 0.005 rad/s` nghĩa là sau 60 giây đứng yên, gyro có thể báo lệch `0.005 × 60 = 0.3 rad ≈ 17°`.

#### Encoder (`warthog_control/config/control.yaml`)

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `twist_covariance_diagonal[0]` | 0.001 | Variance vận tốc vx (m/s)² |
| `twist_covariance_diagonal[5]` | 0.03 | Variance vận tốc angular z (rad/s)² |
| `wheel_separation_multiplier` | 1.125 | Hệ số hiệu chỉnh khoảng cách bánh → ảnh hưởng góc quay |

---

### 9. Hiệu chỉnh khi kết quả kém

#### LCE % > 4% hoặc Heading Error > 15°

```yaml
# ekf_odom.yaml — giảm Q(yaw) để EKF tin mô hình hơn, ít drift hơn
process_noise_covariance: [1e-3, 0, ..., 0.001, ...]
#                                         ↑ index 80 = Q(yaw,yaw), giảm từ 0.01
```

#### RMSE (EKF vs GT) lớn dù LCE nhỏ

EKF đang tự tin quá mức. Tăng `initial_estimate_covariance` để EKF thận trọng hơn lúc đầu:

```yaml
initial_estimate_covariance: [1e-3, 0, ..., 1.0, ...]
#                              ↑ x,y tăng lên       ↑ yaw giữ nguyên
```

#### Heading Error lớn sau mỗi lần quay

Kiểm tra `wheel_separation_multiplier` trong `control.yaml` — sai số góc thường do khoảng cách bánh xe không chính xác:

```yaml
wheel_separation_multiplier: 1.125   # thử điều chỉnh ±0.05
```

---

### 10. File kết quả lưu ở đâu

Mặc định tại `/tmp/ekf_eval/` (xóa khi reboot):

```
/tmp/ekf_eval/
├── result_sim_1234567.yaml        ← chỉ số kết quả
├── ekf_path_sim_1234567.csv       ← tọa độ EKF theo thời gian
├── gt_path_sim_1234567.csv        ← tọa độ GT theo thời gian (chỉ sim)
└── result_sim_1234567_plot.png    ← đồ thị (tạo khi chạy plot_results.py)
```

Xem file ảnh:
```bash
eog /tmp/ekf_eval/result_sim_*_plot.png
```

Lưu lâu dài:
```bash
roslaunch warthog_ekf_eval eval_sim.launch output_dir:=/home/cuong/ekf_results
```
