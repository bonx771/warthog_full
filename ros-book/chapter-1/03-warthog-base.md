# Chapter 3

# Làm chủ warthog

### Mục tiêu

Build workspace `cnn25ws_dev` (chứa các gói Clearpath Warthog) trên Ubuntu 20.04 x86_64.

---

### Bước 1 — Cài ROS Noetic

Ubuntu 20.04 dùng **ROS Noetic** (bản ROS 1 cuối cùng).

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Bước 2 — Cài rosdep và khởi tạo

```bash
sudo apt install python3-rosdep python3-catkin-tools
sudo rosdep init
rosdep update
```

---

### Bước 3 — Cài dependencies của workspace tự động

Lệnh này đọc toàn bộ `package.xml` trong `src/` và cài dependencies còn thiếu qua apt:

```bash
cd ~/cnn25ws_dev
rosdep install --from-paths src --ignore-src -r -y
```

---

### Bước 4 — Cài thêm các gói đặc thù của Warthog

Một số dependencies của workspace không được `rosdep` phát hiện tự động:

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs \
                 ros-noetic-gazebo-ros-control \
                 ros-noetic-hector-gazebo-plugins \
                 ros-noetic-controller-manager \
                 ros-noetic-diff-drive-controller \
                 ros-noetic-joint-state-controller \
                 ros-noetic-robot-localization \
                 ros-noetic-twist-mux \
                 ros-noetic-teleop-twist-joy \
                 ros-noetic-interactive-marker-twist-server \
                 ros-noetic-navigation \
                 ros-noetic-slam-gmapping
```

Gói `cpr_onav_description` của Clearpath không có trên apt, cần clone thủ công vào `src/`:

```bash
cd ~/cnn25ws_dev/src
git clone https://github.com/clearpathrobotics/cpr_onav_description
```

---

### Bước 5 — Build workspace

```bash
cd ~/cnn25ws_dev
catkin_make
```

Hoặc dùng `catkin build` (khuyến nghị hơn vì build từng gói độc lập):

```bash
catkin build
```

---

### Tóm tắt thứ tự

| Bước | Lệnh |
|------|-------|
| 1 | Cài ROS Noetic |
| 2 | `rosdep init && rosdep update` |
| 3 | `rosdep install --from-paths src --ignore-src -r -y` |
| 5 | `catkin_make` hoặc `catkin build` |
