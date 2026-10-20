# Chapter 2  
## Kiến trúc ROS

---

## 1. Tổng quan

ROS (Robot Operating System) là một framework cho phép xây dựng hệ thống robot theo mô hình phân tán.

---

## 2. Thành phần chính

### 2.1 Node

Node là đơn vị xử lý độc lập.

---

### 2.2 Topic

Topic là kênh truyền dữ liệu giữa các node.

---

### 2.3 TF

TF quản lý các hệ tọa độ và biến đổi giữa chúng.

---

##  Các bước cần làm trước khi phát triển

1. Cài đặt ROS Noetic (nếu chưa có)

```bash
# Ubuntu 20.04
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

2. Cài đặt dependencies bắt buộc:
```bash
sudo apt install python3-catkin-pkg python3-rosdep python3-rosinstall
sudo rosdep init
rosdep update
```
Trong hệ sinh thái ROS, mỗi gói (package) thường phụ thuộc vào nhiều thư viện khác nhau: từ thư viện hệ thống (C++, Python) cho đến các package ROS khác. Việc cài đặt thủ công từng phụ thuộc này là không thực tế. Đó là lý do công cụ rosdep được sử dụng.

`rosdep` là một công cụ giúp:
- Phân tích file package.xml của các package ROS
- Xác định các thư viện cần thiết
- Tự động cài đặt chúng thông qua hệ quản lý gói của hệ điều hành (ví dụ: apt, pip)

Trước khi sử dụng rosdep, cần thực hiện bước khởi tạo ban đầu:
```bash
sudo rosdep init
```

Lệnh này có vai trò:

- Tạo file cấu hình mặc định tại:
```bash
/etc/ros/rosdep/sources.list.d/20-default.list
```
- Thiết lập nguồn dữ liệu (database) chứa thông tin về các dependency
- Chuẩn bị môi trường để rosdep hoạt động
`Lưu ý:` Lệnh này chỉ cần thực hiện một lần duy nhất trên mỗi hệ thống.

**Cập nhật dữ liệu

Sau khi khởi tạo, cần cập nhật danh sách dependency:
```bash
rosdep update
```
Lệnh này sẽ:
- Tải dữ liệu mới nhất từ server của ROS
- Đồng bộ về máy local
- Đảm bảo các dependency được nhận diện chính xác

*Quy trình sử dụng điển hình

Khi làm việc với một workspace ROS:
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
rosdep sẽ:
- Quét tất cả package trong thư mục src
- Đọc file package.xml
- Tự động cài đặt các thư viện còn thiếu

*Một số lưu ý
- Nếu chạy lại sudo rosdep init và gặp lỗi: 
```bash
default sources list file already exists
````
- Điều này cho thấy hệ thống đã được khởi tạo trước đó.
- rosdep không build code, chỉ cài dependency.
*Tóm tắt
- `sudo rosdep init`:	Khởi tạo hệ thống (chạy 1 lần)
- `rosdep update`:	Cập nhật dữ liệu
- `rosdep install`:	Cài đặt dependency