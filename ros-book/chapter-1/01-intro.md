# Chapter 1  
## Tổng quan hệ thống Warthog

---

## 1. Giới thiệu

Clearpath Warthog là một nền tảng robot di động ngoài trời, được thiết kế cho các ứng dụng nghiên cứu và công nghiệp. Hệ thống hỗ trợ tích hợp nhiều loại cảm biến và có khả năng vận hành trong môi trường phức tạp.

---

## 2. Thành phần phần cứng

Một hệ thống điển hình bao gồm:

- Bộ điều khiển chính (onboard computer)
- Cảm biến:
  - LiDAR
  - IMU
  - GPS (RTK nếu cần độ chính xác cao)
  - Encoder bánh xe
- Bộ truyền động (motor + driver)

---

## 3. Kiến trúc phần mềm

Hệ thống sử dụng ROS làm middleware, trong đó:

- Node: xử lý dữ liệu
- Topic: truyền thông tin giữa các node
- TF: quản lý hệ tọa độ

---

## 4. Luồng dữ liệu cơ bản

Sensor → Processing → State Estimation → Planning → Control

---

## 5. Kết luận

Chương này giới thiệu tổng quan hệ thống. Các chương tiếp theo sẽ đi sâu vào từng thành phần cụ thể.