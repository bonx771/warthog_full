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

## 3. Ví dụ cơ bản

Khởi động ROS master:

```bash
roscore