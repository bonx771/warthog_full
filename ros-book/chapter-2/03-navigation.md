
---

# 📄 `chapter-2/02-navigation.md`

```markdown
# Chapter 4  
## Navigation

---

## 1. Tổng quan

Navigation cho phép robot di chuyển từ vị trí hiện tại đến mục tiêu một cách tự động.

---

## 2. Thành phần

- Global planner
- Local planner
- Costmap

---

## 3. Localization

Trước khi navigation, robot cần biết vị trí của mình. Điều này thường được thực hiện bằng AMCL.

---

## 4. Pipeline

Map → Localization → Planning → Control

---

## 5. Triển khai

```bash
roslaunch move_base move_base.launch