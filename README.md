# 🤖 Real Mobile Robot
---
![image](https://github.com/user-attachments/assets/dba095c5-e330-4fcc-bf3f-af333f7754f1)

## 🛡️ Giấy phép & Từ chối trách nhiệm

Dự án này được chia sẻ công khai **với mục đích học tập và nghiên cứu**.

Tất cả mã nguồn, bao gồm phần tôi tự viết và các tệp tham khảo từ nguồn mở khác (như SLAM Toolbox, GMapping, Cartographer...), đều **không đi kèm bất kỳ cam kết hay bảo đảm nào**.

> ⚠️ **Tác giả không chịu bất kỳ trách nhiệm nào đối với mọi rủi ro, thiệt hại, mất mát hoặc hậu quả phát sinh từ việc sử dụng mã nguồn trong dự án này, dù là trực tiếp hay gián tiếp.**  
> Việc sử dụng là hoàn toàn tự nguyện và bạn tự chịu trách nhiệm với các hành động của mình.

Bạn được phép sử dụng, chỉnh sửa và chia sẻ lại mã nguồn trong dự án này với điều kiện **không yêu cầu trách nhiệm từ phía tác giả**.

---

Dự án này xây dựng một robot di động hai bánh sử dụng **ROS 2 Foxy**, **ESP32**, và **SLAM Toolbox** để điều hướng và tạo bản đồ môi trường trong thời gian thực.

---

## 📌 Mục tiêu

- Điều khiển robot hai bánh thực tế qua UART sử dụng ESP32.
- Đọc encoder và xuất `Odometry`.
- Tạo bản đồ bằng các thuật toán SLAM: `slam_toolbox`, `gmapping`, `cartographer`.
- Sử dụng `Nav2` để định vị và điều hướng.

---

## ⚙️ Phần cứng sử dụng

- 🧠 **Jetson Nano B01** (chạy Ubuntu 20.04 + ROS 2 Foxy)
- 🔌 **ESP32 DevKit 30 chân** (firmware bằng PlatformIO)
- 🔁 **Động cơ JGB37-520 có encoder**
- 🔺 **Driver L298N**
- 📡 **Cảm biến RPLIDAR C1**
- 🔋 **4 cell pin 18650 (16.8V) + buck chuyển 5V cấp cho Jetson**

### 📷 Hình ảnh robot

![Real Robot](https://github.com/user-attachments/assets/632166e1-1cce-4fc4-a510-a40c349a663b)
![Sơ đồ phần cứng](https://github.com/user-attachments/assets/07672d40-98bf-4043-8ade-fe9123b17f3b)

---

## 🧩 Thành phần phần mềm chính

- `odometry_publisher/`: Node đọc encoder từ ESP32 (UART), xuất bản `nav_msgs/Odometry`.
- `robot_description/`: File URDF mô tả cấu trúc robot (base, bánh xe, lidar...).
- `launch/`: Gồm 3 launch file chính:
  - `slam_toolbox_all_bringup.launch.py`
  - `slam_gmapping_all_bringup.launch.py`
  - `slam_cartographer_all_bringup.launch.py`

---

## 🚀 Cài đặt nhanh

```bash
# Cài đặt ROS 2 Foxy trên Ubuntu 20.04
# (Xem hướng dẫn tại: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

# Tạo workspace và clone mã nguồn
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/Cao-Tien-Dat/Real-mobile-robot.git

# Biên dịch
cd ~/colcon_ws
colcon build
source install/setup.bash
