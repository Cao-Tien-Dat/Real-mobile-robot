![image](https://github.com/user-attachments/assets/f9dde0d6-98b0-4d2c-be16-18c3bf96c243)# Real Mobile Robot

Dự án này xây dựng một robot di động hai bánh sử dụng **ROS 2 Foxy**, **ESP32**, và **SLAM Toolbox** để điều hướng và tạo bản đồ môi trường trong thời gian thực.

## 📌 Mục tiêu
- Điều khiển robot hai bánh thực tế qua UART sử dụng ESP32.
- Đọc encoder và xuất Odometry.
- Tạo bản đồ bằng các thuật toán SLAM: `slam_toolbox`, `gmapping`, và `cartographer`.
- Sử dụng `nav2` để điều hướng trong môi trường.

## ⚙️ Phần cứng sử dụng
- **Jetson Nano B01** (chạy ROS 2 Foxy)
- **ESP32 30 chân** (PlatformIO firmware)
- **Động cơ JGB37-520 có encoder**
- **Driver L298N**
- **Cảm biến RPLIDAR C1**
- **2 hộp pin 18650 12V + module buck 5V cho Jetson**

![Real Robot]
![image](https://github.com/user-attachments/assets/632166e1-1cce-4fc4-a510-a40c349a663b)

![Sơ đồ phần cứng]
![image](https://github.com/user-attachments/assets/07672d40-98bf-4043-8ade-fe9123b17f3b)


> 📌 *Bạn cần thêm các ảnh này vào thư mục `image/` trong repo để hiển thị đúng.*

## 🧩 Thành phần phần mềm chính
- `odometry_publisher/`: Node đọc encoder từ ESP32 qua UART, xuất `Odometry`.
- `robot_description/`: File URDF mô tả cấu trúc robot.
- `launch/`: Gồm 3 file launch:
  - `slam_toolbox_all_bringup.launch.py`
  - `slam_gmapping_all_bringup.launch.py`
  - `slam_cartographer_all_bringup.launch.py`

## 🚀 Cài đặt
```bash
# Cài đặt ROS 2 Foxy trên Ubuntu 20.04
(https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

# Clone repo
git clone https://github.com/Cao-Tien-Dat/Real-mobile-robot.git

# Build
cd ~/colcon_ws
colcon build
source install/setup.bash
