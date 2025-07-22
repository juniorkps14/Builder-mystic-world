# Dino Core Robot Control System

> **ระบบควบคุมหุ่นยนต์แบบสมบูรณ์** - Production Ready Robot Control System

![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)
![ROS](https://img.shields.io/badge/ROS-Noetic%20%7C%20Humble-green.svg)
![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

## 🎯 ภาพรวมของระบบ

Dino Core เป็นระบบควบคุมหุ่นยนต์แบบครบวงจรที่พัฒนาด้วย **React TypeScript** สำหรับ Web Interface และ **Python ROS** สำหรับการควบคุมฮาร์ดแวร์ ออกแบบมาเพื่อใช้งานจริงในสภาพแวดล้อม Production

### ✨ คุณสมบัติหลัก

- 🤖 **การควบคุมหุ่นยนต์แบบ Holonomic Drive** - เคลื่อนที่ได้ทุกทิศทาง
- 🗺️ **ระบบนำทางอัตโนมัติ** - SLAM, Path Planning, Obstacle Avoidance
- 📹 **การส่งภาพแบบ Real-time** - Web streaming ด้วย WebSocket
- 🛡️ **ระบบความปลอดภัย** - Emergency Stop, Collision Detection
- 📊 **การตรวจสอบระบบ** - Real-time monitoring และ Health Check
- 🎮 **Web Interface** - ควบคุมผ่าน browser ได้ทุกที่
- 🔌 **API ครบ���รัน** - REST API และ Python SDK
- 🐳 **Docker Support** - Deploy ง่ายด้วย containerization

## 🏗️ สถาปัตยกรรมระบบ

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Web Interface │    │  Robot Control  │    │   Hardware      │
│   (React TS)    │◄──►│   (Python ROS)  │◄──►│   (Sensors)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   WebSocket     │    │   ROS Topics    │    │   Serial/USB    │
│   (Port 9090)   │    │   (11311)       │    │   (Hardware)    │
└─────────────────┘    └─────────────────┘    └───────���─────────┘
```

### 🔧 เทคโนโลยีที่ใช้

#### Frontend

- **React 18** with TypeScript
- **Tailwind CSS** + Flat Vector Theme
- **Shadcn/ui** Components
- **React Router** for navigation
- **WebSocket** for real-time communication

#### Backend & Control

- **ROS Noetic/Humble** - Robot Operating System
- **Python 3.8+** - Control logic
- **OpenCV** - Computer vision
- **WebSocket Bridge** - Real-time communication
- **PostgreSQL/Redis** - Data storage & caching

#### Hardware Integration

- **Holonomic Drive System** - 4x Mecanum wheels
- **RPLiDAR A1/A2** - 360° laser scanner
- **RGB Camera** - USB 3.0 vision system
- **9-DOF IMU** - Inertial measurement
- **Jetson Xavier NX** - Computing platform

## 🚀 การติดตั้งและใช้งาน

### ⚡ Quick Start (แนะนำ)

```bash
# Clone repository
git clone https://github.com/your-org/dino-core-robot.git
cd dino-core-robot

# ติดตั้งและเรียกใช้แบบ Docker (ง่ายที่สุด)
docker-compose up -d

# เข้าถึงระบบที่ http://localhost:8080
```

### 🔧 การติดตั้งแบบ Manual

#### 1. ติดตั้ง ROS Environment

```bash
# Ubuntu 20.04 - ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# ตั้งค่า environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ติดตั้ง dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install ros-noetic-navigation ros-noetic-slam-gmapping ros-noetic-move-base -y
sudo apt install ros-noetic-cv-camera ros-noetic-rplidar-ros ros-noetic-joy -y

# ติดตั้ง Python packages
pip3 install opencv-python numpy websockets psutil pyserial pyyaml
```

#### 2. สร้าง ROS Workspace

```bash
# สร้าง catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# Clone robot packages
cd ~/catkin_ws/src
git clone https://github.com/your-org/robot-control.git
git clone https://github.com/your-org/robot-description.git
git clone https://github.com/your-org/robot-navigation.git

# Build packages
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### 3. ติดตั้ง Web Interface

```bash
# ติดตั้ง Node.js และ dependencies
cd web-interface
npm install

# Build production version
npm run build

# หรือเรียกใช้ development server
npm run dev
```

#### 4. การตั้งค่าอัตโนมัติ

```bash
# ตั้งค่า auto-start service
sudo cp scripts/robot-control.service /etc/systemd/system/
sudo systemctl enable robot-control.service
sudo systemctl start robot-control.service

# ตรวจสอบสถานะ
sudo systemctl status robot-control.service
```

## 🎮 การใช้งาน

### 1. เปิดระบบ

```bash
# เรียกใช้ ROS Master
roscore

# เปิดระบบหุ่นยนต์ (terminal ใหม่)
roslaunch robot_control complete_robot.launch

# เปิด Web Interface (terminal ใหม่)
cd web-interface && npm start
```

### 2. เข้าถึง Web Interface

- **Local**: http://localhost:8080
- **Remote**: http://ROBOT_IP:8080

### 3. การควบคุมผ่าน API

```python
from robot_api_sdk import RobotAPI

# เชื่อมต่อกับหุ่นยนต์
robot = RobotAPI("http://localhost:8080")

# ตรวจสอบสถานะ
status = robot.get_robot_status()
print(f"Robot state: {status['state']}")

# เคลื่อนที่หุ่นยนต์
robot.send_velocity(0.5, 0.0, 0.0)  # เดินหน้า 0.5 m/s

# สั่งให้ไปยังตำแหน่งเป้าหมาย
result = robot.navigate_to(5.0, 3.0, 1.57)

# ถ่ายรูป
image = robot.get_camera_image()
image.save("robot_photo.jpg")
```

## 📡 API Reference

### REST API Endpoints

| Method | Endpoint                    | Description               |
| ------ | --------------------------- | ------------------------- |
| `GET`  | `/api/robot/status`         | ดึงสถานะหุ่นยนต์          |
| `POST` | `/api/robot/cmd_vel`        | ส่งคำสั่งการเคลื่อนที่    |
| `POST` | `/api/robot/emergency_stop` | หยุดฉุกเฉิน               |
| `POST` | `/api/navigation/goto`      | สั่งนำทางไปยังตำแหน่ง     |
| `GET`  | `/api/navigation/map`       | ดึงแผนที่ปัจจุบัน         |
| `GET`  | `/api/sensors/camera/image` | ดึงภาพจากกล้อง            |
| `GET`  | `/api/sensors/all`          | ดึงข้อมูลเซ็นเซอร์ทั้งหมด |
| `GET`  | `/api/system/health`        | ตรวจสอบสุขภาพระบบ         |

### WebSocket API

- **Endpoint**: `ws://localhost:9090`
- **Real-time data**: Robot status, camera feed, sensor data
- **Commands**: Velocity control, navigation commands

### Python SDK

```bash
# ดาวน์โหลด SDK
wget https://github.com/your-org/dino-core-robot/releases/latest/download/robot_api_sdk.py

# ติดตั้ง dependencies
pip install requests websockets pillow

# ใช้งาน
from robot_api_sdk import RobotAPI
robot = RobotAPI()
```

## 🔧 การกำหนดค่า

### 1. ไฟล์ Configuration หลัก

```yaml
# config/robot_config.yaml
robot:
  max_linear_speed: 1.0
  max_angular_speed: 2.0
  wheel_base: 0.5
  wheel_radius: 0.1

network:
  ros_master_uri: "http://localhost:11311"
  websocket_port: 9090
  web_port: 8080

safety:
  min_obstacle_distance: 0.3
  emergency_stop_distance: 0.15
  max_battery_temp: 60.0

camera:
  width: 640
  height: 480
  fps: 30
```

### 2. การตั้งค่า Environment Variables

```bash
# .env file
ROS_MASTER_URI=http://localhost:11311
ROS_HOSTNAME=localhost
ROBOT_IP=192.168.1.100
WEB_PORT=8080
WEBSOCKET_PORT=9090
```

### 3. Network Configuration

```bash
# การตั้งค่า Static IP
sudo nano /etc/netplan/01-network-manager-all.yaml

# เพิ่มการกำหนดค่า
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# Apply changes
sudo netplan apply
```

## 🐳 Docker Deployment

### 1. Docker Compose (แนะนำ)

```yaml
# docker-compose.yml
version: "3.8"
services:
  roscore:
    image: ros:noetic-robot
    command: roscore
    ports:
      - "11311:11311"

  robot-control:
    build: .
    depends_on:
      - roscore
    ports:
      - "9090:9090"
      - "8080:8080"
    volumes:
      - ./config:/opt/robot/config
      - /dev:/dev
    privileged: true

  database:
    image: postgres:14
    environment:
      POSTGRES_DB: robotdb
      POSTGRES_USER: robot
      POSTGRES_PASSWORD: robot123
    volumes:
      - postgres_data:/var/lib/postgresql/data

volumes:
  postgres_data:
```

### 2. การใช้งาน Docker

```bash
# Build และรัน
docker-compose up -d

# ดู logs
docker-compose logs -f robot-control

# Restart services
docker-compose restart

# Stop ทั้งหมด
docker-compose down
```

## 📊 การ Monitor และ Debug

### 1. System Health Check

```bash
# ตรวจสอบสถานะ ROS
rostopic list
rosnode list

# ตรว��สอบการเชื่อมต่อ
rostopic echo /robot_status

# ตรวจสอบ system resources
htop
iotop
```

### 2. Log Files

```bash
# ROS logs
~/.ros/log/

# System logs
journalctl -f -u robot-control

# Application logs
/opt/robot/logs/
```

### 3. Web Monitoring

- **System Monitor**: http://localhost:8080/system-monitoring
- **API Management**: http://localhost:8080/api-management
- **Terminal**: http://localhost:8080/terminal

## 🛠️ การแก้ไขปัญหา

### ปัญหาที่พบบ่อย

#### 1. Robot ไม่ตอบสนองคำสั่ง

```bash
# ตรวจสอบ emergency stop
rostopic echo /emergency_stop

# Reset emergency stop
rostopic pub /emergency_stop std_msgs/Bool "data: false"

# ตรวจสอบการเชื่อมต่อฮาร์ดแวร์
ls /dev/ttyUSB* /dev/ttyACM*
```

#### 2. Navigation ไม่ทำงาน

```bash
# ตรวจสอบแผนที่
rostopic echo /map

# ตั้งค่า initial pose
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped ...

# Clear costmaps
rosservice call /move_base/clear_costmaps
```

#### 3. Camera ไม่แสดงภาพ

```bash
# ตรวจสอบ camera device
ls /dev/video*

# เพิ่ม user เข้า video group
sudo usermod -a -G video $USER

# Restart camera node
rosnode kill /camera_node
rosrun cv_camera cv_camera_node
```

#### 4. WebSocket ขาดการเชื่อมต่อ

```bash
# ตรวจสอบ port
netstat -tulpn | grep 9090

# Restart WebSocket bridge
pkill -f websocket_bridge
rosrun robot_control websocket_bridge.py
```

## 🔒 Security & Best Practices

### 1. Network Security

```bash
# ตั้งค่า firewall
sudo ufw enable
sudo ufw allow 22
sudo ufw allow 8080
sudo ufw allow 9090
sudo ufw allow 11311

# ใช้ SSL สำหรับ production
# ดูไฟล์ config/ssl/README.md
```

### 2. User Management

```bash
# สร้าง robot user
sudo adduser robot
sudo usermod -a -G dialout,video,audio robot

# ตั้งค่า sudo permissions
echo "robot ALL=(ALL) NOPASSWD: /usr/bin/systemctl" | sudo tee /etc/sudoers.d/robot
```

### 3. Auto Updates

```bash
# ตั้งค่า auto update script
sudo cp scripts/update-robot.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/update-robot.sh

# เพิ่มใน crontab
echo "0 2 * * 0 /usr/local/bin/update-robot.sh" | sudo crontab -
```

## 📚 Documentation

- **[API Documentation](docs/API.md)** - Complete API reference
- **[Hardware Setup](docs/HARDWARE.md)** - Hardware installation guide
- **[Development Guide](docs/DEVELOPMENT.md)** - Development workflows
- **[Deployment Guide](docs/DEPLOYMENT.md)** - Production deployment
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and solutions

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **GitHub Issues**: [Create an issue](https://github.com/your-org/dino-core-robot/issues)
- **Documentation**: [Wiki](https://github.com/your-org/dino-core-robot/wiki)
- **Community**: [Discord](https://discord.gg/robot-community)

## 🔄 Version History

- **v2.0.0** - Complete system with Web Interface และ API
- **v1.5.0** - Navigation และ SLAM integration
- **v1.0.0** - Basic robot control และ safety systems

---

**Built with ❤️ for robotics community**

> **Ready for Production** - Deploy และใช้งานได้ทันที หรือปรับแต่งตามความต้องการ
