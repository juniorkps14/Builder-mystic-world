# Quick Installation Guide

> คู่มือติดตั้งแบบรวดเร็วสำหรับระบบหุ่นยนต์ Dino Core

## ⚡ Quick Start (5 นาที)

### แบบที่ 1: Docker (แนะนำ)

```bash
# Clone และรัน
git clone https://github.com/your-org/dino-core-robot.git
cd dino-core-robot
docker-compose up -d

# เข้าถึงที่ http://localhost:8080
```

### แบบที่ 2: One-liner Script

```bash
curl -sSL https://raw.githubusercontent.com/your-org/dino-core-robot/main/scripts/quick-install.sh | bash
```

## 🔧 Manual Installation

### Ubuntu 20.04 + ROS Noetic

```bash
# 1. ROS Installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# 2. Dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install ros-noetic-navigation ros-noetic-slam-gmapping ros-noetic-move-base -y
pip3 install opencv-python numpy websockets psutil pyserial pyyaml

# 3. Clone และ Build
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/your-org/dino-core-robot.git
cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# 4. Web Interface
cd ~/catkin_ws/src/dino-core-robot/web-interface
npm install && npm run build

# 5. Start System
roslaunch robot_control complete_robot.launch
```

## 🚀 Production Setup

### Raspberry Pi 4 / Jetson Xavier

```bash
# 1. Flash Ubuntu 20.04 Server
# 2. Enable SSH และ Network
sudo systemctl enable ssh
sudo systemctl start ssh

# 3. Run installation script
wget https://raw.githubusercontent.com/your-org/dino-core-robot/main/scripts/install-production.sh
chmod +x install-production.sh
sudo ./install-production.sh

# 4. Configure auto-start
sudo systemctl enable robot-control.service
sudo systemctl start robot-control.service
```

## 🎯 Hardware Setup

### Required Hardware

- **Computing**: Jetson Xavier NX หรือ Raspberry Pi 4 (8GB)
- **LiDAR**: RPLiDAR A1/A2
- **Camera**: USB 3.0 RGB Camera
- **IMU**: 9-DOF MPU9250
- **Motors**: 4x Mecanum wheels with encoders
- **Power**: 24V Battery system

### Connection Diagram

```
Jetson Xavier NX
├── USB 0: Camera
├── USB 1: LiDAR (/dev/ttyUSB0)
├── USB 2: Motor Controller (/dev/ttyACM0)
├── I2C: IMU (Address 0x68)
├── GPIO: Emergency Stop (Pin 18)
└── Ethernet: Network connection
```

## 🔧 Configuration Files

### 1. Robot Configuration

```yaml
# config/robot_config.yaml
robot:
  max_linear_speed: 1.0 # m/s
  max_angular_speed: 2.0 # rad/s
  wheel_base: 0.5 # meters
  wheel_radius: 0.1 # meters

camera:
  device: "/dev/video0"
  width: 640
  height: 480
  fps: 30

lidar:
  port: "/dev/ttyUSB0"
  baudrate: 115200
  frame_id: "laser"

safety:
  min_obstacle_distance: 0.3 # meters
  emergency_stop_pin: 18
```

### 2. Network Configuration

```yaml
# config/network_config.yaml
network:
  static_ip: "192.168.1.100"
  netmask: "255.255.255.0"
  gateway: "192.168.1.1"
  dns: ["8.8.8.8", "8.8.4.4"]

services:
  web_port: 8080
  websocket_port: 9090
  ros_master_port: 11311
```

## 🏃‍♂️ Quick Test

### Test Commands

```bash
# 1. Test ROS connectivity
rostopic list
rosnode list

# 2. Test robot status
rostopic echo /robot_status

# 3. Test camera
rostopic echo /camera/image_raw

# 4. Test movement (CAREFUL!)
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}" -1

# 5. Test emergency stop
rostopic pub /emergency_stop std_msgs/Bool "data: true" -1

# 6. Test web interface
curl http://localhost:8080/api/robot/status
```

### Python API Test

```python
# test_api.py
from robot_api_sdk import RobotAPI

# Connect to robot
robot = RobotAPI("http://localhost:8080")

# Test status
status = robot.get_robot_status()
print(f"Robot state: {status}")

# Test movement (2 seconds forward)
robot.send_velocity(0.2, 0.0, 0.0)
import time; time.sleep(2)
robot.send_velocity(0.0, 0.0, 0.0)

print("API test completed!")
```

## 🔍 Troubleshooting

### Common Issues

#### 1. ROS Master not found

```bash
# Check ROS environment
echo $ROS_MASTER_URI
source /opt/ros/noetic/setup.bash
roscore
```

#### 2. Permission denied for devices

```bash
# Add user to groups
sudo usermod -a -G dialout,video $USER
sudo chmod 666 /dev/ttyUSB0 /dev/ttyACM0 /dev/video0
```

#### 3. Camera not detected

```bash
# List camera devices
ls /dev/video*
v4l2-ctl --list-devices

# Test camera
cheese  # GUI camera test
```

#### 4. Network connection issues

```bash
# Check network status
ip addr show
ping 8.8.8.8

# Restart network
sudo systemctl restart networking
```

#### 5. Port already in use

```bash
# Check ports
sudo netstat -tulpn | grep :8080
sudo netstat -tulpn | grep :9090

# Kill process
sudo pkill -f "port 8080"
```

## 📱 Access Points

### Local Access

- **Web Interface**: http://localhost:8080
- **API Documentation**: http://localhost:8080/api-management
- **System Monitor**: http://localhost:8080/system-monitoring
- **Terminal**: http://localhost:8080/terminal

### Remote Access

- **Robot IP**: http://192.168.1.100:8080
- **SSH**: ssh robot@192.168.1.100
- **VNC** (if enabled): vnc://192.168.1.100:5901

## 🔧 Development Setup

### IDE Configuration

```bash
# VS Code extensions
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-iot.vscode-ros

# ROS development environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
```

### Build Commands

```bash
# Build ROS packages
cd ~/catkin_ws
catkin_make

# Build specific package
catkin_make --only-pkg-with-deps robot_control

# Build web interface
cd web-interface
npm run build

# Build for production
npm run build:prod
```

## 📊 System Status

### Check Services

```bash
# System services
sudo systemctl status robot-control.service
sudo systemctl status nginx
sudo systemctl status postgresql

# ROS nodes
rosnode list
rostopic list

# Docker services
docker-compose ps
docker-compose logs robot-control
```

### Performance Check

```bash
# CPU and Memory
htop
free -h

# Disk usage
df -h

# Network
iftop
netstat -i

# ROS topics bandwidth
rostopic bw /camera/image_raw
rostopic hz /scan
```

## 🆘 Emergency Commands

```bash
# STOP ROBOT IMMEDIATELY
rostopic pub /emergency_stop std_msgs/Bool "data: true" -1

# Kill all ROS processes
pkill -f ros

# Restart robot system
sudo systemctl restart robot-control.service

# Restart all services
docker-compose restart

# Factory reset
sudo rm -rf ~/.ros/log/*
sudo systemctl stop robot-control.service
sudo systemctl start robot-control.service
```

## 📞 Support

### Quick Help

- **Documentation**: [README.md](README.md)
- **API Guide**: [docs/API.md](docs/API.md)
- **Hardware Guide**: [docs/HARDWARE.md](docs/HARDWARE.md)
- **Troubleshooting**: [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

### Contact

- **Issues**: https://github.com/your-org/dino-core-robot/issues
- **Discussions**: https://github.com/your-org/dino-core-robot/discussions
- **Email**: support@robotcompany.com

---

> **Ready to Go!** - Your robot system should be running now at http://localhost:8080
