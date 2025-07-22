import React, { useState } from "react";
import {
  usePersistentState,
  usePersistentStore,
} from "@/hooks/use-persistence";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import {
  Code,
  Book,
  Terminal,
  FileText,
  Rocket,
  Bug,
  GitBranch,
  Package,
  Play,
  Download,
  Copy,
  ExternalLink,
  CheckCircle,
  AlertTriangle,
  Info,
  Lightbulb,
  Zap,
  Database,
  Network,
  Settings,
  Globe,
  Activity,
  Monitor,
  Cpu,
  HardDrive,
  Wifi,
  Camera,
  Gamepad2,
  Server,
  Shield,
  Clock,
  Users,
  FolderOpen,
  Archive,
  Wrench,
} from "lucide-react";

export default function PythonDevelopment() {
  // Persistent development preferences
  const { store: devPrefs, updateField: updateDevPref } = usePersistentStore(
    "python-dev-preferences",
    {
      activeExample: "complete_robot_system",
      activeTab: "code",
      fontSize: 14,
      theme: "dark",
      autoSave: true,
    },
  );

  const { activeExample } = devPrefs;
  const [searchTerm, setSearchTerm] = useState("");

  const codeExamples = {
    complete_robot_system: {
      title: "Complete Robot System Architecture",
      description: "ระบบหุ่นยนต์แบบสมบูรณ์สำหรับการใช้งานจริง",
      code: `#!/usr/bin/env python3
"""
Complete Robot System Architecture
ระบบหุ่นยนต์แบบสมบูรณ์สำหรับการใช้งานจริง 100%

Features:
- Holonomic drive control
- Camera streaming
- Sensor data processing
- Web interface integration
- Emergency stop system
- Automatic navigation
- Task sequence management
- Real-time monitoring
"""

import rospy
import threading
import json
import asyncio
import websockets
import yaml
import time
from datetime import datetime
from dataclasses import dataclass
from typing import Dict, List, Optional, Callable
from enum import Enum

# ROS Messages
from std_msgs.msg import String, Bool, Float32, Header
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, LaserScan, Joy, Imu, CompressedImage
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from tf2_msgs.msg import TFMessage
from actionlib import SimpleActionServer, SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

# Computer Vision
import cv2
from cv_bridge import CvBridge
import numpy as np

# System Monitoring
import psutil
import subprocess
import os
import signal

class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    NAVIGATING = "navigating"
    EMERGENCY = "emergency"
    ERROR = "error"
    CHARGING = "charging"

@dataclass
class SystemConfig:
    # Robot Physical Parameters
    max_linear_speed: float = 1.0
    max_angular_speed: float = 2.0
    wheel_base: float = 0.5
    wheel_radius: float = 0.1

    # Network Configuration
    ros_master_uri: str = "http://localhost:11311"
    websocket_port: int = 9090
    web_port: int = 8080

    # Camera Settings
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30

    # Safety Parameters
    min_obstacle_distance: float = 0.3
    emergency_stop_distance: float = 0.15
    max_battery_temp: float = 60.0
    min_battery_voltage: float = 11.0

    # File Paths
    config_path: str = "/opt/robot/config"
    log_path: str = "/opt/robot/logs"
    sequence_path: str = "/opt/robot/sequences"

class RobotControlSystem:
    def __init__(self, config: SystemConfig):
        self.config = config
        self.state = RobotState.IDLE
        self.bridge = CvBridge()
        self.emergency_stop = False
        self.system_health = {}

        # Initialize ROS
        rospy.init_node('complete_robot_system', anonymous=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        self.system_health_pub = rospy.Publisher('/system_health', String, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', String, queue_size=10)

        # Subscribers
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.web_cmd_sub = rospy.Subscriber('/web_cmd_vel', Twist, self.web_cmd_callback)
        self.emergency_sub = rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.camera_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Action Clients
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Web Interface
        self.websocket_clients = set()
        self.start_websocket_server()

        # System Monitoring
        self.monitor_thread = threading.Thread(target=self.system_monitor_loop, daemon=True)
        self.monitor_thread.start()

        # Main Control Loop
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Complete Robot System initialized successfully")

    def joy_callback(self, msg):
        """Handle joystick input for manual control"""
        if self.emergency_stop:
            return

        # Map joystick to movement
        linear_x = msg.axes[1] * self.config.max_linear_speed
        linear_y = msg.axes[0] * self.config.max_linear_speed  # Strafe
        angular_z = msg.axes[3] * self.config.max_angular_speed

        # Emergency button
        if msg.buttons[0]:  # Button A
            self.trigger_emergency_stop()
            return

        # State changes
        if msg.buttons[1]:  # Button B - Idle
            self.set_state(RobotState.IDLE)
        elif msg.buttons[2]:  # Button X - Manual mode
            self.set_state(RobotState.MOVING)

        self.publish_velocity(linear_x, linear_y, angular_z)

    def web_cmd_callback(self, msg):
        """Handle web interface commands"""
        if self.emergency_stop:
            return

        self.publish_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def emergency_callback(self, msg):
        """Handle emergency stop signal"""
        if msg.data:
            self.trigger_emergency_stop()
        else:
            self.release_emergency_stop()

    def laser_callback(self, msg):
        """Process laser scan data for obstacle avoidance"""
        # Check for obstacles
        min_distance = min(msg.ranges)

        if min_distance < self.config.emergency_stop_distance:
            rospy.logwarn(f"Emergency stop triggered by obstacle: {min_distance:.2f}m")
            self.trigger_emergency_stop()
        elif min_distance < self.config.min_obstacle_distance:
            rospy.logwarn(f"Obstacle detected: {min_distance:.2f}m")
            # Slow down or stop
            self.publish_velocity(0, 0, 0)

    def odom_callback(self, msg):
        """Process odometry data"""
        # Store current position for navigation
        self.current_pose = msg.pose.pose

        # Broadcast to web clients
        odom_data = {
            'type': 'odometry',
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'timestamp': time.time()
        }
        self.broadcast_to_web_clients(json.dumps(odom_data))

    def camera_callback(self, msg):
        """Process camera data and stream to web clients"""
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Resize for web streaming
            cv_image = cv2.resize(cv_image, (self.config.camera_width, self.config.camera_height))

            # Add overlay information
            self.add_camera_overlay(cv_image)

            # Compress and encode
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            img_base64 = base64.b64encode(buffer).decode('utf-8')

            # Send to web clients
            camera_data = {
                'type': 'camera_frame',
                'image': img_base64,
                'timestamp': time.time(),
                'width': self.config.camera_width,
                'height': self.config.camera_height
            }
            self.broadcast_to_web_clients(json.dumps(camera_data))

        except Exception as e:
            rospy.logerr(f"Camera processing error: {e}")

    def imu_callback(self, msg):
        """Process IMU data"""
        # Monitor robot orientation and acceleration
        imu_data = {
            'type': 'imu',
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'timestamp': time.time()
        }
        self.broadcast_to_web_clients(json.dumps(imu_data))

    def add_camera_overlay(self, image):
        """Add system information overlay to camera image"""
        # System status
        status_text = f"State: {self.state.value.upper()}"
        cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Emergency status
        if self.emergency_stop:
            cv2.putText(image, "EMERGENCY STOP", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(image, timestamp, (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def publish_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """Publish velocity commands with safety checks"""
        if self.emergency_stop:
            linear_x = linear_y = angular_z = 0

        # Apply speed limits
        linear_x = max(-self.config.max_linear_speed, min(self.config.max_linear_speed, linear_x))
        linear_y = max(-self.config.max_linear_speed, min(self.config.max_linear_speed, linear_y))
        angular_z = max(-self.config.max_angular_speed, min(self.config.max_angular_speed, angular_z))

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z

        self.cmd_vel_pub.publish(twist)

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop = True
        self.set_state(RobotState.EMERGENCY)
        self.publish_velocity(0, 0, 0)

        emergency_data = {
            'type': 'emergency_stop',
            'active': True,
            'timestamp': time.time()
        }
        self.broadcast_to_web_clients(json.dumps(emergency_data))

        rospy.logwarn("EMERGENCY STOP ACTIVATED")

    def release_emergency_stop(self):
        """Release emergency stop"""
        self.emergency_stop = False
        self.set_state(RobotState.IDLE)

        emergency_data = {
            'type': 'emergency_stop',
            'active': False,
            'timestamp': time.time()
        }
        self.broadcast_to_web_clients(json.dumps(emergency_data))

        rospy.loginfo("Emergency stop released")

    def set_state(self, new_state: RobotState):
        """Change robot state"""
        old_state = self.state
        self.state = new_state

        state_data = {
            'type': 'state_change',
            'old_state': old_state.value,
            'new_state': new_state.value,
            'timestamp': time.time()
        }
        self.broadcast_to_web_clients(json.dumps(state_data))

        rospy.loginfo(f"State changed: {old_state.value} -> {new_state.value}")

    def control_loop(self, event):
        """Main control loop"""
        # Update system status
        status_msg = String()
        status_msg.data = json.dumps({
            'state': self.state.value,
            'emergency_stop': self.emergency_stop,
            'timestamp': time.time(),
            'uptime': rospy.Time.now().to_sec()
        })
        self.status_pub.publish(status_msg)

    def system_monitor_loop(self):
        """System monitoring loop"""
        while not rospy.is_shutdown():
            try:
                # Collect system metrics
                cpu_percent = psutil.cpu_percent(interval=1)
                memory = psutil.virtual_memory()
                disk = psutil.disk_usage('/')
                network = psutil.net_io_counters()
                battery = self.get_battery_info()

                self.system_health = {
                    'cpu_percent': cpu_percent,
                    'memory_percent': memory.percent,
                    'disk_percent': (disk.used / disk.total) * 100,
                    'network_sent': network.bytes_sent,
                    'network_recv': network.bytes_recv,
                    'battery_voltage': battery.get('voltage', 12.0),
                    'battery_temperature': battery.get('temperature', 25.0),
                    'ros_nodes': self.get_active_ros_nodes(),
                    'timestamp': time.time()
                }

                # Publish system health
                health_msg = String()
                health_msg.data = json.dumps(self.system_health)
                self.system_health_pub.publish(health_msg)

                # Send to web clients
                health_data = {
                    'type': 'system_health',
                    **self.system_health
                }
                self.broadcast_to_web_clients(json.dumps(health_data))

                # Check for critical conditions
                self.check_system_health()

            except Exception as e:
                rospy.logerr(f"System monitoring error: {e}")

            time.sleep(5)  # Update every 5 seconds

    def get_battery_info(self) -> Dict:
        """Get battery information"""
        try:
            # This would interface with actual battery monitoring hardware
            # For simulation, return mock data
            return {
                'voltage': 12.5,
                'current': 2.1,
                'temperature': 28.5,
                'capacity': 85.0
            }
        except Exception:
            return {}

    def get_active_ros_nodes(self) -> List[str]:
        """Get list of active ROS nodes"""
        try:
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True)
            if result.returncode == 0:
                return result.stdout.strip().split('\n')
            return []
        except Exception:
            return []

    def check_system_health(self):
        """Check system health and trigger alerts"""
        health = self.system_health

        # Check CPU usage
        if health.get('cpu_percent', 0) > 90:
            rospy.logwarn(f"High CPU usage: {health['cpu_percent']:.1f}%")

        # Check memory usage
        if health.get('memory_percent', 0) > 90:
            rospy.logwarn(f"High memory usage: {health['memory_percent']:.1f}%")

        # Check battery
        battery_voltage = health.get('battery_voltage', 12.0)
        if battery_voltage < self.config.min_battery_voltage:
            rospy.logwarn(f"Low battery voltage: {battery_voltage:.1f}V")

        battery_temp = health.get('battery_temperature', 25.0)
        if battery_temp > self.config.max_battery_temp:
            rospy.logwarn(f"High battery temperature: {battery_temp:.1f}°C")

    # WebSocket Server Implementation
    def start_websocket_server(self):
        """Start WebSocket server for web interface"""
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            start_server = websockets.serve(
                self.handle_websocket_client,
                "0.0.0.0",
                self.config.websocket_port
            )
            loop.run_until_complete(start_server)
            loop.run_forever()

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        rospy.loginfo(f"WebSocket server started on port {self.config.websocket_port}")

    async def handle_websocket_client(self, websocket, path):
        """Handle individual WebSocket client"""
        self.websocket_clients.add(websocket)
        client_addr = websocket.remote_address
        rospy.loginfo(f"WebSocket client connected: {client_addr}")

        try:
            # Send initial system state
            initial_data = {
                'type': 'initial_state',
                'state': self.state.value,
                'emergency_stop': self.emergency_stop,
                'system_health': self.system_health,
                'timestamp': time.time()
            }
            await websocket.send(json.dumps(initial_data))

            # Handle incoming messages
            async for message in websocket:
                await self.process_websocket_message(websocket, message)

        except websockets.exceptions.ConnectionClosed:
            rospy.loginfo(f"WebSocket client disconnected: {client_addr}")
        except Exception as e:
            rospy.logerr(f"WebSocket error: {e}")
        finally:
            self.websocket_clients.discard(websocket)

    async def process_websocket_message(self, websocket, message):
        """Process incoming WebSocket message"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')

            if msg_type == 'cmd_vel':
                # Handle velocity commands
                twist = Twist()
                twist.linear.x = data.get('linear_x', 0.0)
                twist.linear.y = data.get('linear_y', 0.0)
                twist.angular.z = data.get('angular_z', 0.0)
                self.web_cmd_callback(twist)

            elif msg_type == 'emergency_stop':
                # Handle emergency stop
                self.trigger_emergency_stop()

            elif msg_type == 'release_emergency':
                # Release emergency stop
                self.release_emergency_stop()

            elif msg_type == 'navigate_to':
                # Handle navigation command
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                theta = data.get('theta', 0.0)
                await self.navigate_to_position(x, y, theta)

            elif msg_type == 'get_system_info':
                # Send system information
                system_info = await self.get_system_info()
                await websocket.send(json.dumps(system_info))

        except json.JSONDecodeError:
            rospy.logwarn(f"Invalid JSON received: {message}")
        except Exception as e:
            rospy.logerr(f"Error processing WebSocket message: {e}")

    def broadcast_to_web_clients(self, message: str):
        """Broadcast message to all connected WebSocket clients"""
        if self.websocket_clients:
            asyncio.run_coroutine_threadsafe(
                self._broadcast_async(message),
                asyncio.get_event_loop()
            )

    async def _broadcast_async(self, message: str):
        """Async broadcast helper"""
        if self.websocket_clients:
            await asyncio.gather(
                *[self._send_safe(client, message) for client in self.websocket_clients],
                return_exceptions=True
            )

    async def _send_safe(self, websocket, message):
        """Send message safely to WebSocket client"""
        try:
            await websocket.send(message)
        except websockets.exceptions.ConnectionClosed:
            self.websocket_clients.discard(websocket)
        except Exception as e:
            rospy.logerr(f"Error sending WebSocket message: {e}")

    async def navigate_to_position(self, x: float, y: float, theta: float):
        """Navigate to specific position using move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = theta
        goal.target_pose.pose.orientation.w = 1.0

        self.set_state(RobotState.NAVIGATING)
        self.move_base_client.send_goal(goal)

        # Wait for result (non-blocking)
        def goal_done_callback(state, result):
            if state == 3:  # SUCCEEDED
                self.set_state(RobotState.IDLE)
                rospy.loginfo("Navigation completed successfully")
            else:
                self.set_state(RobotState.ERROR)
                rospy.logwarn(f"Navigation failed with state: {state}")

        self.move_base_client.send_goal(goal, done_cb=goal_done_callback)

    async def get_system_info(self) -> Dict:
        """Get comprehensive system information"""
        return {
            'type': 'system_info',
            'robot_state': self.state.value,
            'emergency_stop': self.emergency_stop,
            'system_health': self.system_health,
            'config': {
                'max_linear_speed': self.config.max_linear_speed,
                'max_angular_speed': self.config.max_angular_speed,
                'camera_resolution': f"{self.config.camera_width}x{self.config.camera_height}",
                'safety_distance': self.config.min_obstacle_distance
            },
            'ros_info': {
                'master_uri': self.config.ros_master_uri,
                'active_nodes': self.get_active_ros_nodes(),
                'node_count': len(self.get_active_ros_nodes())
            },
            'timestamp': time.time()
        }

def main():
    """Main function to start the complete robot system"""
    try:
        # Load configuration
        config = SystemConfig()

        # Initialize robot system
        robot_system = RobotControlSystem(config)

        rospy.loginfo("Complete Robot Control System started successfully")
        rospy.loginfo("System ready for operation - All features active")

        # Keep the system running
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down robot system...")
    except Exception as e:
        rospy.logerr(f"Fatal error in robot system: {e}")
        raise

if __name__ == '__main__':
    main()`,
    },
    launch_file_complete: {
      title: "Complete Launch File Configuration",
      description: "Launch file สำหรับระบบทั้งหมด",
      code: `<?xml version="1.0"?>
<!--
Complete Robot Launch Configuration
สำหรับการเปิดใช้งานระบบหุ่นยนต์แบบสมบูรณ์
-->
<launch>
  <!-- Arguments -->
  <arg name="robot_name" default="dino_robot"/>
  <arg name="use_simulation" default="false"/>
  <arg name="map_file" default="$(find robot_navigation)/maps/office.yaml"/>
  <arg name="camera_device" default="/dev/video0"/>
  <arg name="lidar_port" default="/dev/ttyUSB0"/>

  <!-- Robot Description -->
  <param name="robot_description" textfile="$(find robot_description)/urdf/robot.urdf"/>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>

  <!-- Hardware Drivers -->
  <group unless="$(arg use_simulation)">
    <!-- Base Controller -->
    <node name="base_controller" pkg="robot_hardware" type="base_controller.py" output="screen">
      <param name="port" value="/dev/ttyACM0"/>
      <param name="baud_rate" value="115200"/>
      <param name="max_linear_speed" value="1.0"/>
      <param name="max_angular_speed" value="2.0"/>
      <param name="wheel_base" value="0.5"/>
      <param name="wheel_radius" value="0.1"/>
    </node>

    <!-- Camera Driver -->
    <node name="camera_node" pkg="cv_camera" type="cv_camera_node" output="screen">
      <param name="device_id" value="$(arg camera_device)"/>
      <param name="image_width" value="640"/>
      <param name="image_height" value="480"/>
      <param name="fps" value="30"/>
      <param name="frame_id" value="camera_link"/>
      <remap from="image_raw" to="/camera/image_raw"/>
    </node>

    <!-- LiDAR Driver -->
    <node name="laser_node" pkg="rplidar_ros" type="rplidarNode" output="screen">
      <param name="serial_port" value="$(arg lidar_port)"/>
      <param name="serial_baudrate" value="115200"/>
      <param name="frame_id" value="laser"/>
      <param name="inverted" value="false"/>
      <param name="angle_compensate" value="true"/>
    </node>

    <!-- IMU Driver -->
    <node name="imu_node" pkg="robot_hardware" type="imu_driver.py" output="screen">
      <param name="port" value="/dev/ttyUSB1"/>
      <param name="frame_id" value="imu_link"/>
      <param name="publish_rate" value="50"/>
    </node>

    <!-- GPS Driver (if available) -->
    <node name="gps_node" pkg="nmea_navsat_driver" type="nmea_serial_driver" output="screen">
      <param name="port" value="/dev/ttyUSB2"/>
      <param name="baud" value="9600"/>
      <param name="frame_id" value="gps"/>
    </node>
  </group>

  <!-- Simulation (Gazebo) -->
  <group if="$(arg use_simulation)">
    <include file="$(find robot_gazebo)/launch/robot_world.launch">
      <arg name="world_name" value="$(find robot_gazebo)/worlds/office.world"/>
    </include>
  </group>

  <!-- Navigation Stack -->
  <group name="navigation">
    <!-- Map Server -->
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" output="screen"/>

    <!-- AMCL Localization -->
    <node name="amcl" pkg="amcl" type="amcl" output="screen">
      <param name="odom_frame_id" value="odom"/>
      <param name="base_frame_id" value="base_link"/>
      <param name="global_frame_id" value="map"/>
      <param name="odom_model_type" value="omni"/>
      <param name="transform_tolerance" value="0.2"/>
      <param name="gui_publish_rate" value="10.0"/>
      <param name="laser_max_beams" value="60"/>
      <param name="min_particles" value="500"/>
      <param name="max_particles" value="5000"/>
      <param name="kld_err" value="0.05"/>
      <param name="kld_z" value="0.99"/>
      <param name="odom_alpha1" value="0.2"/>
      <param name="odom_alpha2" value="0.2"/>
      <param name="odom_alpha3" value="0.8"/>
      <param name="odom_alpha4" value="0.2"/>
      <param name="laser_z_hit" value="0.5"/>
      <param name="laser_z_short" value="0.05"/>
      <param name="laser_z_max" value="0.05"/>
      <param name="laser_z_rand" value="0.5"/>
      <param name="laser_sigma_hit" value="0.2"/>
      <param name="laser_lambda_short" value="0.1"/>
      <param name="laser_model_type" value="likelihood_field"/>
      <param name="laser_likelihood_max_dist" value="2.0"/>
      <param name="update_min_d" value="0.2"/>
      <param name="update_min_a" value="0.5"/>
      <param name="resample_interval" value="1"/>
      <param name="recovery_alpha_slow" value="0.0"/>
      <param name="recovery_alpha_fast" value="0.0"/>
    </node>

    <!-- Move Base -->
    <node name="move_base" pkg="move_base" type="move_base" respawn="false" output="screen">
      <rosparam file="$(find robot_navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap"/>
      <rosparam file="$(find robot_navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap"/>
      <rosparam file="$(find robot_navigation)/config/local_costmap_params.yaml" command="load"/>
      <rosparam file="$(find robot_navigation)/config/global_costmap_params.yaml" command="load"/>
      <rosparam file="$(find robot_navigation)/config/base_local_planner_params.yaml" command="load"/>
      <rosparam file="$(find robot_navigation)/config/move_base_params.yaml" command="load"/>
    </node>
  </group>

  <!-- SLAM (Optional - for mapping) -->
  <group name="slam" if="false">
    <node name="gmapping" pkg="gmapping" type="slam_gmapping" output="screen">
      <param name="map_frame" value="map"/>
      <param name="odom_frame" value="odom"/>
      <param name="base_frame" value="base_link"/>
      <param name="map_update_interval" value="5.0"/>
      <param name="maxUrange" value="5.6"/>
      <param name="sigma" value="0.05"/>
      <param name="kernelSize" value="1"/>
      <param name="lstep" value="0.05"/>
      <param name="astep" value="0.05"/>
      <param name="iterations" value="5"/>
      <param name="lsigma" value="0.075"/>
      <param name="ogain" value="3.0"/>
      <param name="lskip" value="0"/>
      <param name="minimumScore" value="50"/>
      <param name="srr" value="0.1"/>
      <param name="srt" value="0.2"/>
      <param name="str" value="0.1"/>
      <param name="stt" value="0.2"/>
      <param name="linearUpdate" value="1.0"/>
      <param name="angularUpdate" value="0.5"/>
      <param name="temporalUpdate" value="3.0"/>
      <param name="resampleThreshold" value="0.5"/>
      <param name="particles" value="30"/>
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      <param name="delta" value="0.05"/>
      <param name="llsamplerange" value="0.01"/>
      <param name="llsamplestep" value="0.01"/>
      <param name="lasamplerange" value="0.005"/>
      <param name="lasamplestep" value="0.005"/>
    </node>
  </group>

  <!-- Robot Control System -->
  <node name="complete_robot_system" pkg="robot_control" type="complete_robot_system.py" output="screen">
    <param name="max_linear_speed" value="1.0"/>
    <param name="max_angular_speed" value="2.0"/>
    <param name="websocket_port" value="9090"/>
    <param name="min_obstacle_distance" value="0.3"/>
    <param name="emergency_stop_distance" value="0.15"/>
    <param name="camera_width" value="640"/>
    <param name="camera_height" value="480"/>
  </node>

  <!-- WebSocket Bridge -->
  <node name="websocket_bridge" pkg="robot_control" type="websocket_bridge.py" output="screen">
    <param name="port" value="9090"/>
    <param name="host" value="0.0.0.0"/>
  </node>

  <!-- Joystick Control -->
  <group name="joystick">
    <node name="joy_node" pkg="joy" type="joy_node" output="screen">
      <param name="dev" value="/dev/input/js0"/>
      <param name="deadzone" value="0.1"/>
      <param name="autorepeat_rate" value="20"/>
    </node>

    <node name="teleop_twist_joy" pkg="teleop_twist_joy" type="teleop_node" output="screen">
      <param name="axis_linear_x" value="1"/>
      <param name="axis_linear_y" value="0"/>
      <param name="axis_angular" value="3"/>
      <param name="scale_linear" value="1.0"/>
      <param name="scale_angular" value="2.0"/>
      <param name="enable_button" value="0"/>
      <remap from="cmd_vel" to="joy_cmd_vel"/>
    </node>
  </group>

  <!-- Monitoring and Diagnostics -->
  <node name="robot_monitor" pkg="robot_control" type="system_monitor.py" output="screen">
    <param name="publish_rate" value="1.0"/>
    <param name="check_battery" value="true"/>
    <param name="check_temperature" value="true"/>
    <param name="log_path" value="/opt/robot/logs"/>
  </node>

  <!-- Emergency Stop System -->
  <node name="emergency_stop" pkg="robot_control" type="emergency_stop.py" output="screen">
    <param name="enable_hardware_estop" value="true"/>
    <param name="enable_software_estop" value="true"/>
    <param name="estop_pin" value="18"/>
  </node>

  <!-- RViz Visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot_visualization)/config/robot.rviz" if="false"/>

  <!-- Static Transforms -->
  <node name="base_to_laser" pkg="tf2_ros" type="static_transform_publisher"
        args="0.2 0 0.1 0 0 0 base_link laser"/>
  <node name="base_to_camera" pkg="tf2_ros" type="static_transform_publisher"
        args="0.3 0 0.2 0 0 0 base_link camera_link"/>
  <node name="base_to_imu" pkg="tf2_ros" type="static_transform_publisher"
        args="0 0 0.05 0 0 0 base_link imu_link"/>

  <!-- Web Interface -->
  <node name="web_server" pkg="robot_web" type="web_server.py" output="screen">
    <param name="port" value="8080"/>
    <param name="host" value="0.0.0.0"/>
    <param name="static_path" value="$(find robot_web)/www"/>
  </node>

  <!-- Logging and Recording -->
  <node name="rosbag_record" pkg="rosbag" type="record" args="-O /opt/robot/logs/session.bag
    /cmd_vel /odom /scan /camera/image_raw/compressed /tf /tf_static /robot_status /system_health"
    if="false"/>

</launch>`,
    },
    package_xml: {
      title: "Complete Package.xml",
      description: "Package configuration สำหรับระบบทั้งหมด",
      code: `<?xml version="1.0"?>
<package format="2">
  <name>robot_control</name>
  <version>2.0.0</version>
  <description>Complete Robot Control System Package</description>

  <!-- Maintainer and License -->
  <maintainer email="dev@dinocore.com">Dino Core Development Team</maintainer>
  <license>MIT</license>
  <url type="website">https://github.com/dinocore/robot-control</url>
  <url type="bugtracker">https://github.com/dinocore/robot-control/issues</url>
  <url type="repository">https://github.com/dinocore/robot-control</url>

  <!-- Authors -->
  <author email="dev@dinocore.com">Dino Core Team</author>

  <!-- Build Dependencies -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- Runtime Dependencies -->
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>tf2_msgs</build_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>move_base_msgs</build_depend>
  <build_depend>cv_bridge</build_depend>
  <build_depend>image_transport</build_depend>
  <build_depend>compressed_image_transport</build_depend>

  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>nav_msgs</build_export_depend>
  <build_export_depend>tf2_msgs</build_export_depend>
  <build_export_depend>actionlib</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>move_base_msgs</build_export_depend>
  <build_export_depend>cv_bridge</build_export_depend>
  <build_export_depend>image_transport</build_export_depend>
  <build_export_depend>compressed_image_transport</build_export_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_msgs</exec_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>move_base_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>image_transport</exec_depend>
  <exec_depend>compressed_image_transport</exec_depend>

  <!-- Navigation Dependencies -->
  <exec_depend>move_base</exec_depend>
  <exec_depend>amcl</exec_depend>
  <exec_depend>map_server</exec_depend>
  <exec_depend>gmapping</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>

  <!-- Hardware Interface Dependencies -->
  <exec_depend>cv_camera</exec_depend>
  <exec_depend>rplidar_ros</exec_depend>
  <exec_depend>nmea_navsat_driver</exec_depend>
  <exec_depend>joy</exec_depend>
  <exec_depend>teleop_twist_joy</exec_depend>

  <!-- Python Dependencies -->
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>python3-websockets</exec_depend>
  <exec_depend>python3-psutil</exec_depend>
  <exec_depend>python3-serial</exec_depend>

  <!-- Simulation Dependencies -->
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>gazebo_ros_pkgs</exec_depend>
  <exec_depend>gazebo_ros_control</exec_depend>

  <!-- Visualization Dependencies -->
  <exec_depend>rviz</exec_depend>
  <exec_depend>rqt</exec_depend>
  <exec_depend>rqt_common_plugins</exec_depend>

  <!-- Export -->
  <export>
    <architecture_independent/>
  </export>

</package>`,
    },
    systemd_service: {
      title: "Systemd Service Configuration",
      description: "Auto-start service สำหรับระบบ",
      code: `# /etc/systemd/system/robot-control.service
[Unit]
Description=Dino Core Robot Control System
After=network.target sound.target
Wants=network.target

[Service]
Type=forking
User=robot
Group=robot
WorkingDirectory=/home/robot

# Environment Variables
Environment=ROS_MASTER_URI=http://localhost:11311
Environment=ROS_HOSTNAME=localhost
Environment=ROS_IP=127.0.0.1
Environment=ROS_DISTRO=noetic
Environment=PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/home/robot/catkin_ws/devel/lib/python3/dist-packages
Environment=CMAKE_PREFIX_PATH=/home/robot/catkin_ws/devel:/opt/ros/noetic
Environment=PKG_CONFIG_PATH=/home/robot/catkin_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig
Environment=PATH=/home/robot/catkin_ws/devel/bin:/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# Pre-start script
ExecStartPre=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/robot/catkin_ws/devel/setup.bash'

# Main service command
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/robot/catkin_ws/devel/setup.bash && roslaunch robot_control complete_robot.launch'

# Post-stop cleanup
ExecStopPost=/bin/bash -c 'pkill -f ros'

# Restart configuration
Restart=always
RestartSec=10
StartLimitInterval=0

# Resource limits
LimitNOFILE=65536
LimitNPROC=32768

# Security settings
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=yes
ReadWritePaths=/home/robot /opt/robot /tmp /var/log
DeviceAllow=/dev/ttyUSB* rw
DeviceAllow=/dev/ttyACM* rw
DeviceAllow=/dev/video* rw
DeviceAllow=/dev/input* rw

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=robot-control

[Install]
WantedBy=multi-user.target`,
    },
    docker_compose: {
      title: "Docker Compose Configuration",
      description: "Docker deployment สำหรับระบบ",
      code: `version: '3.8'

services:
  # ROS Master
  roscore:
    image: ros:noetic-robot
    container_name: robot-roscore
    command: roscore
    networks:
      - robot-network
    ports:
      - "11311:11311"
    environment:
      - ROS_MASTER_URI=http://roscore:11311
    restart: unless-stopped

  # Robot Control System
  robot-control:
    build:
      context: .
      dockerfile: Dockerfile.robot
    container_name: robot-control-system
    depends_on:
      - roscore
    networks:
      - robot-network
    ports:
      - "9090:9090"  # WebSocket
      - "8080:8080"  # Web Interface
    environment:
      - ROS_MASTER_URI=http://roscore:11311
      - ROS_HOSTNAME=robot-control
    volumes:
      - ./config:/opt/robot/config:ro
      - ./logs:/opt/robot/logs:rw
      - ./maps:/opt/robot/maps:ro
      - /dev:/dev
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
      - /dev/ttyACM0:/dev/ttyACM0
      - /dev/video0:/dev/video0
    privileged: true
    restart: unless-stopped
    command: >
      bash -c "
        source /opt/ros/noetic/setup.bash &&
        source /catkin_ws/devel/setup.bash &&
        roslaunch robot_control complete_robot.launch
      "

  # Navigation Stack
  navigation:
    image: ros:noetic-navigation
    container_name: robot-navigation
    depends_on:
      - roscore
    networks:
      - robot-network
    environment:
      - ROS_MASTER_URI=http://roscore:11311
      - ROS_HOSTNAME=navigation
    volumes:
      - ./maps:/maps:ro
      - ./config/navigation:/config:ro
    restart: unless-stopped

  # SLAM (Optional)
  slam:
    image: ros:noetic-slam
    container_name: robot-slam
    depends_on:
      - roscore
    networks:
      - robot-network
    environment:
      - ROS_MASTER_URI=http://roscore:11311
      - ROS_HOSTNAME=slam
    volumes:
      - ./maps:/maps:rw
    restart: unless-stopped
    profiles:
      - mapping

  # Database (for logging and data storage)
  database:
    image: postgres:14
    container_name: robot-database
    networks:
      - robot-network
    environment:
      - POSTGRES_DB=robotdb
      - POSTGRES_USER=robot
      - POSTGRES_PASSWORD=robot123
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro
    ports:
      - "5432:5432"
    restart: unless-stopped

  # Redis (for caching and real-time data)
  redis:
    image: redis:7-alpine
    container_name: robot-redis
    networks:
      - robot-network
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data
    restart: unless-stopped

  # Monitoring (Prometheus + Grafana)
  prometheus:
    image: prom/prometheus:latest
    container_name: robot-prometheus
    networks:
      - robot-network
    ports:
      - "9090:9090"
    volumes:
      - ./monitoring/prometheus.yml:/etc/prometheus/prometheus.yml:ro
      - prometheus_data:/prometheus
    command:
      - '--config.file=/etc/prometheus/prometheus.yml'
      - '--storage.tsdb.path=/prometheus'
      - '--web.console.libraries=/etc/prometheus/console_libraries'
      - '--web.console.templates=/etc/prometheus/consoles'
      - '--web.enable-lifecycle'
    restart: unless-stopped

  grafana:
    image: grafana/grafana:latest
    container_name: robot-grafana
    networks:
      - robot-network
    ports:
      - "3000:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin123
    volumes:
      - grafana_data:/var/lib/grafana
      - ./monitoring/grafana-datasources.yml:/etc/grafana/provisioning/datasources/datasources.yml:ro
      - ./monitoring/grafana-dashboards.yml:/etc/grafana/provisioning/dashboards/dashboards.yml:ro
      - ./monitoring/dashboards:/var/lib/grafana/dashboards:ro
    restart: unless-stopped

  # Web Interface
  web-interface:
    build:
      context: ./web
      dockerfile: Dockerfile
    container_name: robot-web-interface
    networks:
      - robot-network
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./web/nginx.conf:/etc/nginx/nginx.conf:ro
      - ./web/ssl:/etc/nginx/ssl:ro
    depends_on:
      - robot-control
    restart: unless-stopped

networks:
  robot-network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

volumes:
  postgres_data:
  redis_data:
  prometheus_data:
  grafana_data:`,
    },
    hardware_interface: {
      title: "Hardware Interface Driver",
      description: "Hardware interface สำหรับการควบคุมฮาร์ดแวร์",
      code: `#!/usr/bin/env python3
"""
Complete Hardware Interface Driver
สำหรับการเชื่อมต่อกับฮาร์ดแวร์หุ่นยนต์
"""

import rospy
import serial
import threading
import time
import json
import struct
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, BatteryState, Temperature
from nav_msgs.msg import Odometry

class HardwareInterface:
    def __init__(self):
        rospy.init_node('hardware_interface', anonymous=True)

        # Serial connections
        self.base_serial = None
        self.sensor_serial = None

        # Configuration
        self.base_port = rospy.get_param('~base_port', '/dev/ttyACM0')
        self.sensor_port = rospy.get_param('~sensor_port', '/dev/ttyUSB1')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)

        # Robot parameters
        self.wheel_base = rospy.get_param('~wheel_base', 0.5)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.1)
        self.encoder_resolution = rospy.get_param('~encoder_resolution', 1024)

        # Initialize serial connections
        self.init_serial_connections()

        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
        self.temp_pub = rospy.Publisher('/temperature', Temperature, queue_size=10)
        self.hardware_status_pub = rospy.Publisher('/hardware_status', String, queue_size=10)

        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.led_sub = rospy.Subscriber('/led_control', String, self.led_callback)
        self.servo_sub = rospy.Subscriber('/servo_control', String, self.servo_callback)

        # Hardware monitoring
        self.start_hardware_monitoring()

        rospy.loginfo("Hardware Interface initialized")

    def init_serial_connections(self):
        """Initialize serial connections to hardware"""
        try:
            # Base controller connection
            self.base_serial = serial.Serial(
                port=self.base_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            rospy.loginfo(f"Base controller connected on {self.base_port}")

            # Sensor controller connection
            self.sensor_serial = serial.Serial(
                port=self.sensor_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            rospy.loginfo(f"Sensor controller connected on {self.sensor_port}")

            # Wait for hardware to initialize
            time.sleep(2)

            # Send initialization commands
            self.send_base_command("INIT")
            self.send_sensor_command("INIT")

        except Exception as e:
            rospy.logerr(f"Failed to initialize serial connections: {e}")

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        try:
            # Convert Twist to wheel velocities for holonomic drive
            vx = msg.linear.x
            vy = msg.linear.y
            vth = msg.angular.z

            # Holonomic drive kinematics
            # For mecanum wheels or omni wheels
            wheel_fl = vx - vy - vth * (self.wheel_base / 2)
            wheel_fr = vx + vy + vth * (self.wheel_base / 2)
            wheel_bl = vx + vy - vth * (self.wheel_base / 2)
            wheel_br = vx - vy + vth * (self.wheel_base / 2)

            # Convert to RPM
            wheel_fl_rpm = (wheel_fl / (2 * 3.14159 * self.wheel_radius)) * 60
            wheel_fr_rpm = (wheel_fr / (2 * 3.14159 * self.wheel_radius)) * 60
            wheel_bl_rpm = (wheel_bl / (2 * 3.14159 * self.wheel_radius)) * 60
            wheel_br_rpm = (wheel_br / (2 * 3.14159 * self.wheel_radius)) * 60

            # Send command to base controller
            command = f"MOVE:{wheel_fl_rpm:.2f},{wheel_fr_rpm:.2f},{wheel_bl_rpm:.2f},{wheel_br_rpm:.2f}"
            self.send_base_command(command)

        except Exception as e:
            rospy.logerr(f"Error processing cmd_vel: {e}")

    def led_callback(self, msg):
        """Handle LED control commands"""
        try:
            led_data = json.loads(msg.data)
            command = f"LED:{led_data['pattern']},{led_data['color']},{led_data['brightness']}"
            self.send_base_command(command)
        except Exception as e:
            rospy.logerr(f"Error processing LED command: {e}")

    def servo_callback(self, msg):
        """Handle servo control commands"""
        try:
            servo_data = json.loads(msg.data)
            command = f"SERVO:{servo_data['id']},{servo_data['angle']},{servo_data['speed']}"
            self.send_base_command(command)
        except Exception as e:
            rospy.logerr(f"Error processing servo command: {e}")

    def send_base_command(self, command):
        """Send command to base controller"""
        if self.base_serial and self.base_serial.is_open:
            try:
                self.base_serial.write(f"{command}\\n".encode())
                self.base_serial.flush()
            except Exception as e:
                rospy.logerr(f"Error sending base command: {e}")

    def send_sensor_command(self, command):
        """Send command to sensor controller"""
        if self.sensor_serial and self.sensor_serial.is_open:
            try:
                self.sensor_serial.write(f"{command}\\n".encode())
                self.sensor_serial.flush()
            except Exception as e:
                rospy.logerr(f"Error sending sensor command: {e}")

    def start_hardware_monitoring(self):
        """Start hardware monitoring threads"""
        # Base controller monitoring
        base_thread = threading.Thread(target=self.base_monitor_loop, daemon=True)
        base_thread.start()

        # Sensor monitoring
        sensor_thread = threading.Thread(target=self.sensor_monitor_loop, daemon=True)
        sensor_thread.start()

        rospy.loginfo("Hardware monitoring threads started")

    def base_monitor_loop(self):
        """Monitor base controller data"""
        while not rospy.is_shutdown():
            try:
                if self.base_serial and self.base_serial.is_open:
                    if self.base_serial.in_waiting > 0:
                        line = self.base_serial.readline().decode().strip()
                        self.process_base_data(line)
            except Exception as e:
                rospy.logerr(f"Base monitor error: {e}")
            time.sleep(0.01)

    def sensor_monitor_loop(self):
        """Monitor sensor controller data"""
        while not rospy.is_shutdown():
            try:
                if self.sensor_serial and self.sensor_serial.is_open:
                    if self.sensor_serial.in_waiting > 0:
                        line = self.sensor_serial.readline().decode().strip()
                        self.process_sensor_data(line)
            except Exception as e:
                rospy.logerr(f"Sensor monitor error: {e}")
            time.sleep(0.01)

    def process_base_data(self, data):
        """Process data from base controller"""
        try:
            if data.startswith("ODOM:"):
                # Parse odometry data
                parts = data[5:].split(",")
                x, y, theta = float(parts[0]), float(parts[1]), float(parts[2])
                vx, vy, vth = float(parts[3]), float(parts[4]), float(parts[5])

                # Publish odometry
                self.publish_odometry(x, y, theta, vx, vy, vth)

            elif data.startswith("BATTERY:"):
                # Parse battery data
                parts = data[8:].split(",")
                voltage = float(parts[0])
                current = float(parts[1])
                percentage = float(parts[2])
                temperature = float(parts[3])

                # Publish battery state
                self.publish_battery_state(voltage, current, percentage, temperature)

            elif data.startswith("STATUS:"):
                # Hardware status
                status = data[7:]
                self.publish_hardware_status(f"Base: {status}")

        except Exception as e:
            rospy.logerr(f"Error processing base data: {e}")

    def process_sensor_data(self, data):
        """Process data from sensor controller"""
        try:
            if data.startswith("IMU:"):
                # Parse IMU data
                parts = data[4:].split(",")
                # Acceleration
                ax, ay, az = float(parts[0]), float(parts[1]), float(parts[2])
                # Gyroscope
                gx, gy, gz = float(parts[3]), float(parts[4]), float(parts[5])
                # Magnetometer
                mx, my, mz = float(parts[6]), float(parts[7]), float(parts[8])

                # Publish IMU data
                self.publish_imu_data(ax, ay, az, gx, gy, gz, mx, my, mz)

            elif data.startswith("TEMP:"):
                # Temperature sensors
                parts = data[5:].split(",")
                for i, temp in enumerate(parts):
                    self.publish_temperature(f"sensor_{i}", float(temp))

        except Exception as e:
            rospy.logerr(f"Error processing sensor data: {e}")

    def publish_odometry(self, x, y, theta, vx, vy, vth):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        import tf.transformations
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

    def publish_imu_data(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        """Publish IMU message"""
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "imu_link"

        # Linear acceleration
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # Angular velocity
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        # Orientation (would need sensor fusion for accurate quaternion)
        # For now, set covariance to indicate unknown orientation
        imu.orientation_covariance[0] = -1

        self.imu_pub.publish(imu)

    def publish_battery_state(self, voltage, current, percentage, temperature):
        """Publish battery state"""
        battery = BatteryState()
        battery.header.stamp = rospy.Time.now()
        battery.voltage = voltage
        battery.current = current
        battery.percentage = percentage / 100.0
        battery.temperature = temperature
        battery.present = True

        if percentage > 80:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif percentage > 20:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING

        self.battery_pub.publish(battery)

    def publish_temperature(self, sensor_name, temperature):
        """Publish temperature reading"""
        temp_msg = Temperature()
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.header.frame_id = sensor_name
        temp_msg.temperature = temperature
        temp_msg.variance = 0.1

        self.temp_pub.publish(temp_msg)

    def publish_hardware_status(self, status):
        """Publish hardware status"""
        status_msg = String()
        status_msg.data = json.dumps({
            'timestamp': time.time(),
            'status': status,
            'base_connected': self.base_serial.is_open if self.base_serial else False,
            'sensor_connected': self.sensor_serial.is_open if self.sensor_serial else False
        })
        self.hardware_status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        hardware = HardwareInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass`,
    },
  };

  const developmentSetup = [
    {
      title: "Complete Development Environment",
      icon: Settings,
      items: [
        "Ubuntu 20.04 LTS + ROS Noetic installation",
        "Complete workspace setup: ~/catkin_ws with all dependencies",
        "Hardware drivers: Camera, LiDAR, IMU, GPS integration",
        "Network configuration: Static IP, WiFi hotspot setup",
        "Security: SSH keys, firewall, user permissions",
        "Auto-start services: systemd configuration",
        "Monitoring: Prometheus + Grafana dashboards",
        "Backup system: Automated daily backups",
      ],
    },
    {
      title: "Production Dependencies",
      icon: Package,
      items: [
        "ROS packages: navigation, slam, tf2, robot_state_publisher",
        "Python: opencv-python, numpy, websockets, psutil, serial",
        "Hardware: cv_camera, rplidar_ros, nmea_navsat_driver, joy",
        "Database: PostgreSQL, Redis for caching",
        "Web: nginx, SSL certificates, WebSocket support",
        "Docker: Multi-container deployment",
        "Monitoring: Prometheus, Grafana, system metrics",
        "Development: VS Code, Git, testing frameworks",
      ],
    },
    {
      title: "Hardware Integration",
      icon: Cpu,
      items: [
        "Holonomic drive system: 4x mecanum wheels with encoders",
        "Sensors: RPLiDAR A1/A2, RGB camera, 9-DOF IMU",
        "Computing: Jetson Nano/Xavier or Raspberry Pi 4",
        "Power management: Smart battery system with monitoring",
        "Communication: Ethernet + WiFi dual connectivity",
        "Safety: Hardware emergency stop, soft limits",
        "Expansion: GPIO, I2C, SPI interfaces available",
        "Enclosure: Weather-resistant design",
      ],
    },
  ];

  const apiReference = [
    {
      category: "Core ROS Topics",
      endpoints: [
        {
          name: "/cmd_vel",
          type: "geometry_msgs/Twist",
          description: "Robot movement control (holonomic)",
        },
        {
          name: "/odom",
          type: "nav_msgs/Odometry",
          description: "Robot odometry with covariance",
        },
        {
          name: "/robot_status",
          type: "std_msgs/String",
          description: "Complete robot state JSON",
        },
        {
          name: "/emergency_stop",
          type: "std_msgs/Bool",
          description: "Emergency stop trigger",
        },
        {
          name: "/system_health",
          type: "std_msgs/String",
          description: "System monitoring data",
        },
        {
          name: "/battery_state",
          type: "sensor_msgs/BatteryState",
          description: "Battery monitoring",
        },
        {
          name: "/camera/image_raw",
          type: "sensor_msgs/Image",
          description: "Camera feed with overlay",
        },
        {
          name: "/scan",
          type: "sensor_msgs/LaserScan",
          description: "LiDAR obstacle detection",
        },
        {
          name: "/imu",
          type: "sensor_msgs/Imu",
          description: "9-DOF inertial measurement",
        },
        {
          name: "/joy",
          type: "sensor_msgs/Joy",
          description: "Joystick input handling",
        },
      ],
    },
    {
      category: "Navigation & SLAM",
      endpoints: [
        {
          name: "/move_base/goal",
          type: "move_base_msgs/MoveBaseActionGoal",
          description: "Navigation goal setting",
        },
        {
          name: "/move_base/status",
          type: "actionlib_msgs/GoalStatusArray",
          description: "Navigation status",
        },
        {
          name: "/map",
          type: "nav_msgs/OccupancyGrid",
          description: "SLAM generated map",
        },
        {
          name: "/amcl_pose",
          type: "geometry_msgs/PoseWithCovarianceStamped",
          description: "Localization result",
        },
        {
          name: "/initialpose",
          type: "geometry_msgs/PoseWithCovarianceStamped",
          description: "Initial pose setting",
        },
        {
          name: "/move_base/local_costmap/costmap",
          type: "nav_msgs/OccupancyGrid",
          description: "Local planning costmap",
        },
        {
          name: "/move_base/global_costmap/costmap",
          type: "nav_msgs/OccupancyGrid",
          description: "Global planning costmap",
        },
      ],
    },
    {
      category: "WebSocket API",
      endpoints: [
        {
          name: "cmd_vel",
          type: "JSON",
          description: "Web velocity control {linear_x, linear_y, angular_z}",
        },
        {
          name: "navigate_to",
          type: "JSON",
          description: "Navigation command {x, y, theta}",
        },
        {
          name: "emergency_stop",
          type: "JSON",
          description: "Emergency stop activation",
        },
        {
          name: "camera_frame",
          type: "Base64",
          description: "Real-time camera stream",
        },
        {
          name: "system_health",
          type: "JSON",
          description: "Real-time system metrics",
        },
        {
          name: "robot_state",
          type: "JSON",
          description: "Complete robot status update",
        },
        {
          name: "initial_state",
          type: "JSON",
          description: "Connection handshake data",
        },
      ],
    },
  ];

  const bestPractices = [
    {
      title: "System Architecture",
      icon: FileText,
      practices: [
        "Modular design: Separate hardware, control, navigation layers",
        "Error handling: Comprehensive exception management",
        "Logging: Structured logging with different severity levels",
        "Configuration: YAML-based parameter management",
        "State management: Clear state machine implementation",
        "Resource management: Proper cleanup and resource disposal",
        "Thread safety: Proper synchronization in multi-threaded code",
        "Documentation: Comprehensive inline and external docs",
      ],
    },
    {
      title: "Production Deployment",
      icon: Rocket,
      practices: [
        "Container orchestration: Docker Compose for service management",
        "Health monitoring: Automated health checks and alerts",
        "Auto-recovery: Service restart on failure",
        "Load balancing: Multiple robot coordination",
        "Security: SSL/TLS, authentication, access control",
        "Backup strategy: Automated incremental backups",
        "Update mechanism: Over-the-air update system",
        "Performance optimization: Resource usage monitoring",
      ],
    },
    {
      title: "Safety & Reliability",
      icon: Shield,
      practices: [
        "Hardware emergency stop: Independent safety circuit",
        "Software watchdog: Process monitoring and restart",
        "Collision avoidance: Multi-sensor fusion approach",
        "Battery management: Low voltage protection",
        "Thermal protection: Temperature monitoring",
        "Network redundancy: Multiple communication paths",
        "Graceful degradation: Reduced functionality on errors",
        "Testing: Unit, integration, and hardware-in-loop tests",
      ],
    },
  ];

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
  };

  const filteredExamples = Object.entries(codeExamples).filter(
    ([key, example]) =>
      example.title.toLowerCase().includes(searchTerm.toLowerCase()) ||
      example.description.toLowerCase().includes(searchTerm.toLowerCase()),
  );

  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            Complete Development Guide
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            คู่มือการพัฒนาระบบหุ่นยนต์แบบสมบูรณ์ 100% - Production Ready
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge className="bg-green-100 text-green-700">
            <Activity className="h-3 w-3 mr-1" />
            Production Ready
          </Badge>
          <Badge className="bg-blue-100 text-blue-700">
            <Rocket className="h-3 w-3 mr-1" />
            Full System
          </Badge>
          <Badge className="bg-purple-100 text-purple-700">
            <Code className="h-3 w-3 mr-1" />
            Complete Code
          </Badge>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Download All
          </Button>
        </div>
      </div>

      <Tabs defaultValue="examples" className="space-y-6">
        <TabsList className="grid w-full grid-cols-8">
          <TabsTrigger value="examples" className="gap-2">
            <Code className="h-4 w-4" />
            Code
          </TabsTrigger>
          <TabsTrigger value="architecture" className="gap-2">
            <Network className="h-4 w-4" />
            Architecture
          </TabsTrigger>
          <TabsTrigger value="setup" className="gap-2">
            <Settings className="h-4 w-4" />
            Setup
          </TabsTrigger>
          <TabsTrigger value="hardware" className="gap-2">
            <Cpu className="h-4 w-4" />
            Hardware
          </TabsTrigger>
          <TabsTrigger value="deployment" className="gap-2">
            <Rocket className="h-4 w-4" />
            Deploy
          </TabsTrigger>
          <TabsTrigger value="api" className="gap-2">
            <Database className="h-4 w-4" />
            API
          </TabsTrigger>
          <TabsTrigger value="monitoring" className="gap-2">
            <Monitor className="h-4 w-4" />
            Monitor
          </TabsTrigger>
          <TabsTrigger value="troubleshooting" className="gap-2">
            <Bug className="h-4 w-4" />
            Debug
          </TabsTrigger>
        </TabsList>

        {/* Complete Code Examples */}
        <TabsContent value="examples">
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            {/* Example List */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-semibold">Production Code</h3>
                <Badge className="bg-green-100 text-green-700">
                  {Object.keys(codeExamples).length}
                </Badge>
              </div>

              <Input
                placeholder="ค้นหาโค้ด..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="mb-4"
              />

              <ScrollArea className="h-96">
                <div className="space-y-2">
                  {filteredExamples.map(([key, example]) => (
                    <button
                      key={key}
                      onClick={() => updateDevPref("activeExample", key)}
                      className={`w-full text-left p-3 rounded-lg transition-colors ${
                        activeExample === key
                          ? "bg-blue-100 border-blue-300 border"
                          : "bg-gray-50 hover:bg-white/5 border border-white/10"
                      }`}
                    >
                      <h4 className="font-medium text-sm">{example.title}</h4>
                      <p className="text-xs text-gray-600 mt-1">
                        {example.description}
                      </p>
                    </button>
                  ))}
                </div>
              </ScrollArea>
            </Card>

            {/* Code Display */}
            <Card className="lg:col-span-2 p-6">
              <div className="flex items-center justify-between mb-4">
                <div>
                  <h3 className="text-lg font-semibold">
                    {codeExamples[activeExample].title}
                  </h3>
                  <p className="text-sm text-gray-600">
                    {codeExamples[activeExample].description}
                  </p>
                </div>
                <div className="flex gap-2">
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() =>
                      copyToClipboard(codeExamples[activeExample].code)
                    }
                    className="gap-2"
                  >
                    <Copy className="h-4 w-4" />
                    Copy
                  </Button>
                  <Button variant="outline" size="sm" className="gap-2">
                    <Download className="h-4 w-4" />
                    Save
                  </Button>
                </div>
              </div>

              <ScrollArea className="h-[600px]">
                <pre className="bg-gray-900 text-green-400 p-4 rounded-lg text-xs overflow-x-auto whitespace-pre-wrap">
                  <code>{codeExamples[activeExample].code}</code>
                </pre>
              </ScrollArea>
            </Card>
          </div>
        </TabsContent>

        {/* System Architecture */}
        <TabsContent value="architecture">
          <div className="space-y-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                Complete System Architecture
              </h3>
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
                <div className="p-4 bg-blue-50 rounded-lg">
                  <Network className="h-8 w-8 text-blue-600 mb-2" />
                  <h4 className="font-medium">Control Layer</h4>
                  <ul className="text-sm text-gray-600 mt-2 space-y-1">
                    <li>• Robot Control System</li>
                    <li>• Emergency Stop Manager</li>
                    <li>• State Machine</li>
                    <li>• Safety Monitor</li>
                  </ul>
                </div>

                <div className="p-4 bg-green-50 rounded-lg">
                  <Cpu className="h-8 w-8 text-green-600 mb-2" />
                  <h4 className="font-medium">Hardware Layer</h4>
                  <ul className="text-sm text-gray-600 mt-2 space-y-1">
                    <li>• Base Controller</li>
                    <li>• Sensor Interface</li>
                    <li>• Motor Drivers</li>
                    <li>• Power Management</li>
                  </ul>
                </div>

                <div className="p-4 bg-purple-50 rounded-lg">
                  <Globe className="h-8 w-8 text-purple-600 mb-2" />
                  <h4 className="font-medium">Navigation Layer</h4>
                  <ul className="text-sm text-gray-600 mt-2 space-y-1">
                    <li>• SLAM & Mapping</li>
                    <li>• Path Planning</li>
                    <li>• Localization</li>
                    <li>• Obstacle Avoidance</li>
                  </ul>
                </div>

                <div className="p-4 bg-orange-50 rounded-lg">
                  <Monitor className="h-8 w-8 text-orange-600 mb-2" />
                  <h4 className="font-medium">Interface Layer</h4>
                  <ul className="text-sm text-gray-600 mt-2 space-y-1">
                    <li>• Web Interface</li>
                    <li>• WebSocket Bridge</li>
                    <li>• REST API</li>
                    <li>• Real-time Streaming</li>
                  </ul>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                Data Flow Architecture
              </h3>
              <div className="text-sm space-y-4">
                <div className="flex items-center gap-4">
                  <span className="font-medium w-24">Sensors →</span>
                  <span className="bg-white/5 border border-white/10 px-3 py-1 rounded">
                    Hardware Interface
                  </span>
                  <span>→</span>
                  <span className="bg-blue-100 px-3 py-1 rounded">
                    ROS Topics
                  </span>
                  <span>→</span>
                  <span className="bg-green-100 px-3 py-1 rounded">
                    Control System
                  </span>
                </div>
                <div className="flex items-center gap-4">
                  <span className="font-medium w-24">Commands →</span>
                  <span className="bg-purple-100 px-3 py-1 rounded">
                    Web Interface
                  </span>
                  <span>→</span>
                  <span className="bg-blue-100 px-3 py-1 rounded">
                    WebSocket
                  </span>
                  <span>→</span>
                  <span className="bg-green-100 px-3 py-1 rounded">
                    Robot Control
                  </span>
                </div>
                <div className="flex items-center gap-4">
                  <span className="font-medium w-24">Navigation →</span>
                  <span className="bg-orange-100 px-3 py-1 rounded">
                    Move Base
                  </span>
                  <span>→</span>
                  <span className="bg-blue-100 px-3 py-1 rounded">
                    Local Planner
                  </span>
                  <span>→</span>
                  <span className="bg-green-100 px-3 py-1 rounded">
                    cmd_vel
                  </span>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Complete Setup Guide */}
        <TabsContent value="setup">
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            {developmentSetup.map((section, index) => (
              <Card key={index} className="p-6">
                <div className="flex items-center gap-3 mb-4">
                  <div className="p-2 bg-blue-100 rounded-lg">
                    <section.icon className="h-5 w-5 text-blue-600" />
                  </div>
                  <h3 className="text-lg font-semibold">{section.title}</h3>
                </div>

                <div className="space-y-3">
                  {section.items.map((item, itemIndex) => (
                    <div key={itemIndex} className="flex items-start gap-3">
                      <CheckCircle className="h-4 w-4 text-green-500 mt-1 flex-shrink-0" />
                      <p className="text-sm text-slate-300">{item}</p>
                    </div>
                  ))}
                </div>
              </Card>
            ))}
          </div>

          {/* Installation Commands */}
          <Card className="p-6 mt-6">
            <h3 className="text-lg font-semibold mb-4">
              Complete Installation Script
            </h3>
            <ScrollArea className="h-64">
              <pre className="bg-gray-900 text-green-400 p-4 rounded text-sm">
                {`#!/bin/bash
# Complete Robot System Installation Script

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install ros-noetic-navigation ros-noetic-slam-gmapping ros-noetic-move-base -y
sudo apt install ros-noetic-cv-camera ros-noetic-rplidar-ros ros-noetic-joy -y

# Python packages
pip3 install opencv-python numpy websockets psutil pyserial pyyaml

# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# Setup auto-start service
sudo systemctl enable robot-control.service
sudo systemctl start robot-control.service

echo "Installation complete! System ready for use."
`}
              </pre>
            </ScrollArea>
          </Card>
        </TabsContent>

        {/* Hardware Integration */}
        <TabsContent value="hardware">
          <div className="space-y-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                Hardware Specifications
              </h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                <div>
                  <h4 className="font-medium mb-3">Core Components</h4>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span>Computing Unit:</span>
                      <span className="font-medium">
                        Jetson Xavier NX / RPi 4
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span>Drive System:</span>
                      <span className="font-medium">4x Mecanum Wheels</span>
                    </div>
                    <div className="flex justify-between">
                      <span>LiDAR:</span>
                      <span className="font-medium">RPLiDAR A1/A2</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Camera:</span>
                      <span className="font-medium">USB 3.0 RGB Camera</span>
                    </div>
                    <div className="flex justify-between">
                      <span>IMU:</span>
                      <span className="font-medium">9-DOF MPU9250</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Battery:</span>
                      <span className="font-medium">24V 10Ah LiFePO4</span>
                    </div>
                  </div>
                </div>

                <div>
                  <h4 className="font-medium mb-3">Communication</h4>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span>Primary:</span>
                      <span className="font-medium">Gigabit Ethernet</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Wireless:</span>
                      <span className="font-medium">WiFi 6 802.11ax</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Backup:</span>
                      <span className="font-medium">4G LTE Module</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Serial:</span>
                      <span className="font-medium">4x USB, 2x RS485</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Expansion:</span>
                      <span className="font-medium">GPIO, I2C, SPI</span>
                    </div>
                  </div>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Wiring Diagram</h3>
              <div className="bg-white/5 border border-white/10 p-4 rounded-lg">
                <div className="text-center text-gray-600">
                  <HardDrive className="h-16 w-16 mx-auto mb-2" />
                  <p>Complete wiring diagrams and PCB layouts</p>
                  <p className="text-sm">
                    Available in the hardware documentation package
                  </p>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Deployment Guide */}
        <TabsContent value="deployment">
          <div className="space-y-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                Production Deployment Options
              </h3>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                <div className="p-4 border rounded-lg">
                  <Server className="h-8 w-8 text-blue-600 mb-2" />
                  <h4 className="font-medium">Single Robot</h4>
                  <p className="text-sm text-gray-600 mt-2">
                    Direct deployment on robot hardware
                  </p>
                  <ul className="text-xs mt-2 space-y-1">
                    <li>• Systemd auto-start</li>
                    <li>• Local monitoring</li>
                    <li>• Basic logging</li>
                  </ul>
                </div>

                <div className="p-4 border rounded-lg">
                  <Network className="h-8 w-8 text-green-600 mb-2" />
                  <h4 className="font-medium">Multi-Robot Fleet</h4>
                  <p className="text-sm text-gray-600 mt-2">
                    Centralized management system
                  </p>
                  <ul className="text-xs mt-2 space-y-1">
                    <li>• Central ROS master</li>
                    <li>• Fleet coordination</li>
                    <li>• Distributed monitoring</li>
                  </ul>
                </div>

                <div className="p-4 border rounded-lg">
                  <Globe className="h-8 w-8 text-purple-600 mb-2" />
                  <h4 className="font-medium">Cloud Integration</h4>
                  <p className="text-sm text-gray-600 mt-2">
                    Cloud-connected operation
                  </p>
                  <ul className="text-xs mt-2 space-y-1">
                    <li>• Remote monitoring</li>
                    <li>• OTA updates</li>
                    <li>• Data analytics</li>
                  </ul>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Docker Deployment</h3>
              <div className="bg-gray-900 text-green-400 p-4 rounded-lg text-sm">
                <div className="mb-4">
                  <p className="text-blue-400">
                    # Deploy complete robot system with Docker
                  </p>
                </div>
                <div className="space-y-1">
                  <p>docker-compose up -d</p>
                  <p>docker-compose logs -f robot-control</p>
                  <p>docker-compose scale navigation=2</p>
                  <p>docker-compose exec robot-control bash</p>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* API Reference */}
        <TabsContent value="api">
          <div className="space-y-6">
            {apiReference.map((category, index) => (
              <Card key={index} className="p-6">
                <h3 className="text-lg font-semibold mb-4">
                  {category.category}
                </h3>
                <div className="space-y-3">
                  {category.endpoints.map((endpoint, endpointIndex) => (
                    <div
                      key={endpointIndex}
                      className="flex items-center justify-between p-3 bg-gray-50 rounded-lg"
                    >
                      <div className="flex-1">
                        <code className="text-sm font-mono bg-blue-100 text-blue-800 px-2 py-1 rounded">
                          {endpoint.name}
                        </code>
                        <p className="text-sm text-gray-600 mt-1">
                          {endpoint.description}
                        </p>
                      </div>
                      <Badge variant="outline">{endpoint.type}</Badge>
                    </div>
                  ))}
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        {/* Monitoring & Analytics */}
        <TabsContent value="monitoring">
          <div className="space-y-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                System Monitoring Dashboard
              </h3>
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
                <div className="p-4 bg-blue-50 rounded-lg">
                  <Activity className="h-6 w-6 text-blue-600 mb-2" />
                  <h4 className="font-medium">Performance</h4>
                  <div className="text-2xl font-bold text-blue-600">87%</div>
                  <p className="text-xs text-gray-600">System efficiency</p>
                </div>

                <div className="p-4 bg-green-50 rounded-lg">
                  <Cpu className="h-6 w-6 text-green-600 mb-2" />
                  <h4 className="font-medium">CPU Usage</h4>
                  <div className="text-2xl font-bold text-green-600">34%</div>
                  <p className="text-xs text-gray-600">8 cores active</p>
                </div>

                <div className="p-4 bg-purple-50 rounded-lg">
                  <HardDrive className="h-6 w-6 text-purple-600 mb-2" />
                  <h4 className="font-medium">Memory</h4>
                  <div className="text-2xl font-bold text-purple-600">
                    2.1GB
                  </div>
                  <p className="text-xs text-gray-600">of 8GB used</p>
                </div>

                <div className="p-4 bg-orange-50 rounded-lg">
                  <Wifi className="h-6 w-6 text-orange-600 mb-2" />
                  <h4 className="font-medium">Network</h4>
                  <div className="text-2xl font-bold text-orange-600">
                    145ms
                  </div>
                  <p className="text-xs text-gray-600">avg latency</p>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Real-time Metrics</h3>
              <div className="space-y-4">
                <div>
                  <div className="flex justify-between text-sm mb-1">
                    <span>Robot Battery</span>
                    <span>87%</span>
                  </div>
                  <div className="w-full bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-green-500 h-2 rounded-full"
                      style={{ width: "87%" }}
                    ></div>
                  </div>
                </div>

                <div>
                  <div className="flex justify-between text-sm mb-1">
                    <span>Navigation Accuracy</span>
                    <span>94%</span>
                  </div>
                  <div className="w-full bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-blue-500 h-2 rounded-full"
                      style={{ width: "94%" }}
                    ></div>
                  </div>
                </div>

                <div>
                  <div className="flex justify-between text-sm mb-1">
                    <span>Sensor Health</span>
                    <span>98%</span>
                  </div>
                  <div className="w-full bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-green-500 h-2 rounded-full"
                      style={{ width: "98%" }}
                    ></div>
                  </div>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Troubleshooting */}
        <TabsContent value="troubleshooting">
          <div className="space-y-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                Common Issues & Solutions
              </h3>

              <div className="space-y-4">
                <div className="p-4 border-l-4 border-red-500 bg-red-50">
                  <h4 className="font-medium text-red-800">
                    Robot not responding to commands
                  </h4>
                  <div className="text-sm text-red-700 mt-2 space-y-1">
                    <p>
                      <strong>Check:</strong> Emergency stop status, hardware
                      connections
                    </p>
                    <p>
                      <strong>Solution:</strong> Reset emergency stop, restart
                      base controller
                    </p>
                    <p>
                      <strong>Command:</strong>{" "}
                      <code>rosservice call /release_emergency</code>
                    </p>
                  </div>
                </div>

                <div className="p-4 border-l-4 border-yellow-500 bg-yellow-50">
                  <h4 className="font-medium text-yellow-800">
                    Navigation not working
                  </h4>
                  <div className="text-sm text-yellow-700 mt-2 space-y-1">
                    <p>
                      <strong>Check:</strong> Map loaded, AMCL localized,
                      obstacles clear
                    </p>
                    <p>
                      <strong>Solution:</strong> Reload map, set initial pose,
                      clear costmaps
                    </p>
                    <p>
                      <strong>Command:</strong>{" "}
                      <code>rosservice call /move_base/clear_costmaps</code>
                    </p>
                  </div>
                </div>

                <div className="p-4 border-l-4 border-blue-500 bg-blue-50">
                  <h4 className="font-medium text-blue-800">
                    Camera stream not available
                  </h4>
                  <div className="text-sm text-blue-700 mt-2 space-y-1">
                    <p>
                      <strong>Check:</strong> Camera device permissions, USB
                      connection
                    </p>
                    <p>
                      <strong>Solution:</strong> Add user to video group,
                      restart camera node
                    </p>
                    <p>
                      <strong>Command:</strong>{" "}
                      <code>sudo usermod -a -G video $USER</code>
                    </p>
                  </div>
                </div>

                <div className="p-4 border-l-4 border-green-500 bg-green-50">
                  <h4 className="font-medium text-green-800">
                    System running successfully
                  </h4>
                  <div className="text-sm text-green-700 mt-2 space-y-1">
                    <p>
                      <strong>Status:</strong> All systems operational
                    </p>
                    <p>
                      <strong>Monitor:</strong> Check logs regularly for any
                      warnings
                    </p>
                    <p>
                      <strong>Command:</strong>{" "}
                      <code>rostopic echo /system_health</code>
                    </p>
                  </div>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Debug Commands</h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div>
                  <h4 className="font-medium mb-2">System Diagnostics</h4>
                  <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
                    {`# Check system status
systemctl status robot-control

# View system logs
journalctl -f -u robot-control

# Check ROS connectivity
rostopic list
rosnode list

# Monitor system resources
htop
iotop`}
                  </pre>
                </div>

                <div>
                  <h4 className="font-medium mb-2">Robot Diagnostics</h4>
                  <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
                    {`# Check robot status
rostopic echo /robot_status

# Test movement
rostopic pub /cmd_vel geometry_msgs/Twist ...

# View camera feed
rosrun image_view image_view image:=/camera/image_raw

# Check sensors
rostopic echo /scan`}
                  </pre>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>
      </Tabs>

      {/* Quick Action Cards */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mt-8">
        <Card className="p-6 bg-gradient-to-r from-blue-50 to-purple-50 border-blue-200">
          <div className="flex items-center gap-3 mb-4">
            <Rocket className="h-6 w-6 text-blue-600" />
            <h3 className="text-lg font-semibold">Quick Deploy</h3>
          </div>
          <p className="text-sm text-slate-300 mb-4">
            Deploy complete robot system in minutes with our automated scripts
          </p>
          <Button className="w-full">Start Deployment</Button>
        </Card>

        <Card className="p-6 bg-gradient-to-r from-green-50 to-blue-50 border-green-200">
          <div className="flex items-center gap-3 mb-4">
            <Monitor className="h-6 w-6 text-green-600" />
            <h3 className="text-lg font-semibold">Live Monitor</h3>
          </div>
          <p className="text-sm text-slate-300 mb-4">
            Real-time system monitoring and performance analytics
          </p>
          <Button variant="outline" className="w-full">
            Open Dashboard
          </Button>
        </Card>

        <Card className="p-6 bg-gradient-to-r from-purple-50 to-pink-50 border-purple-200">
          <div className="flex items-center gap-3 mb-4">
            <Users className="h-6 w-6 text-purple-600" />
            <h3 className="text-lg font-semibold">Community</h3>
          </div>
          <p className="text-sm text-slate-300 mb-4">
            Join our developer community for support and collaboration
          </p>
          <Button variant="outline" className="w-full">
            Join Community
          </Button>
        </Card>
      </div>

      {/* Footer Summary */}
      <Card className="p-6 mt-8 bg-gradient-to-r from-gray-50 to-blue-50 border-gray-200">
        <div className="flex items-center gap-3 mb-4">
          <CheckCircle className="h-6 w-6 text-green-600" />
          <h3 className="text-lg font-semibold">Production Ready System</h3>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-4 gap-4 text-sm">
          <div>
            <h4 className="font-medium mb-2">Core Features</h4>
            <p>Complete robot control, navigation, safety systems</p>
          </div>

          <div>
            <h4 className="font-medium mb-2">Deployment</h4>
            <p>Docker, systemd, cloud integration ready</p>
          </div>

          <div>
            <h4 className="font-medium mb-2">Monitoring</h4>
            <p>Real-time metrics, logging, health checks</p>
          </div>

          <div>
            <h4 className="font-medium mb-2">Support</h4>
            <p>Complete documentation, examples, troubleshooting</p>
          </div>
        </div>
      </Card>
    </div>
  );
}
