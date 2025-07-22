import React, { useState } from "react";
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
} from "lucide-react";

export default function PythonDevelopment() {
  const [activeExample, setActiveExample] = useState("basic_node");
  const [searchTerm, setSearchTerm] = useState("");

  const codeExamples = {
    basic_node: {
      title: "Basic ROS Node",
      description: "สร้าง ROS node พื้นฐานด้วย Python",
      code: `#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class BasicRobotNode:
    def __init__(self):
        rospy.init_node('basic_robot_node', anonymous=True)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        
        # Subscribers
        self.cmd_sub = rospy.Subscriber('/robot_commands', String, self.command_callback)
        
        # Timer for periodic tasks
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        rospy.loginfo("Basic Robot Node initialized")
    
    def command_callback(self, msg):
        """Handle incoming commands"""
        command = msg.data.lower()
        
        if command == "move_forward":
            self.move_robot(0.5, 0.0)
        elif command == "turn_left":
            self.move_robot(0.0, 0.5)
        elif command == "stop":
            self.move_robot(0.0, 0.0)
        
        rospy.loginfo(f"Received command: {command}")
    
    def move_robot(self, linear_x, angular_z):
        """Send movement commands to robot"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
    
    def timer_callback(self, event):
        """Periodic status updates"""
        status_msg = String()
        status_msg.data = f"Robot status: OK - Time: {rospy.Time.now()}"
        self.status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        node = BasicRobotNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass`
    },
    holonomic_control: {
      title: "Holonomic Drive Control",
      description: "ควบคุมการเคลื่อนที่แบบ holonomic drive",
      code: `#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class HolonomicController:
    def __init__(self):
        rospy.init_node('holonomic_controller', anonymous=True)
        
        # Parameters
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.web_cmd_sub = rospy.Subscriber('/web_cmd_vel', Twist, self.web_cmd_callback)
        
        rospy.loginfo("Holonomic Controller initialized")
    
    def joy_callback(self, msg):
        """Handle joystick input"""
        # Map joystick axes to robot movement
        linear_x = msg.axes[1] * self.max_linear_speed  # Forward/backward
        linear_y = msg.axes[0] * self.max_linear_speed  # Strafe left/right
        angular_z = msg.axes[3] * self.max_angular_speed  # Rotation
        
        self.publish_velocity(linear_x, linear_y, angular_z)
    
    def web_cmd_callback(self, msg):
        """Handle web interface commands"""
        self.publish_velocity(msg.linear.x, msg.linear.y, msg.angular.z)
    
    def publish_velocity(self, linear_x, linear_y, angular_z):
        """Publish velocity commands"""
        twist = Twist()
        twist.linear.x = self.clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
        twist.linear.y = self.clamp(linear_y, -self.max_linear_speed, self.max_linear_speed)
        twist.angular.z = self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)
        
        self.cmd_vel_pub.publish(twist)
        
        rospy.logdebug(f"Published cmd_vel: x={twist.linear.x:.2f}, "
                      f"y={twist.linear.y:.2f}, z={twist.angular.z:.2f}")
    
    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(value, max_val))
    
    def move_to_position(self, target_x, target_y, target_theta):
        """Move to specific position (simple P controller)"""
        current_x, current_y, current_theta = self.get_current_pose()
        
        # Calculate errors
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_theta = self.normalize_angle(target_theta - current_theta)
        
        # Simple proportional control
        kp_linear = 0.5
        kp_angular = 1.0
        
        linear_x = kp_linear * error_x
        linear_y = kp_linear * error_y
        angular_z = kp_angular * error_theta
        
        self.publish_velocity(linear_x, linear_y, angular_z)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_current_pose(self):
        """Get current robot pose (placeholder - implement with tf)"""
        # TODO: Implement actual pose getting using tf
        return 0.0, 0.0, 0.0

if __name__ == '__main__':
    try:
        controller = HolonomicController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass`
    },
    websocket_bridge: {
      title: "WebSocket ROS Bridge",
      description: "สร้างการเชื่อมต่อระหว่าง Web Interface และ ROS",
      code: `#!/usr/bin/env python3
import rospy
import json
import asyncio
import websockets
from threading import Thread
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import base64
import cv2
from cv_bridge import CvBridge

class WebSocketROSBridge:
    def __init__(self):
        rospy.init_node('websocket_ros_bridge', anonymous=True)
        
        self.bridge = CvBridge()
        self.connected_clients = set()
        
        # ROS Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.camera_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.status_sub = rospy.Subscriber('/robot_status', String, self.status_callback)
        
        # ROS Publishers
        self.web_cmd_pub = rospy.Publisher('/web_cmd_vel', Twist, queue_size=1)
        self.web_command_pub = rospy.Publisher('/web_commands', String, queue_size=10)
        
        # Start WebSocket server
        self.start_websocket_server()
        
        rospy.loginfo("WebSocket ROS Bridge initialized")
    
    def start_websocket_server(self):
        """Start WebSocket server in separate thread"""
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            start_server = websockets.serve(self.handle_client, "0.0.0.0", 9090)
            loop.run_until_complete(start_server)
            loop.run_forever()
        
        server_thread = Thread(target=run_server, daemon=True)
        server_thread.start()
    
    async def handle_client(self, websocket, path):
        """Handle WebSocket client connection"""
        self.connected_clients.add(websocket)
        rospy.loginfo(f"Client connected from {websocket.remote_address}")
        
        try:
            async for message in websocket:
                await self.process_web_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            rospy.loginfo(f"Client disconnected: {websocket.remote_address}")
        finally:
            self.connected_clients.discard(websocket)
    
    async def process_web_message(self, websocket, message):
        """Process incoming WebSocket message"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'cmd_vel':
                # Handle velocity commands from web
                twist = Twist()
                twist.linear.x = data.get('linear_x', 0.0)
                twist.linear.y = data.get('linear_y', 0.0)
                twist.angular.z = data.get('angular_z', 0.0)
                self.web_cmd_pub.publish(twist)
                
            elif msg_type == 'command':
                # Handle general commands
                cmd_msg = String()
                cmd_msg.data = data.get('command', '')
                self.web_command_pub.publish(cmd_msg)
                
            elif msg_type == 'get_status':
                # Send current robot status
                await self.send_robot_status(websocket)
                
        except json.JSONDecodeError:
            rospy.logwarn(f"Invalid JSON received: {message}")
        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")
    
    def cmd_vel_callback(self, msg):
        """Forward cmd_vel to web clients"""
        data = {
            'type': 'cmd_vel',
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
            'timestamp': rospy.Time.now().to_sec()
        }
        asyncio.run_coroutine_threadsafe(
            self.broadcast_to_clients(json.dumps(data)),
            asyncio.get_event_loop()
        )
    
    def image_callback(self, msg):
        """Forward camera images to web clients"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Resize for web transmission
            cv_image = cv2.resize(cv_image, (640, 480))
            
            # Encode to base64
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            data = {
                'type': 'camera_image',
                'image': img_base64,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            asyncio.run_coroutine_threadsafe(
                self.broadcast_to_clients(json.dumps(data)),
                asyncio.get_event_loop()
            )
            
        except Exception as e:
            rospy.logerr(f"Error processing camera image: {e}")
    
    def status_callback(self, msg):
        """Forward status messages to web clients"""
        data = {
            'type': 'robot_status',
            'status': msg.data,
            'timestamp': rospy.Time.now().to_sec()
        }
        asyncio.run_coroutine_threadsafe(
            self.broadcast_to_clients(json.dumps(data)),
            asyncio.get_event_loop()
        )
    
    async def broadcast_to_clients(self, message):
        """Broadcast message to all connected clients"""
        if self.connected_clients:
            await asyncio.gather(
                *[self.send_safe(client, message) for client in self.connected_clients],
                return_exceptions=True
            )
    
    async def send_safe(self, websocket, message):
        """Send message safely to client"""
        try:
            await websocket.send(message)
        except websockets.exceptions.ConnectionClosed:
            self.connected_clients.discard(websocket)

if __name__ == '__main__':
    try:
        bridge = WebSocketROSBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass`
    },
    sequence_manager: {
      title: "Sequence Manager",
      description: "จัดการ sequence และ automation",
      code: `#!/usr/bin/env python3
import rospy
import json
import yaml
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from actionlib import SimpleActionServer, SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SequenceManager:
    def __init__(self):
        rospy.init_node('sequence_manager', anonymous=True)
        
        self.current_sequence = None
        self.sequence_running = False
        self.step_index = 0
        
        # Action clients
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sequence_status_pub = rospy.Publisher('/sequence_status', String, queue_size=10)
        
        # Subscribers
        self.start_sequence_sub = rospy.Subscriber('/start_sequence', String, self.start_sequence_callback)
        self.stop_sequence_sub = rospy.Subscriber('/stop_sequence', Bool, self.stop_sequence_callback)
        
        # Load sequences from file
        self.load_sequences()
        
        rospy.loginfo("Sequence Manager initialized")
    
    def load_sequences(self):
        """Load sequence definitions from YAML file"""
        try:
            with open('/opt/ros/sequences.yaml', 'r') as file:
                self.sequences = yaml.safe_load(file)
            rospy.loginfo(f"Loaded {len(self.sequences)} sequences")
        except Exception as e:
            rospy.logwarn(f"Could not load sequences: {e}")
            self.sequences = {}
    
    def start_sequence_callback(self, msg):
        """Start executing a sequence"""
        sequence_name = msg.data
        
        if sequence_name in self.sequences:
            self.current_sequence = self.sequences[sequence_name]
            self.sequence_running = True
            self.step_index = 0
            
            rospy.loginfo(f"Starting sequence: {sequence_name}")
            self.execute_next_step()
        else:
            rospy.logwarn(f"Sequence not found: {sequence_name}")
    
    def stop_sequence_callback(self, msg):
        """Stop current sequence execution"""
        if msg.data and self.sequence_running:
            self.sequence_running = False
            self.current_sequence = None
            self.step_index = 0
            
            # Stop robot movement
            self.stop_robot()
            
            rospy.loginfo("Sequence stopped")
            self.publish_status("stopped")
    
    def execute_next_step(self):
        """Execute the next step in current sequence"""
        if not self.sequence_running or not self.current_sequence:
            return
        
        if self.step_index >= len(self.current_sequence.get('steps', [])):
            # Sequence completed
            self.sequence_running = False
            rospy.loginfo("Sequence completed")
            self.publish_status("completed")
            return
        
        step = self.current_sequence['steps'][self.step_index]
        step_type = step.get('type')
        
        rospy.loginfo(f"Executing step {self.step_index + 1}: {step_type}")
        
        if step_type == 'move':
            self.execute_move_step(step)
        elif step_type == 'wait':
            self.execute_wait_step(step)
        elif step_type == 'rotate':
            self.execute_rotate_step(step)
        elif step_type == 'goto':
            self.execute_goto_step(step)
        elif step_type == 'custom':
            self.execute_custom_step(step)
        else:
            rospy.logwarn(f"Unknown step type: {step_type}")
            self.next_step()
    
    def execute_move_step(self, step):
        """Execute movement step"""
        duration = step.get('duration', 1.0)
        linear_x = step.get('linear_x', 0.0)
        linear_y = step.get('linear_y', 0.0)
        angular_z = step.get('angular_z', 0.0)
        
        # Send velocity command
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        # Set timer for step completion
        rospy.Timer(rospy.Duration(duration), self.step_completed, oneshot=True)
    
    def execute_wait_step(self, step):
        """Execute wait step"""
        duration = step.get('duration', 1.0)
        rospy.Timer(rospy.Duration(duration), self.step_completed, oneshot=True)
    
    def execute_rotate_step(self, step):
        """Execute rotation step"""
        angle = step.get('angle', 0.0)  # radians
        speed = step.get('speed', 0.5)   # rad/s
        
        duration = abs(angle / speed)
        
        twist = Twist()
        twist.angular.z = speed if angle > 0 else -speed
        self.cmd_vel_pub.publish(twist)
        
        rospy.Timer(rospy.Duration(duration), self.step_completed, oneshot=True)
    
    def execute_goto_step(self, step):
        """Execute goto waypoint step"""
        x = step.get('x', 0.0)
        y = step.get('y', 0.0)
        theta = step.get('theta', 0.0)
        
        # Create goal for move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = theta
        
        # Send goal
        self.move_base_client.send_goal(goal, done_cb=self.goto_completed)
    
    def execute_custom_step(self, step):
        """Execute custom step"""
        command = step.get('command', '')
        
        # Publish custom command
        cmd_msg = String()
        cmd_msg.data = command
        rospy.Publisher('/custom_commands', String, queue_size=1).publish(cmd_msg)
        
        # Complete step immediately (or implement custom logic)
        self.next_step()
    
    def step_completed(self, event):
        """Called when a timed step is completed"""
        self.stop_robot()
        self.next_step()
    
    def goto_completed(self, state, result):
        """Called when move_base goal is completed"""
        rospy.loginfo(f"Goto completed with state: {state}")
        self.next_step()
    
    def next_step(self):
        """Move to next step in sequence"""
        self.step_index += 1
        self.publish_status(f"step_{self.step_index}")
        rospy.Timer(rospy.Duration(0.1), lambda event: self.execute_next_step(), oneshot=True)
    
    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def publish_status(self, status):
        """Publish sequence status"""
        status_msg = String()
        status_msg.data = json.dumps({
            'status': status,
            'step': self.step_index,
            'total_steps': len(self.current_sequence.get('steps', [])) if self.current_sequence else 0,
            'sequence_name': self.current_sequence.get('name', '') if self.current_sequence else ''
        })
        self.sequence_status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        manager = SequenceManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass`
    }
  };

  const developmentSetup = [
    {
      title: "ROS Environment Setup",
      icon: Settings,
      steps: [
        "ติดตั้ง ROS Noetic หรือ ROS2 Humble",
        "สร้าง catkin workspace: mkdir -p ~/catkin_ws/src",
        "cd ~/catkin_ws && catkin_make",
        "source ~/catkin_ws/devel/setup.bash",
        "เพิ่ม source command ใน ~/.bashrc"
      ]
    },
    {
      title: "Python Dependencies",
      icon: Package,
      steps: [
        "pip install rospy rospkg",
        "pip install opencv-python cv_bridge",
        "pip install websockets asyncio",
        "pip install numpy scipy matplotlib",
        "pip install pyyaml json"
      ]
    },
    {
      title: "Development Tools",
      icon: Code,
      steps: [
        "ติดตั้ง VS Code พร้อม Python extension",
        "ติดตั้ง ROS extension สำหรับ VS Code",
        "ตั้งค่า Python interpreter path",
        "ใช้ pylint หรือ flake8 สำหรับ code quality",
        "ตั้งค่า Git สำหรับ version control"
      ]
    }
  ];

  const apiReference = [
    {
      category: "ROS Topics",
      endpoints: [
        { name: "/cmd_vel", type: "geometry_msgs/Twist", description: "ควบคุมการเคลื่อนที่ของหุ่นยนต์" },
        { name: "/robot_status", type: "std_msgs/String", description: "สถานะปัจจุบันของหุ่นยนต์" },
        { name: "/camera/image_raw", type: "sensor_msgs/Image", description: "ภาพจากกล้อง" },
        { name: "/scan", type: "sensor_msgs/LaserScan", description: "ข้อมูลจาก LiDAR" },
        { name: "/odom", type: "nav_msgs/Odometry", description: "ข้อมูล odometry" }
      ]
    },
    {
      category: "ROS Services",
      endpoints: [
        { name: "/start_sequence", type: "std_srvs/Trigger", description: "เริ่มต้น sequence" },
        { name: "/emergency_stop", type: "std_srvs/Empty", description: "หยุดฉุกเฉิน" },
        { name: "/get_robot_state", type: "custom_msgs/RobotState", description: "ดึงสถานะหุ่นยนต์" },
        { name: "/set_mode", type: "custom_msgs/SetMode", description: "เปลี่ยนโหมดการทำงาน" }
      ]
    },
    {
      category: "WebSocket API",
      endpoints: [
        { name: "ws://localhost:9090", type: "WebSocket", description: "Real-time communication" },
        { name: "cmd_vel", type: "JSON", description: "ส่งคำสั่งการเคลื่อนที่" },
        { name: "camera_stream", type: "Base64", description: "รับภาพจากกล้อง" },
        { name: "robot_status", type: "JSON", description: "รับสถานะหุ่นยนต์" }
      ]
    }
  ];

  const bestPractices = [
    {
      title: "Code Structure",
      icon: FileText,
      practices: [
        "แยก business logic ออกจาก ROS-specific code",
        "ใช้ class-based approach สำหรับ ROS nodes",
        "สร้าง configuration files แยกต่างหาก",
        "ใช้ logging แทน print statements",
        "Implement proper error handling"
      ]
    },
    {
      title: "Performance",
      icon: Zap,
      practices: [
        "ใช้ rospy.Timer สำหรับ periodic tasks",
        "จำกัด publishing rate ให้เหมาะสม",
        "ใช้ queue_size ที่เหมาะสมสำหรับ publishers",
        "Avoid blocking operations ใน callbacks",
        "ใช้ threading สำหรับ long-running tasks"
      ]
    },
    {
      title: "Testing",
      icon: Bug,
      practices: [
        "เขียน unit tests ด้วย pytest",
        "ใช้ rosbag สำหรับ integration testing",
        "Test กับ simulated robot ก่อน",
        "ใช้ rostest สำหรับ system testing",
        "Monitor system resources"
      ]
    }
  ];

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
  };

  const filteredExamples = Object.entries(codeExamples).filter(([key, example]) =>
    example.title.toLowerCase().includes(searchTerm.toLowerCase()) ||
    example.description.toLowerCase().includes(searchTerm.toLowerCase())
  );

  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            Python Development Guide
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            คู่มือการพัฒนาระบบหุ่นยนต์ด้วย Python และ ROS
          </p>
        </div>
        
        <div className="flex items-center gap-3">
          <Badge className="bg-green-100 text-green-700">
            <Code className="h-3 w-3 mr-1" />
            Python 3.8+
          </Badge>
          <Badge className="bg-blue-100 text-blue-700">
            <Rocket className="h-3 w-3 mr-1" />
            ROS Noetic
          </Badge>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Download PDF
          </Button>
        </div>
      </div>

      <Tabs defaultValue="examples" className="space-y-6">
        <TabsList className="grid w-full grid-cols-6">
          <TabsTrigger value="examples" className="gap-2">
            <Code className="h-4 w-4" />
            Examples
          </TabsTrigger>
          <TabsTrigger value="setup" className="gap-2">
            <Settings className="h-4 w-4" />
            Setup
          </TabsTrigger>
          <TabsTrigger value="api" className="gap-2">
            <Network className="h-4 w-4" />
            API Reference
          </TabsTrigger>
          <TabsTrigger value="best-practices" className="gap-2">
            <Lightbulb className="h-4 w-4" />
            Best Practices
          </TabsTrigger>
          <TabsTrigger value="deployment" className="gap-2">
            <Rocket className="h-4 w-4" />
            Deployment
          </TabsTrigger>
          <TabsTrigger value="troubleshooting" className="gap-2">
            <Bug className="h-4 w-4" />
            Troubleshooting
          </TabsTrigger>
        </TabsList>

        {/* Code Examples */}
        <TabsContent value="examples">
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            {/* Example List */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-semibold">Code Examples</h3>
                <Badge>{Object.keys(codeExamples).length}</Badge>
              </div>
              
              <Input
                placeholder="ค้นหาตัวอย่าง..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="mb-4"
              />
              
              <ScrollArea className="h-96">
                <div className="space-y-2">
                  {filteredExamples.map(([key, example]) => (
                    <button
                      key={key}
                      onClick={() => setActiveExample(key)}
                      className={`w-full text-left p-3 rounded-lg transition-colors ${
                        activeExample === key 
                          ? 'bg-blue-100 border-blue-300 border' 
                          : 'bg-gray-50 hover:bg-gray-100'
                      }`}
                    >
                      <h4 className="font-medium text-sm">{example.title}</h4>
                      <p className="text-xs text-gray-600 mt-1">{example.description}</p>
                    </button>
                  ))}
                </div>
              </ScrollArea>
            </Card>

            {/* Code Display */}
            <Card className="lg:col-span-2 p-6">
              <div className="flex items-center justify-between mb-4">
                <div>
                  <h3 className="text-lg font-semibold">{codeExamples[activeExample].title}</h3>
                  <p className="text-sm text-gray-600">{codeExamples[activeExample].description}</p>
                </div>
                <div className="flex gap-2">
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => copyToClipboard(codeExamples[activeExample].code)}
                    className="gap-2"
                  >
                    <Copy className="h-4 w-4" />
                    Copy
                  </Button>
                  <Button variant="outline" size="sm" className="gap-2">
                    <Download className="h-4 w-4" />
                    Download
                  </Button>
                </div>
              </div>
              
              <ScrollArea className="h-96">
                <pre className="bg-gray-900 text-green-400 p-4 rounded-lg text-xs overflow-x-auto">
                  <code>{codeExamples[activeExample].code}</code>
                </pre>
              </ScrollArea>
            </Card>
          </div>
        </TabsContent>

        {/* Setup Guide */}
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
                  {section.steps.map((step, stepIndex) => (
                    <div key={stepIndex} className="flex items-start gap-3">
                      <div className="w-6 h-6 bg-gray-200 rounded-full flex items-center justify-center text-xs font-medium mt-0.5">
                        {stepIndex + 1}
                      </div>
                      <p className="text-sm text-gray-700 flex-1">{step}</p>
                    </div>
                  ))}
                </div>
              </Card>
            ))}
          </div>

          {/* Quick Start Commands */}
          <Card className="p-6 mt-6">
            <h3 className="text-lg font-semibold mb-4">Quick Start Commands</h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div>
                <h4 className="font-medium mb-2">สร้าง ROS Package</h4>
                <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
{`cd ~/catkin_ws/src
catkin_create_pkg my_robot_pkg rospy std_msgs geometry_msgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash`}
                </pre>
              </div>
              
              <div>
                <h4 className="font-medium mb-2">รัน Python Node</h4>
                <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
{`chmod +x scripts/my_node.py
roscore
rosrun my_robot_pkg my_node.py`}
                </pre>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* API Reference */}
        <TabsContent value="api">
          <div className="space-y-6">
            {apiReference.map((category, index) => (
              <Card key={index} className="p-6">
                <h3 className="text-lg font-semibold mb-4">{category.category}</h3>
                <div className="space-y-3">
                  {category.endpoints.map((endpoint, endpointIndex) => (
                    <div key={endpointIndex} className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                      <div>
                        <code className="text-sm font-mono bg-blue-100 text-blue-800 px-2 py-1 rounded">
                          {endpoint.name}
                        </code>
                        <p className="text-sm text-gray-600 mt-1">{endpoint.description}</p>
                      </div>
                      <Badge variant="outline">{endpoint.type}</Badge>
                    </div>
                  ))}
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        {/* Best Practices */}
        <TabsContent value="best-practices">
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            {bestPractices.map((section, index) => (
              <Card key={index} className="p-6">
                <div className="flex items-center gap-3 mb-4">
                  <div className="p-2 bg-green-100 rounded-lg">
                    <section.icon className="h-5 w-5 text-green-600" />
                  </div>
                  <h3 className="text-lg font-semibold">{section.title}</h3>
                </div>
                
                <div className="space-y-3">
                  {section.practices.map((practice, practiceIndex) => (
                    <div key={practiceIndex} className="flex items-start gap-3">
                      <CheckCircle className="h-4 w-4 text-green-500 mt-1 flex-shrink-0" />
                      <p className="text-sm text-gray-700">{practice}</p>
                    </div>
                  ))}
                </div>
              </Card>
            ))}
          </div>

          {/* Code Quality Tips */}
          <Card className="p-6 mt-6">
            <h3 className="text-lg font-semibold mb-4">Code Quality Tips</h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div>
                <h4 className="font-medium mb-3 flex items-center gap-2">
                  <CheckCircle className="h-4 w-4 text-green-500" />
                  Good Practices
                </h4>
                <div className="space-y-2 text-sm">
                  <p>✅ ใช้ type hints ใน function signatures</p>
                  <p>✅ เขียน docstrings สำหรับ functions และ classes</p>
                  <p>✅ ใช้ meaningful variable names</p>
                  <p>✅ Handle exceptions properly</p>
                  <p>✅ Use rospy.loginfo() แทน print()</p>
                </div>
              </div>
              
              <div>
                <h4 className="font-medium mb-3 flex items-center gap-2">
                  <AlertTriangle className="h-4 w-4 text-red-500" />
                  Avoid These
                </h4>
                <div className="space-y-2 text-sm">
                  <p>❌ Hardcoding values ใน code</p>
                  <p>❌ Using global variables unnecessarily</p>
                  <p>❌ Blocking operations ใน ROS callbacks</p>
                  <p>❌ Ignoring ROS shutdown signals</p>
                  <p>❌ Not validating input parameters</p>
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Deployment */}
        <TabsContent value="deployment">
          <div className="space-y-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Production Deployment</h3>
              
              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                <div>
                  <h4 className="font-medium mb-3">Launch File Configuration</h4>
                  <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
{`<launch>
  <node name="robot_controller" 
        pkg="my_robot_pkg" 
        type="robot_controller.py" 
        output="screen">
    <param name="max_speed" value="1.0"/>
    <param name="config_file" value="$(find my_robot_pkg)/config/robot.yaml"/>
  </node>
  
  <node name="websocket_bridge" 
        pkg="my_robot_pkg" 
        type="websocket_bridge.py" 
        output="screen"/>
</launch>`}
                  </pre>
                </div>
                
                <div>
                  <h4 className="font-medium mb-3">Systemd Service</h4>
                  <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
{`[Unit]
Description=Robot Control System
After=network.target

[Service]
Type=simple
User=robot
Environment=ROS_MASTER_URI=http://localhost:11311
ExecStart=/opt/ros/noetic/bin/roslaunch my_robot_pkg robot.launch
Restart=always

[Install]
WantedBy=multi-user.target`}
                  </pre>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Monitoring & Logging</h3>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                <div className="p-4 bg-blue-50 rounded-lg">
                  <Database className="h-6 w-6 text-blue-600 mb-2" />
                  <h4 className="font-medium">ROS Logs</h4>
                  <p className="text-sm text-gray-600">ใช้ rospy.loginfo, logwarn, logerr</p>
                </div>
                
                <div className="p-4 bg-green-50 rounded-lg">
                  <Activity className="h-6 w-6 text-green-600 mb-2" />
                  <h4 className="font-medium">System Monitoring</h4>
                  <p className="text-sm text-gray-600">Monitor CPU, memory, network</p>
                </div>
                
                <div className="p-4 bg-purple-50 rounded-lg">
                  <Globe className="h-6 w-6 text-purple-600 mb-2" />
                  <h4 className="font-medium">Remote Monitoring</h4>
                  <p className="text-sm text-gray-600">WebSocket real-time updates</p>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Troubleshooting */}
        <TabsContent value="troubleshooting">
          <div className="space-y-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Common Issues & Solutions</h3>
              
              <div className="space-y-4">
                <div className="p-4 border-l-4 border-red-500 bg-red-50">
                  <h4 className="font-medium text-red-800">ImportError: No module named 'rospy'</h4>
                  <p className="text-sm text-red-700 mt-1">
                    <strong>Solution:</strong> ตรวจสอบว่า source ROS environment แล้วหรือยัง: source /opt/ros/noetic/setup.bash
                  </p>
                </div>
                
                <div className="p-4 border-l-4 border-yellow-500 bg-yellow-50">
                  <h4 className="font-medium text-yellow-800">Node ไม่สามารถเชื่อมต่อกับ ROS Master</h4>
                  <p className="text-sm text-yellow-700 mt-1">
                    <strong>Solution:</strong> ตรวจสอบ ROS_MASTER_URI และเริ่ม roscore ก่อน: roscore
                  </p>
                </div>
                
                <div className="p-4 border-l-4 border-blue-500 bg-blue-50">
                  <h4 className="font-medium text-blue-800">WebSocket connection failed</h4>
                  <p className="text-sm text-blue-700 mt-1">
                    <strong>Solution:</strong> ตรวจสอบ firewall settings และ port 9090 เปิดอยู่หรือไม่
                  </p>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Debug Tools</h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div>
                  <h4 className="font-medium mb-2">ROS Debug Commands</h4>
                  <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
{`# ดู active nodes
rosnode list

# ดู topics ที่มี
rostopic list

# Monitor topic data
rostopic echo /cmd_vel

# ดู node information
rosnode info /my_node`}
                  </pre>
                </div>
                
                <div>
                  <h4 className="font-medium mb-2">Python Debug</h4>
                  <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm">
{`# ใช้ pdb debugger
import pdb; pdb.set_trace()

# Print debugging
rospy.logdebug("Debug message")

# Exception handling
try:
    # code here
except Exception as e:
    rospy.logerr(f"Error: {e}")`}
                  </pre>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>
      </Tabs>

      {/* Quick Reference Card */}
      <Card className="p-6 mt-8 bg-gradient-to-r from-blue-50 to-purple-50 border-blue-200">
        <div className="flex items-center gap-3 mb-4">
          <Info className="h-6 w-6 text-blue-600" />
          <h3 className="text-lg font-semibold">Quick Reference</h3>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-4 gap-4 text-sm">
          <div>
            <h4 className="font-medium mb-2">Essential Commands</h4>
            <p><code>roscore</code> - Start ROS master</p>
            <p><code>rosrun</code> - Run single node</p>
            <p><code>roslaunch</code> - Run launch file</p>
          </div>
          
          <div>
            <h4 className="font-medium mb-2">File Locations</h4>
            <p><code>~/catkin_ws/src</code> - Source code</p>
            <p><code>/opt/ros/noetic</code> - ROS installation</p>
            <p><code>~/.ros/log</code> - Log files</p>
          </div>
          
          <div>
            <h4 className="font-medium mb-2">Important Ports</h4>
            <p><code>11311</code> - ROS Master</p>
            <p><code>9090</code> - WebSocket Bridge</p>
            <p><code>8080</code> - Web Interface</p>
          </div>
          
          <div>
            <h4 className="font-medium mb-2">Environment Variables</h4>
            <p><code>ROS_MASTER_URI</code></p>
            <p><code>ROS_PACKAGE_PATH</code></p>
            <p><code>PYTHONPATH</code></p>
          </div>
        </div>
      </Card>
    </div>
  );
}
