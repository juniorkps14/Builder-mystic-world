import React, { useState, useEffect } from "react";
import {
  usePersistentState,
  usePersistentArray,
} from "@/hooks/use-persistence";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { ScrollArea } from "@/components/ui/scroll-area";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Database,
  Network,
  Send,
  PlayCircle,
  Download,
  Copy,
  CheckCircle,
  AlertTriangle,
  Clock,
  Zap,
  Globe,
  Terminal,
  Code,
  FileText,
  Activity,
  Monitor,
  Settings,
} from "lucide-react";

interface APIEndpoint {
  id: string;
  name: string;
  method: "GET" | "POST" | "PUT" | "DELETE" | "WEBSOCKET";
  endpoint: string;
  description: string;
  category: string;
  status: "active" | "inactive" | "deprecated";
  requestExample?: any;
  responseExample?: any;
  pythonExample?: string;
}

interface APICall {
  id: string;
  timestamp: Date;
  endpoint: string;
  method: string;
  status: number;
  duration: number;
  success: boolean;
}

export default function APIManagement() {
  // Persistent API testing state
  const { state: selectedCategory, setState: setSelectedCategory } =
    usePersistentState({
      key: "api-selected-category",
      defaultValue: "all",
    });

  const { state: testEndpoint, setState: setTestEndpoint } = usePersistentState(
    {
      key: "api-test-endpoint",
      defaultValue: "",
    },
  );

  const { state: testMethod, setState: setTestMethod } = usePersistentState({
    key: "api-test-method",
    defaultValue: "GET",
  });

  const { state: testPayload, setState: setTestPayload } = usePersistentState({
    key: "api-test-payload",
    defaultValue: "",
  });

  const { state: testResponse, setState: setTestResponse } = usePersistentState(
    {
      key: "api-test-response",
      defaultValue: "",
    },
  );

  const [isLoading, setIsLoading] = useState(false);

  // Persistent API call history
  const {
    items: apiCalls,
    addItem: addApiCall,
    clearItems: clearApiCalls,
  } = usePersistentArray<APICall>(
    "api-call-history",
    50, // Keep last 50 API calls
  );

  const apiEndpoints: APIEndpoint[] = [
    // Robot Control APIs
    {
      id: "1",
      name: "Send Velocity Command",
      method: "POST",
      endpoint: "/api/robot/cmd_vel",
      description: "Send velocity commands to the robot",
      category: "robot_control",
      status: "active",
      requestExample: {
        linear: { x: 0.5, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.5 },
      },
      responseExample: { success: true, message: "Command sent" },
      pythonExample: `import requests
import json

# ส่งคำสั่งการเคลื่อนที่
def send_velocity_command(linear_x, linear_y, angular_z):
    url = "http://localhost:8080/api/robot/cmd_vel"
    payload = {
        "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
    }

    response = requests.post(url, json=payload)
    return response.json()

# ตัวอย่างการใช้งาน
result = send_velocity_command(0.5, 0.0, 0.5)  # เดินหน้า + เลี้ยวขวา
print(result)`,
    },
    {
      id: "2",
      name: "Get Robot Status",
      method: "GET",
      endpoint: "/api/robot/status",
      description: "Get current robot status and position",
      category: "robot_control",
      status: "active",
      responseExample: {
        state: "idle",
        position: { x: 1.5, y: 2.3, theta: 0.5 },
        battery: 87,
        emergency_stop: false,
      },
      pythonExample: `import requests

def get_robot_status():
    url = "http://localhost:8080/api/robot/status"
    response = requests.get(url)
    return response.json()

# ตัวอย่างการใช้งาน
status = get_robot_status()
print(f"Robot state: {status['state']}")
print(f"Battery: {status['battery']}%")
print(f"Position: x={status['position']['x']}, y={status['position']['y']}")`,
    },
    {
      id: "3",
      name: "Emergency Stop",
      method: "POST",
      endpoint: "/api/robot/emergency_stop",
      description: "Emergency stop for robot systems",
      category: "safety",
      status: "active",
      requestExample: { activate: true },
      responseExample: { success: true, emergency_active: true },
      pythonExample: `import requests

def emergency_stop(activate=True):
    url = "http://localhost:8080/api/robot/emergency_stop"
    payload = {"activate": activate}

    response = requests.post(url, json=payload)
    return response.json()

# ตัวอย่างการใช้งาน
result = emergency_stop(True)   # เปิดใช้งาน emergency stop
result = emergency_stop(False)  # ปิด emergency stop`,
    },
    // Navigation APIs
    {
      id: "4",
      name: "Navigate to Position",
      method: "POST",
      endpoint: "/api/navigation/goto",
      description: "สั่งให้หุ่นยนต์เดินทางไปยังตำแหน่งที่กำหนด",
      category: "navigation",
      status: "active",
      requestExample: {
        target: { x: 5.0, y: 3.0, theta: 1.57 },
        frame_id: "map",
      },
      responseExample: { goal_id: "goal_123", status: "sent" },
      pythonExample: `import requests

def navigate_to_position(x, y, theta=0.0):
    url = "http://localhost:8080/api/navigation/goto"
    payload = {
        "target": {"x": x, "y": y, "theta": theta},
        "frame_id": "map"
    }

    response = requests.post(url, json=payload)
    return response.json()

# ตัวอย่างการใช้งาน
result = navigate_to_position(5.0, 3.0, 1.57)
print(f"Goal ID: {result['goal_id']}")`,
    },
    {
      id: "5",
      name: "Get Current Map",
      method: "GET",
      endpoint: "/api/navigation/map",
      description: "ดึงแผนที่ปัจจุบัน",
      category: "navigation",
      status: "active",
      responseExample: {
        map_data: "base64_encoded_map",
        resolution: 0.05,
        width: 400,
        height: 400,
      },
      pythonExample: `import requests
import base64
import numpy as np
from PIL import Image

def get_current_map():
    url = "http://localhost:8080/api/navigation/map"
    response = requests.get(url)
    return response.json()

def save_map_image(map_data, filename="map.png"):
    # แปลง base64 เป็นรูปภาพ
    map_bytes = base64.b64decode(map_data['map_data'])

    # สร้างรูปภาพ
    img_array = np.frombuffer(map_bytes, dtype=np.uint8)
    img_array = img_array.reshape(map_data['height'], map_data['width'])

    # บันทึกเป็นไฟ��์รูปภาพ
    img = Image.fromarray(img_array)
    img.save(filename)

# ตัวอย่างการใช้งาน
map_data = get_current_map()
save_map_image(map_data, "current_map.png")`,
    },
    // Sensor APIs
    {
      id: "6",
      name: "Get Camera Feed",
      method: "GET",
      endpoint: "/api/sensors/camera/image",
      description: "ดึงภาพจากกล้อง",
      category: "sensors",
      status: "active",
      responseExample: {
        image: "base64_encoded_image",
        timestamp: 1703123456.789,
        width: 640,
        height: 480,
      },
      pythonExample: `import requests
import base64
from PIL import Image
import io

def get_camera_image():
    url = "http://localhost:8080/api/sensors/camera/image"
    response = requests.get(url)
    return response.json()

def save_camera_image(image_data, filename="camera.jpg"):
    # แปลง base64 เป็นรูปภาพ
    image_bytes = base64.b64decode(image_data['image'])

    # สร้างและบันทึกรูปภาพ
    img = Image.open(io.BytesIO(image_bytes))
    img.save(filename)

    return img

# ตัวอย่างการใช้งาน
image_data = get_camera_image()
img = save_camera_image(image_data, "latest_photo.jpg")
print(f"Image size: {img.size}")`,
    },
    {
      id: "7",
      name: "Get Sensor Data",
      method: "GET",
      endpoint: "/api/sensors/all",
      description: "ดึงข้อมูลเซ็นเซอร์ทั้งหมด",
      category: "sensors",
      status: "active",
      responseExample: {
        lidar: { ranges: [1.2, 1.5, 2.0], angle_min: -1.57, angle_max: 1.57 },
        imu: { acceleration: { x: 0.1, y: 0.2, z: 9.8 } },
        battery: { voltage: 12.5, percentage: 87 },
      },
      pythonExample: `import requests

def get_all_sensors():
    url = "http://localhost:8080/api/sensors/all"
    response = requests.get(url)
    return response.json()

def monitor_sensors():
    while True:
        sensors = get_all_sensors()

        # แสดงข้อมูลเซ็นเซอร์
        print(f"Battery: {sensors['battery']['percentage']}%")
        print(f"IMU Acceleration: {sensors['imu']['acceleration']}")
        print(f"LiDAR min distance: {min(sensors['lidar']['ranges']):.2f}m")

        time.sleep(1)

# ตัวอย่างการใช้งาน
monitor_sensors()`,
    },
    // System APIs
    {
      id: "8",
      name: "Get System Health",
      method: "GET",
      endpoint: "/api/system/health",
      description: "ตรวจสอบสุขภาพระบบ",
      category: "system",
      status: "active",
      responseExample: {
        cpu_usage: 45.2,
        memory_usage: 68.5,
        disk_usage: 23.1,
        ros_nodes: 12,
        uptime: 86400,
      },
      pythonExample: `import requests

def get_system_health():
    url = "http://localhost:8080/api/system/health"
    response = requests.get(url)
    return response.json()

def check_system_status():
    health = get_system_health()

    # ตรวจสอบสถานะระ��บ
    if health['cpu_usage'] > 80:
        print("Warning: High CPU usage!")

    if health['memory_usage'] > 90:
        print("Warning: High memory usage!")

    print(f"System uptime: {health['uptime']/3600:.1f} hours")
    print(f"Active ROS nodes: {health['ros_nodes']}")

# ตัวอย่างการใช้งาน
check_system_status()`,
    },
    // Sequence APIs
    {
      id: "9",
      name: "Execute Sequence",
      method: "POST",
      endpoint: "/api/sequences/execute",
      description: "เรียกใช้ sequence ที่กำหนด",
      category: "sequences",
      status: "active",
      requestExample: {
        sequence_name: "patrol_routine",
        parameters: { speed: 0.5, loops: 3 },
      },
      responseExample: {
        execution_id: "exec_123",
        status: "started",
        estimated_duration: 300,
      },
      pythonExample: `import requests

def execute_sequence(sequence_name, parameters=None):
    url = "http://localhost:8080/api/sequences/execute"
    payload = {
        "sequence_name": sequence_name,
        "parameters": parameters or {}
    }

    response = requests.post(url, json=payload)
    return response.json()

def get_sequence_status(execution_id):
    url = f"http://localhost:8080/api/sequences/status/{execution_id}"
    response = requests.get(url)
    return response.json()

# ���ัวอย่างการใช้งาน
result = execute_sequence("patrol_routine", {"speed": 0.5, "loops": 3})
exec_id = result['execution_id']

# ติดตามสถานะ
status = get_sequence_status(exec_id)
print(f"Sequence status: {status['status']}")`,
    },
    // WebSocket APIs
    {
      id: "10",
      name: "Real-time Data Stream",
      method: "WEBSOCKET",
      endpoint: "ws://localhost:9090",
      description: "รับข้อมูลแบบ real-time ผ่าน WebSocket",
      category: "realtime",
      status: "active",
      pythonExample: `import asyncio
import websockets
import json

async def robot_data_stream():
    uri = "ws://localhost:9090"

    async with websockets.connect(uri) as websocket:
        # สมัครรับข้อมูล
        subscribe_msg = {
            "type": "subscribe",
            "topics": ["robot_status", "camera_frame", "sensor_data"]
        }
        await websocket.send(json.dumps(subscribe_msg))

        # รับข้อมูลแบบ real-time
        async for message in websocket:
            data = json.loads(message)

            if data['type'] == 'robot_status':
                print(f"Robot: {data['state']} at position {data['position']}")

            elif data['type'] == 'camera_frame':
                print(f"New camera frame received: {data['width']}x{data['height']}")

            elif data['type'] == 'sensor_data':
                print(f"Sensors updated: {data.keys()}")

# ตัวอย่างการใช้งาน
asyncio.run(robot_data_stream())`,
    },
  ];

  const categories = [
    { value: "all", label: "ทั้งหมด", icon: Globe },
    { value: "robot_control", label: "ควบคุมหุ่นยนต์", icon: Activity },
    { value: "navigation", label: "การนำทาง", icon: Network },
    { value: "sensors", label: "เซ็นเซอร์", icon: Monitor },
    { value: "system", label: "ระบบ", icon: Settings },
    { value: "sequences", label: "ลำดับงาน", icon: PlayCircle },
    { value: "safety", label: "ความปลอดภัย", icon: AlertTriangle },
    { value: "realtime", label: "ข้อมูลแบบ Real-time", icon: Zap },
  ];

  const filteredEndpoints =
    selectedCategory === "all"
      ? apiEndpoints
      : apiEndpoints.filter((ep) => ep.category === selectedCategory);

  const executeAPITest = async () => {
    setIsLoading(true);
    setTestResponse("");

    try {
      const startTime = Date.now();

      // Simulate API call
      await new Promise((resolve) =>
        setTimeout(resolve, Math.random() * 1000 + 500),
      );

      const endTime = Date.now();
      const duration = endTime - startTime;

      // Mock response
      const mockResponse = {
        success: true,
        data: {
          message: "API test successful",
          timestamp: new Date().toISOString(),
        },
        duration: duration,
      };

      setTestResponse(JSON.stringify(mockResponse, null, 2));

      // Add to call history
      const newCall: APICall = {
        id: Date.now().toString(),
        timestamp: new Date(),
        endpoint: testEndpoint,
        method: testMethod,
        status: 200,
        duration,
        success: true,
      };

      addApiCall(newCall);
    } catch (error) {
      setTestResponse(JSON.stringify({ error: "API call failed" }, null, 2));
    } finally {
      setIsLoading(false);
    }
  };

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
  };

  const generatePythonSDK = () => {
    const sdkCode = `"""
Robot Control System Python SDK
SDK สำหรับกา��เชื่อมต่อและควบคุมหุ่นยนต์
"""

import requests
import asyncio
import websockets
import json
import base64
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from PIL import Image
import io

class RobotAPI:
    def __init__(self, base_url: str = "http://localhost:8080", ws_url: str = "ws://localhost:9090"):
        self.base_url = base_url.rstrip('/')
        self.ws_url = ws_url
        self.session = requests.Session()

    # Robot Control Methods
    def send_velocity(self, linear_x: float, linear_y: float = 0.0, angular_z: float = 0.0) -> Dict:
        """ส่งคำสั่งความเร็วไปยังหุ่นยนต์"""
        payload = {
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
        }
        response = self.session.post(f"{self.base_url}/api/robot/cmd_vel", json=payload)
        return response.json()

    def get_robot_status(self) -> Dict:
        """ดึงสถานะปัจจุบันของหุ่นยนต์"""
        response = self.session.get(f"{self.base_url}/api/robot/status")
        return response.json()

    def emergency_stop(self, activate: bool = True) -> Dict:
        """เปิด/ปิด emergency stop"""
        payload = {"activate": activate}
        response = self.session.post(f"{self.base_url}/api/robot/emergency_stop", json=payload)
        return response.json()

    # Navigation Methods
    def navigate_to(self, x: float, y: float, theta: float = 0.0) -> Dict:
        """สั่งให้หุ่นยนต์เดินทางไปยังตำแหน่งที่กำหนด"""
        payload = {
            "target": {"x": x, "y": y, "theta": theta},
            "frame_id": "map"
        }
        response = self.session.post(f"{self.base_url}/api/navigation/goto", json=payload)
        return response.json()

    def get_map(self) -> Dict:
        """ดึงแผนที่ปัจจุบัน"""
        response = self.session.get(f"{self.base_url}/api/navigation/map")
        return response.json()

    # Sensor Methods
    def get_camera_image(self) -> Image.Image:
        """ดึงภาพจากกล้องและคืนค่าเป็น PIL Image"""
        response = self.session.get(f"{self.base_url}/api/sensors/camera/image")
        data = response.json()

        image_bytes = base64.b64decode(data['image'])
        img = Image.open(io.BytesIO(image_bytes))
        return img

    def get_all_sensors(self) -> Dict:
        """ดึงข้อมูลเซ็นเซอร์ทั้งหมด"""
        response = self.session.get(f"{self.base_url}/api/sensors/all")
        return response.json()

    # System Methods
    def get_system_health(self) -> Dict:
        """ตรวจสอบสุขภาพระบบ"""
        response = self.session.get(f"{self.base_url}/api/system/health")
        return response.json()

    # Sequence Methods
    def execute_sequence(self, sequence_name: str, parameters: Optional[Dict] = None) -> Dict:
        """เรียกใช้ sequence ที่กำหนด"""
        payload = {
            "sequence_name": sequence_name,
            "parameters": parameters or {}
        }
        response = self.session.post(f"{self.base_url}/api/sequences/execute", json=payload)
        return response.json()

    def get_sequence_status(self, execution_id: str) -> Dict:
        """ตรวจสอบสถานะการทำงานของ sequence"""
        response = self.session.get(f"{self.base_url}/api/sequences/status/{execution_id}")
        return response.json()

    # WebSocket Methods
    async def subscribe_realtime_data(self, topics: List[str], callback):
        """สมัครรับข้อมูลแบบ real-time ผ่าน WebSocket"""
        async with websockets.connect(self.ws_url) as websocket:
            # ส่งคำขอ subscribe
            subscribe_msg = {
                "type": "subscribe",
                "topics": topics
            }
            await websocket.send(json.dumps(subscribe_msg))

            # รับข้อมูลและเรียก callback
            async for message in websocket:
                data = json.loads(message)
                await callback(data)

# ตัวอย่างการใช้งาน
if __name__ == "__main__":
    # สร้าง instance ขอ��� API
    robot = RobotAPI()

    # ตรวจสอบสถานะหุ่นยนต์
    status = robot.get_robot_status()
    print(f"Robot state: {status['state']}")

    # เคลื่อนที่หุ่นยนต์
    robot.send_velocity(0.5, 0.0, 0.0)  # เดินหน้า

    # ถ่ายรูป
    image = robot.get_camera_image()
    image.save("robot_photo.jpg")

    # สั่���ให้ไปยังตำแหน่งที่กำหนด
    result = robot.navigate_to(5.0, 3.0, 1.57)
    print(f"Navigation goal: {result['goal_id']}")
`;

    return sdkCode;
  };

  const downloadSDK = () => {
    const sdkCode = generatePythonSDK();
    const blob = new Blob([sdkCode], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "robot_api_sdk.py";
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            API Management
          </h1>
          <p className="text-slate-300 font-light mt-2">
            Manage all robot system APIs and test connections
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge className="bg-green-100 text-green-700">
            <Activity className="h-3 w-3 mr-1" />
            {apiEndpoints.filter((ep) => ep.status === "active").length} Active
          </Badge>
          <Button onClick={downloadSDK} className="gap-2">
            <Download className="h-4 w-4" />
            Download Python SDK
          </Button>
        </div>
      </div>

      <Tabs defaultValue="endpoints" className="space-y-6">
        <TabsList className="grid w-full grid-cols-4">
          <TabsTrigger value="endpoints" className="gap-2">
            <Database className="h-4 w-4" />
            API Endpoints
          </TabsTrigger>
          <TabsTrigger value="testing" className="gap-2">
            <Send className="h-4 w-4" />
            API Testing
          </TabsTrigger>
          <TabsTrigger value="python" className="gap-2">
            <Code className="h-4 w-4" />
            Python Examples
          </TabsTrigger>
          <TabsTrigger value="logs" className="gap-2">
            <Activity className="h-4 w-4" />
            API Logs
          </TabsTrigger>
        </TabsList>

        {/* API Endpoints */}
        <TabsContent value="endpoints">
          <div className="flex gap-6">
            {/* Category Filter */}
            <div className="w-64">
              <Card className="p-4">
                <h3 className="font-semibold mb-4">หมวดหมู่ API</h3>
                <div className="space-y-2">
                  {categories.map((category) => {
                    const IconComponent = category.icon;
                    const count =
                      category.value === "all"
                        ? apiEndpoints.length
                        : apiEndpoints.filter(
                            (ep) => ep.category === category.value,
                          ).length;

                    return (
                      <button
                        key={category.value}
                        onClick={() => setSelectedCategory(category.value)}
                        className={`w-full flex items-center justify-between p-3 rounded-lg text-left transition-colors ${
                          selectedCategory === category.value
                            ? "bg-blue-500/20 text-blue-300 border-blue-500/30 border"
                            : "hover:bg-white/10"
                        }`}
                      >
                        <div className="flex items-center gap-2">
                          <IconComponent className="h-4 w-4" />
                          <span className="text-sm">{category.label}</span>
                        </div>
                        <Badge variant="outline" className="text-xs">
                          {count}
                        </Badge>
                      </button>
                    );
                  })}
                </div>
              </Card>
            </div>

            {/* API List */}
            <div className="flex-1">
              <div className="space-y-4">
                {filteredEndpoints.map((endpoint) => (
                  <Card key={endpoint.id} className="p-6">
                    <div className="flex items-start justify-between mb-4">
                      <div>
                        <div className="flex items-center gap-3 mb-2">
                          <Badge
                            className={
                              endpoint.method === "GET"
                                ? "bg-green-100 text-green-700"
                                : endpoint.method === "POST"
                                  ? "bg-blue-100 text-blue-700"
                                  : endpoint.method === "PUT"
                                    ? "bg-orange-100 text-orange-700"
                                    : endpoint.method === "DELETE"
                                      ? "bg-red-100 text-red-700"
                                      : "bg-purple-100 text-purple-700"
                            }
                          >
                            {endpoint.method}
                          </Badge>
                          <h3 className="font-semibold">{endpoint.name}</h3>
                        </div>
                        <code className="text-sm bg-white/10 px-2 py-1 rounded text-slate-300">
                          {endpoint.endpoint}
                        </code>
                        <p className="text-sm text-gray-600 mt-2">
                          {endpoint.description}
                        </p>
                      </div>

                      <Badge
                        className={
                          endpoint.status === "active"
                            ? "bg-green-100 text-green-700"
                            : endpoint.status === "inactive"
                              ? "bg-white/5 border border-white/10 text-slate-300"
                              : "bg-yellow-100 text-yellow-700"
                        }
                      >
                        {endpoint.status}
                      </Badge>
                    </div>

                    {/* Request/Response Examples */}
                    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                      {endpoint.requestExample && (
                        <div>
                          <h4 className="font-medium text-sm mb-2">
                            Request Example:
                          </h4>
                          <pre className="bg-gray-900 text-green-400 p-3 rounded text-xs overflow-x-auto">
                            {JSON.stringify(endpoint.requestExample, null, 2)}
                          </pre>
                        </div>
                      )}

                      {endpoint.responseExample && (
                        <div>
                          <h4 className="font-medium text-sm mb-2">
                            Response Example:
                          </h4>
                          <pre className="bg-gray-900 text-green-400 p-3 rounded text-xs overflow-x-auto">
                            {JSON.stringify(endpoint.responseExample, null, 2)}
                          </pre>
                        </div>
                      )}
                    </div>
                  </Card>
                ))}
              </div>
            </div>
          </div>
        </TabsContent>

        {/* API Testing */}
        <TabsContent value="testing">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {/* Test Form */}
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">ทดสอบ API</h3>

              <div className="space-y-4">
                <div>
                  <label className="block text-sm font-medium mb-2">
                    HTTP Method
                  </label>
                  <Select value={testMethod} onValueChange={setTestMethod}>
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="GET">GET</SelectItem>
                      <SelectItem value="POST">POST</SelectItem>
                      <SelectItem value="PUT">PUT</SelectItem>
                      <SelectItem value="DELETE">DELETE</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <label className="block text-sm font-medium mb-2">
                    Endpoint
                  </label>
                  <Input
                    value={testEndpoint}
                    onChange={(e) => setTestEndpoint(e.target.value)}
                    placeholder="/api/robot/status"
                  />
                </div>

                {testMethod !== "GET" && (
                  <div>
                    <label className="block text-sm font-medium mb-2">
                      Request Body (JSON)
                    </label>
                    <Textarea
                      value={testPayload}
                      onChange={(e) => setTestPayload(e.target.value)}
                      placeholder='{"key": "value"}'
                      rows={4}
                    />
                  </div>
                )}

                <Button
                  onClick={executeAPITest}
                  disabled={isLoading}
                  className="w-full gap-2"
                >
                  {isLoading ? (
                    <>
                      <Clock className="h-4 w-4 animate-spin" />
                      Executing...
                    </>
                  ) : (
                    <>
                      <Send className="h-4 w-4" />
                      Execute API Call
                    </>
                  )}
                </Button>
              </div>
            </Card>

            {/* Response */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-semibold">Response</h3>
                {testResponse && (
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => copyToClipboard(testResponse)}
                    className="gap-2"
                  >
                    <Copy className="h-4 w-4" />
                    Copy
                  </Button>
                )}
              </div>

              <ScrollArea className="h-80">
                <pre className="bg-gray-900 text-green-400 p-4 rounded text-sm whitespace-pre-wrap">
                  {testResponse ||
                    "No response yet. Execute an API call to see the result."}
                </pre>
              </ScrollArea>
            </Card>
          </div>
        </TabsContent>

        {/* Python Examples */}
        <TabsContent value="python">
          <div className="space-y-6">
            {/* SDK Download */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <div>
                  <h3 className="text-lg font-semibold">Python SDK</h3>
                  <p className="text-sm text-gray-600">
                    ดาวน์โหลด SDK สำหรับการเชื่อมต่อกับ API ทั้งหมด
                  </p>
                </div>
                <Button onClick={downloadSDK} className="gap-2">
                  <Download className="h-4 w-4" />
                  Download robot_api_sdk.py
                </Button>
              </div>

              <div className="bg-blue-50 border border-blue-200 rounded-lg p-4">
                <h4 className="font-medium text-blue-900 mb-2">
                  การติดตั้งและใช้งา��:
                </h4>
                <pre className="text-sm text-blue-800">
                  {`# ติดตั้��� dependencies
pip install requests websockets pillow

# ใช้งาน SDK
from robot_api_sdk import RobotAPI

robot = RobotAPI()
status = robot.get_robot_status()
print(status)`}
                </pre>
              </div>
            </Card>

            {/* Individual Examples */}
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              {filteredEndpoints
                .filter((ep) => ep.pythonExample)
                .map((endpoint) => (
                  <Card key={endpoint.id} className="p-6">
                    <div className="flex items-center justify-between mb-4">
                      <h4 className="font-medium">{endpoint.name}</h4>
                      <Button
                        variant="outline"
                        size="sm"
                        onClick={() => copyToClipboard(endpoint.pythonExample!)}
                        className="gap-2"
                      >
                        <Copy className="h-4 w-4" />
                        Copy
                      </Button>
                    </div>

                    <ScrollArea className="h-64">
                      <pre className="bg-gray-900 text-green-400 p-3 rounded text-xs whitespace-pre-wrap">
                        {endpoint.pythonExample}
                      </pre>
                    </ScrollArea>
                  </Card>
                ))}
            </div>
          </div>
        </TabsContent>

        {/* API Logs */}
        <TabsContent value="logs">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">
              การเรียกใช้ API ล��าสุด
            </h3>

            {apiCalls.length === 0 ? (
              <div className="text-center py-8 text-gray-500">
                <Activity className="h-12 w-12 mx-auto mb-4 opacity-50" />
                <p>ยังไม่มีการเรียกใช้ API</p>
                <p className="text-sm">ใช้แท็บ "API Testing" เพื่อทดสอบ API</p>
              </div>
            ) : (
              <div className="space-y-3">
                {apiCalls.map((call) => (
                  <div
                    key={call.id}
                    className="flex items-center justify-between p-4 bg-white/5 rounded-lg border border-white/10"
                  >
                    <div className="flex items-center gap-4">
                      <Badge
                        className={
                          call.method === "GET"
                            ? "bg-green-100 text-green-700"
                            : call.method === "POST"
                              ? "bg-blue-100 text-blue-700"
                              : "bg-purple-100 text-purple-700"
                        }
                      >
                        {call.method}
                      </Badge>

                      <div>
                        <code className="text-sm font-mono">
                          {call.endpoint}
                        </code>
                        <p className="text-xs text-gray-500">
                          {call.timestamp.toLocaleString()}
                        </p>
                      </div>
                    </div>

                    <div className="flex items-center gap-4">
                      <span className="text-sm text-gray-600">
                        {call.duration}ms
                      </span>

                      <Badge
                        className={
                          call.success
                            ? "bg-green-100 text-green-700"
                            : "bg-red-100 text-red-700"
                        }
                      >
                        {call.status}
                      </Badge>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
}
