import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Code,
  Play,
  Square,
  Save,
  FolderOpen,
  Download,
  Upload,
  Terminal,
  FileText,
  Zap,
  CheckCircle,
  AlertTriangle,
  Clock,
  Trash2,
  Copy,
  Edit,
  Settings,
  BookOpen,
  Eye,
  Bug,
} from "lucide-react";

interface CodeProject {
  id: string;
  name: string;
  description: string;
  language: "python" | "javascript" | "cpp" | "bash";
  type: "script" | "api" | "service" | "automation";
  code: string;
  isRunning: boolean;
  lastRun: Date | null;
  status: "ready" | "running" | "error" | "success";
  output: string[];
  apiEndpoint?: string;
  dependencies: string[];
  autoStart: boolean;
}

interface APIEndpoint {
  id: string;
  path: string;
  method: "GET" | "POST" | "PUT" | "DELETE";
  description: string;
  parameters: { name: string; type: string; required: boolean }[];
  response: any;
  enabled: boolean;
}

const CodeDevelopment = () => {
  const { t } = useLanguage();

  const [projects, setProjects] = useState<CodeProject[]>([
    {
      id: "proj_001",
      name: "Robot Controller",
      description: "Main robot control logic with serial communication",
      language: "python",
      type: "service",
      code: `#!/usr/bin/env python3
import serial
import json
import time
import threading
from flask import Flask, request, jsonify

app = Flask(__name__)

class RobotController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial_conn = None
        self.port = port
        self.baudrate = baudrate
        self.is_connected = False
        self.connect()
    
    def connect(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.is_connected = True
            print(f"Connected to {self.port}")
        except Exception as e:
            print(f"Connection failed: {e}")
            self.is_connected = False
    
    def send_command(self, command, params=None):
        if not self.is_connected:
            return {"error": "Not connected"}
        
        try:
            cmd_data = {
                "command": command,
                "params": params or {}
            }
            cmd_str = json.dumps(cmd_data) + '\\n'
            self.serial_conn.write(cmd_str.encode())
            
            # Read response
            response = self.serial_conn.readline().decode().strip()
            if response:
                return json.loads(response)
            return {"status": "no_response"}
            
        except Exception as e:
            return {"error": str(e)}

# Global robot controller instance
robot = RobotController()

@app.route('/api/robot/move', methods=['POST'])
def move_robot():
    data = request.json
    result = robot.send_command('MOVE', {
        'linear_x': data.get('linear_x', 0),
        'linear_y': data.get('linear_y', 0),
        'angular_z': data.get('angular_z', 0)
    })
    return jsonify(result)

@app.route('/api/robot/status', methods=['GET'])
def get_status():
    result = robot.send_command('GET_STATUS')
    return jsonify(result)

@app.route('/api/robot/stop', methods=['POST'])
def emergency_stop():
    result = robot.send_command('EMERGENCY_STOP')
    return jsonify(result)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)`,
      isRunning: false,
      lastRun: new Date(Date.now() - 300000),
      status: "ready",
      output: [
        "Starting robot controller service...",
        "Connected to /dev/ttyUSB0",
        "Flask server starting on port 5000",
        "API endpoints ready",
      ],
      apiEndpoint: "http://localhost:5000",
      dependencies: ["pyserial", "flask", "json"],
      autoStart: true,
    },
    {
      id: "proj_002",
      name: "Sensor Data Logger",
      description: "Collect and log sensor data from multiple sources",
      language: "python",
      type: "automation",
      code: `#!/usr/bin/env python3
import serial
import json
import csv
import time
from datetime import datetime
import threading

class SensorLogger:
    def __init__(self):
        self.sensors = {
            'imu': {'port': '/dev/ttyUSB1', 'baudrate': 9600},
            'environment': {'port': '/dev/ttyUSB2', 'baudrate': 9600}
        }
        self.connections = {}
        self.logging = False
        self.data_file = f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
    def connect_sensors(self):
        for sensor_name, config in self.sensors.items():
            try:
                conn = serial.Serial(
                    port=config['port'],
                    baudrate=config['baudrate'],
                    timeout=1
                )
                self.connections[sensor_name] = conn
                print(f"Connected to {sensor_name}: {config['port']}")
            except Exception as e:
                print(f"Failed to connect {sensor_name}: {e}")
    
    def start_logging(self):
        self.logging = True
        self.log_thread = threading.Thread(target=self._log_data)
        self.log_thread.start()
        print("Started sensor logging")
    
    def stop_logging(self):
        self.logging = False
        if hasattr(self, 'log_thread'):
            self.log_thread.join()
        print("Stopped sensor logging")
    
    def _log_data(self):
        with open(self.data_file, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'sensor', 'data']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            while self.logging:
                timestamp = datetime.now().isoformat()
                
                for sensor_name, conn in self.connections.items():
                    try:
                        # Send data request
                        if sensor_name == 'imu':
                            conn.write(b'GET_IMU\\n')
                        elif sensor_name == 'environment':
                            conn.write(b'GET_ALL_ENV\\n')
                        
                        # Read response
                        response = conn.readline().decode().strip()
                        if response:
                            writer.writerow({
                                'timestamp': timestamp,
                                'sensor': sensor_name,
                                'data': response
                            })
                            print(f"{timestamp} - {sensor_name}: {response}")
                            
                    except Exception as e:
                        print(f"Error reading {sensor_name}: {e}")
                
                time.sleep(0.1)  # 10Hz logging rate

# Main execution
logger = SensorLogger()
logger.connect_sensors()
logger.start_logging()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    logger.stop_logging()
    print("Sensor logging stopped")`,
      isRunning: false,
      lastRun: null,
      status: "ready",
      output: [],
      dependencies: ["pyserial", "csv"],
      autoStart: false,
    },
    {
      id: "proj_003",
      name: "Navigation API",
      description: "RESTful API for robot navigation commands",
      language: "python",
      type: "api",
      code: `#!/usr/bin/env python3
from flask import Flask, request, jsonify
import serial
import json
import math
import threading
import time

app = Flask(__name__)

class NavigationController:
    def __init__(self):
        self.robot_conn = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.current_position = {'x': 0, 'y': 0, 'theta': 0}
        self.target_position = None
        self.is_navigating = False
        
    def get_position(self):
        """Get current robot position"""
        try:
            self.robot_conn.write(b'GET_ODOM\\n')
            response = self.robot_conn.readline().decode().strip()
            if response:
                data = json.loads(response)
                self.current_position = data
            return self.current_position
        except Exception as e:
            return {"error": str(e)}
    
    def move_to_position(self, x, y, theta=None):
        """Navigate to target position"""
        self.target_position = {'x': x, 'y': y, 'theta': theta}
        self.is_navigating = True
        
        # Start navigation thread
        nav_thread = threading.Thread(target=self._navigate)
        nav_thread.start()
        
        return {"status": "navigation_started", "target": self.target_position}
    
    def _navigate(self):
        """Internal navigation logic"""
        tolerance = 0.1  # 10cm tolerance
        
        while self.is_navigating:
            current = self.get_position()
            target = self.target_position
            
            # Calculate distance to target
            dx = target['x'] - current['x']
            dy = target['y'] - current['y']
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < tolerance:
                # Reached target
                self.stop_robot()
                self.is_navigating = False
                break
            
            # Calculate heading to target
            target_heading = math.atan2(dy, dx)
            heading_error = target_heading - current['theta']
            
            # Normalize heading error
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            
            # Simple P controller
            linear_speed = min(0.5, distance * 0.5)  # Max 0.5 m/s
            angular_speed = heading_error * 0.5  # Proportional turning
            
            # Send movement command
            self.send_velocity(linear_speed, 0, angular_speed)
            time.sleep(0.1)
    
    def send_velocity(self, linear_x, linear_y, angular_z):
        """Send velocity command to robot"""
        try:
            cmd = {
                "command": "MOVE",
                "params": {
                    "linear_x": linear_x,
                    "linear_y": linear_y,
                    "angular_z": angular_z
                }
            }
            self.robot_conn.write((json.dumps(cmd) + '\\n').encode())
        except Exception as e:
            print(f"Velocity command failed: {e}")
    
    def stop_robot(self):
        """Stop robot movement"""
        self.send_velocity(0, 0, 0)
        self.is_navigating = False

# Global navigation controller
nav_controller = NavigationController()

@app.route('/api/navigation/position', methods=['GET'])
def get_position():
    return jsonify(nav_controller.get_position())

@app.route('/api/navigation/goto', methods=['POST'])
def goto_position():
    data = request.json
    x = data.get('x', 0)
    y = data.get('y', 0)
    theta = data.get('theta')
    
    result = nav_controller.move_to_position(x, y, theta)
    return jsonify(result)

@app.route('/api/navigation/stop', methods=['POST'])
def stop_navigation():
    nav_controller.stop_robot()
    return jsonify({"status": "stopped"})

@app.route('/api/navigation/status', methods=['GET'])
def navigation_status():
    return jsonify({
        "is_navigating": nav_controller.is_navigating,
        "current_position": nav_controller.current_position,
        "target_position": nav_controller.target_position
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)`,
      isRunning: false,
      lastRun: null,
      status: "ready",
      output: [],
      apiEndpoint: "http://localhost:5001",
      dependencies: ["flask", "pyserial", "math"],
      autoStart: false,
    },
  ]);

  const [selectedProject, setSelectedProject] = useState<string | null>(null);
  const [newProject, setNewProject] = useState({
    name: "",
    description: "",
    language: "python" as const,
    type: "script" as const,
  });
  const [showNewProject, setShowNewProject] = useState(false);
  const [consoleOutput, setConsoleOutput] = useState<string[]>([]);

  const [apiEndpoints, setApiEndpoints] = useState<APIEndpoint[]>([
    {
      id: "api_001",
      path: "/api/robot/move",
      method: "POST",
      description: "Move robot with velocity commands",
      parameters: [
        { name: "linear_x", type: "float", required: true },
        { name: "linear_y", type: "float", required: false },
        { name: "angular_z", type: "float", required: true },
      ],
      response: { status: "success", message: "Robot moving" },
      enabled: true,
    },
    {
      id: "api_002",
      path: "/api/robot/status",
      method: "GET",
      description: "Get current robot status and position",
      parameters: [],
      response: {
        position: { x: 0, y: 0, theta: 0 },
        velocity: { linear: 0, angular: 0 },
        battery: 85,
      },
      enabled: true,
    },
    {
      id: "api_003",
      path: "/api/navigation/goto",
      method: "POST",
      description: "Navigate to target position",
      parameters: [
        { name: "x", type: "float", required: true },
        { name: "y", type: "float", required: true },
        { name: "theta", type: "float", required: false },
      ],
      response: { status: "navigation_started", target: { x: 0, y: 0 } },
      enabled: true,
    },
  ]);

  const selectedProjectData = projects.find((p) => p.id === selectedProject);

  const handleRunProject = (projectId: string) => {
    setProjects((prev) =>
      prev.map((project) => {
        if (project.id === projectId) {
          const newOutput = [
            ...project.output,
            `[${new Date().toLocaleTimeString()}] Starting ${project.name}...`,
            `[${new Date().toLocaleTimeString()}] Checking dependencies...`,
            `[${new Date().toLocaleTimeString()}] ${project.language} interpreter ready`,
          ];

          // Simulate running
          setTimeout(() => {
            setProjects((prev2) =>
              prev2.map((p) =>
                p.id === projectId
                  ? {
                      ...p,
                      isRunning: false,
                      status: "success" as const,
                      lastRun: new Date(),
                      output: [
                        ...newOutput,
                        `[${new Date().toLocaleTimeString()}] Execution completed successfully`,
                      ],
                    }
                  : p,
              ),
            );
          }, 3000);

          return {
            ...project,
            isRunning: true,
            status: "running" as const,
            output: newOutput,
          };
        }
        return project;
      }),
    );
  };

  const handleStopProject = (projectId: string) => {
    setProjects((prev) =>
      prev.map((project) =>
        project.id === projectId
          ? {
              ...project,
              isRunning: false,
              status: "ready" as const,
              output: [
                ...project.output,
                `[${new Date().toLocaleTimeString()}] Process stopped by user`,
              ],
            }
          : project,
      ),
    );
  };

  const handleCreateProject = () => {
    const project: CodeProject = {
      id: `proj_${Date.now()}`,
      name: newProject.name,
      description: newProject.description,
      language: newProject.language,
      type: newProject.type,
      code: `# ${newProject.name}\n# ${newProject.description}\n\ndef main():\n    print("Hello from ${newProject.name}!")\n\nif __name__ == "__main__":\n    main()`,
      isRunning: false,
      lastRun: null,
      status: "ready",
      output: [],
      dependencies: [],
      autoStart: false,
    };

    setProjects((prev) => [...prev, project]);
    setNewProject({
      name: "",
      description: "",
      language: "python",
      type: "script",
    });
    setShowNewProject(false);
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "running":
        return "text-blue-500";
      case "success":
        return "text-green-500";
      case "error":
        return "text-red-500";
      default:
        return "text-muted-foreground";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "running":
        return Clock;
      case "success":
        return CheckCircle;
      case "error":
        return AlertTriangle;
      default:
        return Code;
    }
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Code className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            Code Development Environment
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Integrated development environment for robot control scripts and
            APIs
          </p>
        </div>

        <div className="flex gap-2">
          <Button
            onClick={() => setShowNewProject(!showNewProject)}
            className="gap-2"
          >
            <Code className="h-4 w-4" />
            New Project
          </Button>
          <Button variant="outline" className="gap-2">
            <FolderOpen className="h-4 w-4" />
            Import
          </Button>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export All
          </Button>
        </div>
      </div>

      {/* New Project Form */}
      {showNewProject && (
        <Card className="p-6 border-primary/30 fade-in-up">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Code className="h-5 w-5 text-primary" />
            Create New Project
          </h3>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div>
              <Label>Project Name</Label>
              <Input
                value={newProject.name}
                onChange={(e) =>
                  setNewProject((prev) => ({ ...prev, name: e.target.value }))
                }
                placeholder="e.g., Robot Control API"
              />
            </div>

            <div>
              <Label>Description</Label>
              <Input
                value={newProject.description}
                onChange={(e) =>
                  setNewProject((prev) => ({
                    ...prev,
                    description: e.target.value,
                  }))
                }
                placeholder="Brief description of the project"
              />
            </div>

            <div>
              <Label>Language</Label>
              <Select
                value={newProject.language}
                onValueChange={(value: any) =>
                  setNewProject((prev) => ({ ...prev, language: value }))
                }
              >
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="python">Python</SelectItem>
                  <SelectItem value="javascript">JavaScript</SelectItem>
                  <SelectItem value="cpp">C++</SelectItem>
                  <SelectItem value="bash">Bash</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div>
              <Label>Project Type</Label>
              <Select
                value={newProject.type}
                onValueChange={(value: any) =>
                  setNewProject((prev) => ({ ...prev, type: value }))
                }
              >
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="script">Script</SelectItem>
                  <SelectItem value="api">API Service</SelectItem>
                  <SelectItem value="service">Background Service</SelectItem>
                  <SelectItem value="automation">Automation</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </div>

          <div className="flex gap-2 mt-4">
            <Button onClick={handleCreateProject} className="gap-2">
              <Save className="h-4 w-4" />
              Create Project
            </Button>
            <Button variant="outline" onClick={() => setShowNewProject(false)}>
              Cancel
            </Button>
          </div>
        </Card>
      )}

      <Tabs defaultValue="projects" className="space-y-4">
        <TabsList className="grid w-full grid-cols-3">
          <TabsTrigger value="projects" className="gap-2">
            <FileText className="h-4 w-4" />
            Projects
          </TabsTrigger>
          <TabsTrigger value="api" className="gap-2">
            <Zap className="h-4 w-4" />
            API Endpoints
          </TabsTrigger>
          <TabsTrigger value="console" className="gap-2">
            <Terminal className="h-4 w-4" />
            Console
          </TabsTrigger>
        </TabsList>

        {/* Projects Tab */}
        <TabsContent value="projects" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            {/* Projects List */}
            <Card className="lg:col-span-2 p-4">
              <h3 className="text-lg font-light mb-4 flex items-center gap-2">
                <FileText className="h-5 w-5 text-primary" />
                Code Projects ({projects.length})
              </h3>

              <ScrollArea className="h-[600px]">
                <div className="space-y-3">
                  {projects.map((project, index) => {
                    const StatusIcon = getStatusIcon(project.status);
                    const isSelected = selectedProject === project.id;

                    return (
                      <div
                        key={project.id}
                        className={`p-4 rounded-lg border cursor-pointer transition-all hover-lift stagger-item ${
                          isSelected
                            ? "bg-primary/5 border-primary/30"
                            : "hover:bg-muted/50"
                        }`}
                        style={{ animationDelay: `${index * 0.1}s` }}
                        onClick={() =>
                          setSelectedProject(isSelected ? null : project.id)
                        }
                      >
                        <div className="flex items-center justify-between">
                          <div className="flex items-center gap-3 flex-1">
                            <StatusIcon
                              className={`h-5 w-5 ${getStatusColor(project.status)} ${project.isRunning ? "animate-spin" : ""}`}
                            />
                            <div className="flex-1 min-w-0">
                              <div className="font-medium text-sm">
                                {project.name}
                              </div>
                              <div className="text-xs text-muted-foreground">
                                {project.description}
                              </div>
                            </div>
                          </div>

                          <div className="flex items-center gap-2">
                            <Badge
                              variant="outline"
                              className="text-xs capitalize"
                            >
                              {project.language}
                            </Badge>
                            <Badge
                              variant={
                                project.type === "api"
                                  ? "default"
                                  : project.type === "service"
                                    ? "secondary"
                                    : "outline"
                              }
                              className="text-xs capitalize"
                            >
                              {project.type}
                            </Badge>
                          </div>
                        </div>

                        {isSelected && (
                          <div className="mt-4 pt-4 border-t fade-in-up">
                            <div className="space-y-3">
                              <div>
                                <Label className="text-xs font-medium">
                                  Code
                                </Label>
                                <Textarea
                                  value={project.code}
                                  onChange={(e) =>
                                    setProjects((prev) =>
                                      prev.map((p) =>
                                        p.id === project.id
                                          ? { ...p, code: e.target.value }
                                          : p,
                                      ),
                                    )
                                  }
                                  className="mt-1 font-mono text-xs"
                                  rows={10}
                                />
                              </div>

                              <div className="flex flex-wrap gap-2">
                                {!project.isRunning ? (
                                  <Button
                                    size="sm"
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      handleRunProject(project.id);
                                    }}
                                    className="gap-1"
                                  >
                                    <Play className="h-3 w-3" />
                                    Run
                                  </Button>
                                ) : (
                                  <Button
                                    size="sm"
                                    variant="destructive"
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      handleStopProject(project.id);
                                    }}
                                    className="gap-1"
                                  >
                                    <Square className="h-3 w-3" />
                                    Stop
                                  </Button>
                                )}
                                <Button
                                  size="sm"
                                  variant="outline"
                                  className="gap-1"
                                >
                                  <Save className="h-3 w-3" />
                                  Save
                                </Button>
                                <Button
                                  size="sm"
                                  variant="outline"
                                  className="gap-1"
                                >
                                  <Download className="h-3 w-3" />
                                  Export
                                </Button>
                                <Button
                                  size="sm"
                                  variant="outline"
                                  className="gap-1"
                                >
                                  <Settings className="h-3 w-3" />
                                  Config
                                </Button>
                              </div>
                            </div>
                          </div>
                        )}
                      </div>
                    );
                  })}
                </div>
              </ScrollArea>
            </Card>

            {/* Project Details */}
            <div className="space-y-4">
              <Card className="p-4">
                <h3 className="text-lg font-light mb-4 flex items-center gap-2">
                  <Eye className="h-5 w-5 text-primary" />
                  Project Details
                </h3>

                {selectedProjectData ? (
                  <div className="space-y-4">
                    <div>
                      <Label className="text-sm font-medium">
                        Project Name
                      </Label>
                      <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                        {selectedProjectData.name}
                      </div>
                    </div>

                    <div>
                      <Label className="text-sm font-medium">
                        Language & Type
                      </Label>
                      <div className="flex gap-2 mt-1">
                        <Badge className="capitalize">
                          {selectedProjectData.language}
                        </Badge>
                        <Badge variant="outline" className="capitalize">
                          {selectedProjectData.type}
                        </Badge>
                      </div>
                    </div>

                    <div>
                      <Label className="text-sm font-medium">Status</Label>
                      <div className="flex items-center gap-2 mt-1">
                        <Badge
                          variant={
                            selectedProjectData.status === "success"
                              ? "default"
                              : selectedProjectData.status === "error"
                                ? "destructive"
                                : "secondary"
                          }
                        >
                          {selectedProjectData.status.toUpperCase()}
                        </Badge>
                        {selectedProjectData.isRunning && (
                          <Clock className="h-4 w-4 animate-spin text-primary" />
                        )}
                      </div>
                    </div>

                    {selectedProjectData.apiEndpoint && (
                      <div>
                        <Label className="text-sm font-medium">
                          API Endpoint
                        </Label>
                        <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                          {selectedProjectData.apiEndpoint}
                        </div>
                      </div>
                    )}

                    <div>
                      <Label className="text-sm font-medium">
                        Dependencies
                      </Label>
                      <div className="flex flex-wrap gap-1 mt-1">
                        {selectedProjectData.dependencies.map((dep) => (
                          <Badge
                            key={dep}
                            variant="outline"
                            className="text-xs"
                          >
                            {dep}
                          </Badge>
                        ))}
                      </div>
                    </div>

                    <div>
                      <Label className="text-sm font-medium">Output Log</Label>
                      <ScrollArea className="h-32 mt-1">
                        <div className="bg-black text-green-400 p-2 rounded font-mono text-xs">
                          {selectedProjectData.output.map((line, i) => (
                            <div key={i}>{line}</div>
                          ))}
                        </div>
                      </ScrollArea>
                    </div>
                  </div>
                ) : (
                  <div className="text-center text-muted-foreground py-8">
                    Select a project to view details
                  </div>
                )}
              </Card>
            </div>
          </div>
        </TabsContent>

        {/* API Endpoints Tab */}
        <TabsContent value="api" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Zap className="h-5 w-5 text-primary" />
              Available API Endpoints
            </h3>

            <div className="space-y-4">
              {apiEndpoints.map((endpoint, index) => (
                <div
                  key={endpoint.id}
                  className={`p-4 border rounded-lg stagger-item`}
                  style={{ animationDelay: `${index * 0.1}s` }}
                >
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center gap-3">
                      <Badge
                        variant={
                          endpoint.method === "GET"
                            ? "secondary"
                            : endpoint.method === "POST"
                              ? "default"
                              : "outline"
                        }
                        className="font-mono"
                      >
                        {endpoint.method}
                      </Badge>
                      <code className="font-mono text-sm">{endpoint.path}</code>
                    </div>
                    <Badge variant={endpoint.enabled ? "default" : "secondary"}>
                      {endpoint.enabled ? "Active" : "Disabled"}
                    </Badge>
                  </div>

                  <p className="text-sm text-muted-foreground mb-3">
                    {endpoint.description}
                  </p>

                  {endpoint.parameters.length > 0 && (
                    <div className="mb-3">
                      <Label className="text-xs font-medium">Parameters:</Label>
                      <div className="flex flex-wrap gap-1 mt-1">
                        {endpoint.parameters.map((param) => (
                          <Badge
                            key={param.name}
                            variant={param.required ? "default" : "outline"}
                            className="text-xs"
                          >
                            {param.name}: {param.type}
                            {param.required && " *"}
                          </Badge>
                        ))}
                      </div>
                    </div>
                  )}

                  <div>
                    <Label className="text-xs font-medium">Response:</Label>
                    <pre className="bg-muted p-2 rounded mt-1 text-xs overflow-x-auto">
                      {JSON.stringify(endpoint.response, null, 2)}
                    </pre>
                  </div>
                </div>
              ))}
            </div>
          </Card>
        </TabsContent>

        {/* Console Tab */}
        <TabsContent value="console" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Terminal className="h-5 w-5 text-primary" />
              System Console
            </h3>

            <div className="bg-black text-green-400 p-4 rounded font-mono text-sm h-96 overflow-y-auto">
              <div>$ Robot Development Environment Console</div>
              <div>$ Python 3.9.7 | Flask 2.0.1 | Serial 3.5</div>
              <div>$ All systems ready for development</div>
              <div className="mt-2">
                {projects
                  .filter((p) => p.output.length > 0)
                  .map((project) =>
                    project.output.map((line, i) => (
                      <div key={`${project.id}-${i}`}>
                        [{project.name}] {line}
                      </div>
                    )),
                  )}
              </div>
              <div className="mt-2">$ _</div>
            </div>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
};

export default CodeDevelopment;
