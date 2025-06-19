import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import {
  Activity,
  AlertTriangle,
  Cpu,
  HardDrive,
  MemoryStick,
  Network,
  Play,
  Pause,
  RotateCcw,
  Thermometer,
  Zap,
  Camera,
  Radio,
  MapPin,
  Battery,
  Wifi,
  Signal,
} from "lucide-react";

export function Dashboard() {
  // Mock real-time data
  const systemMetrics = {
    cpu: 34,
    memory: 67,
    disk: 45,
    temperature: 42,
    battery: 87,
    network: {
      strength: 85,
      latency: "12ms",
    },
  };

  const robotStatus = {
    position: { x: 2.34, y: 1.67, theta: 0.45 },
    velocity: { linear: 0.25, angular: 0.1 },
    battery: 87,
    mode: "autonomous",
    state: "navigating",
  };

  const activeNodes = [
    {
      name: "/robot_state_publisher",
      status: "active",
      cpu: 2.1,
      memory: 15.4,
    },
    { name: "/move_base", status: "active", cpu: 8.3, memory: 45.2 },
    { name: "/amcl", status: "active", cpu: 5.7, memory: 32.1 },
    { name: "/camera_node", status: "active", cpu: 12.4, memory: 67.8 },
    { name: "/laser_scan", status: "active", cpu: 3.2, memory: 12.6 },
    { name: "/tf2_ros", status: "warning", cpu: 1.8, memory: 8.9 },
  ];

  const activeTopics = [
    { name: "/cmd_vel", rate: "50 Hz", type: "geometry_msgs/Twist" },
    { name: "/scan", rate: "10 Hz", type: "sensor_msgs/LaserScan" },
    { name: "/odom", rate: "50 Hz", type: "nav_msgs/Odometry" },
    { name: "/camera/image_raw", rate: "30 Hz", type: "sensor_msgs/Image" },
    { name: "/map", rate: "1 Hz", type: "nav_msgs/OccupancyGrid" },
  ];

  return (
    <div className="space-y-6">
      {/* System Overview */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
        <Card className="p-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <div className="p-2 rounded-lg bg-primary/10">
                <Cpu className="h-5 w-5 text-primary" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">CPU Usage</p>
                <p className="text-2xl font-bold">{systemMetrics.cpu}%</p>
              </div>
            </div>
            <Progress value={systemMetrics.cpu} className="w-16" />
          </div>
        </Card>

        <Card className="p-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <div className="p-2 rounded-lg bg-accent/10">
                <MemoryStick className="h-5 w-5 text-accent" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">Memory</p>
                <p className="text-2xl font-bold">{systemMetrics.memory}%</p>
              </div>
            </div>
            <Progress value={systemMetrics.memory} className="w-16" />
          </div>
        </Card>

        <Card className="p-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <div className="p-2 rounded-lg bg-ros-success/10">
                <Battery className="h-5 w-5 text-ros-success" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">Battery</p>
                <p className="text-2xl font-bold">{robotStatus.battery}%</p>
              </div>
            </div>
            <Progress value={robotStatus.battery} className="w-16" />
          </div>
        </Card>

        <Card className="p-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <div className="p-2 rounded-lg bg-blue-500/10">
                <Thermometer className="h-5 w-5 text-blue-500" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">Temperature</p>
                <p className="text-2xl font-bold">
                  {systemMetrics.temperature}°C
                </p>
              </div>
            </div>
          </div>
        </Card>
      </div>

      {/* Robot Status and Quick Controls */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Robot Status */}
        <Card className="lg:col-span-2 p-6">
          <div className="flex items-center justify-between mb-6">
            <h3 className="text-lg font-semibold">Robot Status</h3>
            <Badge variant="outline" className="gap-2">
              <div className="ros-status-indicator ros-status-active" />
              {robotStatus.state}
            </Badge>
          </div>

          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div className="text-center">
              <p className="text-sm text-muted-foreground mb-1">Position X</p>
              <p className="text-xl font-mono">
                {robotStatus.position.x.toFixed(2)}
              </p>
            </div>
            <div className="text-center">
              <p className="text-sm text-muted-foreground mb-1">Position Y</p>
              <p className="text-xl font-mono">
                {robotStatus.position.y.toFixed(2)}
              </p>
            </div>
            <div className="text-center">
              <p className="text-sm text-muted-foreground mb-1">Linear Vel</p>
              <p className="text-xl font-mono">
                {robotStatus.velocity.linear.toFixed(2)}
              </p>
            </div>
            <div className="text-center">
              <p className="text-sm text-muted-foreground mb-1">Angular Vel</p>
              <p className="text-xl font-mono">
                {robotStatus.velocity.angular.toFixed(2)}
              </p>
            </div>
          </div>

          <Separator className="my-6" />

          <div className="flex items-center gap-4">
            <Button size="sm" className="gap-2">
              <Play className="h-4 w-4" />
              Start Mission
            </Button>
            <Button size="sm" variant="secondary" className="gap-2">
              <Pause className="h-4 w-4" />
              Pause
            </Button>
            <Button size="sm" variant="outline" className="gap-2">
              <RotateCcw className="h-4 w-4" />
              Reset
            </Button>
            <Button size="sm" variant="outline" className="gap-2">
              <MapPin className="h-4 w-4" />
              Home
            </Button>
          </div>
        </Card>

        {/* Quick Actions */}
        <Card className="p-6">
          <h3 className="text-lg font-semibold mb-6">Quick Actions</h3>
          <div className="space-y-3">
            <Button variant="outline" className="w-full justify-start gap-3">
              <Camera className="h-4 w-4" />
              View Cameras
            </Button>
            <Button variant="outline" className="w-full justify-start gap-3">
              <Activity className="h-4 w-4" />
              Sensor Data
            </Button>
            <Button variant="outline" className="w-full justify-start gap-3">
              <Network className="h-4 w-4" />
              Node Manager
            </Button>
            <Button variant="outline" className="w-full justify-start gap-3">
              <Radio className="h-4 w-4" />
              Topic Monitor
            </Button>
          </div>
        </Card>
      </div>

      {/* Active Nodes and Topics */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Active Nodes */}
        <Card className="p-6">
          <div className="flex items-center justify-between mb-6">
            <h3 className="text-lg font-semibold">Active Nodes</h3>
            <Badge variant="secondary">{activeNodes.length} running</Badge>
          </div>

          <div className="space-y-3">
            {activeNodes.map((node, index) => (
              <div
                key={index}
                className="flex items-center justify-between p-3 rounded-lg border border-border"
              >
                <div className="flex items-center gap-3">
                  <div
                    className={`ros-status-indicator ${node.status === "active" ? "ros-status-active" : "ros-status-error"}`}
                  />
                  <div>
                    <p className="font-mono text-sm">{node.name}</p>
                    <p className="text-xs text-muted-foreground">
                      CPU: {node.cpu}% | MEM: {node.memory} MB
                    </p>
                  </div>
                </div>
                {node.status === "warning" && (
                  <AlertTriangle className="h-4 w-4 text-ros-warning" />
                )}
              </div>
            ))}
          </div>
        </Card>

        {/* Active Topics */}
        <Card className="p-6">
          <div className="flex items-center justify-between mb-6">
            <h3 className="text-lg font-semibold">Active Topics</h3>
            <Badge variant="secondary">{activeTopics.length} topics</Badge>
          </div>

          <div className="space-y-3">
            {activeTopics.map((topic, index) => (
              <div
                key={index}
                className="flex items-center justify-between p-3 rounded-lg border border-border"
              >
                <div className="flex items-center gap-3">
                  <Radio className="h-4 w-4 text-ros-topic" />
                  <div>
                    <p className="font-mono text-sm">{topic.name}</p>
                    <p className="text-xs text-muted-foreground">
                      {topic.type}
                    </p>
                  </div>
                </div>
                <Badge variant="outline" className="text-xs">
                  {topic.rate}
                </Badge>
              </div>
            ))}
          </div>
        </Card>
      </div>
    </div>
  );
}
