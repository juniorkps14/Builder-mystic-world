import React, { useState } from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Card } from "@/components/ui/card";
import {
  Activity,
  Bot,
  Wifi,
  WifiOff,
  Battery,
  Thermometer,
  Cpu,
  HardDrive,
  Play,
  Pause,
  Square,
  AlertTriangle,
  CheckCircle,
  Clock,
  MapPin,
  Eye,
  Settings,
  Zap,
  Shield,
  TrendingUp,
  Calendar,
  Users,
} from "lucide-react";
import "../styles/flat-vector-theme.css";

export default function FlatDashboard() {
  const [robotStatus, setRobotStatus] = useState<"idle" | "running" | "error">(
    "idle",
  );

  // Mock data
  const systemStats = {
    cpu: 42,
    memory: 68,
    disk: 73,
    temperature: 45,
    battery: 87,
    uptime: "2d 14h 32m",
  };

  const robotData = {
    position: { x: 2.45, y: 1.23, rotation: 45 },
    speed: 0.5,
    lastCommand: "Navigate to Point A",
    activeTasks: 3,
    completedTasks: 28,
  };

  const quickActions = [
    {
      title: "Start Robot",
      icon: Play,
      color: "bg-green-500",
      action: () => setRobotStatus("running"),
    },
    {
      title: "Stop Robot",
      icon: Square,
      color: "bg-red-500",
      action: () => setRobotStatus("idle"),
    },
    {
      title: "Emergency Stop",
      icon: AlertTriangle,
      color: "bg-orange-500",
      action: () => setRobotStatus("error"),
    },
    {
      title: "Go Home",
      icon: MapPin,
      color: "bg-blue-500",
      action: () => console.log("Going home"),
    },
  ];

  const recentActivities = [
    {
      id: 1,
      title: "Navigation completed",
      time: "2 minutes ago",
      icon: CheckCircle,
      color: "text-green-500",
    },
    {
      id: 2,
      title: "Sensor data updated",
      time: "5 minutes ago",
      icon: Activity,
      color: "text-blue-500",
    },
    {
      id: 3,
      title: "Task sequence started",
      time: "10 minutes ago",
      icon: Play,
      color: "text-purple-500",
    },
    {
      id: 4,
      title: "System health check",
      time: "15 minutes ago",
      icon: Shield,
      color: "text-orange-500",
    },
  ];

  const getStatusColor = (status: string) => {
    switch (status) {
      case "running":
        return "bg-green-100 text-green-700 border-green-200";
      case "error":
        return "bg-red-100 text-red-700 border-red-200";
      default:
        return "bg-gray-100 text-gray-700 border-gray-200";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "running":
        return <Activity className="w-5 h-5 animate-pulse" />;
      case "error":
        return <AlertTriangle className="w-5 h-5" />;
      default:
        return <Pause className="w-5 h-5" />;
    }
  };

  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Welcome Header */}
      <div className="mb-8">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="flat-title">Welcome back! 👋</h1>
            <p className="flat-subtitle">
              Your robot is ready and systems are running smoothly
            </p>
          </div>
          <div className="flex items-center gap-3">
            <Badge className="flat-badge-success">All Systems OK</Badge>
            <Button className="flat-button-primary">
              <Settings className="flat-icon" />
              Settings
            </Button>
          </div>
        </div>
      </div>

      {/* Status Overview */}
      <div className="flat-grid flat-grid-4 mb-8">
        {/* Robot Status */}
        <Card className="flat-card">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-3">
              <div className="w-12 h-12 bg-gradient-to-br from-blue-500 to-purple-600 rounded-xl flex items-center justify-center">
                <Bot className="w-6 h-6 text-white" />
              </div>
              <div>
                <h3 className="font-semibold text-gray-900">Robot Status</h3>
                <p className="text-sm text-gray-500">Current state</p>
              </div>
            </div>
          </div>
          <div
            className={`flex items-center gap-2 px-4 py-3 rounded-lg border-2 ${getStatusColor(
              robotStatus,
            )}`}
          >
            {getStatusIcon(robotStatus)}
            <span className="font-medium capitalize">{robotStatus}</span>
          </div>
        </Card>

        {/* Connection Status */}
        <Card className="flat-card">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-3">
              <div className="w-12 h-12 bg-gradient-to-br from-green-500 to-teal-600 rounded-xl flex items-center justify-center">
                <Wifi className="w-6 h-6 text-white" />
              </div>
              <div>
                <h3 className="font-semibold text-gray-900">Connection</h3>
                <p className="text-sm text-gray-500">ROS Bridge</p>
              </div>
            </div>
          </div>
          <div className="flex items-center gap-2 px-4 py-3 rounded-lg bg-green-100 text-green-700 border-2 border-green-200">
            <Wifi className="w-5 h-5" />
            <span className="font-medium">Connected</span>
          </div>
        </Card>

        {/* Battery Level */}
        <Card className="flat-card">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-3">
              <div className="w-12 h-12 bg-gradient-to-br from-orange-500 to-red-600 rounded-xl flex items-center justify-center">
                <Battery className="w-6 h-6 text-white" />
              </div>
              <div>
                <h3 className="font-semibold text-gray-900">Battery</h3>
                <p className="text-sm text-gray-500">Power level</p>
              </div>
            </div>
          </div>
          <div className="space-y-2">
            <div className="flex justify-between items-center">
              <span className="text-2xl font-bold text-gray-900">
                {systemStats.battery}%
              </span>
              <Badge className="flat-badge-success">Good</Badge>
            </div>
            <Progress value={systemStats.battery} className="flat-progress" />
          </div>
        </Card>

        {/* Temperature */}
        <Card className="flat-card">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-3">
              <div className="w-12 h-12 bg-gradient-to-br from-purple-500 to-pink-600 rounded-xl flex items-center justify-center">
                <Thermometer className="w-6 h-6 text-white" />
              </div>
              <div>
                <h3 className="font-semibold text-gray-900">Temperature</h3>
                <p className="text-sm text-gray-500">System temp</p>
              </div>
            </div>
          </div>
          <div className="text-2xl font-bold text-gray-900">
            {systemStats.temperature}°C
          </div>
          <p className="text-sm text-gray-500 mt-1">Normal range</p>
        </Card>
      </div>

      {/* Main Content Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
        {/* Quick Actions */}
        <Card className="flat-card lg:col-span-1">
          <h3 className="text-lg font-semibold text-gray-900 mb-4">
            Quick Actions
          </h3>
          <div className="space-y-3">
            {quickActions.map((action, index) => {
              const IconComponent = action.icon;
              return (
                <Button
                  key={index}
                  onClick={action.action}
                  className={`flat-button w-full justify-start text-white ${action.color} hover:opacity-90`}
                >
                  <IconComponent className="flat-icon" />
                  {action.title}
                </Button>
              );
            })}
          </div>
        </Card>

        {/* System Performance */}
        <Card className="flat-card lg:col-span-1">
          <h3 className="text-lg font-semibold text-gray-900 mb-4">
            System Performance
          </h3>
          <div className="space-y-4">
            {/* CPU */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center gap-2">
                  <Cpu className="w-4 h-4 text-blue-500" />
                  <span className="text-sm font-medium">CPU Usage</span>
                </div>
                <span className="text-sm text-gray-600">
                  {systemStats.cpu}%
                </span>
              </div>
              <Progress value={systemStats.cpu} className="flat-progress" />
            </div>

            {/* Memory */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center gap-2">
                  <HardDrive className="w-4 h-4 text-green-500" />
                  <span className="text-sm font-medium">Memory</span>
                </div>
                <span className="text-sm text-gray-600">
                  {systemStats.memory}%
                </span>
              </div>
              <Progress value={systemStats.memory} className="flat-progress" />
            </div>

            {/* Disk */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center gap-2">
                  <HardDrive className="w-4 h-4 text-purple-500" />
                  <span className="text-sm font-medium">Disk Space</span>
                </div>
                <span className="text-sm text-gray-600">
                  {systemStats.disk}%
                </span>
              </div>
              <Progress value={systemStats.disk} className="flat-progress" />
            </div>
          </div>
        </Card>

        {/* Recent Activities */}
        <Card className="flat-card lg:col-span-1">
          <h3 className="text-lg font-semibold text-gray-900 mb-4">
            Recent Activities
          </h3>
          <div className="space-y-3">
            {recentActivities.map((activity) => {
              const IconComponent = activity.icon;
              return (
                <div
                  key={activity.id}
                  className="flex items-center gap-3 p-3 rounded-lg bg-gray-50 hover:bg-gray-100 transition-colors"
                >
                  <div className="w-8 h-8 bg-white rounded-lg flex items-center justify-center">
                    <IconComponent className={`w-4 h-4 ${activity.color}`} />
                  </div>
                  <div className="flex-1">
                    <p className="text-sm font-medium text-gray-900">
                      {activity.title}
                    </p>
                    <p className="text-xs text-gray-500">{activity.time}</p>
                  </div>
                </div>
              );
            })}
          </div>
        </Card>
      </div>

      {/* Bottom Statistics */}
      <div className="flat-grid flat-grid-4">
        <Card className="flat-card">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 bg-blue-100 rounded-lg flex items-center justify-center">
              <Clock className="w-5 h-5 text-blue-600" />
            </div>
            <div>
              <p className="text-sm text-gray-500">Uptime</p>
              <p className="text-lg font-semibold text-gray-900">
                {systemStats.uptime}
              </p>
            </div>
          </div>
        </Card>

        <Card className="flat-card">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 bg-green-100 rounded-lg flex items-center justify-center">
              <TrendingUp className="w-5 h-5 text-green-600" />
            </div>
            <div>
              <p className="text-sm text-gray-500">Tasks Completed</p>
              <p className="text-lg font-semibold text-gray-900">
                {robotData.completedTasks}
              </p>
            </div>
          </div>
        </Card>

        <Card className="flat-card">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 bg-purple-100 rounded-lg flex items-center justify-center">
              <Activity className="w-5 h-5 text-purple-600" />
            </div>
            <div>
              <p className="text-sm text-gray-500">Active Tasks</p>
              <p className="text-lg font-semibold text-gray-900">
                {robotData.activeTasks}
              </p>
            </div>
          </div>
        </Card>

        <Card className="flat-card">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 bg-orange-100 rounded-lg flex items-center justify-center">
              <Zap className="w-5 h-5 text-orange-600" />
            </div>
            <div>
              <p className="text-sm text-gray-500">Speed</p>
              <p className="text-lg font-semibold text-gray-900">
                {robotData.speed} m/s
              </p>
            </div>
          </div>
        </Card>
      </div>
    </div>
  );
}
