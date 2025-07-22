import React, { useState, useEffect } from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Card } from "@/components/ui/card";
import { usePersistentStore } from "@/hooks/use-persistence";
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
  Navigation,
  Camera,
  Layers,
  Globe,
  Radio,
  Target,
  Move,
  RotateCcw,
  Home,
  Power,
  ChevronRight,
  ArrowUp,
  ArrowDown,
  Minus,
} from "lucide-react";

export default function FlatDashboard() {
  const { store: dashboardPrefs, updateField } = usePersistentStore(
    "dashboard-preferences",
    {
      robotStatus: "idle" as "idle" | "running" | "paused" | "error",
      selectedView: "overview" as "overview" | "detailed" | "monitoring",
      autoRefresh: true,
    }
  );

  const [currentTime, setCurrentTime] = useState(new Date());
  const [isOnline, setIsOnline] = useState(true);

  useEffect(() => {
    const timer = setInterval(() => setCurrentTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  // Mock real-time data
  const systemStats = {
    cpu: 42 + Math.sin(Date.now() / 10000) * 10,
    memory: 68 + Math.cos(Date.now() / 8000) * 5,
    disk: 73,
    temperature: 45 + Math.sin(Date.now() / 15000) * 3,
    battery: 87 - Math.floor(Date.now() / 100000) % 20,
    uptime: "2d 14h 32m",
    networkLatency: 12 + Math.random() * 8,
  };

  const robotData = {
    position: { 
      x: 2.45 + Math.sin(Date.now() / 5000) * 0.1, 
      y: 1.23 + Math.cos(Date.now() / 6000) * 0.1, 
      rotation: 45 + Math.sin(Date.now() / 4000) * 10 
    },
    speed: 0.5 + Math.sin(Date.now() / 3000) * 0.3,
    lastCommand: "Navigate to Point A",
    activeTasks: 3,
    completedTasks: 28,
    currentTask: "Environmental Scanning",
    taskProgress: 65 + Math.sin(Date.now() / 2000) * 10,
  };

  const sensorData = [
    { name: "LIDAR", status: "active", value: "360°", quality: 98 },
    { name: "Camera", status: "active", value: "1080p", quality: 94 },
    { name: "IMU", status: "active", value: "9-axis", quality: 99 },
    { name: "GPS", status: "active", value: "RTK", quality: 89 },
    { name: "Ultrasonic", status: "active", value: "8 sensors", quality: 92 },
  ];

  const getStatusColor = (status: string) => {
    switch (status) {
      case "running":
        return "from-emerald-500 to-teal-500";
      case "error":
        return "from-red-500 to-pink-500";
      case "paused":
        return "from-yellow-500 to-orange-500";
      default:
        return "from-blue-500 to-cyan-500";
    }
  };

  const getStatusIcon = () => {
    switch (dashboardPrefs.robotStatus) {
      case "running":
        return Play;
      case "error":
        return AlertTriangle;
      case "paused":
        return Pause;
      default:
        return Square;
    }
  };

  const StatusIcon = getStatusIcon();

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Header with Time and Status */}
      <div className="mb-8">
        <div className="bg-white/5 backdrop-blur-xl border border-white/10 rounded-3xl p-6 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="space-y-2">
              <h1 className="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
                Robot Command Center
              </h1>
              <p className="text-slate-400 font-light">
                Autonomous system control and monitoring dashboard
              </p>
            </div>
            
            <div className="flex items-center gap-6">
              {/* Time Display */}
              <div className="text-right">
                <p className="text-2xl font-light text-white">
                  {currentTime.toLocaleTimeString('en-US', { 
                    hour12: false,
                    hour: '2-digit',
                    minute: '2-digit',
                    second: '2-digit'
                  })}
                </p>
                <p className="text-sm text-slate-400">
                  {currentTime.toLocaleDateString('en-US', {
                    weekday: 'long',
                    month: 'short',
                    day: 'numeric'
                  })}
                </p>
              </div>

              {/* Online Status */}
              <div className={`flex items-center gap-2 px-4 py-2 rounded-full ${
                isOnline 
                  ? "bg-emerald-500/10 text-emerald-400 border border-emerald-500/20" 
                  : "bg-red-500/10 text-red-400 border border-red-500/20"
              }`}>
                {isOnline ? <Wifi className="w-4 h-4" /> : <WifiOff className="w-4 h-4" />}
                <span className="text-sm font-medium">
                  {isOnline ? "Connected" : "Offline"}
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Main Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 mb-8">
        {/* Robot Status - Large Card */}
        <div className="lg:col-span-8">
          <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-8 shadow-2xl hover:bg-white/10 transition-all duration-300">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
              {/* Robot Visualization */}
              <div className="space-y-6">
                <div className="flex items-center gap-4">
                  <div className={`w-16 h-16 rounded-full bg-gradient-to-br ${getStatusColor(dashboardPrefs.robotStatus)} flex items-center justify-center shadow-lg`}>
                    <StatusIcon className="h-8 w-8 text-white" />
                  </div>
                  <div>
                    <h2 className="text-2xl font-light text-white">DINO-01</h2>
                    <Badge className={`bg-gradient-to-r ${getStatusColor(dashboardPrefs.robotStatus)} text-white border-0`}>
                      {dashboardPrefs.robotStatus.toUpperCase()}
                    </Badge>
                  </div>
                </div>

                {/* Current Task */}
                <div className="bg-white/5 rounded-2xl p-4 border border-white/10">
                  <div className="flex items-center justify-between mb-3">
                    <span className="text-slate-400 text-sm">Current Task</span>
                    <span className="text-blue-400 text-sm font-mono">{robotData.taskProgress.toFixed(1)}%</span>
                  </div>
                  <p className="text-white font-medium mb-3">{robotData.currentTask}</p>
                  <Progress 
                    value={robotData.taskProgress} 
                    className="h-2 bg-slate-700"
                  />
                </div>

                {/* Quick Actions */}
                <div className="grid grid-cols-2 gap-3">
                  <Button 
                    onClick={() => updateField("robotStatus", "running")}
                    className="bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600 text-white"
                  >
                    <Play className="h-4 w-4 mr-2" />
                    Start
                  </Button>
                  <Button 
                    onClick={() => updateField("robotStatus", "paused")}
                    className="bg-white/10 hover:bg-white/20 border border-white/20 text-white"
                  >
                    <Pause className="h-4 w-4 mr-2" />
                    Pause
                  </Button>
                  <Button 
                    onClick={() => updateField("robotStatus", "idle")}
                    className="bg-white/10 hover:bg-white/20 border border-white/20 text-white"
                  >
                    <Square className="h-4 w-4 mr-2" />
                    Stop
                  </Button>
                  <Button 
                    onClick={() => updateField("robotStatus", "error")}
                    className="bg-gradient-to-r from-red-500 to-pink-500 hover:from-red-600 hover:to-pink-600 text-white"
                  >
                    <AlertTriangle className="h-4 w-4 mr-2" />
                    E-Stop
                  </Button>
                </div>
              </div>

              {/* Robot Data */}
              <div className="space-y-4">
                <h3 className="text-lg font-light text-white mb-4">System Metrics</h3>
                
                <div className="grid grid-cols-2 gap-4">
                  <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                    <div className="flex items-center gap-2 mb-2">
                      <MapPin className="h-4 w-4 text-blue-400" />
                      <span className="text-slate-400 text-sm">Position</span>
                    </div>
                    <p className="text-white font-mono text-sm">
                      X: {robotData.position.x.toFixed(2)}m<br/>
                      Y: {robotData.position.y.toFixed(2)}m<br/>
                      θ: {robotData.position.rotation.toFixed(1)}°
                    </p>
                  </div>

                  <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                    <div className="flex items-center gap-2 mb-2">
                      <Zap className="h-4 w-4 text-green-400" />
                      <span className="text-slate-400 text-sm">Speed</span>
                    </div>
                    <p className="text-white font-mono text-lg">
                      {robotData.speed.toFixed(2)} m/s
                    </p>
                  </div>

                  <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                    <div className="flex items-center gap-2 mb-2">
                      <Target className="h-4 w-4 text-purple-400" />
                      <span className="text-slate-400 text-sm">Tasks</span>
                    </div>
                    <p className="text-white font-mono">
                      {robotData.activeTasks} active<br/>
                      {robotData.completedTasks} completed
                    </p>
                  </div>

                  <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                    <div className="flex items-center gap-2 mb-2">
                      <Radio className="h-4 w-4 text-cyan-400" />
                      <span className="text-slate-400 text-sm">Network</span>
                    </div>
                    <p className="text-white font-mono text-lg">
                      {systemStats.networkLatency.toFixed(0)}ms
                    </p>
                  </div>
                </div>
              </div>
            </div>
          </Card>
        </div>

        {/* System Health */}
        <div className="lg:col-span-4 space-y-6">
          {/* Battery */}
          <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-6 shadow-xl hover:bg-white/10 transition-all duration-300">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-2">
                <Battery className="h-5 w-5 text-green-400" />
                <span className="text-white font-medium">Battery</span>
              </div>
              <span className="text-2xl font-light text-white">{systemStats.battery}%</span>
            </div>
            <Progress value={systemStats.battery} className="h-3 bg-slate-700" />
            <p className="text-slate-400 text-sm mt-2">Charging not required</p>
          </Card>

          {/* Temperature */}
          <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-6 shadow-xl hover:bg-white/10 transition-all duration-300">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-2">
                <Thermometer className="h-5 w-5 text-orange-400" />
                <span className="text-white font-medium">Temperature</span>
              </div>
              <span className="text-2xl font-light text-white">{systemStats.temperature.toFixed(1)}°C</span>
            </div>
            <div className="flex justify-between text-sm text-slate-400">
              <span>Optimal</span>
              <span>Max: 85°C</span>
            </div>
          </Card>

          {/* CPU & Memory */}
          <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-6 shadow-xl hover:bg-white/10 transition-all duration-300">
            <div className="space-y-4">
              <div>
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center gap-2">
                    <Cpu className="h-4 w-4 text-blue-400" />
                    <span className="text-white text-sm">CPU</span>
                  </div>
                  <span className="text-white text-sm">{systemStats.cpu.toFixed(1)}%</span>
                </div>
                <Progress value={systemStats.cpu} className="h-2 bg-slate-700" />
              </div>

              <div>
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center gap-2">
                    <HardDrive className="h-4 w-4 text-purple-400" />
                    <span className="text-white text-sm">Memory</span>
                  </div>
                  <span className="text-white text-sm">{systemStats.memory.toFixed(1)}%</span>
                </div>
                <Progress value={systemStats.memory} className="h-2 bg-slate-700" />
              </div>
            </div>
          </Card>
        </div>
      </div>

      {/* Sensors Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-5 gap-4 mb-8">
        {sensorData.map((sensor, index) => (
          <Card key={sensor.name} className="bg-white/5 backdrop-blur-xl border border-white/10 p-4 shadow-xl hover:bg-white/10 transition-all duration-300">
            <div className="flex items-center justify-between mb-3">
              <span className="text-white font-medium text-sm">{sensor.name}</span>
              <div className="w-3 h-3 rounded-full bg-emerald-500 shadow-lg"></div>
            </div>
            <p className="text-slate-400 text-xs mb-2">{sensor.value}</p>
            <div className="flex items-center justify-between">
              <span className="text-xs text-slate-400">Quality</span>
              <span className="text-xs font-mono text-emerald-400">{sensor.quality}%</span>
            </div>
          </Card>
        ))}
      </div>

      {/* Quick Navigation */}
      <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-6 shadow-2xl">
        <h3 className="text-lg font-light text-white mb-4">Quick Navigation</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-6 gap-4">
          {[
            { name: "Sequences", icon: Layers, path: "/sequences" },
            { name: "Terminal", icon: Activity, path: "/terminal" },
            { name: "Cameras", icon: Camera, path: "/cameras" },
            { name: "Navigation", icon: Navigation, path: "/navigation" },
            { name: "Settings", icon: Settings, path: "/settings" },
            { name: "Monitoring", icon: TrendingUp, path: "/system-monitoring" },
          ].map((item) => {
            const IconComponent = item.icon;
            return (
              <Button
                key={item.name}
                variant="ghost"
                className="h-20 bg-white/5 hover:bg-white/10 border border-white/10 flex-col gap-2 text-white"
                onClick={() => window.location.href = item.path}
              >
                <IconComponent className="h-6 w-6" />
                <span className="text-xs">{item.name}</span>
              </Button>
            );
          })}
        </div>
      </Card>
    </div>
  );
}
