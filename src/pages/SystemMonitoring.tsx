import React, { useState, useEffect } from "react";
import { usePersistentStore } from "@/hooks/use-persistence";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Button } from "@/components/ui/button";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { ScrollArea } from "@/components/ui/scroll-area";
import {
  LineChart,
  Line,
  AreaChart,
  Area,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  PieChart,
  Pie,
  Cell,
  BarChart,
  Bar,
} from "recharts";
import {
  Cpu,
  HardDrive,
  Network,
  Activity,
  Thermometer,
  Zap,
  Wifi,
  Database,
  Server,
  AlertTriangle,
  CheckCircle,
  RefreshCw,
  Settings,
  Download,
  Eye,
  Clock,
  TrendingUp,
  TrendingDown,
  Minus,
  BarChart3,
  Monitor,
} from "lucide-react";

interface SystemMetrics {
  cpu: {
    usage: number;
    cores: number;
    frequency: number;
    temperature: number;
    processes: number;
  };
  memory: {
    total: number;
    used: number;
    free: number;
    cached: number;
    swap: { total: number; used: number };
  };
  disk: {
    total: number;
    used: number;
    free: number;
    io: { read: number; write: number };
  };
  network: {
    interfaces: Array<{
      name: string;
      status: "up" | "down";
      rx: number;
      tx: number;
      speed: number;
    }>;
  };
  system: {
    uptime: number;
    loadAverage: [number, number, number];
    bootTime: Date;
    processes: {
      total: number;
      running: number;
      sleeping: number;
      zombie: number;
    };
  };
  ros: {
    nodes: Array<{
      name: string;
      status: "active" | "inactive" | "error";
      cpu: number;
      memory: number;
    }>;
    topics: number;
    services: number;
    masterRunning: boolean;
  };
}

export default function SystemMonitoring() {
  // Apply Tesla theme
  React.useEffect(() => {
    document.body.classList.add("tesla-ui");
  }, []);

  // Persistent monitoring preferences
  const { store: monitoringPrefs, updateField: updateMonitoringPref } =
    usePersistentStore("monitoring-preferences", {
      autoRefresh: true,
      refreshInterval: 5000,
      showCharts: true,
      compactView: false,
      theme: "dark",
    });

  const { autoRefresh, refreshInterval } = monitoringPrefs;
  const [lastUpdate, setLastUpdate] = useState(new Date());

  const [metrics, setMetrics] = useState<SystemMetrics>({
    cpu: {
      usage: 45,
      cores: 8,
      frequency: 2400,
      temperature: 65,
      processes: 156,
    },
    memory: {
      total: 16384,
      used: 8192,
      free: 6144,
      cached: 2048,
      swap: { total: 8192, used: 1024 },
    },
    disk: {
      total: 512000,
      used: 256000,
      free: 256000,
      io: { read: 125, write: 89 },
    },
    network: {
      interfaces: [
        { name: "eth0", status: "up", rx: 1024, tx: 512, speed: 1000 },
        { name: "wlan0", status: "up", rx: 256, tx: 128, speed: 150 },
        { name: "lo", status: "up", rx: 64, tx: 64, speed: 0 },
      ],
    },
    system: {
      uptime: 86400 * 3,
      loadAverage: [1.2, 1.5, 1.8],
      bootTime: new Date(Date.now() - 86400 * 3 * 1000),
      processes: { total: 156, running: 3, sleeping: 150, zombie: 3 },
    },
    ros: {
      nodes: [
        {
          name: "/robot_state_publisher",
          status: "active",
          cpu: 2.1,
          memory: 25.6,
        },
        { name: "/move_base", status: "active", cpu: 12.3, memory: 128.4 },
        { name: "/camera_node", status: "active", cpu: 8.7, memory: 64.2 },
        { name: "/laser_node", status: "inactive", cpu: 0, memory: 0 },
        { name: "/navigation", status: "error", cpu: 5.2, memory: 32.1 },
      ],
      topics: 24,
      services: 18,
      masterRunning: true,
    },
  });

  const [historicalData, setHistoricalData] = useState<
    Array<{
      time: string;
      cpu: number;
      memory: number;
      disk: number;
      network: number;
    }>
  >([]);

  useEffect(() => {
    const generateHistoricalData = () => {
      const data = [];
      const now = Date.now();
      for (let i = 23; i >= 0; i--) {
        data.push({
          time: new Date(now - i * 60000).toLocaleTimeString("en-US", {
            hour12: false,
            hour: "2-digit",
            minute: "2-digit",
          }),
          cpu: Math.random() * 100,
          memory: Math.random() * 100,
          disk: Math.random() * 100,
          network: Math.random() * 100,
        });
      }
      setHistoricalData(data);
    };

    generateHistoricalData();
  }, []);

  useEffect(() => {
    if (!autoRefresh) return;

    const interval = setInterval(() => {
      // Simulate real-time data updates
      setMetrics((prev) => ({
        ...prev,
        cpu: {
          ...prev.cpu,
          usage: Math.max(
            0,
            Math.min(100, prev.cpu.usage + (Math.random() - 0.5) * 10),
          ),
          temperature: Math.max(
            40,
            Math.min(85, prev.cpu.temperature + (Math.random() - 0.5) * 5),
          ),
        },
        memory: {
          ...prev.memory,
          used: Math.max(
            0,
            Math.min(
              prev.memory.total,
              prev.memory.used + (Math.random() - 0.5) * 512,
            ),
          ),
        },
      }));

      setLastUpdate(new Date());
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [autoRefresh, refreshInterval]);

  const formatBytes = (bytes: number): string => {
    if (bytes === 0) return "0 B";
    const k = 1024;
    const sizes = ["B", "KB", "MB", "GB", "TB"];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return `${parseFloat((bytes / Math.pow(k, i)).toFixed(1))} ${sizes[i]}`;
  };

  const formatUptime = (seconds: number): string => {
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const mins = Math.floor((seconds % 3600) / 60);
    return `${days}d ${hours}h ${mins}m`;
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "active":
      case "up":
        return "text-emerald-400 bg-emerald-500/20 border-emerald-500/30";
      case "inactive":
      case "down":
        return "text-gray-400 bg-white/5 border border-white/100/20 border-gray-500/30";
      case "error":
        return "text-red-400 bg-red-500/20 border-red-500/30";
      default:
        return "text-blue-400 bg-blue-500/20 border-blue-500/30";
    }
  };

  const getTrendIcon = (current: number, threshold: number) => {
    if (current > threshold + 10)
      return <TrendingUp className="h-4 w-4 text-red-400" />;
    if (current < threshold - 10)
      return <TrendingDown className="h-4 w-4 text-emerald-400" />;
    return <Minus className="h-4 w-4 text-gray-400" />;
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Tesla Header */}
      <div className="mb-8">
        <div className="bg-white/10 backdrop-blur-xl border border-white/20 rounded-3xl p-6 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="space-y-2">
              <h1 className="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
                System Monitoring
              </h1>
              <p className="text-slate-300 font-light">
                Real-time system performance and health monitoring
              </p>
            </div>

            <div className="flex items-center gap-4">
              <Badge
                className={`px-4 py-2 ${
                  autoRefresh
                    ? "bg-emerald-500/20 text-emerald-300 border border-emerald-500/30"
                    : "bg-white/5 border border-white/100/20 text-gray-300 border border-gray-500/30"
                }`}
              >
                {autoRefresh ? (
                  <>
                    <RefreshCw className="h-3 w-3 mr-1 animate-spin" />
                    Auto-Refresh
                  </>
                ) : (
                  <>
                    <Clock className="h-3 w-3 mr-1" />
                    Paused
                  </>
                )}
              </Badge>

              <Button
                onClick={() =>
                  updateMonitoringPref("autoRefresh", !autoRefresh)
                }
                className="bg-white/10 hover:bg-white/20 border border-white/20 text-white gap-2"
              >
                {autoRefresh ? (
                  <>
                    <RefreshCw className="h-4 w-4 animate-spin" />
                    Pause
                  </>
                ) : (
                  <>
                    <RefreshCw className="h-4 w-4" />
                    Resume
                  </>
                )}
              </Button>

              <Button className="bg-white/10 hover:bg-white/20 border border-white/20 text-white gap-2">
                <Download className="h-4 w-4" />
                Export
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* Overview Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">CPU Usage</p>
              <p className="text-2xl font-light text-white">
                {metrics.cpu.usage.toFixed(1)}%
              </p>
              <p className="text-xs text-slate-400">
                {metrics.cpu.cores} cores @ {metrics.cpu.frequency}MHz
              </p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center">
              <Cpu className="h-6 w-6 text-blue-400" />
            </div>
          </div>
          <div className="mt-4">
            <Progress value={metrics.cpu.usage} className="h-2 bg-slate-700" />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-slate-400">
              Temp: {metrics.cpu.temperature}°C
            </span>
            {getTrendIcon(metrics.cpu.usage, 50)}
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Memory</p>
              <p className="text-2xl font-light text-white">
                {((metrics.memory.used / metrics.memory.total) * 100).toFixed(
                  1,
                )}
                %
              </p>
              <p className="text-xs text-slate-400">
                {formatBytes(metrics.memory.used * 1024 * 1024)} /{" "}
                {formatBytes(metrics.memory.total * 1024 * 1024)}
              </p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-emerald-500/20 to-teal-500/20 rounded-xl flex items-center justify-center">
              <Database className="h-6 w-6 text-emerald-400" />
            </div>
          </div>
          <div className="mt-4">
            <Progress
              value={(metrics.memory.used / metrics.memory.total) * 100}
              className="h-2 bg-slate-700"
            />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-slate-400">
              Cached: {formatBytes(metrics.memory.cached * 1024 * 1024)}
            </span>
            {getTrendIcon(
              (metrics.memory.used / metrics.memory.total) * 100,
              70,
            )}
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Disk Usage</p>
              <p className="text-2xl font-light text-white">
                {((metrics.disk.used / metrics.disk.total) * 100).toFixed(1)}%
              </p>
              <p className="text-xs text-slate-400">
                {formatBytes(metrics.disk.used * 1024 * 1024)} /{" "}
                {formatBytes(metrics.disk.total * 1024 * 1024)}
              </p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center">
              <HardDrive className="h-6 w-6 text-purple-400" />
            </div>
          </div>
          <div className="mt-4">
            <Progress
              value={(metrics.disk.used / metrics.disk.total) * 100}
              className="h-2 bg-slate-700"
            />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-slate-400">
              Free: {formatBytes(metrics.disk.free * 1024 * 1024)}
            </span>
            {getTrendIcon((metrics.disk.used / metrics.disk.total) * 100, 80)}
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">System Load</p>
              <p className="text-2xl font-light text-white">
                {metrics.system.loadAverage[0].toFixed(1)}
              </p>
              <p className="text-xs text-slate-400">
                {formatUptime(metrics.system.uptime)} uptime
              </p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-orange-500/20 to-red-500/20 rounded-xl flex items-center justify-center">
              <Activity className="h-6 w-6 text-orange-400" />
            </div>
          </div>
          <div className="mt-4">
            <Progress
              value={metrics.system.loadAverage[0] * 20}
              className="h-2 bg-slate-700"
            />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-slate-400">
              Processes: {metrics.system.processes.total}
            </span>
            {getTrendIcon(metrics.system.loadAverage[0] * 20, 60)}
          </div>
        </Card>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Charts Section */}
        <div className="lg:col-span-2">
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-2xl">
            <div className="p-6">
              <h3 className="text-xl font-light text-white mb-6">
                Performance Trends
              </h3>

              {/* Chart Placeholder */}
              <div className="aspect-video bg-gradient-to-br from-slate-800 to-slate-900 rounded-xl border border-white/10 flex items-center justify-center">
                <div className="text-center">
                  <BarChart3 className="h-16 w-16 text-blue-400 mx-auto mb-4" />
                  <p className="text-slate-300">Real-time Performance Charts</p>
                  <p className="text-slate-400 text-sm mt-2">
                    CPU, Memory, and Network usage over time
                  </p>
                </div>
              </div>
            </div>
          </Card>
        </div>

        {/* ROS Nodes & System Info */}
        <div className="space-y-6">
          {/* ROS Nodes */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">ROS Nodes</h3>
              <ScrollArea className="h-[200px]">
                <div className="space-y-3">
                  {metrics.ros.nodes.map((node, index) => (
                    <div
                      key={index}
                      className="flex items-center justify-between p-3 bg-white/5 rounded-lg border border-white/10"
                    >
                      <div className="flex items-center gap-3">
                        <div
                          className={`w-3 h-3 rounded-full ${
                            node.status === "active"
                              ? "bg-emerald-400"
                              : node.status === "error"
                                ? "bg-red-400"
                                : "bg-gray-400"
                          }`}
                        />
                        <div>
                          <p className="text-white text-sm font-medium">
                            {node.name}
                          </p>
                          <p className="text-slate-400 text-xs">
                            {node.cpu.toFixed(1)}% CPU |{" "}
                            {node.memory.toFixed(1)}MB
                          </p>
                        </div>
                      </div>
                      <Badge className={getStatusColor(node.status)}>
                        {node.status}
                      </Badge>
                    </div>
                  ))}
                </div>
              </ScrollArea>
            </div>
          </Card>

          {/* Network Interfaces */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                Network Interfaces
              </h3>
              <div className="space-y-3">
                {metrics.network.interfaces.map((iface, index) => (
                  <div
                    key={index}
                    className="flex items-center justify-between p-3 bg-white/5 rounded-lg border border-white/10"
                  >
                    <div className="flex items-center gap-3">
                      <div
                        className={`w-3 h-3 rounded-full ${
                          iface.status === "up"
                            ? "bg-emerald-400"
                            : "bg-red-400"
                        }`}
                      />
                      <div>
                        <p className="text-white text-sm font-medium">
                          {iface.name}
                        </p>
                        <p className="text-slate-400 text-xs">
                          ↑{iface.tx}KB/s ↓{iface.rx}KB/s
                        </p>
                      </div>
                    </div>
                    <Badge className={getStatusColor(iface.status)}>
                      {iface.status}
                    </Badge>
                  </div>
                ))}
              </div>
            </div>
          </Card>
        </div>
      </div>

      {/* Footer Status */}
      <div className="mt-8 flex items-center justify-between text-sm text-slate-400 bg-white/10 p-4 rounded-lg border border-white/20">
        <div className="flex items-center gap-4">
          <span>Last Update: {lastUpdate.toLocaleTimeString()}</span>
          <span>•</span>
          <span>
            ROS Master: {metrics.ros.masterRunning ? "Running" : "Stopped"}
          </span>
          <span>•</span>
          <span>Topics: {metrics.ros.topics}</span>
          <span>•</span>
          <span>Services: {metrics.ros.services}</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-2 h-2 bg-emerald-400 rounded-full animate-pulse" />
          <span>System Healthy</span>
        </div>
      </div>
    </div>
  );
}
