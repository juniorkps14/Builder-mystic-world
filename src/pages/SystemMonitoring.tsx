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

  // Persistent monitoring preferences
  const { store: monitoringPrefs, updateField: updateMonitoringPref } = usePersistentStore(
    "monitoring-preferences",
    {
      autoRefresh: true,
      refreshInterval: 5000,
      showCharts: true,
      compactView: false,
      theme: "dark",
    }
  );

  const { autoRefresh, refreshInterval } = monitoringPrefs;
  const [lastUpdate, setLastUpdate] = useState(new Date());

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

      // Update historical data
      setHistoricalData((prev) => {
        const newData = [...prev.slice(1)];
        newData.push({
          time: new Date().toLocaleTimeString("en-US", {
            hour12: false,
            hour: "2-digit",
            minute: "2-digit",
          }),
          cpu: metrics.cpu.usage,
          memory: (metrics.memory.used / metrics.memory.total) * 100,
          disk: (metrics.disk.used / metrics.disk.total) * 100,
          network: Math.random() * 100,
        });
        return newData;
      });

      setLastUpdate(new Date());
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [
    autoRefresh,
    refreshInterval,
    metrics.cpu.usage,
    metrics.memory.used,
    metrics.disk.used,
  ]);

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
        return "text-green-600 bg-green-100";
      case "inactive":
      case "down":
        return "text-gray-600 bg-gray-100";
      case "error":
        return "text-red-600 bg-red-100";
      default:
        return "text-blue-600 bg-blue-100";
    }
  };

  const getTrendIcon = (current: number, threshold: number) => {
    if (current > threshold + 10)
      return <TrendingUp className="h-4 w-4 text-red-500" />;
    if (current < threshold - 10)
      return <TrendingDown className="h-4 w-4 text-green-500" />;
    return <Minus className="h-4 w-4 text-gray-500" />;
  };

  const pieData = [
    { name: "Used", value: metrics.memory.used, color: "#ef4444" },
    { name: "Cached", value: metrics.memory.cached, color: "#f59e0b" },
    { name: "Free", value: metrics.memory.free, color: "#10b981" },
  ];

  const processData = [
    {
      name: "Running",
      value: metrics.system.processes.running,
      color: "#10b981",
    },
    {
      name: "Sleeping",
      value: metrics.system.processes.sleeping,
      color: "#3b82f6",
    },
    {
      name: "Zombie",
      value: metrics.system.processes.zombie,
      color: "#ef4444",
    },
  ];

  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            System Monitoring
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Real-time system performance and ROS node monitoring
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge
            className={
              autoRefresh
                ? "bg-green-100 text-green-700"
                : "bg-gray-100 text-gray-700"
            }
          >
            {autoRefresh ? (
              <>
                <Activity className="h-3 w-3 mr-1 animate-pulse" />
                Live
              </>
            ) : (
              <>
                <Clock className="h-3 w-3 mr-1" />
                Paused
              </>
            )}
          </Badge>

          <Button
            variant="outline"
            size="sm"
            onClick={() => setAutoRefresh(!autoRefresh)}
            className="gap-2"
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

          <Button variant="outline" size="sm" className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
        </div>
      </div>

      {/* Overview Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        <Card className="p-6">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm text-gray-600">CPU Usage</p>
              <p className="text-2xl font-bold text-gray-900">
                {metrics.cpu.usage.toFixed(1)}%
              </p>
              <p className="text-xs text-gray-500">
                {metrics.cpu.cores} cores @ {metrics.cpu.frequency}MHz
              </p>
            </div>
            <div className="p-3 bg-blue-100 rounded-lg">
              <Cpu className="h-6 w-6 text-blue-600" />
            </div>
          </div>
          <div className="mt-4">
            <Progress value={metrics.cpu.usage} className="h-2" />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-gray-500">
              Temp: {metrics.cpu.temperature}°C
            </span>
            {getTrendIcon(metrics.cpu.usage, 50)}
          </div>
        </Card>

        <Card className="p-6">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm text-gray-600">Memory</p>
              <p className="text-2xl font-bold text-gray-900">
                {((metrics.memory.used / metrics.memory.total) * 100).toFixed(
                  1,
                )}
                %
              </p>
              <p className="text-xs text-gray-500">
                {formatBytes(metrics.memory.used * 1024 * 1024)} /{" "}
                {formatBytes(metrics.memory.total * 1024 * 1024)}
              </p>
            </div>
            <div className="p-3 bg-green-100 rounded-lg">
              <Database className="h-6 w-6 text-green-600" />
            </div>
          </div>
          <div className="mt-4">
            <Progress
              value={(metrics.memory.used / metrics.memory.total) * 100}
              className="h-2"
            />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-gray-500">
              Cached: {formatBytes(metrics.memory.cached * 1024 * 1024)}
            </span>
            {getTrendIcon(
              (metrics.memory.used / metrics.memory.total) * 100,
              70,
            )}
          </div>
        </Card>

        <Card className="p-6">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm text-gray-600">Disk Usage</p>
              <p className="text-2xl font-bold text-gray-900">
                {((metrics.disk.used / metrics.disk.total) * 100).toFixed(1)}%
              </p>
              <p className="text-xs text-gray-500">
                {formatBytes(metrics.disk.used * 1024 * 1024)} /{" "}
                {formatBytes(metrics.disk.total * 1024 * 1024)}
              </p>
            </div>
            <div className="p-3 bg-purple-100 rounded-lg">
              <HardDrive className="h-6 w-6 text-purple-600" />
            </div>
          </div>
          <div className="mt-4">
            <Progress
              value={(metrics.disk.used / metrics.disk.total) * 100}
              className="h-2"
            />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-gray-500">
              I/O: {metrics.disk.io.read}MB/s ↓ {metrics.disk.io.write}MB/s ↑
            </span>
            {getTrendIcon((metrics.disk.used / metrics.disk.total) * 100, 80)}
          </div>
        </Card>

        <Card className="p-6">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm text-gray-600">Load Average</p>
              <p className="text-2xl font-bold text-gray-900">
                {metrics.system.loadAverage[0].toFixed(2)}
              </p>
              <p className="text-xs text-gray-500">
                {metrics.system.loadAverage[1].toFixed(2)} |{" "}
                {metrics.system.loadAverage[2].toFixed(2)}
              </p>
            </div>
            <div className="p-3 bg-orange-100 rounded-lg">
              <Activity className="h-6 w-6 text-orange-600" />
            </div>
          </div>
          <div className="mt-4">
            <Progress
              value={(metrics.system.loadAverage[0] / metrics.cpu.cores) * 100}
              className="h-2"
            />
          </div>
          <div className="flex items-center justify-between mt-2">
            <span className="text-xs text-gray-500">
              Uptime: {formatUptime(metrics.system.uptime)}
            </span>
            {getTrendIcon(metrics.system.loadAverage[0], 2)}
          </div>
        </Card>
      </div>

      {/* Detailed Monitoring */}
      <Tabs defaultValue="performance" className="space-y-6">
        <TabsList className="grid w-full grid-cols-4">
          <TabsTrigger value="performance">Performance</TabsTrigger>
          <TabsTrigger value="processes">Processes</TabsTrigger>
          <TabsTrigger value="network">Network</TabsTrigger>
          <TabsTrigger value="ros">ROS Nodes</TabsTrigger>
        </TabsList>

        {/* Performance Charts */}
        <TabsContent value="performance">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                System Performance (24h)
              </h3>
              <ResponsiveContainer width="100%" height={300}>
                <LineChart data={historicalData}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="time" />
                  <YAxis domain={[0, 100]} />
                  <Tooltip
                    formatter={(value: number) => [`${value.toFixed(1)}%`]}
                  />
                  <Line
                    type="monotone"
                    dataKey="cpu"
                    stroke="#3b82f6"
                    strokeWidth={2}
                    name="CPU"
                  />
                  <Line
                    type="monotone"
                    dataKey="memory"
                    stroke="#10b981"
                    strokeWidth={2}
                    name="Memory"
                  />
                  <Line
                    type="monotone"
                    dataKey="disk"
                    stroke="#8b5cf6"
                    strokeWidth={2}
                    name="Disk"
                  />
                </LineChart>
              </ResponsiveContainer>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                Memory Distribution
              </h3>
              <ResponsiveContainer width="100%" height={300}>
                <PieChart>
                  <Pie
                    data={pieData}
                    cx="50%"
                    cy="50%"
                    innerRadius={60}
                    outerRadius={120}
                    paddingAngle={5}
                    dataKey="value"
                  >
                    {pieData.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={entry.color} />
                    ))}
                  </Pie>
                  <Tooltip
                    formatter={(value: number) => [
                      formatBytes(value * 1024 * 1024),
                    ]}
                  />
                </PieChart>
              </ResponsiveContainer>
              <div className="flex justify-center gap-4 mt-4">
                {pieData.map((entry, index) => (
                  <div key={index} className="flex items-center gap-2">
                    <div
                      className="w-3 h-3 rounded"
                      style={{ backgroundColor: entry.color }}
                    />
                    <span className="text-sm">{entry.name}</span>
                  </div>
                ))}
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Process Management */}
        <TabsContent value="processes">
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Process Overview</h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Total Processes</span>
                  <Badge>{metrics.system.processes.total}</Badge>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Running</span>
                  <Badge className="bg-green-100 text-green-700">
                    {metrics.system.processes.running}
                  </Badge>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Sleeping</span>
                  <Badge className="bg-blue-100 text-blue-700">
                    {metrics.system.processes.sleeping}
                  </Badge>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Zombie</span>
                  <Badge className="bg-red-100 text-red-700">
                    {metrics.system.processes.zombie}
                  </Badge>
                </div>
              </div>
            </Card>

            <Card className="p-6 lg:col-span-2">
              <h3 className="text-lg font-semibold mb-4">
                Process Distribution
              </h3>
              <ResponsiveContainer width="100%" height={200}>
                <BarChart data={processData} layout="horizontal">
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis type="number" />
                  <YAxis dataKey="name" type="category" width={80} />
                  <Tooltip />
                  <Bar dataKey="value" fill="#3b82f6" />
                </BarChart>
              </ResponsiveContainer>
            </Card>
          </div>
        </TabsContent>

        {/* Network Status */}
        <TabsContent value="network">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">Network Interfaces</h3>
            <div className="space-y-4">
              {metrics.network.interfaces.map((iface, index) => (
                <div
                  key={index}
                  className="flex items-center justify-between p-4 bg-gray-50 rounded-lg"
                >
                  <div className="flex items-center gap-4">
                    <div className="p-2 bg-blue-100 rounded-lg">
                      <Network className="h-5 w-5 text-blue-600" />
                    </div>
                    <div>
                      <h4 className="font-medium">{iface.name}</h4>
                      <p className="text-sm text-gray-500">
                        Speed: {iface.speed > 0 ? `${iface.speed} Mbps` : "N/A"}
                      </p>
                    </div>
                  </div>

                  <div className="flex items-center gap-6">
                    <div className="text-center">
                      <p className="text-sm text-gray-500">RX</p>
                      <p className="font-medium">{formatBytes(iface.rx)}/s</p>
                    </div>
                    <div className="text-center">
                      <p className="text-sm text-gray-500">TX</p>
                      <p className="font-medium">{formatBytes(iface.tx)}/s</p>
                    </div>
                    <Badge className={getStatusColor(iface.status)}>
                      {iface.status.toUpperCase()}
                    </Badge>
                  </div>
                </div>
              ))}
            </div>
          </Card>
        </TabsContent>

        {/* ROS Node Status */}
        <TabsContent value="ros">
          <div className="space-y-6">
            <Card className="p-6">
              <div className="flex items-center justify-between mb-6">
                <h3 className="text-lg font-semibold">ROS System Status</h3>
                <div className="flex items-center gap-4">
                  <Badge
                    className={
                      metrics.ros.masterRunning
                        ? "bg-green-100 text-green-700"
                        : "bg-red-100 text-red-700"
                    }
                  >
                    {metrics.ros.masterRunning ? (
                      <>
                        <CheckCircle className="h-3 w-3 mr-1" />
                        Master Running
                      </>
                    ) : (
                      <>
                        <AlertTriangle className="h-3 w-3 mr-1" />
                        Master Down
                      </>
                    )}
                  </Badge>
                  <Badge>Topics: {metrics.ros.topics}</Badge>
                  <Badge>Services: {metrics.ros.services}</Badge>
                </div>
              </div>

              <div className="space-y-3">
                {metrics.ros.nodes.map((node, index) => (
                  <div
                    key={index}
                    className="flex items-center justify-between p-4 bg-gray-50 rounded-lg"
                  >
                    <div className="flex items-center gap-4">
                      <div className="p-2 bg-purple-100 rounded-lg">
                        <Server className="h-5 w-5 text-purple-600" />
                      </div>
                      <div>
                        <h4 className="font-medium">{node.name}</h4>
                        <p className="text-sm text-gray-500">
                          CPU: {node.cpu.toFixed(1)}% | Memory:{" "}
                          {node.memory.toFixed(1)} MB
                        </p>
                      </div>
                    </div>

                    <div className="flex items-center gap-4">
                      <div className="text-right">
                        <Progress value={node.cpu} className="w-20 h-2" />
                        <p className="text-xs text-gray-500 mt-1">CPU Usage</p>
                      </div>
                      <Badge className={getStatusColor(node.status)}>
                        {node.status.toUpperCase()}
                      </Badge>
                    </div>
                  </div>
                ))}
              </div>
            </Card>
          </div>
        </TabsContent>
      </Tabs>

      {/* Footer Status */}
      <div className="mt-8 flex items-center justify-between text-sm text-gray-500 bg-white p-4 rounded-lg">
        <div className="flex items-center gap-4">
          <span>Last updated: {lastUpdate.toLocaleTimeString()}</span>
          <span>Refresh interval: {refreshInterval / 1000}s</span>
        </div>
        <div className="flex items-center gap-4">
          <Badge className="bg-blue-100 text-blue-700">
            <Eye className="h-3 w-3 mr-1" />
            Monitoring {Object.keys(metrics).length} systems
          </Badge>
        </div>
      </div>
    </div>
  );
}
