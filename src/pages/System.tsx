import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { ScrollArea } from "@/components/ui/scroll-area";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  BarChart3,
  Cpu,
  HardDrive,
  MemoryStick,
  Wifi,
  Thermometer,
  Activity,
  Zap,
  AlertTriangle,
  CheckCircle,
  RefreshCw,
  Settings,
  Download,
  Server,
  Clock,
  Battery,
  Network,
  Monitor,
} from "lucide-react";

interface SystemMetrics {
  cpu: {
    usage: number;
    cores: number;
    frequency: number;
    temperature: number;
  };
  memory: {
    used: number;
    total: number;
    available: number;
    percentage: number;
  };
  disk: {
    used: number;
    total: number;
    available: number;
    percentage: number;
  };
  network: {
    bytesIn: number;
    bytesOut: number;
    packetsIn: number;
    packetsOut: number;
    latency: number;
  };
  ros: {
    nodes: number;
    topics: number;
    services: number;
    uptime: number;
  };
  power: {
    voltage: number;
    current: number;
    percentage: number;
    timeRemaining: number;
  };
}

interface ProcessInfo {
  pid: number;
  name: string;
  cpu: number;
  memory: number;
  status: "running" | "sleeping" | "stopped";
}

const System = () => {
  const { t } = useLanguage();

  const [metrics, setMetrics] = useState<SystemMetrics>({
    cpu: { usage: 45.2, cores: 4, frequency: 2.4, temperature: 58.3 },
    memory: { used: 3.2, total: 8.0, available: 4.8, percentage: 40 },
    disk: { used: 45.6, total: 128.0, available: 82.4, percentage: 35.6 },
    network: {
      bytesIn: 1024000,
      bytesOut: 512000,
      packetsIn: 8540,
      packetsOut: 6230,
      latency: 12.5,
    },
    ros: { nodes: 15, topics: 42, services: 28, uptime: 7320 },
    power: { voltage: 24.6, current: 3.2, percentage: 78, timeRemaining: 150 },
  });

  const [processes, setProcesses] = useState<ProcessInfo[]>([
    { pid: 1234, name: "roscore", cpu: 5.2, memory: 125.4, status: "running" },
    {
      pid: 1235,
      name: "move_base",
      cpu: 12.1,
      memory: 89.2,
      status: "running",
    },
    { pid: 1236, name: "amcl", cpu: 8.7, memory: 67.8, status: "running" },
    {
      pid: 1237,
      name: "laser_scan",
      cpu: 3.4,
      memory: 45.6,
      status: "running",
    },
    {
      pid: 1238,
      name: "camera_node",
      cpu: 15.8,
      memory: 156.7,
      status: "running",
    },
    { pid: 1239, name: "tf2_ros", cpu: 2.1, memory: 34.2, status: "running" },
    {
      pid: 1240,
      name: "robot_localization",
      cpu: 6.9,
      memory: 78.3,
      status: "running",
    },
    {
      pid: 1241,
      name: "joint_state_publisher",
      cpu: 1.2,
      memory: 23.1,
      status: "sleeping",
    },
  ]);

  const [isAutoRefresh, setIsAutoRefresh] = useState(true);
  const [refreshInterval, setRefreshInterval] = useState(2000);

  // Simulate real-time metrics updates
  useEffect(() => {
    if (!isAutoRefresh) return;

    const interval = setInterval(() => {
      setMetrics((prev) => ({
        ...prev,
        cpu: {
          ...prev.cpu,
          usage: Math.max(
            0,
            Math.min(100, prev.cpu.usage + (Math.random() - 0.5) * 10),
          ),
          temperature: Math.max(
            30,
            Math.min(80, prev.cpu.temperature + (Math.random() - 0.5) * 5),
          ),
        },
        memory: {
          ...prev.memory,
          percentage: Math.max(
            0,
            Math.min(100, prev.memory.percentage + (Math.random() - 0.5) * 5),
          ),
        },
        network: {
          ...prev.network,
          bytesIn: prev.network.bytesIn + Math.floor(Math.random() * 10000),
          bytesOut: prev.network.bytesOut + Math.floor(Math.random() * 5000),
          latency: Math.max(
            1,
            Math.min(100, prev.network.latency + (Math.random() - 0.5) * 2),
          ),
        },
        ros: {
          ...prev.ros,
          uptime: prev.ros.uptime + 2,
        },
        power: {
          ...prev.power,
          percentage: Math.max(0, Math.min(100, prev.power.percentage - 0.01)),
          timeRemaining: Math.max(0, prev.power.timeRemaining - 0.5),
        },
      }));

      // Update processes
      setProcesses((prev) =>
        prev.map((process) => ({
          ...process,
          cpu: Math.max(
            0,
            Math.min(100, process.cpu + (Math.random() - 0.5) * 3),
          ),
          memory: Math.max(0, process.memory + (Math.random() - 0.5) * 10),
        })),
      );
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [isAutoRefresh, refreshInterval]);

  const formatBytes = (bytes: number) => {
    if (bytes < 1024) return `${bytes.toFixed(1)} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    if (bytes < 1024 * 1024 * 1024)
      return `${(bytes / 1024 / 1024).toFixed(1)} MB`;
    return `${(bytes / 1024 / 1024 / 1024).toFixed(1)} GB`;
  };

  const formatUptime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return `${hours}h ${minutes}m`;
  };

  const getStatusColor = (
    value: number,
    thresholds: { warning: number; critical: number },
  ) => {
    if (value >= thresholds.critical) return "text-red-500";
    if (value >= thresholds.warning) return "text-yellow-500";
    return "text-green-500";
  };

  const getProgressVariant = (
    value: number,
    thresholds: { warning: number; critical: number },
  ) => {
    if (value >= thresholds.critical) return "destructive";
    if (value >= thresholds.warning) return "secondary";
    return "default";
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <BarChart3 className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            {t("nav.system")}
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Real-time system performance and health monitoring
          </p>
        </div>

        <div className="flex gap-2">
          <Button
            variant={isAutoRefresh ? "default" : "outline"}
            onClick={() => setIsAutoRefresh(!isAutoRefresh)}
            className="gap-2"
          >
            <RefreshCw
              className={`h-4 w-4 ${isAutoRefresh ? "animate-spin" : ""}`}
            />
            Auto Refresh
          </Button>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
          <Button variant="outline" className="gap-2">
            <Settings className="h-4 w-4" />
            Settings
          </Button>
        </div>
      </div>

      {/* System Overview Cards */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
        {/* CPU Usage */}
        <Card className="p-4 hover-lift">
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-2">
              <Cpu className="h-5 w-5 text-blue-500" />
              <span className="font-medium">CPU</span>
            </div>
            <Badge variant="outline" className="text-xs">
              {metrics.cpu.cores} cores
            </Badge>
          </div>

          <div className="space-y-2">
            <div className="flex justify-between text-sm">
              <span className="text-muted-foreground">Usage:</span>
              <span
                className={`font-mono ${getStatusColor(metrics.cpu.usage, { warning: 70, critical: 90 })}`}
              >
                {metrics.cpu.usage.toFixed(1)}%
              </span>
            </div>
            <Progress
              value={metrics.cpu.usage}
              className="h-2"
              variant={getProgressVariant(metrics.cpu.usage, {
                warning: 70,
                critical: 90,
              })}
            />

            <div className="flex justify-between text-xs text-muted-foreground">
              <span>Freq: {metrics.cpu.frequency} GHz</span>
              <span>Temp: {metrics.cpu.temperature.toFixed(1)}°C</span>
            </div>
          </div>
        </Card>

        {/* Memory Usage */}
        <Card className="p-4 hover-lift">
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-2">
              <MemoryStick className="h-5 w-5 text-green-500" />
              <span className="font-medium">Memory</span>
            </div>
            <Badge variant="outline" className="text-xs">
              {metrics.memory.total.toFixed(1)} GB
            </Badge>
          </div>

          <div className="space-y-2">
            <div className="flex justify-between text-sm">
              <span className="text-muted-foreground">Used:</span>
              <span
                className={`font-mono ${getStatusColor(metrics.memory.percentage, { warning: 80, critical: 95 })}`}
              >
                {metrics.memory.percentage.toFixed(1)}%
              </span>
            </div>
            <Progress
              value={metrics.memory.percentage}
              className="h-2"
              variant={getProgressVariant(metrics.memory.percentage, {
                warning: 80,
                critical: 95,
              })}
            />

            <div className="flex justify-between text-xs text-muted-foreground">
              <span>Used: {metrics.memory.used.toFixed(1)} GB</span>
              <span>Free: {metrics.memory.available.toFixed(1)} GB</span>
            </div>
          </div>
        </Card>

        {/* Disk Usage */}
        <Card className="p-4 hover-lift">
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-2">
              <HardDrive className="h-5 w-5 text-purple-500" />
              <span className="font-medium">Storage</span>
            </div>
            <Badge variant="outline" className="text-xs">
              {metrics.disk.total.toFixed(0)} GB
            </Badge>
          </div>

          <div className="space-y-2">
            <div className="flex justify-between text-sm">
              <span className="text-muted-foreground">Used:</span>
              <span
                className={`font-mono ${getStatusColor(metrics.disk.percentage, { warning: 80, critical: 95 })}`}
              >
                {metrics.disk.percentage.toFixed(1)}%
              </span>
            </div>
            <Progress
              value={metrics.disk.percentage}
              className="h-2"
              variant={getProgressVariant(metrics.disk.percentage, {
                warning: 80,
                critical: 95,
              })}
            />

            <div className="flex justify-between text-xs text-muted-foreground">
              <span>Used: {metrics.disk.used.toFixed(1)} GB</span>
              <span>Free: {metrics.disk.available.toFixed(1)} GB</span>
            </div>
          </div>
        </Card>

        {/* Power Status */}
        <Card className="p-4 hover-lift">
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-2">
              <Battery className="h-5 w-5 text-yellow-500" />
              <span className="font-medium">Power</span>
            </div>
            <Badge variant="outline" className="text-xs">
              {metrics.power.voltage.toFixed(1)}V
            </Badge>
          </div>

          <div className="space-y-2">
            <div className="flex justify-between text-sm">
              <span className="text-muted-foreground">Battery:</span>
              <span
                className={`font-mono ${getStatusColor(100 - metrics.power.percentage, { warning: 50, critical: 80 })}`}
              >
                {metrics.power.percentage.toFixed(1)}%
              </span>
            </div>
            <Progress
              value={metrics.power.percentage}
              className="h-2"
              variant={getProgressVariant(100 - metrics.power.percentage, {
                warning: 50,
                critical: 80,
              })}
            />

            <div className="flex justify-between text-xs text-muted-foreground">
              <span>Current: {metrics.power.current.toFixed(1)}A</span>
              <span>Time: {Math.floor(metrics.power.timeRemaining)}m</span>
            </div>
          </div>
        </Card>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* ROS System Status */}
        <Card className="p-4 md:p-6">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Server className="h-5 w-5 text-primary" />
            ROS System
          </h3>

          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">
                System Status:
              </span>
              <Badge className="gap-1">
                <CheckCircle className="h-3 w-3" />
                RUNNING
              </Badge>
            </div>

            <div className="space-y-3">
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">
                  Active Nodes:
                </span>
                <Badge variant="outline">{metrics.ros.nodes}</Badge>
              </div>
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">Topics:</span>
                <Badge variant="outline">{metrics.ros.topics}</Badge>
              </div>
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">Services:</span>
                <Badge variant="outline">{metrics.ros.services}</Badge>
              </div>
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">Uptime:</span>
                <span className="text-sm font-mono">
                  {formatUptime(metrics.ros.uptime)}
                </span>
              </div>
            </div>
          </div>
        </Card>

        {/* Network Status */}
        <Card className="p-4 md:p-6">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Network className="h-5 w-5 text-primary" />
            Network
          </h3>

          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">Connection:</span>
              <Badge className="gap-1">
                <Wifi className="h-3 w-3" />
                CONNECTED
              </Badge>
            </div>

            <div className="space-y-3">
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">Bytes In:</span>
                <span className="text-sm font-mono">
                  {formatBytes(metrics.network.bytesIn)}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">
                  Bytes Out:
                </span>
                <span className="text-sm font-mono">
                  {formatBytes(metrics.network.bytesOut)}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">
                  Packets In:
                </span>
                <span className="text-sm font-mono">
                  {metrics.network.packetsIn.toLocaleString()}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">Latency:</span>
                <span
                  className={`text-sm font-mono ${getStatusColor(metrics.network.latency, { warning: 50, critical: 100 })}`}
                >
                  {metrics.network.latency.toFixed(1)}ms
                </span>
              </div>
            </div>
          </div>
        </Card>

        {/* Temperature Monitoring */}
        <Card className="p-4 md:p-6">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Thermometer className="h-5 w-5 text-primary" />
            Temperature
          </h3>

          <div className="space-y-4">
            <div className="space-y-3">
              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-muted-foreground">CPU:</span>
                  <span
                    className={`font-mono ${getStatusColor(metrics.cpu.temperature, { warning: 70, critical: 85 })}`}
                  >
                    {metrics.cpu.temperature.toFixed(1)}°C
                  </span>
                </div>
                <Progress
                  value={(metrics.cpu.temperature / 100) * 100}
                  className="h-2"
                  variant={getProgressVariant(metrics.cpu.temperature, {
                    warning: 70,
                    critical: 85,
                  })}
                />
              </div>

              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-muted-foreground">GPU:</span>
                  <span className="font-mono text-green-500">45.2°C</span>
                </div>
                <Progress value={45} className="h-2" />
              </div>

              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-muted-foreground">Motherboard:</span>
                  <span className="font-mono text-green-500">38.7°C</span>
                </div>
                <Progress value={39} className="h-2" />
              </div>
            </div>
          </div>
        </Card>
      </div>

      {/* Process Monitor */}
      <Card className="p-4 md:p-6">
        <h3 className="text-lg font-light mb-4 flex items-center gap-2">
          <Activity className="h-5 w-5 text-primary" />
          ROS Process Monitor
        </h3>

        <div className="overflow-x-auto">
          <ScrollArea className="h-64">
            <table className="w-full text-sm">
              <thead>
                <tr className="border-b text-left">
                  <th className="pb-2 font-medium">PID</th>
                  <th className="pb-2 font-medium">Process Name</th>
                  <th className="pb-2 font-medium">CPU %</th>
                  <th className="pb-2 font-medium">Memory (MB)</th>
                  <th className="pb-2 font-medium">Status</th>
                </tr>
              </thead>
              <tbody>
                {processes.map((process, index) => (
                  <tr
                    key={process.pid}
                    className={`border-b hover:bg-muted/50 stagger-item`}
                    style={{ animationDelay: `${index * 0.05}s` }}
                  >
                    <td className="py-2 font-mono text-xs">{process.pid}</td>
                    <td className="py-2 font-mono">{process.name}</td>
                    <td className="py-2">
                      <span
                        className={`font-mono ${getStatusColor(process.cpu, { warning: 15, critical: 30 })}`}
                      >
                        {process.cpu.toFixed(1)}%
                      </span>
                    </td>
                    <td className="py-2">
                      <span className="font-mono">
                        {process.memory.toFixed(1)}
                      </span>
                    </td>
                    <td className="py-2">
                      <Badge
                        variant={
                          process.status === "running" ? "default" : "secondary"
                        }
                        className="text-xs"
                      >
                        {process.status === "running" && (
                          <Activity className="h-3 w-3 mr-1" />
                        )}
                        {process.status === "sleeping" && (
                          <Clock className="h-3 w-3 mr-1" />
                        )}
                        {process.status.toUpperCase()}
                      </Badge>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </ScrollArea>
        </div>
      </Card>
    </div>
  );
};

export default System;
