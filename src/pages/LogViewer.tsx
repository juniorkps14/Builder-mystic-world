import { useState, useEffect, useRef } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Switch } from "@/components/ui/switch";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  FileText,
  Play,
  Pause,
  Square,
  Download,
  Search,
  Filter,
  Trash2,
  Settings,
  AlertTriangle,
  Info,
  CheckCircle,
  Clock,
  Bug,
  Zap,
  RefreshCw,
  Eye,
  EyeOff,
} from "lucide-react";

interface LogEntry {
  id: string;
  timestamp: Date;
  level: "DEBUG" | "INFO" | "WARN" | "ERROR" | "FATAL";
  source: string;
  message: string;
  category: "ros" | "system" | "application" | "hardware";
  details?: any;
}

const LogViewer = () => {
  const { t } = useLanguage();
  const scrollAreaRef = useRef<HTMLDivElement>(null);

  const [logs, setLogs] = useState<LogEntry[]>([
    {
      id: "log_001",
      timestamp: new Date(),
      level: "INFO",
      source: "roscore",
      message: "ROS Master started successfully on port 11311",
      category: "ros",
      details: { port: 11311, pid: 12345 },
    },
    {
      id: "log_002",
      timestamp: new Date(Date.now() - 1000),
      level: "INFO",
      source: "move_base",
      message: "Navigation node initialized",
      category: "ros",
    },
    {
      id: "log_003",
      timestamp: new Date(Date.now() - 2000),
      level: "WARN",
      source: "sensor_driver",
      message: "IMU sensor calibration drift detected",
      category: "hardware",
      details: { drift: 0.05, threshold: 0.1 },
    },
    {
      id: "log_004",
      timestamp: new Date(Date.now() - 3000),
      level: "ERROR",
      source: "camera_node",
      message: "Failed to initialize camera device /dev/video0",
      category: "hardware",
      details: { device: "/dev/video0", error_code: -1 },
    },
    {
      id: "log_005",
      timestamp: new Date(Date.now() - 4000),
      level: "DEBUG",
      source: "robot_controller",
      message: "Velocity command received: linear=0.5, angular=0.2",
      category: "application",
      details: { linear_x: 0.5, angular_z: 0.2 },
    },
  ]);

  const [isLiveMode, setIsLiveMode] = useState(true);
  const [searchQuery, setSearchQuery] = useState("");
  const [levelFilter, setLevelFilter] = useState("ALL");
  const [sourceFilter, setSourceFilter] = useState("ALL");
  const [categoryFilter, setCategoryFilter] = useState("ALL");
  const [autoScroll, setAutoScroll] = useState(true);
  const [maxLogs, setMaxLogs] = useState(1000);

  // Simulate real-time log updates
  useEffect(() => {
    if (!isLiveMode) return;

    const interval = setInterval(() => {
      const sources = [
        "roscore",
        "move_base",
        "amcl",
        "sensor_driver",
        "camera_node",
        "robot_controller",
        "navigation",
        "tf2_ros",
      ];
      const levels: LogEntry["level"][] = [
        "DEBUG",
        "INFO",
        "WARN",
        "ERROR",
        "FATAL",
      ];
      const categories: LogEntry["category"][] = [
        "ros",
        "system",
        "application",
        "hardware",
      ];
      const messages = [
        "Processing navigation goal",
        "Sensor data received",
        "Transform published",
        "Path planning completed",
        "Motor command sent",
        "Collision detected",
        "Battery level: 75%",
        "Temperature reading: 45°C",
        "Network connection established",
        "Memory usage: 45%",
      ];

      const newLog: LogEntry = {
        id: `log_${Date.now()}`,
        timestamp: new Date(),
        level: levels[Math.floor(Math.random() * levels.length)],
        source: sources[Math.floor(Math.random() * sources.length)],
        message: messages[Math.floor(Math.random() * messages.length)],
        category: categories[Math.floor(Math.random() * categories.length)],
      };

      setLogs((prev) => {
        const updated = [newLog, ...prev];
        return updated.slice(0, maxLogs); // Keep only recent logs
      });
    }, 2000);

    return () => clearInterval(interval);
  }, [isLiveMode, maxLogs]);

  // Auto-scroll to top when new logs arrive
  useEffect(() => {
    if (autoScroll && scrollAreaRef.current) {
      scrollAreaRef.current.scrollTop = 0;
    }
  }, [logs, autoScroll]);

  const filteredLogs = logs.filter((log) => {
    const matchesSearch =
      log.message.toLowerCase().includes(searchQuery.toLowerCase()) ||
      log.source.toLowerCase().includes(searchQuery.toLowerCase());
    const matchesLevel = levelFilter === "ALL" || log.level === levelFilter;
    const matchesSource = sourceFilter === "ALL" || log.source === sourceFilter;
    const matchesCategory =
      categoryFilter === "ALL" || log.category === categoryFilter;

    return matchesSearch && matchesLevel && matchesSource && matchesCategory;
  });

  const getLevelColor = (level: string) => {
    switch (level) {
      case "DEBUG":
        return "text-gray-500";
      case "INFO":
        return "text-blue-500";
      case "WARN":
        return "text-yellow-500";
      case "ERROR":
        return "text-red-500";
      case "FATAL":
        return "text-red-700";
      default:
        return "text-muted-foreground";
    }
  };

  const getLevelIcon = (level: string) => {
    switch (level) {
      case "DEBUG":
        return Bug;
      case "INFO":
        return Info;
      case "WARN":
        return AlertTriangle;
      case "ERROR":
        return AlertTriangle;
      case "FATAL":
        return AlertTriangle;
      default:
        return Info;
    }
  };

  const getCategoryColor = (category: string) => {
    switch (category) {
      case "ros":
        return "bg-blue-100 text-blue-800";
      case "system":
        return "bg-green-100 text-green-800";
      case "application":
        return "bg-purple-100 text-purple-800";
      case "hardware":
        return "bg-orange-100 text-orange-800";
      default:
        return "bg-white/5 border border-white/10 text-white";
    }
  };

  const clearLogs = () => {
    setLogs([]);
  };

  const exportLogs = () => {
    const dataStr = JSON.stringify(filteredLogs, null, 2);
    const dataUri =
      "data:application/json;charset=utf-8," + encodeURIComponent(dataStr);
    const exportFileDefaultName = `logs_${new Date().toISOString().split("T")[0]}.json`;

    const linkElement = document.createElement("a");
    linkElement.setAttribute("href", dataUri);
    linkElement.setAttribute("download", exportFileDefaultName);
    linkElement.click();
  };

  const uniqueSources = Array.from(new Set(logs.map((log) => log.source)));
  const uniqueCategories = Array.from(new Set(logs.map((log) => log.category)));

  const logCounts = {
    total: logs.length,
    debug: logs.filter((l) => l.level === "DEBUG").length,
    info: logs.filter((l) => l.level === "INFO").length,
    warn: logs.filter((l) => l.level === "WARN").length,
    error: logs.filter((l) => l.level === "ERROR").length,
    fatal: logs.filter((l) => l.level === "FATAL").length,
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <FileText className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            Log Viewer
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Real-time system logs with filtering and search capabilities
          </p>
        </div>

        <div className="flex gap-2">
          <Button
            variant={isLiveMode ? "default" : "outline"}
            onClick={() => setIsLiveMode(!isLiveMode)}
            className="gap-2"
          >
            {isLiveMode ? (
              <Pause className="h-4 w-4" />
            ) : (
              <Play className="h-4 w-4" />
            )}
            {isLiveMode ? "Pause" : "Resume"} Live
          </Button>
          <Button onClick={exportLogs} variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
          <Button onClick={clearLogs} variant="outline" className="gap-2">
            <Trash2 className="h-4 w-4" />
            Clear
          </Button>
          <Button variant="outline" className="gap-2">
            <Settings className="h-4 w-4" />
            Settings
          </Button>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 sm:grid-cols-6 gap-4">
        <Card className="p-3 hover-lift">
          <div className="text-center">
            <div className="text-xl font-extralight text-primary">
              {logCounts.total}
            </div>
            <div className="text-xs text-muted-foreground">Total</div>
          </div>
        </Card>
        <Card className="p-3 hover-lift">
          <div className="text-center">
            <div className="text-xl font-extralight text-gray-500">
              {logCounts.debug}
            </div>
            <div className="text-xs text-muted-foreground">Debug</div>
          </div>
        </Card>
        <Card className="p-3 hover-lift">
          <div className="text-center">
            <div className="text-xl font-extralight text-blue-500">
              {logCounts.info}
            </div>
            <div className="text-xs text-muted-foreground">Info</div>
          </div>
        </Card>
        <Card className="p-3 hover-lift">
          <div className="text-center">
            <div className="text-xl font-extralight text-yellow-500">
              {logCounts.warn}
            </div>
            <div className="text-xs text-muted-foreground">Warn</div>
          </div>
        </Card>
        <Card className="p-3 hover-lift">
          <div className="text-center">
            <div className="text-xl font-extralight text-red-500">
              {logCounts.error}
            </div>
            <div className="text-xs text-muted-foreground">Error</div>
          </div>
        </Card>
        <Card className="p-3 hover-lift">
          <div className="text-center">
            <div className="text-xl font-extralight text-red-700">
              {logCounts.fatal}
            </div>
            <div className="text-xs text-muted-foreground">Fatal</div>
          </div>
        </Card>
      </div>

      {/* Filters */}
      <Card className="p-4">
        <div className="grid grid-cols-1 lg:grid-cols-6 gap-4">
          <div className="lg:col-span-2">
            <Label className="text-sm">Search</Label>
            <div className="relative mt-1">
              <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
              <Input
                placeholder="Search logs..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="pl-10"
              />
            </div>
          </div>

          <div>
            <Label className="text-sm">Level</Label>
            <Select value={levelFilter} onValueChange={setLevelFilter}>
              <SelectTrigger className="mt-1">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="ALL">All Levels</SelectItem>
                <SelectItem value="DEBUG">Debug</SelectItem>
                <SelectItem value="INFO">Info</SelectItem>
                <SelectItem value="WARN">Warning</SelectItem>
                <SelectItem value="ERROR">Error</SelectItem>
                <SelectItem value="FATAL">Fatal</SelectItem>
              </SelectContent>
            </Select>
          </div>

          <div>
            <Label className="text-sm">Source</Label>
            <Select value={sourceFilter} onValueChange={setSourceFilter}>
              <SelectTrigger className="mt-1">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="ALL">All Sources</SelectItem>
                {uniqueSources.map((source) => (
                  <SelectItem key={source} value={source}>
                    {source}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          <div>
            <Label className="text-sm">Category</Label>
            <Select value={categoryFilter} onValueChange={setCategoryFilter}>
              <SelectTrigger className="mt-1">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="ALL">All Categories</SelectItem>
                {uniqueCategories.map((category) => (
                  <SelectItem key={category} value={category}>
                    {category}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          <div className="flex items-end gap-2">
            <div className="flex items-center space-x-2">
              <Switch checked={autoScroll} onCheckedChange={setAutoScroll} />
              <Label className="text-sm">Auto Scroll</Label>
            </div>
          </div>
        </div>
      </Card>

      {/* Log Display */}
      <Card className="p-4">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-light flex items-center gap-2">
            <Eye className="h-5 w-5 text-primary" />
            Live Logs ({filteredLogs.length})
          </h3>
          <div className="flex items-center gap-2">
            {isLiveMode && (
              <div className="flex items-center gap-2">
                <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse" />
                <span className="text-sm text-green-500">Live</span>
              </div>
            )}
            <Badge variant="outline" className="text-xs">
              {filteredLogs.length} / {logs.length}
            </Badge>
          </div>
        </div>

        <ScrollArea className="h-[600px]" ref={scrollAreaRef}>
          <div className="space-y-1">
            {filteredLogs.map((log, index) => {
              const LevelIcon = getLevelIcon(log.level);

              return (
                <div
                  key={log.id}
                  className={`p-3 border-l-4 bg-gradient-to-r from-background to-muted/20 hover:bg-muted/40 transition-colors stagger-item`}
                  style={{
                    borderLeftColor:
                      log.level === "ERROR" || log.level === "FATAL"
                        ? "rgb(239 68 68)"
                        : log.level === "WARN"
                          ? "rgb(245 158 11)"
                          : log.level === "INFO"
                            ? "rgb(59 130 246)"
                            : "rgb(107 114 128)",
                    animationDelay: `${index * 0.02}s`,
                  }}
                >
                  <div className="flex items-start gap-3">
                    <LevelIcon
                      className={`h-4 w-4 mt-0.5 ${getLevelColor(log.level)}`}
                    />
                    <div className="flex-1 min-w-0">
                      <div className="flex items-center gap-2 mb-1">
                        <span className="font-mono text-xs text-muted-foreground">
                          {log.timestamp.toLocaleTimeString()}
                        </span>
                        <Badge
                          variant="outline"
                          className={`text-xs ${getLevelColor(log.level)}`}
                        >
                          {log.level}
                        </Badge>
                        <Badge
                          className={`text-xs ${getCategoryColor(log.category)}`}
                        >
                          {log.category}
                        </Badge>
                        <Badge variant="secondary" className="text-xs">
                          {log.source}
                        </Badge>
                      </div>
                      <div className="text-sm">{log.message}</div>
                      {log.details && (
                        <details className="mt-2">
                          <summary className="text-xs text-muted-foreground cursor-pointer hover:text-foreground">
                            Show details
                          </summary>
                          <pre className="text-xs bg-muted p-2 rounded mt-1 overflow-x-auto">
                            {JSON.stringify(log.details, null, 2)}
                          </pre>
                        </details>
                      )}
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </ScrollArea>
      </Card>
    </div>
  );
};

export default LogViewer;
