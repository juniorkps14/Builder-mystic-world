import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Network,
  Play,
  Pause,
  Square,
  RefreshCw,
  Search,
  Settings,
  AlertTriangle,
  CheckCircle,
  Clock,
  Activity,
  Zap,
  Link,
  Terminal,
  Download,
  Filter,
  Eye,
  EyeOff,
} from "lucide-react";

interface ROSNode {
  name: string;
  namespace: string;
  pid: number;
  status: "running" | "stopped" | "error" | "starting" | "stopping";
  uptime: number;
  cpuUsage: number;
  memoryUsage: number;
  publishedTopics: string[];
  subscribedTopics: string[];
  providedServices: string[];
  parameters: Record<string, any>;
  lastSeen: Date;
}

const Nodes = () => {
  const { t } = useLanguage();

  const [nodes, setNodes] = useState<ROSNode[]>([
    {
      name: "roscore",
      namespace: "/",
      pid: 1234,
      status: "running",
      uptime: 7320,
      cpuUsage: 2.1,
      memoryUsage: 45.2,
      publishedTopics: ["/rosout", "/rosout_agg"],
      subscribedTopics: [],
      providedServices: ["/rosout/get_loggers", "/rosout/set_logger_level"],
      parameters: { rosdistro: "noetic", roslaunch_uuids: [] },
      lastSeen: new Date(),
    },
    {
      name: "move_base",
      namespace: "/",
      pid: 1235,
      status: "running",
      uptime: 6890,
      cpuUsage: 12.5,
      memoryUsage: 128.7,
      publishedTopics: ["/move_base/goal", "/move_base/feedback", "/cmd_vel"],
      subscribedTopics: ["/map", "/amcl_pose", "/scan"],
      providedServices: ["/move_base/make_plan", "/move_base/clear_costmaps"],
      parameters: {
        base_global_planner: "navfn/NavfnROS",
        controller_frequency: 20.0,
      },
      lastSeen: new Date(),
    },
    {
      name: "amcl",
      namespace: "/",
      pid: 1236,
      status: "running",
      uptime: 6750,
      cpuUsage: 8.3,
      memoryUsage: 67.4,
      publishedTopics: ["/amcl_pose", "/particlecloud"],
      subscribedTopics: ["/scan", "/tf", "/initialpose"],
      providedServices: ["/global_localization", "/request_nomotion_update"],
      parameters: { min_particles: 500, max_particles: 5000, kld_err: 0.05 },
      lastSeen: new Date(),
    },
    {
      name: "laser_scan_matcher",
      namespace: "/",
      pid: 1237,
      status: "running",
      uptime: 6234,
      cpuUsage: 15.2,
      memoryUsage: 89.1,
      publishedTopics: ["/scan_matched_points2"],
      subscribedTopics: ["/scan"],
      providedServices: [],
      parameters: { fixed_frame: "odom", max_iterations: 10 },
      lastSeen: new Date(),
    },
    {
      name: "camera_node",
      namespace: "/camera",
      pid: 1238,
      status: "running",
      uptime: 5980,
      cpuUsage: 18.7,
      memoryUsage: 156.3,
      publishedTopics: ["/camera/image_raw", "/camera/camera_info"],
      subscribedTopics: [],
      providedServices: ["/camera/set_camera_info"],
      parameters: { frame_rate: 30, image_width: 1920, image_height: 1080 },
      lastSeen: new Date(),
    },
    {
      name: "tf2_ros",
      namespace: "/",
      pid: 1239,
      status: "running",
      uptime: 7200,
      cpuUsage: 3.4,
      memoryUsage: 34.8,
      publishedTopics: ["/tf", "/tf_static"],
      subscribedTopics: ["/tf", "/tf_static"],
      providedServices: [],
      parameters: { cache_time: 10.0 },
      lastSeen: new Date(),
    },
    {
      name: "robot_state_publisher",
      namespace: "/",
      pid: 1240,
      status: "stopped",
      uptime: 0,
      cpuUsage: 0,
      memoryUsage: 0,
      publishedTopics: ["/tf"],
      subscribedTopics: ["/joint_states"],
      providedServices: [],
      parameters: { publish_frequency: 50.0 },
      lastSeen: new Date(Date.now() - 300000),
    },
    {
      name: "joint_state_publisher",
      namespace: "/",
      pid: 1241,
      status: "error",
      uptime: 0,
      cpuUsage: 0,
      memoryUsage: 0,
      publishedTopics: ["/joint_states"],
      subscribedTopics: [],
      providedServices: [],
      parameters: { rate: 50 },
      lastSeen: new Date(Date.now() - 600000),
    },
  ]);

  const [selectedNode, setSelectedNode] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState("");
  const [statusFilter, setStatusFilter] = useState<string>("all");
  const [isAutoRefresh, setIsAutoRefresh] = useState(true);
  const [showDetails, setShowDetails] = useState(false);

  // Simulate real-time updates
  useEffect(() => {
    if (!isAutoRefresh) return;

    const interval = setInterval(() => {
      setNodes((prev) =>
        prev.map((node) => ({
          ...node,
          uptime: node.status === "running" ? node.uptime + 5 : node.uptime,
          cpuUsage:
            node.status === "running"
              ? Math.max(
                  0,
                  Math.min(100, node.cpuUsage + (Math.random() - 0.5) * 3),
                )
              : 0,
          memoryUsage:
            node.status === "running"
              ? Math.max(0, node.memoryUsage + (Math.random() - 0.5) * 5)
              : 0,
          lastSeen: node.status === "running" ? new Date() : node.lastSeen,
        })),
      );
    }, 5000);

    return () => clearInterval(interval);
  }, [isAutoRefresh]);

  const filteredNodes = nodes.filter((node) => {
    const matchesSearch =
      node.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
      node.namespace.toLowerCase().includes(searchQuery.toLowerCase());
    const matchesStatus =
      statusFilter === "all" || node.status === statusFilter;
    return matchesSearch && matchesStatus;
  });

  const selectedNodeData = nodes.find((node) => node.name === selectedNode);

  const handleNodeAction = (
    nodeName: string,
    action: "start" | "stop" | "restart",
  ) => {
    setNodes((prev) =>
      prev.map((node) => {
        if (node.name === nodeName) {
          switch (action) {
            case "start":
              return { ...node, status: "starting" as const };
            case "stop":
              return { ...node, status: "stopping" as const };
            case "restart":
              return { ...node, status: "stopping" as const };
          }
        }
        return node;
      }),
    );

    // Simulate state transition
    setTimeout(() => {
      setNodes((prev) =>
        prev.map((node) => {
          if (node.name === nodeName) {
            switch (action) {
              case "start":
                return {
                  ...node,
                  status: "running" as const,
                  pid: Math.floor(Math.random() * 9000) + 1000,
                  uptime: 0,
                };
              case "stop":
              case "restart":
                return {
                  ...node,
                  status:
                    action === "restart"
                      ? ("running" as const)
                      : ("stopped" as const),
                  pid:
                    action === "restart"
                      ? Math.floor(Math.random() * 9000) + 1000
                      : 0,
                  uptime: action === "restart" ? 0 : 0,
                  cpuUsage: action === "restart" ? Math.random() * 20 : 0,
                  memoryUsage: action === "restart" ? Math.random() * 100 : 0,
                };
            }
          }
          return node;
        }),
      );
    }, 2000);
  };

  const formatUptime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return `${hours}h ${minutes}m`;
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "running":
        return "text-green-500";
      case "stopped":
        return "text-muted-foreground";
      case "error":
        return "text-red-500";
      case "starting":
      case "stopping":
        return "text-yellow-500";
      default:
        return "text-muted-foreground";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "running":
        return CheckCircle;
      case "stopped":
        return Square;
      case "error":
        return AlertTriangle;
      case "starting":
      case "stopping":
        return Clock;
      default:
        return Clock;
    }
  };

  const statusCounts = {
    all: nodes.length,
    running: nodes.filter((n) => n.status === "running").length,
    stopped: nodes.filter((n) => n.status === "stopped").length,
    error: nodes.filter((n) => n.status === "error").length,
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Network className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            {t("nav.nodes")}
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            ROS Node Lifecycle Management and Monitoring
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
          <Button
            variant={showDetails ? "default" : "outline"}
            onClick={() => setShowDetails(!showDetails)}
            className="gap-2"
          >
            {showDetails ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
            Details
          </Button>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
        </div>
      </div>

      {/* Status Overview */}
      <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-primary">
              {statusCounts.all}
            </div>
            <div className="text-sm text-muted-foreground">Total Nodes</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-green-500">
              {statusCounts.running}
            </div>
            <div className="text-sm text-muted-foreground">Running</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-muted-foreground">
              {statusCounts.stopped}
            </div>
            <div className="text-sm text-muted-foreground">Stopped</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-red-500">
              {statusCounts.error}
            </div>
            <div className="text-sm text-muted-foreground">Error</div>
          </div>
        </Card>
      </div>

      {/* Controls */}
      <Card className="p-4">
        <div className="flex flex-col sm:flex-row gap-4">
          <div className="flex-1">
            <div className="relative">
              <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
              <Input
                placeholder="Search nodes..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="pl-10"
              />
            </div>
          </div>
          <div className="flex gap-2">
            {["all", "running", "stopped", "error"].map((status) => (
              <Button
                key={status}
                variant={statusFilter === status ? "default" : "outline"}
                size="sm"
                onClick={() => setStatusFilter(status)}
                className="gap-1"
              >
                <Filter className="h-3 w-3" />
                {status.charAt(0).toUpperCase() + status.slice(1)}
                <Badge variant="secondary" className="ml-1 text-xs">
                  {statusCounts[status as keyof typeof statusCounts]}
                </Badge>
              </Button>
            ))}
          </div>
        </div>
      </Card>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Nodes List */}
        <Card className="lg:col-span-2 p-4">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Activity className="h-5 w-5 text-primary" />
            Active Nodes ({filteredNodes.length})
          </h3>

          <ScrollArea className="h-[600px]">
            <div className="space-y-2">
              {filteredNodes.map((node, index) => {
                const StatusIcon = getStatusIcon(node.status);
                const isSelected = selectedNode === node.name;

                return (
                  <div
                    key={node.name}
                    className={`p-4 rounded-lg border transition-all cursor-pointer hover-lift stagger-item ${
                      isSelected
                        ? "bg-primary/5 border-primary/30"
                        : "hover:bg-muted/50"
                    }`}
                    style={{ animationDelay: `${index * 0.05}s` }}
                    onClick={() =>
                      setSelectedNode(isSelected ? null : node.name)
                    }
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-3 flex-1">
                        <StatusIcon
                          className={`h-5 w-5 ${getStatusColor(node.status)}`}
                        />
                        <div className="flex-1 min-w-0">
                          <div className="font-mono text-sm font-medium truncate">
                            {node.namespace !== "/" && (
                              <span className="text-muted-foreground">
                                {node.namespace}
                              </span>
                            )}
                            {node.name}
                          </div>
                          {showDetails && (
                            <div className="text-xs text-muted-foreground mt-1">
                              PID: {node.pid || "N/A"} • Uptime:{" "}
                              {formatUptime(node.uptime)}
                            </div>
                          )}
                        </div>
                      </div>

                      <div className="flex items-center gap-2">
                        {showDetails && (
                          <div className="text-right text-xs space-y-1">
                            <div className="text-muted-foreground">
                              CPU:{" "}
                              <span className="font-mono">
                                {node.cpuUsage.toFixed(1)}%
                              </span>
                            </div>
                            <div className="text-muted-foreground">
                              Mem:{" "}
                              <span className="font-mono">
                                {node.memoryUsage.toFixed(1)}MB
                              </span>
                            </div>
                          </div>
                        )}

                        <Badge
                          variant={
                            node.status === "running"
                              ? "default"
                              : node.status === "error"
                                ? "destructive"
                                : "secondary"
                          }
                          className="text-xs"
                        >
                          {node.status.toUpperCase()}
                        </Badge>

                        <div className="flex gap-1">
                          {node.status === "stopped" ||
                          node.status === "error" ? (
                            <Button
                              size="sm"
                              variant="outline"
                              onClick={(e) => {
                                e.stopPropagation();
                                handleNodeAction(node.name, "start");
                              }}
                              disabled={node.status === "starting"}
                            >
                              <Play className="h-3 w-3" />
                            </Button>
                          ) : (
                            <Button
                              size="sm"
                              variant="outline"
                              onClick={(e) => {
                                e.stopPropagation();
                                handleNodeAction(node.name, "stop");
                              }}
                              disabled={node.status === "stopping"}
                            >
                              <Square className="h-3 w-3" />
                            </Button>
                          )}
                          <Button
                            size="sm"
                            variant="outline"
                            onClick={(e) => {
                              e.stopPropagation();
                              handleNodeAction(node.name, "restart");
                            }}
                            disabled={
                              node.status === "starting" ||
                              node.status === "stopping"
                            }
                          >
                            <RefreshCw className="h-3 w-3" />
                          </Button>
                        </div>
                      </div>
                    </div>

                    {isSelected && (
                      <div className="mt-4 pt-4 border-t space-y-3 fade-in-up">
                        <div className="grid grid-cols-1 sm:grid-cols-3 gap-4 text-xs">
                          <div>
                            <Label className="text-muted-foreground">
                              Published Topics
                            </Label>
                            <div className="mt-1 space-y-1">
                              {node.publishedTopics.length > 0 ? (
                                node.publishedTopics.map((topic) => (
                                  <div
                                    key={topic}
                                    className="font-mono text-xs bg-muted p-1 rounded"
                                  >
                                    {topic}
                                  </div>
                                ))
                              ) : (
                                <div className="text-muted-foreground">
                                  None
                                </div>
                              )}
                            </div>
                          </div>
                          <div>
                            <Label className="text-muted-foreground">
                              Subscribed Topics
                            </Label>
                            <div className="mt-1 space-y-1">
                              {node.subscribedTopics.length > 0 ? (
                                node.subscribedTopics.map((topic) => (
                                  <div
                                    key={topic}
                                    className="font-mono text-xs bg-muted p-1 rounded"
                                  >
                                    {topic}
                                  </div>
                                ))
                              ) : (
                                <div className="text-muted-foreground">
                                  None
                                </div>
                              )}
                            </div>
                          </div>
                          <div>
                            <Label className="text-muted-foreground">
                              Services
                            </Label>
                            <div className="mt-1 space-y-1">
                              {node.providedServices.length > 0 ? (
                                node.providedServices.map((service) => (
                                  <div
                                    key={service}
                                    className="font-mono text-xs bg-muted p-1 rounded"
                                  >
                                    {service}
                                  </div>
                                ))
                              ) : (
                                <div className="text-muted-foreground">
                                  None
                                </div>
                              )}
                            </div>
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

        {/* Node Details */}
        <div className="space-y-6">
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Settings className="h-5 w-5 text-primary" />
              Node Details
            </h3>

            {selectedNodeData ? (
              <div className="space-y-4">
                <div>
                  <Label className="text-sm font-medium">Node Name</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedNodeData.namespace}
                    {selectedNodeData.name}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Process ID</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedNodeData.pid || "N/A"}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Status</Label>
                  <div className="mt-1">
                    <Badge
                      variant={
                        selectedNodeData.status === "running"
                          ? "default"
                          : "secondary"
                      }
                    >
                      {selectedNodeData.status.toUpperCase()}
                    </Badge>
                  </div>
                </div>

                <Separator />

                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <Label className="text-sm font-medium">CPU Usage</Label>
                    <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                      {selectedNodeData.cpuUsage.toFixed(1)}%
                    </div>
                  </div>
                  <div>
                    <Label className="text-sm font-medium">Memory</Label>
                    <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                      {selectedNodeData.memoryUsage.toFixed(1)}MB
                    </div>
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Uptime</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {formatUptime(selectedNodeData.uptime)}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Last Seen</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedNodeData.lastSeen.toLocaleTimeString()}
                  </div>
                </div>

                <Separator />

                <div className="space-y-2">
                  <Label className="text-sm font-medium">Actions</Label>
                  <div className="flex gap-2">
                    <Button
                      size="sm"
                      variant="outline"
                      className="flex-1 gap-1"
                      onClick={() =>
                        handleNodeAction(selectedNodeData.name, "restart")
                      }
                      disabled={
                        selectedNodeData.status === "starting" ||
                        selectedNodeData.status === "stopping"
                      }
                    >
                      <RefreshCw className="h-3 w-3" />
                      Restart
                    </Button>
                    <Button size="sm" variant="outline" className="gap-1">
                      <Terminal className="h-3 w-3" />
                      Logs
                    </Button>
                  </div>
                </div>
              </div>
            ) : (
              <div className="text-center text-muted-foreground py-8">
                Select a node to view details
              </div>
            )}
          </Card>

          {/* Quick Actions */}
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Zap className="h-5 w-5 text-primary" />
              Quick Actions
            </h3>

            <div className="space-y-2">
              <Button variant="outline" className="w-full justify-start gap-2">
                <Play className="h-4 w-4" />
                Start All Stopped Nodes
              </Button>
              <Button variant="outline" className="w-full justify-start gap-2">
                <RefreshCw className="h-4 w-4" />
                Restart All Nodes
              </Button>
              <Button variant="outline" className="w-full justify-start gap-2">
                <Terminal className="h-4 w-4" />
                View System Logs
              </Button>
              <Button variant="outline" className="w-full justify-start gap-2">
                <Settings className="h-4 w-4" />
                Node Configuration
              </Button>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
};

export default Nodes;
