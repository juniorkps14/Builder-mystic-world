import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Zap,
  Play,
  RefreshCw,
  Search,
  Settings,
  Activity,
  Clock,
  CheckCircle,
  AlertTriangle,
  Download,
  Filter,
  MessageSquare,
  Terminal,
  Code,
  Send,
} from "lucide-react";

interface ROSService {
  name: string;
  type: string;
  node: string;
  isActive: boolean;
  callCount: number;
  lastCalled: Date | null;
  averageResponseTime: number;
  successRate: number;
  description: string;
}

interface ServiceCall {
  id: string;
  serviceName: string;
  request: any;
  response: any;
  timestamp: Date;
  responseTime: number;
  success: boolean;
  error?: string;
}

const Services = () => {
  const { t } = useLanguage();

  const [services, setServices] = useState<ROSService[]>([
    {
      name: "/move_base/make_plan",
      type: "nav_msgs/GetPlan",
      node: "move_base",
      isActive: true,
      callCount: 45,
      lastCalled: new Date(Date.now() - 120000),
      averageResponseTime: 85.3,
      successRate: 96.7,
      description: "Create a navigation plan between two poses",
    },
    {
      name: "/move_base/clear_costmaps",
      type: "std_srvs/Empty",
      node: "move_base",
      isActive: true,
      callCount: 12,
      lastCalled: new Date(Date.now() - 300000),
      averageResponseTime: 12.5,
      successRate: 100.0,
      description: "Clear all costmaps for navigation",
    },
    {
      name: "/global_localization",
      type: "std_srvs/Empty",
      node: "amcl",
      isActive: true,
      callCount: 8,
      lastCalled: new Date(Date.now() - 600000),
      averageResponseTime: 234.7,
      successRate: 87.5,
      description: "Initialize global localization",
    },
    {
      name: "/request_nomotion_update",
      type: "std_srvs/Empty",
      node: "amcl",
      isActive: true,
      callCount: 156,
      lastCalled: new Date(Date.now() - 60000),
      averageResponseTime: 5.2,
      successRate: 99.4,
      description: "Request particle filter update without motion",
    },
    {
      name: "/camera/set_camera_info",
      type: "sensor_msgs/SetCameraInfo",
      node: "camera_node",
      isActive: true,
      callCount: 2,
      lastCalled: new Date(Date.now() - 1800000),
      averageResponseTime: 45.8,
      successRate: 100.0,
      description: "Set camera calibration parameters",
    },
    {
      name: "/rosout/get_loggers",
      type: "roscpp/GetLoggers",
      node: "roscore",
      isActive: true,
      callCount: 23,
      lastCalled: new Date(Date.now() - 180000),
      averageResponseTime: 8.9,
      successRate: 100.0,
      description: "Get logger levels for debugging",
    },
    {
      name: "/robot_state_publisher/set_parameters",
      type: "dynamic_reconfigure/Reconfigure",
      node: "robot_state_publisher",
      isActive: false,
      callCount: 0,
      lastCalled: null,
      averageResponseTime: 0,
      successRate: 0,
      description: "Configure robot state publisher parameters",
    },
    {
      name: "/spawn_model",
      type: "gazebo_msgs/SpawnModel",
      node: "gazebo",
      isActive: true,
      callCount: 5,
      lastCalled: new Date(Date.now() - 7200000),
      averageResponseTime: 156.2,
      successRate: 80.0,
      description: "Spawn a model in Gazebo simulation",
    },
  ]);

  const [selectedService, setSelectedService] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState("");
  const [typeFilter, setTypeFilter] = useState("all");
  const [callHistory, setCallHistory] = useState<ServiceCall[]>([]);
  const [requestData, setRequestData] = useState("");
  const [isAutoRefresh, setIsAutoRefresh] = useState(true);
  const [isCalling, setIsCalling] = useState(false);

  // Simulate real-time updates
  useEffect(() => {
    if (!isAutoRefresh) return;

    const interval = setInterval(() => {
      // Randomly update some services
      setServices((prev) =>
        prev.map((service) => {
          if (Math.random() < 0.1 && service.isActive) {
            // 10% chance to update
            return {
              ...service,
              callCount: service.callCount + 1,
              lastCalled: new Date(),
              averageResponseTime: Math.max(
                1,
                service.averageResponseTime + (Math.random() - 0.5) * 10,
              ),
            };
          }
          return service;
        }),
      );
    }, 5000);

    return () => clearInterval(interval);
  }, [isAutoRefresh]);

  const filteredServices = services.filter((service) => {
    const matchesSearch =
      service.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
      service.type.toLowerCase().includes(searchQuery.toLowerCase()) ||
      service.node.toLowerCase().includes(searchQuery.toLowerCase());
    const matchesType =
      typeFilter === "all" || service.type.includes(typeFilter);
    return matchesSearch && matchesType;
  });

  const selectedServiceData = services.find(
    (service) => service.name === selectedService,
  );

  const serviceTypes = Array.from(
    new Set(services.map((s) => s.type.split("/")[0])),
  );

  const handleServiceCall = async () => {
    if (!selectedServiceData) return;

    setIsCalling(true);

    try {
      let request = {};
      if (requestData.trim()) {
        request = JSON.parse(requestData);
      }

      // Simulate service call
      const responseTime = Math.random() * 200 + 10; // 10-210ms
      const success = Math.random() > 0.1; // 90% success rate

      await new Promise((resolve) => setTimeout(resolve, responseTime));

      const callResult: ServiceCall = {
        id: `call_${Date.now()}`,
        serviceName: selectedServiceData.name,
        request,
        response: success
          ? generateMockResponse(selectedServiceData.type)
          : null,
        timestamp: new Date(),
        responseTime,
        success,
        error: success ? undefined : "Service call failed: connection timeout",
      };

      setCallHistory((prev) => [callResult, ...prev.slice(0, 49)]);

      // Update service statistics
      setServices((prev) =>
        prev.map((service) => {
          if (service.name === selectedServiceData.name) {
            const newCallCount = service.callCount + 1;
            const newSuccessRate =
              (service.successRate * service.callCount + (success ? 100 : 0)) /
              newCallCount;
            const newAvgResponseTime =
              (service.averageResponseTime * service.callCount + responseTime) /
              newCallCount;

            return {
              ...service,
              callCount: newCallCount,
              lastCalled: new Date(),
              successRate: newSuccessRate,
              averageResponseTime: newAvgResponseTime,
            };
          }
          return service;
        }),
      );
    } catch (error) {
      const callResult: ServiceCall = {
        id: `call_${Date.now()}`,
        serviceName: selectedServiceData.name,
        request: requestData,
        response: null,
        timestamp: new Date(),
        responseTime: 0,
        success: false,
        error: "Invalid JSON in request",
      };

      setCallHistory((prev) => [callResult, ...prev.slice(0, 49)]);
    }

    setIsCalling(false);
  };

  const generateMockResponse = (type: string) => {
    switch (type) {
      case "nav_msgs/GetPlan":
        return {
          plan: {
            header: {
              frame_id: "map",
              stamp: { sec: Math.floor(Date.now() / 1000) },
            },
            poses: Array.from({ length: 20 }, (_, i) => ({
              pose: {
                position: { x: i * 0.5, y: Math.sin(i * 0.2) * 2, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 },
              },
            })),
          },
        };
      case "std_srvs/Empty":
        return {};
      case "sensor_msgs/SetCameraInfo":
        return {
          success: true,
          status_message: "Camera info set successfully",
        };
      case "roscpp/GetLoggers":
        return {
          loggers: [
            { name: "ros", level: "info" },
            { name: "ros.amcl", level: "warn" },
            { name: "ros.move_base", level: "debug" },
          ],
        };
      default:
        return { success: true, message: "Service call completed" };
    }
  };

  const formatResponseTime = (ms: number) => {
    if (ms < 1000) return `${ms.toFixed(1)}ms`;
    return `${(ms / 1000).toFixed(2)}s`;
  };

  const getSuccessRateColor = (rate: number) => {
    if (rate >= 95) return "text-green-500";
    if (rate >= 80) return "text-yellow-500";
    return "text-red-500";
  };

  const activeServices = services.filter((s) => s.isActive).length;
  const totalCalls = services.reduce((sum, s) => sum + s.callCount, 0);
  const avgSuccessRate =
    services.reduce((sum, s) => sum + s.successRate, 0) / services.length;

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Zap className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            {t("nav.services")}
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            ROS Service Management, Testing, and Monitoring
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

      {/* Statistics Overview */}
      <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-primary">
              {services.length}
            </div>
            <div className="text-sm text-muted-foreground">Total Services</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-green-500">
              {activeServices}
            </div>
            <div className="text-sm text-muted-foreground">Active</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-blue-500">
              {totalCalls}
            </div>
            <div className="text-sm text-muted-foreground">Total Calls</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div
              className={`text-2xl font-extralight ${getSuccessRateColor(avgSuccessRate)}`}
            >
              {avgSuccessRate.toFixed(1)}%
            </div>
            <div className="text-sm text-muted-foreground">Success Rate</div>
          </div>
        </Card>
      </div>

      {/* Controls */}
      <Card className="p-4">
        <div className="flex flex-col lg:flex-row gap-4">
          <div className="flex-1">
            <div className="relative">
              <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
              <Input
                placeholder="Search services..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="pl-10"
              />
            </div>
          </div>
          <div className="flex gap-2">
            {["all", ...serviceTypes].map((type) => (
              <Button
                key={type}
                variant={typeFilter === type ? "default" : "outline"}
                size="sm"
                onClick={() => setTypeFilter(type)}
                className="gap-1"
              >
                <Filter className="h-3 w-3" />
                {type === "all" ? "All" : type}
              </Button>
            ))}
          </div>
        </div>
      </Card>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Services List */}
        <Card className="lg:col-span-2 p-4">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Activity className="h-5 w-5 text-primary" />
            Available Services ({filteredServices.length})
          </h3>

          <ScrollArea className="h-[600px]">
            <div className="space-y-2">
              {filteredServices.map((service, index) => {
                const isSelected = selectedService === service.name;

                return (
                  <div
                    key={service.name}
                    className={`p-4 rounded-lg border transition-all cursor-pointer hover-lift stagger-item ${
                      isSelected
                        ? "bg-primary/5 border-primary/30"
                        : "hover:bg-muted/50"
                    }`}
                    style={{ animationDelay: `${index * 0.05}s` }}
                    onClick={() =>
                      setSelectedService(isSelected ? null : service.name)
                    }
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-3 flex-1">
                        <div
                          className={`h-3 w-3 rounded-full ${service.isActive ? "bg-green-500 glow-on-hover" : "bg-muted-foreground"}`}
                        />
                        <div className="flex-1 min-w-0">
                          <div className="font-mono text-sm font-medium truncate">
                            {service.name}
                          </div>
                          <div className="text-xs text-muted-foreground truncate">
                            {service.type} • {service.node}
                          </div>
                          <div className="text-xs text-muted-foreground mt-1">
                            {service.description}
                          </div>
                        </div>
                      </div>

                      <div className="flex items-center gap-2">
                        <div className="text-right text-xs space-y-1">
                          <div className="flex items-center gap-1">
                            <MessageSquare className="h-3 w-3 text-muted-foreground" />
                            <span className="font-mono">
                              {service.callCount} calls
                            </span>
                          </div>
                          <div className="flex items-center gap-1">
                            <Clock className="h-3 w-3 text-muted-foreground" />
                            <span className="font-mono">
                              {formatResponseTime(service.averageResponseTime)}
                            </span>
                          </div>
                          <div className="flex items-center gap-1">
                            <CheckCircle className="h-3 w-3 text-muted-foreground" />
                            <span
                              className={`font-mono ${getSuccessRateColor(service.successRate)}`}
                            >
                              {service.successRate.toFixed(1)}%
                            </span>
                          </div>
                        </div>

                        <Badge
                          variant={service.isActive ? "default" : "secondary"}
                          className="text-xs"
                        >
                          {service.isActive ? "ACTIVE" : "INACTIVE"}
                        </Badge>
                      </div>
                    </div>

                    {isSelected && service.isActive && (
                      <div className="mt-4 pt-4 border-t fade-in-up">
                        <div className="flex gap-2">
                          <Button
                            size="sm"
                            className="gap-1"
                            onClick={(e) => {
                              e.stopPropagation();
                              handleServiceCall();
                            }}
                            disabled={isCalling}
                          >
                            {isCalling ? (
                              <RefreshCw className="h-3 w-3 animate-spin" />
                            ) : (
                              <Send className="h-3 w-3" />
                            )}
                            Call Service
                          </Button>
                          <Button size="sm" variant="outline" className="gap-1">
                            <Code className="h-3 w-3" />
                            View Definition
                          </Button>
                          <Button size="sm" variant="outline" className="gap-1">
                            <Terminal className="h-3 w-3" />
                            CLI Info
                          </Button>
                        </div>
                      </div>
                    )}
                  </div>
                );
              })}
            </div>
          </ScrollArea>
        </Card>

        {/* Service Caller */}
        <div className="space-y-6">
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Send className="h-5 w-5 text-primary" />
              Service Caller
            </h3>

            {selectedServiceData ? (
              <div className="space-y-4">
                <div>
                  <Label className="text-sm font-medium">Service</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedServiceData.name}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Type</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedServiceData.type}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Node</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedServiceData.node}
                  </div>
                </div>

                <Separator />

                <div>
                  <Label className="text-sm font-medium">Request (JSON)</Label>
                  <Textarea
                    placeholder='{"param": "value"}'
                    value={requestData}
                    onChange={(e) => setRequestData(e.target.value)}
                    className="mt-1 font-mono text-sm"
                    rows={4}
                  />
                </div>

                <Button
                  onClick={handleServiceCall}
                  disabled={!selectedServiceData.isActive || isCalling}
                  className="w-full gap-2"
                >
                  {isCalling ? (
                    <RefreshCw className="h-4 w-4 animate-spin" />
                  ) : (
                    <Send className="h-4 w-4" />
                  )}
                  {isCalling ? "Calling..." : "Call Service"}
                </Button>

                <div className="text-xs text-muted-foreground">
                  Last called:{" "}
                  {selectedServiceData.lastCalled?.toLocaleTimeString() ||
                    "Never"}
                </div>
              </div>
            ) : (
              <div className="text-center text-muted-foreground py-8">
                Select a service to call
              </div>
            )}
          </Card>

          {/* Call History */}
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Clock className="h-5 w-5 text-primary" />
              Call History
            </h3>

            <ScrollArea className="h-64">
              <div className="space-y-2">
                {callHistory.length > 0 ? (
                  callHistory.map((call, index) => (
                    <div
                      key={call.id}
                      className="text-xs bg-muted p-2 rounded space-y-1"
                    >
                      <div className="flex justify-between items-center">
                        <span className="font-mono text-muted-foreground">
                          {call.timestamp.toLocaleTimeString()}
                        </span>
                        <div className="flex items-center gap-1">
                          {call.success ? (
                            <CheckCircle className="h-3 w-3 text-green-500" />
                          ) : (
                            <AlertTriangle className="h-3 w-3 text-red-500" />
                          )}
                          <span className="font-mono">
                            {formatResponseTime(call.responseTime)}
                          </span>
                        </div>
                      </div>
                      <div className="font-mono">{call.serviceName}</div>
                      {call.error && (
                        <div className="text-red-500">{call.error}</div>
                      )}
                    </div>
                  ))
                ) : (
                  <div className="text-center text-muted-foreground py-4">
                    No calls made yet
                  </div>
                )}
              </div>
            </ScrollArea>
          </Card>
        </div>
      </div>
    </div>
  );
};

export default Services;
