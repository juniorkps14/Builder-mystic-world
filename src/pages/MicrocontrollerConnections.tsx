import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";
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
  Cpu,
  Wifi,
  Cable,
  Router,
  Activity,
  CheckCircle,
  AlertTriangle,
  Clock,
  Zap,
  Settings,
  RefreshCw,
  Save,
  Download,
  Upload,
  Link,
  Monitor,
  Smartphone,
  HardDrive,
  Network,
  Radio,
} from "lucide-react";

interface MicrocontrollerConnection {
  id: string;
  name: string;
  type: "arduino" | "esp32" | "raspberry_pi" | "teensy" | "stm32";
  protocol: "usb" | "wifi" | "ethernet" | "serial" | "bluetooth";
  address: string;
  port: number;
  baudRate?: number;
  status: "connected" | "disconnected" | "error" | "connecting";
  lastSeen: Date;
  dataRate: number;
  functions: string[];
  powerConsumption: number;
  firmware: string;
  serialConfig: {
    baudRate: number;
    dataBits: 8 | 7 | 6 | 5;
    stopBits: 1 | 2;
    parity: "none" | "even" | "odd";
    flowControl: "none" | "hardware" | "software";
    timeout: number;
    reconnectInterval: number;
  };
  commands: {
    command: string;
    description: string;
    parameters: string[];
    frequency: number;
    enabled: boolean;
  }[];
}

const MicrocontrollerConnections = () => {
  const { t } = useLanguage();

  const [connections, setConnections] = useState<MicrocontrollerConnection[]>([
    {
      id: "mcu_001",
      name: "Main Drive Controller",
      type: "arduino",
      protocol: "usb",
      address: "/dev/ttyUSB0",
      port: 0,
      baudRate: 115200,
      status: "connected",
      lastSeen: new Date(),
      dataRate: 125.5,
      functions: ["Motor Control", "Encoder Reading", "Emergency Stop"],
      powerConsumption: 2.3,
      firmware: "v2.1.4",
      serialConfig: {
        baudRate: 115200,
        dataBits: 8,
        stopBits: 1,
        parity: "none",
        flowControl: "none",
        timeout: 1000,
        reconnectInterval: 5000,
      },
      commands: [
        {
          command: "MOVE",
          description: "Move robot with velocity",
          parameters: ["linear_x", "linear_y", "angular_z"],
          frequency: 20,
          enabled: true,
        },
        {
          command: "GET_ODOM",
          description: "Get odometry data",
          parameters: [],
          frequency: 50,
          enabled: true,
        },
        {
          command: "EMERGENCY_STOP",
          description: "Emergency stop command",
          parameters: [],
          frequency: 10,
          enabled: true,
        },
      ],
    },
    {
      id: "mcu_002",
      name: "Sensor Hub",
      type: "esp32",
      protocol: "wifi",
      address: "192.168.1.101",
      port: 8080,
      status: "connected",
      lastSeen: new Date(),
      dataRate: 89.2,
      functions: ["IMU Data", "Environmental Sensors", "Camera Control"],
      powerConsumption: 1.8,
      firmware: "v1.3.2",
      serialConfig: {
        baudRate: 9600,
        dataBits: 8,
        stopBits: 1,
        parity: "none",
        flowControl: "none",
        timeout: 2000,
        reconnectInterval: 3000,
      },
      commands: [
        {
          command: "GET_IMU",
          description: "Get IMU sensor data",
          parameters: [],
          frequency: 100,
          enabled: true,
        },
        {
          command: "GET_TEMP",
          description: "Get temperature reading",
          parameters: [],
          frequency: 1,
          enabled: true,
        },
        {
          command: "CAMERA_TRIGGER",
          description: "Trigger camera capture",
          parameters: ["mode"],
          frequency: 30,
          enabled: true,
        },
      ],
    },
    {
      id: "mcu_003",
      name: "Arm Controller",
      type: "teensy",
      protocol: "usb",
      address: "/dev/ttyACM0",
      port: 0,
      baudRate: 9600,
      status: "connected",
      lastSeen: new Date(),
      dataRate: 67.8,
      functions: ["Joint Control", "Force Feedback", "End Effector"],
      powerConsumption: 3.1,
      firmware: "v3.0.1",
      routes: [
        {
          topic: "/arm/joint_states",
          direction: "publish",
          frequency: 50,
          enabled: true,
        },
        {
          topic: "/arm/joint_commands",
          direction: "subscribe",
          frequency: 20,
          enabled: true,
        },
        {
          topic: "/gripper/force",
          direction: "publish",
          frequency: 10,
          enabled: true,
        },
      ],
    },
    {
      id: "mcu_004",
      name: "Safety Monitor",
      type: "stm32",
      protocol: "ethernet",
      address: "192.168.1.102",
      port: 502,
      status: "error",
      lastSeen: new Date(Date.now() - 300000),
      dataRate: 0,
      functions: ["Safety Monitoring", "Power Management", "Diagnostics"],
      powerConsumption: 1.2,
      firmware: "v1.8.5",
      routes: [
        {
          topic: "/safety/status",
          direction: "publish",
          frequency: 5,
          enabled: false,
        },
        {
          topic: "/power/status",
          direction: "publish",
          frequency: 1,
          enabled: false,
        },
      ],
    },
    {
      id: "mcu_005",
      name: "Wireless Relay",
      type: "esp32",
      protocol: "bluetooth",
      address: "AA:BB:CC:DD:EE:FF",
      port: 1,
      status: "disconnected",
      lastSeen: new Date(Date.now() - 600000),
      dataRate: 0,
      functions: ["Remote Control", "Telemetry", "Debug Interface"],
      powerConsumption: 0.8,
      firmware: "v2.0.3",
      routes: [
        {
          topic: "/remote/cmd",
          direction: "subscribe",
          frequency: 10,
          enabled: false,
        },
        {
          topic: "/telemetry/basic",
          direction: "publish",
          frequency: 2,
          enabled: false,
        },
      ],
    },
  ]);

  const [selectedConnection, setSelectedConnection] = useState<string | null>(
    null,
  );
  const [isAutoRefresh, setIsAutoRefresh] = useState(true);
  const [showAddForm, setShowAddForm] = useState(false);

  // New connection form state
  const [newConnection, setNewConnection] = useState({
    name: "",
    type: "arduino" as const,
    protocol: "usb" as const,
    address: "",
    port: 0,
    baudRate: 115200,
  });

  // Simulate real-time updates
  useEffect(() => {
    if (!isAutoRefresh) return;

    const interval = setInterval(() => {
      setConnections((prev) =>
        prev.map((conn) => {
          if (conn.status === "connected") {
            return {
              ...conn,
              lastSeen: new Date(),
              dataRate: Math.max(0, conn.dataRate + (Math.random() - 0.5) * 20),
            };
          }
          return conn;
        }),
      );
    }, 2000);

    return () => clearInterval(interval);
  }, [isAutoRefresh]);

  const getStatusColor = (status: string) => {
    switch (status) {
      case "connected":
        return "text-green-500";
      case "disconnected":
        return "text-muted-foreground";
      case "error":
        return "text-red-500";
      case "connecting":
        return "text-yellow-500";
      default:
        return "text-muted-foreground";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "connected":
        return CheckCircle;
      case "disconnected":
        return Clock;
      case "error":
        return AlertTriangle;
      case "connecting":
        return RefreshCw;
      default:
        return Clock;
    }
  };

  const getTypeIcon = (type: string) => {
    switch (type) {
      case "arduino":
        return Cpu;
      case "esp32":
        return Wifi;
      case "raspberry_pi":
        return Monitor;
      case "teensy":
        return Zap;
      case "stm32":
        return HardDrive;
      default:
        return Cpu;
    }
  };

  const getProtocolIcon = (protocol: string) => {
    switch (protocol) {
      case "usb":
        return Cable;
      case "wifi":
        return Wifi;
      case "ethernet":
        return Network;
      case "serial":
        return Link;
      case "bluetooth":
        return Radio;
      default:
        return Cable;
    }
  };

  const handleConnectionAction = (
    connectionId: string,
    action: "connect" | "disconnect" | "restart",
  ) => {
    setConnections((prev) =>
      prev.map((conn) => {
        if (conn.id === connectionId) {
          switch (action) {
            case "connect":
              return { ...conn, status: "connecting" as const };
            case "disconnect":
              return { ...conn, status: "disconnected" as const };
            case "restart":
              return { ...conn, status: "connecting" as const };
          }
        }
        return conn;
      }),
    );

    // Simulate state change
    setTimeout(() => {
      setConnections((prev) =>
        prev.map((conn) => {
          if (conn.id === connectionId) {
            switch (action) {
              case "connect":
              case "restart":
                return {
                  ...conn,
                  status: "connected" as const,
                  lastSeen: new Date(),
                  dataRate: Math.random() * 100 + 50,
                };
              case "disconnect":
                return { ...conn, status: "disconnected" as const };
            }
          }
          return conn;
        }),
      );
    }, 2000);
  };

  const handleAddConnection = () => {
    const connection: MicrocontrollerConnection = {
      id: `mcu_${Date.now()}`,
      ...newConnection,
      status: "disconnected",
      lastSeen: new Date(),
      dataRate: 0,
      functions: [],
      powerConsumption: 1.0,
      firmware: "v1.0.0",
      routes: [],
    };

    setConnections((prev) => [...prev, connection]);
    setNewConnection({
      name: "",
      type: "arduino",
      protocol: "usb",
      address: "",
      port: 0,
      baudRate: 115200,
    });
    setShowAddForm(false);
  };

  const selectedConnectionData = connections.find(
    (conn) => conn.id === selectedConnection,
  );

  const connectedCount = connections.filter(
    (c) => c.status === "connected",
  ).length;
  const totalDataRate = connections.reduce((sum, c) => sum + c.dataRate, 0);
  const totalPowerConsumption = connections
    .filter((c) => c.status === "connected")
    .reduce((sum, c) => sum + c.powerConsumption, 0);

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Router className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            Microcontroller Connections
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Manage microcontroller connections, protocols, and data routing
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
            onClick={() => setShowAddForm(!showAddForm)}
            className="gap-2"
          >
            <Zap className="h-4 w-4" />
            Add Connection
          </Button>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
        </div>
      </div>

      {/* Statistics Overview */}
      <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-primary">
              {connections.length}
            </div>
            <div className="text-sm text-muted-foreground">
              Total Controllers
            </div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-green-500">
              {connectedCount}
            </div>
            <div className="text-sm text-muted-foreground">Connected</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-blue-500">
              {totalDataRate.toFixed(1)}
            </div>
            <div className="text-sm text-muted-foreground">KB/s Total</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-purple-500">
              {totalPowerConsumption.toFixed(1)}W
            </div>
            <div className="text-sm text-muted-foreground">Power Usage</div>
          </div>
        </Card>
      </div>

      {/* Add Connection Form */}
      {showAddForm && (
        <Card className="p-4 md:p-6 border-primary/30 fade-in-up">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Zap className="h-5 w-5 text-primary" />
            Add New Connection
          </h3>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
            <div>
              <Label>Controller Name</Label>
              <Input
                value={newConnection.name}
                onChange={(e) =>
                  setNewConnection((prev) => ({
                    ...prev,
                    name: e.target.value,
                  }))
                }
                placeholder="e.g., Main Drive Controller"
              />
            </div>

            <div>
              <Label>Controller Type</Label>
              <Select
                value={newConnection.type}
                onValueChange={(value: any) =>
                  setNewConnection((prev) => ({ ...prev, type: value }))
                }
              >
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="arduino">Arduino</SelectItem>
                  <SelectItem value="esp32">ESP32</SelectItem>
                  <SelectItem value="raspberry_pi">Raspberry Pi</SelectItem>
                  <SelectItem value="teensy">Teensy</SelectItem>
                  <SelectItem value="stm32">STM32</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div>
              <Label>Protocol</Label>
              <Select
                value={newConnection.protocol}
                onValueChange={(value: any) =>
                  setNewConnection((prev) => ({ ...prev, protocol: value }))
                }
              >
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="usb">USB</SelectItem>
                  <SelectItem value="wifi">WiFi</SelectItem>
                  <SelectItem value="ethernet">Ethernet</SelectItem>
                  <SelectItem value="serial">Serial</SelectItem>
                  <SelectItem value="bluetooth">Bluetooth</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div>
              <Label>Address</Label>
              <Input
                value={newConnection.address}
                onChange={(e) =>
                  setNewConnection((prev) => ({
                    ...prev,
                    address: e.target.value,
                  }))
                }
                placeholder={
                  newConnection.protocol === "usb"
                    ? "/dev/ttyUSB0"
                    : "192.168.1.100"
                }
              />
            </div>

            <div>
              <Label>Port</Label>
              <Input
                type="number"
                value={newConnection.port}
                onChange={(e) =>
                  setNewConnection((prev) => ({
                    ...prev,
                    port: parseInt(e.target.value) || 0,
                  }))
                }
              />
            </div>

            {(newConnection.protocol === "usb" ||
              newConnection.protocol === "serial") && (
              <div>
                <Label>Baud Rate</Label>
                <Select
                  value={newConnection.baudRate.toString()}
                  onValueChange={(value) =>
                    setNewConnection((prev) => ({
                      ...prev,
                      baudRate: parseInt(value),
                    }))
                  }
                >
                  <SelectTrigger>
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="9600">9600</SelectItem>
                    <SelectItem value="19200">19200</SelectItem>
                    <SelectItem value="38400">38400</SelectItem>
                    <SelectItem value="57600">57600</SelectItem>
                    <SelectItem value="115200">115200</SelectItem>
                    <SelectItem value="230400">230400</SelectItem>
                  </SelectContent>
                </Select>
              </div>
            )}
          </div>

          <div className="flex gap-2 mt-4">
            <Button onClick={handleAddConnection} className="gap-2">
              <Save className="h-4 w-4" />
              Add Connection
            </Button>
            <Button
              variant="outline"
              onClick={() => setShowAddForm(false)}
              className="gap-2"
            >
              Cancel
            </Button>
          </div>
        </Card>
      )}

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Connections List */}
        <Card className="lg:col-span-2 p-4">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Activity className="h-5 w-5 text-primary" />
            Controllers ({connections.length})
          </h3>

          <ScrollArea className="h-[600px]">
            <div className="space-y-2">
              {connections.map((connection, index) => {
                const StatusIcon = getStatusIcon(connection.status);
                const TypeIcon = getTypeIcon(connection.type);
                const ProtocolIcon = getProtocolIcon(connection.protocol);
                const isSelected = selectedConnection === connection.id;

                return (
                  <div
                    key={connection.id}
                    className={`p-4 rounded-lg border transition-all cursor-pointer hover-lift stagger-item ${
                      isSelected
                        ? "bg-primary/5 border-primary/30"
                        : "hover:bg-muted/50"
                    }`}
                    style={{ animationDelay: `${index * 0.05}s` }}
                    onClick={() =>
                      setSelectedConnection(isSelected ? null : connection.id)
                    }
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-3 flex-1">
                        <StatusIcon
                          className={`h-5 w-5 ${getStatusColor(connection.status)} ${connection.status === "connecting" ? "animate-spin" : ""}`}
                        />
                        <div className="flex-1 min-w-0">
                          <div className="font-medium text-sm truncate">
                            {connection.name}
                          </div>
                          <div className="text-xs text-muted-foreground truncate">
                            {connection.address}
                          </div>
                        </div>
                      </div>

                      <div className="flex items-center gap-2">
                        <div className="text-right text-xs space-y-1">
                          <div className="flex items-center gap-1">
                            <TypeIcon className="h-3 w-3 text-muted-foreground" />
                            <span className="capitalize">
                              {connection.type}
                            </span>
                          </div>
                          <div className="flex items-center gap-1">
                            <ProtocolIcon className="h-3 w-3 text-muted-foreground" />
                            <span className="uppercase">
                              {connection.protocol}
                            </span>
                          </div>
                        </div>

                        <Badge
                          variant={
                            connection.status === "connected"
                              ? "default"
                              : connection.status === "error"
                                ? "destructive"
                                : "secondary"
                          }
                          className="text-xs"
                        >
                          {connection.status.toUpperCase()}
                        </Badge>
                      </div>
                    </div>

                    {isSelected && (
                      <div className="mt-4 pt-4 border-t space-y-3 fade-in-up">
                        <div className="grid grid-cols-1 sm:grid-cols-3 gap-4 text-xs">
                          <div>
                            <Label className="text-muted-foreground">
                              Functions
                            </Label>
                            <div className="mt-1 space-y-1">
                              {connection.functions.length > 0 ? (
                                connection.functions.map((func) => (
                                  <div
                                    key={func}
                                    className="text-xs bg-muted p-1 rounded"
                                  >
                                    {func}
                                  </div>
                                ))
                              ) : (
                                <div className="text-muted-foreground">
                                  No functions configured
                                </div>
                              )}
                            </div>
                          </div>
                          <div>
                            <Label className="text-muted-foreground">
                              Routes ({connection.routes.length})
                            </Label>
                            <div className="mt-1 space-y-1">
                              {connection.routes.slice(0, 3).map((route) => (
                                <div
                                  key={route.topic}
                                  className="text-xs bg-muted p-1 rounded flex items-center justify-between"
                                >
                                  <span className="font-mono truncate">
                                    {route.topic}
                                  </span>
                                  <Badge
                                    variant={
                                      route.enabled ? "default" : "secondary"
                                    }
                                    className="text-xs"
                                  >
                                    {route.direction}
                                  </Badge>
                                </div>
                              ))}
                              {connection.routes.length > 3 && (
                                <div className="text-xs text-muted-foreground">
                                  +{connection.routes.length - 3} more...
                                </div>
                              )}
                            </div>
                          </div>
                          <div>
                            <Label className="text-muted-foreground">
                              Statistics
                            </Label>
                            <div className="mt-1 space-y-1 text-xs">
                              <div>
                                Data Rate:{" "}
                                <span className="font-mono">
                                  {connection.dataRate.toFixed(1)} KB/s
                                </span>
                              </div>
                              <div>
                                Power:{" "}
                                <span className="font-mono">
                                  {connection.powerConsumption.toFixed(1)}W
                                </span>
                              </div>
                              <div>
                                Firmware:{" "}
                                <span className="font-mono">
                                  {connection.firmware}
                                </span>
                              </div>
                            </div>
                          </div>
                        </div>

                        <div className="flex flex-wrap gap-2">
                          {connection.status === "disconnected" ||
                          connection.status === "error" ? (
                            <Button
                              size="sm"
                              onClick={(e) => {
                                e.stopPropagation();
                                handleConnectionAction(
                                  connection.id,
                                  "connect",
                                );
                              }}
                              disabled={connection.status === "connecting"}
                              className="gap-1"
                            >
                              <Zap className="h-3 w-3" />
                              Connect
                            </Button>
                          ) : (
                            <Button
                              size="sm"
                              variant="outline"
                              onClick={(e) => {
                                e.stopPropagation();
                                handleConnectionAction(
                                  connection.id,
                                  "disconnect",
                                );
                              }}
                              className="gap-1"
                            >
                              <Clock className="h-3 w-3" />
                              Disconnect
                            </Button>
                          )}
                          <Button
                            size="sm"
                            variant="outline"
                            onClick={(e) => {
                              e.stopPropagation();
                              handleConnectionAction(connection.id, "restart");
                            }}
                            disabled={connection.status === "connecting"}
                            className="gap-1"
                          >
                            <RefreshCw className="h-3 w-3" />
                            Restart
                          </Button>
                          <Button size="sm" variant="outline" className="gap-1">
                            <Settings className="h-3 w-3" />
                            Configure
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

        {/* Connection Details */}
        <div className="space-y-6">
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Settings className="h-5 w-5 text-primary" />
              Connection Details
            </h3>

            {selectedConnectionData ? (
              <div className="space-y-4">
                <div>
                  <Label className="text-sm font-medium">Controller Name</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedConnectionData.name}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Type & Protocol</Label>
                  <div className="flex gap-2 mt-1">
                    <Badge variant="outline" className="capitalize">
                      {selectedConnectionData.type}
                    </Badge>
                    <Badge variant="outline" className="uppercase">
                      {selectedConnectionData.protocol}
                    </Badge>
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Connection</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedConnectionData.address}
                    {selectedConnectionData.port > 0 &&
                      `:${selectedConnectionData.port}`}
                  </div>
                </div>

                {selectedConnectionData.baudRate && (
                  <div>
                    <Label className="text-sm font-medium">Baud Rate</Label>
                    <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                      {selectedConnectionData.baudRate}
                    </div>
                  </div>
                )}

                <Separator />

                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <Label className="text-sm font-medium">Data Rate</Label>
                    <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                      {selectedConnectionData.dataRate.toFixed(1)} KB/s
                    </div>
                  </div>
                  <div>
                    <Label className="text-sm font-medium">Power</Label>
                    <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                      {selectedConnectionData.powerConsumption.toFixed(1)}W
                    </div>
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Firmware</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedConnectionData.firmware}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Last Seen</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedConnectionData.lastSeen.toLocaleString()}
                  </div>
                </div>
              </div>
            ) : (
              <div className="text-center text-muted-foreground py-8">
                Select a connection to view details
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
                <RefreshCw className="h-4 w-4" />
                Reconnect All
              </Button>
              <Button variant="outline" className="w-full justify-start gap-2">
                <Download className="h-4 w-4" />
                Export Config
              </Button>
              <Button variant="outline" className="w-full justify-start gap-2">
                <Upload className="h-4 w-4" />
                Import Config
              </Button>
              <Button variant="outline" className="w-full justify-start gap-2">
                <Monitor className="h-4 w-4" />
                System Diagnostics
              </Button>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
};

export default MicrocontrollerConnections;
