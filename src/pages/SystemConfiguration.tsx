import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Switch } from "@/components/ui/switch";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { useLanguage } from "@/contexts/LanguageContext";
import { useROSIntegration } from "@/services/ROSIntegrationService";
import {
  Settings,
  Wifi,
  Bot,
  Shield,
  Database,
  Monitor,
  Zap,
  Network,
  Save,
  RefreshCw,
  Download,
  Upload,
  AlertTriangle,
  CheckCircle,
  Info,
  FileText,
  Clock,
  Thermometer,
  Gauge,
  Camera,
  Compass,
  Activity,
  Globe,
  Lock,
  Bell,
  HardDrive,
  Cpu,
  MemoryStick,
} from "lucide-react";

interface SystemConfig {
  ros: {
    masterUri: string;
    rosBridgeUrl: string;
    rosBridgePort: number;
    nodeName: string;
    namespace: string;
    autoStart: boolean;
    heartbeatInterval: number;
    reconnectInterval: number;
    logLevel: "DEBUG" | "INFO" | "WARN" | "ERROR";
  };
  robot: {
    name: string;
    type: "mobile" | "arm" | "humanoid" | "aerial" | "custom";
    maxVelocity: number;
    maxAngularVelocity: number;
    wheelDiameter: number;
    wheelBase: number;
    trackWidth: number;
    safetyEnabled: boolean;
    emergencyStopTimeout: number;
    batteryThreshold: number;
    temperatureThreshold: number;
  };
  network: {
    wifiEnabled: boolean;
    wifiSSID: string;
    wifiPassword: string;
    ethernetEnabled: boolean;
    ethernetIP: string;
    ethernetSubnet: string;
    ethernetGateway: string;
    dnsServers: string[];
    firewallEnabled: boolean;
    sshEnabled: boolean;
    sshPort: number;
  };
  sensors: {
    imuEnabled: boolean;
    imuCalibration: number[];
    lidarEnabled: boolean;
    lidarMaxRange: number;
    cameraEnabled: boolean;
    cameraResolution: string;
    cameraFrameRate: number;
    encodersEnabled: boolean;
    encoderPPR: number;
    gpsEnabled: boolean;
    ultrasonicEnabled: boolean;
  };
  safety: {
    emergencyStopEnabled: boolean;
    collisionDetection: boolean;
    obstacleAvoidance: boolean;
    speedLimiting: boolean;
    geofencing: boolean;
    batteryMonitoring: boolean;
    temperatureMonitoring: boolean;
    connectionMonitoring: boolean;
    safetyZones: Array<{
      name: string;
      x: number;
      y: number;
      radius: number;
      action: "stop" | "slow" | "warn";
    }>;
  };
  logging: {
    enabled: boolean;
    level: "DEBUG" | "INFO" | "WARN" | "ERROR";
    maxFileSize: number;
    maxFiles: number;
    logPath: string;
    remoteLogging: boolean;
    remoteEndpoint: string;
    realTimeDisplay: boolean;
    categories: string[];
  };
  system: {
    timezone: string;
    autoUpdate: boolean;
    telemetryEnabled: boolean;
    diagnosticsInterval: number;
    resourceMonitoring: boolean;
    performanceLogging: boolean;
    debugMode: boolean;
    devMode: boolean;
  };
}

const SystemConfiguration = () => {
  const { t } = useLanguage();
  const { config: rosConfig, updateConfig } = useROSIntegration();

  const [systemConfig, setSystemConfig] = useState<SystemConfig>({
    ros: {
      masterUri: "http://localhost:11311",
      rosBridgeUrl: "ws://localhost:9090",
      rosBridgePort: 9090,
      nodeName: "web_interface",
      namespace: "/web_interface",
      autoStart: true,
      heartbeatInterval: 30000,
      reconnectInterval: 5000,
      logLevel: "INFO",
    },
    robot: {
      name: "DinoBot",
      type: "mobile",
      maxVelocity: 2.0,
      maxAngularVelocity: 1.5,
      wheelDiameter: 0.1,
      wheelBase: 0.5,
      trackWidth: 0.4,
      safetyEnabled: true,
      emergencyStopTimeout: 5000,
      batteryThreshold: 20,
      temperatureThreshold: 75,
    },
    network: {
      wifiEnabled: true,
      wifiSSID: "",
      wifiPassword: "",
      ethernetEnabled: true,
      ethernetIP: "192.168.1.100",
      ethernetSubnet: "255.255.255.0",
      ethernetGateway: "192.168.1.1",
      dnsServers: ["8.8.8.8", "8.8.4.4"],
      firewallEnabled: true,
      sshEnabled: true,
      sshPort: 22,
    },
    sensors: {
      imuEnabled: true,
      imuCalibration: [0, 0, 0, 0, 0, 0],
      lidarEnabled: true,
      lidarMaxRange: 30.0,
      cameraEnabled: true,
      cameraResolution: "1920x1080",
      cameraFrameRate: 30,
      encodersEnabled: true,
      encoderPPR: 1024,
      gpsEnabled: false,
      ultrasonicEnabled: true,
    },
    safety: {
      emergencyStopEnabled: true,
      collisionDetection: true,
      obstacleAvoidance: true,
      speedLimiting: true,
      geofencing: false,
      batteryMonitoring: true,
      temperatureMonitoring: true,
      connectionMonitoring: true,
      safetyZones: [],
    },
    logging: {
      enabled: true,
      level: "INFO",
      maxFileSize: 100,
      maxFiles: 10,
      logPath: "/var/log/robot",
      remoteLogging: false,
      remoteEndpoint: "",
      realTimeDisplay: true,
      categories: ["ros", "system", "sensors", "navigation", "safety"],
    },
    system: {
      timezone: "Asia/Bangkok",
      autoUpdate: false,
      telemetryEnabled: true,
      diagnosticsInterval: 60000,
      resourceMonitoring: true,
      performanceLogging: false,
      debugMode: false,
      devMode: true,
    },
  });

  const [isUnsaved, setIsUnsaved] = useState(false);
  const [lastSaved, setLastSaved] = useState<Date | null>(null);
  const [isImportDialogOpen, setIsImportDialogOpen] = useState(false);
  const [importData, setImportData] = useState("");

  // Monitor changes
  useEffect(() => {
    setIsUnsaved(true);
  }, [systemConfig]);

  const updateNestedConfig = (
    section: keyof SystemConfig,
    key: string,
    value: any,
  ) => {
    setSystemConfig((prev) => ({
      ...prev,
      [section]: {
        ...prev[section],
        [key]: value,
      },
    }));
  };

  const handleSaveConfig = async () => {
    try {
      // Update ROS integration config
      updateConfig({
        rosBridgeUrl: systemConfig.ros.rosBridgeUrl,
        masterUri: systemConfig.ros.masterUri,
        reconnectInterval: systemConfig.ros.reconnectInterval,
        heartbeatInterval: systemConfig.ros.heartbeatInterval,
      });

      // Save to localStorage (in production, save to backend)
      localStorage.setItem("systemConfig", JSON.stringify(systemConfig));

      setIsUnsaved(false);
      setLastSaved(new Date());

      // Show success message
      alert("Configuration saved successfully!");
    } catch (error) {
      alert(`Failed to save configuration: ${error}`);
    }
  };

  const handleLoadConfig = () => {
    try {
      const saved = localStorage.getItem("systemConfig");
      if (saved) {
        const loadedConfig = JSON.parse(saved);
        setSystemConfig(loadedConfig);
        setIsUnsaved(false);
        alert("Configuration loaded successfully!");
      } else {
        alert("No saved configuration found");
      }
    } catch (error) {
      alert(`Failed to load configuration: ${error}`);
    }
  };

  const handleExportConfig = () => {
    const dataStr = JSON.stringify(systemConfig, null, 2);
    const dataUri =
      "data:application/json;charset=utf-8," + encodeURIComponent(dataStr);
    const exportFileDefaultName = `robot_config_${new Date().toISOString().split("T")[0]}.json`;

    const linkElement = document.createElement("a");
    linkElement.setAttribute("href", dataUri);
    linkElement.setAttribute("download", exportFileDefaultName);
    linkElement.click();
  };

  const handleImportConfig = () => {
    try {
      const imported = JSON.parse(importData);
      setSystemConfig(imported);
      setIsImportDialogOpen(false);
      setImportData("");
      alert("Configuration imported successfully!");
    } catch (error) {
      alert(`Invalid configuration file: ${error}`);
    }
  };

  const addSafetyZone = () => {
    setSystemConfig((prev) => ({
      ...prev,
      safety: {
        ...prev.safety,
        safetyZones: [
          ...prev.safety.safetyZones,
          {
            name: `Zone ${prev.safety.safetyZones.length + 1}`,
            x: 0,
            y: 0,
            radius: 1.0,
            action: "stop" as const,
          },
        ],
      },
    }));
  };

  const removeSafetyZone = (index: number) => {
    setSystemConfig((prev) => ({
      ...prev,
      safety: {
        ...prev.safety,
        safetyZones: prev.safety.safetyZones.filter((_, i) => i !== index),
      },
    }));
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Settings className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            System Configuration
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Configure ROS, robot parameters, network, and system settings
          </p>
        </div>

        <div className="flex gap-2">
          <Button
            onClick={handleSaveConfig}
            disabled={!isUnsaved}
            className="gap-2"
          >
            <Save className="h-4 w-4" />
            Save Configuration
            {isUnsaved && <Badge variant="destructive">*</Badge>}
          </Button>
          <Button
            variant="outline"
            onClick={handleLoadConfig}
            className="gap-2"
          >
            <RefreshCw className="h-4 w-4" />
            Load
          </Button>
          <Button
            variant="outline"
            onClick={handleExportConfig}
            className="gap-2"
          >
            <Download className="h-4 w-4" />
            Export
          </Button>
          <Dialog
            open={isImportDialogOpen}
            onOpenChange={setIsImportDialogOpen}
          >
            <DialogTrigger asChild>
              <Button variant="outline" className="gap-2">
                <Upload className="h-4 w-4" />
                Import
              </Button>
            </DialogTrigger>
            <DialogContent className="sm:max-w-[525px]">
              <DialogHeader>
                <DialogTitle>Import Configuration</DialogTitle>
                <DialogDescription>
                  Paste your configuration JSON below to import settings.
                </DialogDescription>
              </DialogHeader>
              <Textarea
                value={importData}
                onChange={(e) => setImportData(e.target.value)}
                placeholder="Paste configuration JSON here..."
                rows={10}
                className="font-mono text-xs"
              />
              <DialogFooter>
                <Button
                  variant="outline"
                  onClick={() => setIsImportDialogOpen(false)}
                >
                  Cancel
                </Button>
                <Button onClick={handleImportConfig}>Import</Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
        </div>
      </div>

      {/* Status Bar */}
      {(isUnsaved || lastSaved) && (
        <Card className="p-3">
          <div className="flex items-center justify-between text-sm">
            <div className="flex items-center gap-2">
              {isUnsaved ? (
                <>
                  <AlertTriangle className="h-4 w-4 text-yellow-500" />
                  <span className="text-yellow-600">
                    You have unsaved changes
                  </span>
                </>
              ) : (
                <>
                  <CheckCircle className="h-4 w-4 text-green-500" />
                  <span className="text-green-600">All changes saved</span>
                </>
              )}
            </div>
            {lastSaved && (
              <span className="text-muted-foreground">
                Last saved: {lastSaved.toLocaleTimeString()}
              </span>
            )}
          </div>
        </Card>
      )}

      <Tabs defaultValue="ros" className="space-y-4">
        <TabsList className="grid w-full grid-cols-3 lg:grid-cols-7">
          <TabsTrigger value="ros" className="gap-1 text-xs">
            <Wifi className="h-3 w-3" />
            ROS
          </TabsTrigger>
          <TabsTrigger value="robot" className="gap-1 text-xs">
            <Bot className="h-3 w-3" />
            Robot
          </TabsTrigger>
          <TabsTrigger value="network" className="gap-1 text-xs">
            <Network className="h-3 w-3" />
            Network
          </TabsTrigger>
          <TabsTrigger value="sensors" className="gap-1 text-xs">
            <Activity className="h-3 w-3" />
            Sensors
          </TabsTrigger>
          <TabsTrigger value="safety" className="gap-1 text-xs">
            <Shield className="h-3 w-3" />
            Safety
          </TabsTrigger>
          <TabsTrigger value="logging" className="gap-1 text-xs">
            <FileText className="h-3 w-3" />
            Logging
          </TabsTrigger>
          <TabsTrigger value="system" className="gap-1 text-xs">
            <Monitor className="h-3 w-3" />
            System
          </TabsTrigger>
        </TabsList>

        {/* ROS Configuration */}
        <TabsContent value="ros" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Wifi className="h-5 w-5 text-primary" />
              ROS Configuration
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>ROS Master URI</Label>
                  <Input
                    value={systemConfig.ros.masterUri}
                    onChange={(e) =>
                      updateNestedConfig("ros", "masterUri", e.target.value)
                    }
                    placeholder="http://localhost:11311"
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>RosBridge WebSocket URL</Label>
                  <Input
                    value={systemConfig.ros.rosBridgeUrl}
                    onChange={(e) =>
                      updateNestedConfig("ros", "rosBridgeUrl", e.target.value)
                    }
                    placeholder="ws://localhost:9090"
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>RosBridge Port</Label>
                  <Input
                    type="number"
                    value={systemConfig.ros.rosBridgePort}
                    onChange={(e) =>
                      updateNestedConfig(
                        "ros",
                        "rosBridgePort",
                        parseInt(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Node Name</Label>
                  <Input
                    value={systemConfig.ros.nodeName}
                    onChange={(e) =>
                      updateNestedConfig("ros", "nodeName", e.target.value)
                    }
                    className="mt-1"
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>Namespace</Label>
                  <Input
                    value={systemConfig.ros.namespace}
                    onChange={(e) =>
                      updateNestedConfig("ros", "namespace", e.target.value)
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Heartbeat Interval (ms)</Label>
                  <Input
                    type="number"
                    value={systemConfig.ros.heartbeatInterval}
                    onChange={(e) =>
                      updateNestedConfig(
                        "ros",
                        "heartbeatInterval",
                        parseInt(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Reconnect Interval (ms)</Label>
                  <Input
                    type="number"
                    value={systemConfig.ros.reconnectInterval}
                    onChange={(e) =>
                      updateNestedConfig(
                        "ros",
                        "reconnectInterval",
                        parseInt(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Log Level</Label>
                  <Select
                    value={systemConfig.ros.logLevel}
                    onValueChange={(value: any) =>
                      updateNestedConfig("ros", "logLevel", value)
                    }
                  >
                    <SelectTrigger className="mt-1">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="DEBUG">Debug</SelectItem>
                      <SelectItem value="INFO">Info</SelectItem>
                      <SelectItem value="WARN">Warning</SelectItem>
                      <SelectItem value="ERROR">Error</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div className="flex items-center justify-between">
                  <Label>Auto Start</Label>
                  <Switch
                    checked={systemConfig.ros.autoStart}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("ros", "autoStart", checked)
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Robot Configuration */}
        <TabsContent value="robot" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Bot className="h-5 w-5 text-primary" />
              Robot Configuration
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>Robot Name</Label>
                  <Input
                    value={systemConfig.robot.name}
                    onChange={(e) =>
                      updateNestedConfig("robot", "name", e.target.value)
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Robot Type</Label>
                  <Select
                    value={systemConfig.robot.type}
                    onValueChange={(value: any) =>
                      updateNestedConfig("robot", "type", value)
                    }
                  >
                    <SelectTrigger className="mt-1">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="mobile">Mobile Robot</SelectItem>
                      <SelectItem value="arm">Robotic Arm</SelectItem>
                      <SelectItem value="humanoid">Humanoid</SelectItem>
                      <SelectItem value="aerial">Aerial Vehicle</SelectItem>
                      <SelectItem value="custom">Custom</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Max Velocity (m/s)</Label>
                  <Input
                    type="number"
                    step="0.1"
                    value={systemConfig.robot.maxVelocity}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "maxVelocity",
                        parseFloat(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Max Angular Velocity (rad/s)</Label>
                  <Input
                    type="number"
                    step="0.1"
                    value={systemConfig.robot.maxAngularVelocity}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "maxAngularVelocity",
                        parseFloat(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Wheel Diameter (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={systemConfig.robot.wheelDiameter}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "wheelDiameter",
                        parseFloat(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>Wheel Base (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={systemConfig.robot.wheelBase}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "wheelBase",
                        parseFloat(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Track Width (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={systemConfig.robot.trackWidth}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "trackWidth",
                        parseFloat(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Emergency Stop Timeout (ms)</Label>
                  <Input
                    type="number"
                    value={systemConfig.robot.emergencyStopTimeout}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "emergencyStopTimeout",
                        parseInt(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Battery Threshold (%)</Label>
                  <Input
                    type="number"
                    value={systemConfig.robot.batteryThreshold}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "batteryThreshold",
                        parseInt(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Temperature Threshold (°C)</Label>
                  <Input
                    type="number"
                    value={systemConfig.robot.temperatureThreshold}
                    onChange={(e) =>
                      updateNestedConfig(
                        "robot",
                        "temperatureThreshold",
                        parseInt(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Safety Enabled</Label>
                  <Switch
                    checked={systemConfig.robot.safetyEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("robot", "safetyEnabled", checked)
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Network Configuration */}
        <TabsContent value="network" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="p-6">
              <h3 className="text-lg font-light mb-4 flex items-center gap-2">
                <Wifi className="h-5 w-5 text-primary" />
                WiFi Configuration
              </h3>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>WiFi Enabled</Label>
                  <Switch
                    checked={systemConfig.network.wifiEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("network", "wifiEnabled", checked)
                    }
                  />
                </div>

                <div>
                  <Label>SSID</Label>
                  <Input
                    value={systemConfig.network.wifiSSID}
                    onChange={(e) =>
                      updateNestedConfig("network", "wifiSSID", e.target.value)
                    }
                    disabled={!systemConfig.network.wifiEnabled}
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Password</Label>
                  <Input
                    type="password"
                    value={systemConfig.network.wifiPassword}
                    onChange={(e) =>
                      updateNestedConfig(
                        "network",
                        "wifiPassword",
                        e.target.value,
                      )
                    }
                    disabled={!systemConfig.network.wifiEnabled}
                    className="mt-1"
                  />
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-light mb-4 flex items-center gap-2">
                <Network className="h-5 w-5 text-primary" />
                Ethernet Configuration
              </h3>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Ethernet Enabled</Label>
                  <Switch
                    checked={systemConfig.network.ethernetEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("network", "ethernetEnabled", checked)
                    }
                  />
                </div>

                <div>
                  <Label>IP Address</Label>
                  <Input
                    value={systemConfig.network.ethernetIP}
                    onChange={(e) =>
                      updateNestedConfig(
                        "network",
                        "ethernetIP",
                        e.target.value,
                      )
                    }
                    disabled={!systemConfig.network.ethernetEnabled}
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Subnet Mask</Label>
                  <Input
                    value={systemConfig.network.ethernetSubnet}
                    onChange={(e) =>
                      updateNestedConfig(
                        "network",
                        "ethernetSubnet",
                        e.target.value,
                      )
                    }
                    disabled={!systemConfig.network.ethernetEnabled}
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Gateway</Label>
                  <Input
                    value={systemConfig.network.ethernetGateway}
                    onChange={(e) =>
                      updateNestedConfig(
                        "network",
                        "ethernetGateway",
                        e.target.value,
                      )
                    }
                    disabled={!systemConfig.network.ethernetEnabled}
                    className="mt-1"
                  />
                </div>
              </div>
            </Card>
          </div>

          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Shield className="h-5 w-5 text-primary" />
              Network Security
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Firewall Enabled</Label>
                  <Switch
                    checked={systemConfig.network.firewallEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("network", "firewallEnabled", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>SSH Enabled</Label>
                  <Switch
                    checked={systemConfig.network.sshEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("network", "sshEnabled", checked)
                    }
                  />
                </div>

                <div>
                  <Label>SSH Port</Label>
                  <Input
                    type="number"
                    value={systemConfig.network.sshPort}
                    onChange={(e) =>
                      updateNestedConfig(
                        "network",
                        "sshPort",
                        parseInt(e.target.value),
                      )
                    }
                    disabled={!systemConfig.network.sshEnabled}
                    className="mt-1"
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>DNS Servers</Label>
                  <div className="space-y-2 mt-1">
                    {systemConfig.network.dnsServers.map((dns, index) => (
                      <div key={index} className="flex gap-2">
                        <Input
                          value={dns}
                          onChange={(e) => {
                            const newDns = [...systemConfig.network.dnsServers];
                            newDns[index] = e.target.value;
                            updateNestedConfig("network", "dnsServers", newDns);
                          }}
                          placeholder="8.8.8.8"
                        />
                        <Button
                          variant="outline"
                          size="sm"
                          onClick={() => {
                            const newDns =
                              systemConfig.network.dnsServers.filter(
                                (_, i) => i !== index,
                              );
                            updateNestedConfig("network", "dnsServers", newDns);
                          }}
                        >
                          Remove
                        </Button>
                      </div>
                    ))}
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => {
                        const newDns = [...systemConfig.network.dnsServers, ""];
                        updateNestedConfig("network", "dnsServers", newDns);
                      }}
                    >
                      Add DNS Server
                    </Button>
                  </div>
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Sensors Configuration */}
        <TabsContent value="sensors" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="p-6">
              <h3 className="text-lg font-light mb-4 flex items-center gap-2">
                <Compass className="h-5 w-5 text-primary" />
                Motion Sensors
              </h3>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>IMU Enabled</Label>
                  <Switch
                    checked={systemConfig.sensors.imuEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("sensors", "imuEnabled", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Encoders Enabled</Label>
                  <Switch
                    checked={systemConfig.sensors.encodersEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("sensors", "encodersEnabled", checked)
                    }
                  />
                </div>

                <div>
                  <Label>Encoder PPR</Label>
                  <Input
                    type="number"
                    value={systemConfig.sensors.encoderPPR}
                    onChange={(e) =>
                      updateNestedConfig(
                        "sensors",
                        "encoderPPR",
                        parseInt(e.target.value),
                      )
                    }
                    disabled={!systemConfig.sensors.encodersEnabled}
                    className="mt-1"
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>GPS Enabled</Label>
                  <Switch
                    checked={systemConfig.sensors.gpsEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("sensors", "gpsEnabled", checked)
                    }
                  />
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-light mb-4 flex items-center gap-2">
                <Activity className="h-5 w-5 text-primary" />
                Perception Sensors
              </h3>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>LiDAR Enabled</Label>
                  <Switch
                    checked={systemConfig.sensors.lidarEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("sensors", "lidarEnabled", checked)
                    }
                  />
                </div>

                <div>
                  <Label>LiDAR Max Range (m)</Label>
                  <Input
                    type="number"
                    step="0.1"
                    value={systemConfig.sensors.lidarMaxRange}
                    onChange={(e) =>
                      updateNestedConfig(
                        "sensors",
                        "lidarMaxRange",
                        parseFloat(e.target.value),
                      )
                    }
                    disabled={!systemConfig.sensors.lidarEnabled}
                    className="mt-1"
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Camera Enabled</Label>
                  <Switch
                    checked={systemConfig.sensors.cameraEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("sensors", "cameraEnabled", checked)
                    }
                  />
                </div>

                <div>
                  <Label>Camera Resolution</Label>
                  <Select
                    value={systemConfig.sensors.cameraResolution}
                    onValueChange={(value) =>
                      updateNestedConfig("sensors", "cameraResolution", value)
                    }
                    disabled={!systemConfig.sensors.cameraEnabled}
                  >
                    <SelectTrigger className="mt-1">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="640x480">640x480</SelectItem>
                      <SelectItem value="1280x720">1280x720</SelectItem>
                      <SelectItem value="1920x1080">1920x1080</SelectItem>
                      <SelectItem value="3840x2160">3840x2160</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Camera Frame Rate</Label>
                  <Input
                    type="number"
                    value={systemConfig.sensors.cameraFrameRate}
                    onChange={(e) =>
                      updateNestedConfig(
                        "sensors",
                        "cameraFrameRate",
                        parseInt(e.target.value),
                      )
                    }
                    disabled={!systemConfig.sensors.cameraEnabled}
                    className="mt-1"
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Ultrasonic Enabled</Label>
                  <Switch
                    checked={systemConfig.sensors.ultrasonicEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig(
                        "sensors",
                        "ultrasonicEnabled",
                        checked,
                      )
                    }
                  />
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Safety Configuration */}
        <TabsContent value="safety" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Shield className="h-5 w-5 text-primary" />
              Safety Systems
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Emergency Stop</Label>
                  <Switch
                    checked={systemConfig.safety.emergencyStopEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig(
                        "safety",
                        "emergencyStopEnabled",
                        checked,
                      )
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Collision Detection</Label>
                  <Switch
                    checked={systemConfig.safety.collisionDetection}
                    onCheckedChange={(checked) =>
                      updateNestedConfig(
                        "safety",
                        "collisionDetection",
                        checked,
                      )
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Obstacle Avoidance</Label>
                  <Switch
                    checked={systemConfig.safety.obstacleAvoidance}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("safety", "obstacleAvoidance", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Speed Limiting</Label>
                  <Switch
                    checked={systemConfig.safety.speedLimiting}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("safety", "speedLimiting", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Geofencing</Label>
                  <Switch
                    checked={systemConfig.safety.geofencing}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("safety", "geofencing", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Battery Monitoring</Label>
                  <Switch
                    checked={systemConfig.safety.batteryMonitoring}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("safety", "batteryMonitoring", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Temperature Monitoring</Label>
                  <Switch
                    checked={systemConfig.safety.temperatureMonitoring}
                    onCheckedChange={(checked) =>
                      updateNestedConfig(
                        "safety",
                        "temperatureMonitoring",
                        checked,
                      )
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Connection Monitoring</Label>
                  <Switch
                    checked={systemConfig.safety.connectionMonitoring}
                    onCheckedChange={(checked) =>
                      updateNestedConfig(
                        "safety",
                        "connectionMonitoring",
                        checked,
                      )
                    }
                  />
                </div>
              </div>
            </div>
          </Card>

          <Card className="p-6">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-lg font-light flex items-center gap-2">
                <Globe className="h-5 w-5 text-primary" />
                Safety Zones
              </h3>
              <Button onClick={addSafetyZone} size="sm" className="gap-2">
                <Info className="h-4 w-4" />
                Add Zone
              </Button>
            </div>

            <div className="space-y-4">
              {systemConfig.safety.safetyZones.map((zone, index) => (
                <div key={index} className="p-4 border rounded-lg">
                  <div className="grid grid-cols-1 md:grid-cols-5 gap-4">
                    <div>
                      <Label>Name</Label>
                      <Input
                        value={zone.name}
                        onChange={(e) => {
                          const newZones = [...systemConfig.safety.safetyZones];
                          newZones[index].name = e.target.value;
                          updateNestedConfig("safety", "safetyZones", newZones);
                        }}
                        className="mt-1"
                      />
                    </div>

                    <div>
                      <Label>X Position</Label>
                      <Input
                        type="number"
                        step="0.1"
                        value={zone.x}
                        onChange={(e) => {
                          const newZones = [...systemConfig.safety.safetyZones];
                          newZones[index].x = parseFloat(e.target.value);
                          updateNestedConfig("safety", "safetyZones", newZones);
                        }}
                        className="mt-1"
                      />
                    </div>

                    <div>
                      <Label>Y Position</Label>
                      <Input
                        type="number"
                        step="0.1"
                        value={zone.y}
                        onChange={(e) => {
                          const newZones = [...systemConfig.safety.safetyZones];
                          newZones[index].y = parseFloat(e.target.value);
                          updateNestedConfig("safety", "safetyZones", newZones);
                        }}
                        className="mt-1"
                      />
                    </div>

                    <div>
                      <Label>Radius</Label>
                      <Input
                        type="number"
                        step="0.1"
                        value={zone.radius}
                        onChange={(e) => {
                          const newZones = [...systemConfig.safety.safetyZones];
                          newZones[index].radius = parseFloat(e.target.value);
                          updateNestedConfig("safety", "safetyZones", newZones);
                        }}
                        className="mt-1"
                      />
                    </div>

                    <div className="flex items-end gap-2">
                      <div className="flex-1">
                        <Label>Action</Label>
                        <Select
                          value={zone.action}
                          onValueChange={(value: any) => {
                            const newZones = [
                              ...systemConfig.safety.safetyZones,
                            ];
                            newZones[index].action = value;
                            updateNestedConfig(
                              "safety",
                              "safetyZones",
                              newZones,
                            );
                          }}
                        >
                          <SelectTrigger className="mt-1">
                            <SelectValue />
                          </SelectTrigger>
                          <SelectContent>
                            <SelectItem value="stop">Stop</SelectItem>
                            <SelectItem value="slow">Slow Down</SelectItem>
                            <SelectItem value="warn">Warning Only</SelectItem>
                          </SelectContent>
                        </Select>
                      </div>
                      <Button
                        variant="outline"
                        size="sm"
                        onClick={() => removeSafetyZone(index)}
                      >
                        Remove
                      </Button>
                    </div>
                  </div>
                </div>
              ))}

              {systemConfig.safety.safetyZones.length === 0 && (
                <div className="text-center py-8 text-muted-foreground">
                  No safety zones configured. Click "Add Zone" to create one.
                </div>
              )}
            </div>
          </Card>
        </TabsContent>

        {/* Logging Configuration */}
        <TabsContent value="logging" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <FileText className="h-5 w-5 text-primary" />
              Logging Configuration
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Logging Enabled</Label>
                  <Switch
                    checked={systemConfig.logging.enabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("logging", "enabled", checked)
                    }
                  />
                </div>

                <div>
                  <Label>Log Level</Label>
                  <Select
                    value={systemConfig.logging.level}
                    onValueChange={(value: any) =>
                      updateNestedConfig("logging", "level", value)
                    }
                    disabled={!systemConfig.logging.enabled}
                  >
                    <SelectTrigger className="mt-1">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="DEBUG">Debug</SelectItem>
                      <SelectItem value="INFO">Info</SelectItem>
                      <SelectItem value="WARN">Warning</SelectItem>
                      <SelectItem value="ERROR">Error</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Max File Size (MB)</Label>
                  <Input
                    type="number"
                    value={systemConfig.logging.maxFileSize}
                    onChange={(e) =>
                      updateNestedConfig(
                        "logging",
                        "maxFileSize",
                        parseInt(e.target.value),
                      )
                    }
                    disabled={!systemConfig.logging.enabled}
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Max Files</Label>
                  <Input
                    type="number"
                    value={systemConfig.logging.maxFiles}
                    onChange={(e) =>
                      updateNestedConfig(
                        "logging",
                        "maxFiles",
                        parseInt(e.target.value),
                      )
                    }
                    disabled={!systemConfig.logging.enabled}
                    className="mt-1"
                  />
                </div>

                <div>
                  <Label>Log Path</Label>
                  <Input
                    value={systemConfig.logging.logPath}
                    onChange={(e) =>
                      updateNestedConfig("logging", "logPath", e.target.value)
                    }
                    disabled={!systemConfig.logging.enabled}
                    className="mt-1"
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Remote Logging</Label>
                  <Switch
                    checked={systemConfig.logging.remoteLogging}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("logging", "remoteLogging", checked)
                    }
                    disabled={!systemConfig.logging.enabled}
                  />
                </div>

                <div>
                  <Label>Remote Endpoint</Label>
                  <Input
                    value={systemConfig.logging.remoteEndpoint}
                    onChange={(e) =>
                      updateNestedConfig(
                        "logging",
                        "remoteEndpoint",
                        e.target.value,
                      )
                    }
                    disabled={
                      !systemConfig.logging.enabled ||
                      !systemConfig.logging.remoteLogging
                    }
                    placeholder="https://logs.example.com/api"
                    className="mt-1"
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Real-time Display</Label>
                  <Switch
                    checked={systemConfig.logging.realTimeDisplay}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("logging", "realTimeDisplay", checked)
                    }
                    disabled={!systemConfig.logging.enabled}
                  />
                </div>

                <div>
                  <Label>Log Categories</Label>
                  <div className="space-y-2 mt-1">
                    {["ros", "system", "sensors", "navigation", "safety"].map(
                      (category) => (
                        <div
                          key={category}
                          className="flex items-center justify-between"
                        >
                          <Label className="capitalize">{category}</Label>
                          <Switch
                            checked={systemConfig.logging.categories.includes(
                              category,
                            )}
                            onCheckedChange={(checked) => {
                              const categories = checked
                                ? [...systemConfig.logging.categories, category]
                                : systemConfig.logging.categories.filter(
                                    (c) => c !== category,
                                  );
                              updateNestedConfig(
                                "logging",
                                "categories",
                                categories,
                              );
                            }}
                            disabled={!systemConfig.logging.enabled}
                          />
                        </div>
                      ),
                    )}
                  </div>
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* System Configuration */}
        <TabsContent value="system" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Monitor className="h-5 w-5 text-primary" />
              System Settings
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>Timezone</Label>
                  <Select
                    value={systemConfig.system.timezone}
                    onValueChange={(value) =>
                      updateNestedConfig("system", "timezone", value)
                    }
                  >
                    <SelectTrigger className="mt-1">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="Asia/Bangkok">Asia/Bangkok</SelectItem>
                      <SelectItem value="UTC">UTC</SelectItem>
                      <SelectItem value="America/New_York">
                        America/New_York
                      </SelectItem>
                      <SelectItem value="Europe/London">
                        Europe/London
                      </SelectItem>
                      <SelectItem value="Asia/Tokyo">Asia/Tokyo</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Diagnostics Interval (ms)</Label>
                  <Input
                    type="number"
                    value={systemConfig.system.diagnosticsInterval}
                    onChange={(e) =>
                      updateNestedConfig(
                        "system",
                        "diagnosticsInterval",
                        parseInt(e.target.value),
                      )
                    }
                    className="mt-1"
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Auto Update</Label>
                  <Switch
                    checked={systemConfig.system.autoUpdate}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("system", "autoUpdate", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Telemetry Enabled</Label>
                  <Switch
                    checked={systemConfig.system.telemetryEnabled}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("system", "telemetryEnabled", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <Label>Resource Monitoring</Label>
                  <Switch
                    checked={systemConfig.system.resourceMonitoring}
                    onCheckedChange={(checked) =>
                      updateNestedConfig(
                        "system",
                        "resourceMonitoring",
                        checked,
                      )
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Performance Logging</Label>
                  <Switch
                    checked={systemConfig.system.performanceLogging}
                    onCheckedChange={(checked) =>
                      updateNestedConfig(
                        "system",
                        "performanceLogging",
                        checked,
                      )
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Debug Mode</Label>
                  <Switch
                    checked={systemConfig.system.debugMode}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("system", "debugMode", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Development Mode</Label>
                  <Switch
                    checked={systemConfig.system.devMode}
                    onCheckedChange={(checked) =>
                      updateNestedConfig("system", "devMode", checked)
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
};

export default SystemConfiguration;
