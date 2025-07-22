import React, { useState } from "react";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { Switch } from "@/components/ui/switch";
import { Badge } from "@/components/ui/badge";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Progress } from "@/components/ui/progress";
import {
  Save,
  Download,
  Upload,
  RefreshCw,
  Settings,
  Cpu,
  HardDrive,
  Network,
  Wifi,
  Terminal,
  Shield,
  Database,
  FileText,
  Zap,
  Monitor,
  Clock,
  Globe,
  Archive,
  AlertTriangle,
  CheckCircle,
  Play,
  Square,
} from "lucide-react";

export default function SystemConfiguration() {
  const [isApplying, setIsApplying] = useState(false);
  const [lastBackup, setLastBackup] = useState(new Date());

  // System Settings State
  const [systemSettings, setSystemSettings] = useState({
    // ROS Configuration
    rosVersion: "noetic",
    rosMasterUri: "http://localhost:11311",
    rosBridgePort: 9090,
    rosWorkspace: "/opt/ros/noetic",
    autoStartRos: true,

    // Network Configuration
    wifiEnabled: true,
    ethernetEnabled: true,
    staticIp: false,
    ipAddress: "192.168.1.100",
    netmask: "255.255.255.0",
    gateway: "192.168.1.1",

    // ESP32 Configuration
    esp32Enabled: false,
    esp32WifiSSID: "",
    esp32WifiPassword: "",
    esp32EthernetEnabled: false,

    // System Performance
    cpuGovernor: "performance",
    maxCpuFreq: 100,
    swappiness: 10,
    enableGpu: true,

    // Security
    firewallEnabled: true,
    sshEnabled: true,
    sshPort: 22,
    autoUpdates: false,

    // Backup & Storage
    autoBackup: true,
    backupInterval: 24,
    maxBackups: 10,
    backupLocation: "/home/robot/backups",

    // Launch Files
    autoLaunchFiles: [],
    customLaunchFiles: [],

    // System Monitoring
    enableLogging: true,
    logLevel: "INFO",
    maxLogSize: 100,
    enableMetrics: true,
  });

  const handleApply = async () => {
    setIsApplying(true);
    console.log("Applying system configuration...", systemSettings);

    // Simulate applying settings
    await new Promise((resolve) => setTimeout(resolve, 2000));

    // Save to JSON
    localStorage.setItem("systemConfiguration", JSON.stringify(systemSettings));

    setIsApplying(false);
  };

  const handleBackup = () => {
    const backupData = {
      timestamp: new Date().toISOString(),
      systemSettings,
      version: "2.0.0",
    };

    const blob = new Blob([JSON.stringify(backupData, null, 2)], {
      type: "application/json",
    });

    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `system-backup-${new Date().toISOString().split("T")[0]}.json`;
    a.click();
    URL.revokeObjectURL(url);

    setLastBackup(new Date());
  };

  const handleRestore = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const backupData = JSON.parse(e.target?.result as string);
        setSystemSettings(backupData.systemSettings);
        console.log("System configuration restored from backup");
      } catch (error) {
        console.error("Failed to restore backup:", error);
      }
    };
    reader.readAsText(file);
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            System Configuration
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Configure system settings, ROS parameters, and network options
          </p>
        </div>
        <div className="flex items-center gap-3">
          <input
            type="file"
            accept=".json"
            onChange={handleRestore}
            className="hidden"
            id="restore-backup"
          />
          <Button
            variant="outline"
            onClick={() => document.getElementById("restore-backup")?.click()}
            className="gap-2"
          >
            <Upload className="h-4 w-4" />
            Restore
          </Button>
          <Button variant="outline" onClick={handleBackup} className="gap-2">
            <Download className="h-4 w-4" />
            Backup
          </Button>
          <Button onClick={handleApply} disabled={isApplying} className="gap-2">
            {isApplying ? (
              <RefreshCw className="h-4 w-4 animate-spin" />
            ) : (
              <Save className="h-4 w-4" />
            )}
            Apply Configuration
          </Button>
        </div>
      </div>

      {/* Configuration Tabs */}
      <Tabs defaultValue="ros" className="space-y-6">
        <TabsList className="grid w-full grid-cols-6">
          <TabsTrigger value="ros" className="gap-2">
            <Terminal className="h-4 w-4" />
            ROS
          </TabsTrigger>
          <TabsTrigger value="network" className="gap-2">
            <Network className="h-4 w-4" />
            Network
          </TabsTrigger>
          <TabsTrigger value="performance" className="gap-2">
            <Cpu className="h-4 w-4" />
            Performance
          </TabsTrigger>
          <TabsTrigger value="security" className="gap-2">
            <Shield className="h-4 w-4" />
            Security
          </TabsTrigger>
          <TabsTrigger value="backup" className="gap-2">
            <Archive className="h-4 w-4" />
            Backup
          </TabsTrigger>
          <TabsTrigger value="launch" className="gap-2">
            <Play className="h-4 w-4" />
            Launch
          </TabsTrigger>
        </TabsList>

        {/* ROS Configuration */}
        <TabsContent value="ros">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">ROS Configuration</h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>ROS Version</Label>
                  <Select
                    value={systemSettings.rosVersion}
                    onValueChange={(value) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        rosVersion: value,
                      }))
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="noetic">
                        ROS Noetic (Ubuntu 20.04)
                      </SelectItem>
                      <SelectItem value="humble">
                        ROS2 Humble (Ubuntu 22.04)
                      </SelectItem>
                      <SelectItem value="iron">
                        ROS2 Iron (Ubuntu 22.04)
                      </SelectItem>
                      <SelectItem value="rolling">
                        ROS2 Rolling (Latest)
                      </SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>ROS Master URI</Label>
                  <Input
                    value={systemSettings.rosMasterUri}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        rosMasterUri: e.target.value,
                      }))
                    }
                    placeholder="http://localhost:11311"
                  />
                </div>

                <div>
                  <Label>ROS Bridge Port</Label>
                  <Input
                    type="number"
                    value={systemSettings.rosBridgePort}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        rosBridgePort: parseInt(e.target.value),
                      }))
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>ROS Workspace</Label>
                  <Input
                    value={systemSettings.rosWorkspace}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        rosWorkspace: e.target.value,
                      }))
                    }
                    placeholder="/opt/ros/noetic"
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Auto-start ROS on boot</Label>
                    <p className="text-sm text-gray-500">
                      Automatically start ROS services when system boots
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.autoStartRos}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        autoStartRos: checked,
                      }))
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Network Configuration */}
        <TabsContent value="network">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {/* WiFi Configuration */}
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-6">WiFi Configuration</h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable WiFi</Label>
                    <p className="text-sm text-gray-500">
                      Enable wireless network interface
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.wifiEnabled}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        wifiEnabled: checked,
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>ESP32 WiFi SSID</Label>
                  <Input
                    value={systemSettings.esp32WifiSSID}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        esp32WifiSSID: e.target.value,
                      }))
                    }
                    placeholder="RobotNetwork"
                  />
                </div>

                <div>
                  <Label>ESP32 WiFi Password</Label>
                  <Input
                    type="password"
                    value={systemSettings.esp32WifiPassword}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        esp32WifiPassword: e.target.value,
                      }))
                    }
                    placeholder="Enter WiFi password"
                  />
                </div>
              </div>
            </Card>

            {/* Ethernet Configuration */}
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-6">
                Ethernet Configuration
              </h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable Ethernet</Label>
                    <p className="text-sm text-gray-500">
                      Enable wired network interface
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.ethernetEnabled}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        ethernetEnabled: checked,
                      }))
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Static IP</Label>
                    <p className="text-sm text-gray-500">
                      Use static IP address
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.staticIp}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        staticIp: checked,
                      }))
                    }
                  />
                </div>

                {systemSettings.staticIp && (
                  <>
                    <div>
                      <Label>IP Address</Label>
                      <Input
                        value={systemSettings.ipAddress}
                        onChange={(e) =>
                          setSystemSettings((prev) => ({
                            ...prev,
                            ipAddress: e.target.value,
                          }))
                        }
                        placeholder="192.168.1.100"
                      />
                    </div>

                    <div>
                      <Label>Netmask</Label>
                      <Input
                        value={systemSettings.netmask}
                        onChange={(e) =>
                          setSystemSettings((prev) => ({
                            ...prev,
                            netmask: e.target.value,
                          }))
                        }
                        placeholder="255.255.255.0"
                      />
                    </div>

                    <div>
                      <Label>Gateway</Label>
                      <Input
                        value={systemSettings.gateway}
                        onChange={(e) =>
                          setSystemSettings((prev) => ({
                            ...prev,
                            gateway: e.target.value,
                          }))
                        }
                        placeholder="192.168.1.1"
                      />
                    </div>
                  </>
                )}
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Performance Configuration */}
        <TabsContent value="performance">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">Performance Settings</h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>CPU Governor</Label>
                  <Select
                    value={systemSettings.cpuGovernor}
                    onValueChange={(value) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        cpuGovernor: value,
                      }))
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="performance">Performance</SelectItem>
                      <SelectItem value="powersave">Power Save</SelectItem>
                      <SelectItem value="ondemand">On Demand</SelectItem>
                      <SelectItem value="conservative">Conservative</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Max CPU Frequency (%)</Label>
                  <div className="space-y-2">
                    <Input
                      type="number"
                      min="10"
                      max="100"
                      value={systemSettings.maxCpuFreq}
                      onChange={(e) =>
                        setSystemSettings((prev) => ({
                          ...prev,
                          maxCpuFreq: parseInt(e.target.value),
                        }))
                      }
                    />
                    <Progress
                      value={systemSettings.maxCpuFreq}
                      className="w-full"
                    />
                  </div>
                </div>

                <div>
                  <Label>Swappiness</Label>
                  <Input
                    type="number"
                    min="0"
                    max="100"
                    value={systemSettings.swappiness}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        swappiness: parseInt(e.target.value),
                      }))
                    }
                  />
                  <p className="text-sm text-gray-500">
                    Lower values prefer RAM over swap
                  </p>
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable GPU Acceleration</Label>
                    <p className="text-sm text-gray-500">
                      Use GPU for vision processing
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.enableGpu}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        enableGpu: checked,
                      }))
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable System Metrics</Label>
                    <p className="text-sm text-gray-500">
                      Collect performance metrics
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.enableMetrics}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        enableMetrics: checked,
                      }))
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Security Configuration */}
        <TabsContent value="security">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">Security Settings</h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable Firewall</Label>
                    <p className="text-sm text-gray-500">
                      UFW firewall protection
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.firewallEnabled}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        firewallEnabled: checked,
                      }))
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable SSH</Label>
                    <p className="text-sm text-gray-500">
                      Remote access via SSH
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.sshEnabled}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        sshEnabled: checked,
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>SSH Port</Label>
                  <Input
                    type="number"
                    value={systemSettings.sshPort}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        sshPort: parseInt(e.target.value),
                      }))
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Auto Updates</Label>
                    <p className="text-sm text-gray-500">
                      Automatic security updates
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.autoUpdates}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        autoUpdates: checked,
                      }))
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Backup Configuration */}
        <TabsContent value="backup">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">Backup & Storage</h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Auto Backup</Label>
                    <p className="text-sm text-gray-500">
                      Automatic system backups
                    </p>
                  </div>
                  <Switch
                    checked={systemSettings.autoBackup}
                    onCheckedChange={(checked) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        autoBackup: checked,
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Backup Interval (hours)</Label>
                  <Input
                    type="number"
                    min="1"
                    max="168"
                    value={systemSettings.backupInterval}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        backupInterval: parseInt(e.target.value),
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Max Backups to Keep</Label>
                  <Input
                    type="number"
                    min="1"
                    max="50"
                    value={systemSettings.maxBackups}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        maxBackups: parseInt(e.target.value),
                      }))
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>Backup Location</Label>
                  <Input
                    value={systemSettings.backupLocation}
                    onChange={(e) =>
                      setSystemSettings((prev) => ({
                        ...prev,
                        backupLocation: e.target.value,
                      }))
                    }
                    placeholder="/home/robot/backups"
                  />
                </div>

                <div className="bg-white/5 border border-white/10 p-4 rounded-lg">
                  <div className="flex items-center justify-between mb-2">
                    <span className="font-medium">Last Backup</span>
                    <Badge variant="outline">
                      {lastBackup.toLocaleDateString()}
                    </Badge>
                  </div>
                  <p className="text-sm text-slate-300">
                    {lastBackup.toLocaleString()}
                  </p>
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Launch Files Configuration */}
        <TabsContent value="launch">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">
              Launch Files & Startup
            </h3>
            <div className="space-y-6">
              <div>
                <Label>Auto-launch Files on Startup</Label>
                <p className="text-sm text-gray-500 mb-3">
                  ROS launch files to automatically start when the system boots
                </p>
                <Textarea
                  placeholder="Enter launch file paths, one per line&#10;/opt/ros/noetic/share/robot_bringup/launch/robot.launch&#10;/home/robot/catkin_ws/src/my_package/launch/sensors.launch"
                  value={systemSettings.autoLaunchFiles.join("\n")}
                  onChange={(e) =>
                    setSystemSettings((prev) => ({
                      ...prev,
                      autoLaunchFiles: e.target.value
                        .split("\n")
                        .filter((line) => line.trim()),
                    }))
                  }
                  rows={5}
                />
              </div>

              <div>
                <Label>Custom Launch Files</Label>
                <p className="text-sm text-gray-500 mb-3">
                  Additional launch files for manual execution
                </p>
                <Textarea
                  placeholder="Enter custom launch file paths&#10;/home/robot/custom_launch/debug.launch&#10;/home/robot/custom_launch/calibration.launch"
                  value={systemSettings.customLaunchFiles.join("\n")}
                  onChange={(e) =>
                    setSystemSettings((prev) => ({
                      ...prev,
                      customLaunchFiles: e.target.value
                        .split("\n")
                        .filter((line) => line.trim()),
                    }))
                  }
                  rows={5}
                />
              </div>
            </div>
          </Card>
        </TabsContent>
      </Tabs>

      {/* Status Summary */}
      <Card className="p-6 mt-8">
        <h3 className="text-lg font-semibold mb-4">Configuration Status</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <div className="flex items-center gap-2">
            <CheckCircle className="h-5 w-5 text-green-500" />
            <span className="text-sm">ROS Configured</span>
          </div>
          <div className="flex items-center gap-2">
            <CheckCircle className="h-5 w-5 text-green-500" />
            <span className="text-sm">Network Ready</span>
          </div>
          <div className="flex items-center gap-2">
            <AlertTriangle className="h-5 w-5 text-orange-500" />
            <span className="text-sm">ESP32 Pending</span>
          </div>
          <div className="flex items-center gap-2">
            <CheckCircle className="h-5 w-5 text-green-500" />
            <span className="text-sm">Security Active</span>
          </div>
        </div>
      </Card>
    </div>
  );
}
