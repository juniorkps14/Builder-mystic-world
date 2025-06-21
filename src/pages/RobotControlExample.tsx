import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import {
  useROSIntegration,
  useROSTopic,
} from "@/services/ROSIntegrationService";
import {
  Bot,
  Play,
  Square,
  RotateCcw,
  Zap,
  Navigation,
  Activity,
  Wifi,
  WifiOff,
  AlertTriangle,
  CheckCircle,
} from "lucide-react";

const RobotControlExample = () => {
  const {
    config,
    connect,
    disconnect,
    sendVelocityCommand,
    emergencyStop,
    sendNavigationGoal,
    getCurrentPose,
    getSystemStatus,
  } = useROSIntegration();

  // Real-time topic subscriptions
  const { data: odomData } = useROSTopic("/odom", config.isConnected);
  const { data: batteryData } = useROSTopic(
    "/battery_state",
    config.isConnected,
  );
  const { data: cmdVelData } = useROSTopic("/cmd_vel", config.isConnected);

  const [velocityControl, setVelocityControl] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  });

  const [navigationGoal, setNavigationGoal] = useState({
    x: 0,
    y: 0,
    yaw: 0,
  });

  const [isMoving, setIsMoving] = useState(false);
  const [systemStatus, setSystemStatus] = useState<any>(null);

  // Update system status periodically
  useEffect(() => {
    const interval = setInterval(async () => {
      if (config.isConnected) {
        try {
          const status = await getSystemStatus();
          if (status.success) {
            setSystemStatus(status.data);
          }
        } catch (error) {
          console.error("Failed to get system status:", error);
        }
      }
    }, 5000);

    return () => clearInterval(interval);
  }, [config.isConnected, getSystemStatus]);

  // Handle velocity command
  const handleSendVelocity = async () => {
    try {
      setIsMoving(true);
      const response = await sendVelocityCommand(
        velocityControl.linear,
        velocityControl.angular,
      );

      if (!response.success) {
        alert(`Failed to send velocity: ${response.error}`);
      }
    } catch (error) {
      alert(`Error sending velocity: ${error}`);
    } finally {
      setTimeout(() => setIsMoving(false), 1000);
    }
  };

  // Handle emergency stop
  const handleEmergencyStop = async () => {
    try {
      const response = await emergencyStop();
      if (response.success) {
        setVelocityControl({
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
      } else {
        alert(`Emergency stop failed: ${response.error}`);
      }
    } catch (error) {
      alert(`Emergency stop error: ${error}`);
    }
  };

  // Handle navigation goal
  const handleSendNavigationGoal = async () => {
    try {
      const response = await sendNavigationGoal(
        navigationGoal.x,
        navigationGoal.y,
        navigationGoal.yaw,
      );

      if (response.success) {
        alert("Navigation goal sent successfully!");
      } else {
        alert(`Navigation failed: ${response.error}`);
      }
    } catch (error) {
      alert(`Navigation error: ${error}`);
    }
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Bot className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            Robot Control (ROS Integrated)
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Control your robot through ROS with real-time feedback
          </p>
        </div>

        <div className="flex gap-2">
          <Button
            variant={config.isConnected ? "default" : "outline"}
            onClick={config.isConnected ? disconnect : connect}
            className="gap-2"
          >
            {config.isConnected ? (
              <Wifi className="h-4 w-4" />
            ) : (
              <WifiOff className="h-4 w-4" />
            )}
            {config.isConnected ? "Connected" : "Connect to ROS"}
          </Button>
        </div>
      </div>

      {/* Connection Status */}
      <Card className="p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div
              className={`w-3 h-3 rounded-full ${
                config.isConnected ? "bg-green-500 animate-pulse" : "bg-red-500"
              }`}
            />
            <span className="font-medium">
              ROS Bridge: {config.isConnected ? "Connected" : "Disconnected"}
            </span>
          </div>
          <div className="text-sm text-muted-foreground">
            {config.rosBridgeUrl}
          </div>
        </div>
      </Card>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Velocity Control */}
        <Card className="p-6">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Activity className="h-5 w-5 text-primary" />
            Velocity Control
          </h3>

          <div className="space-y-4">
            <div>
              <Label className="text-sm">Linear X (m/s)</Label>
              <div className="flex items-center gap-2 mt-1">
                <Slider
                  value={[velocityControl.linear.x]}
                  onValueChange={([value]) =>
                    setVelocityControl((prev) => ({
                      ...prev,
                      linear: { ...prev.linear, x: value },
                    }))
                  }
                  min={-2}
                  max={2}
                  step={0.1}
                  className="flex-1"
                />
                <span className="w-12 text-xs font-mono">
                  {velocityControl.linear.x.toFixed(1)}
                </span>
              </div>
            </div>

            <div>
              <Label className="text-sm">Linear Y (m/s)</Label>
              <div className="flex items-center gap-2 mt-1">
                <Slider
                  value={[velocityControl.linear.y]}
                  onValueChange={([value]) =>
                    setVelocityControl((prev) => ({
                      ...prev,
                      linear: { ...prev.linear, y: value },
                    }))
                  }
                  min={-2}
                  max={2}
                  step={0.1}
                  className="flex-1"
                />
                <span className="w-12 text-xs font-mono">
                  {velocityControl.linear.y.toFixed(1)}
                </span>
              </div>
            </div>

            <div>
              <Label className="text-sm">Angular Z (rad/s)</Label>
              <div className="flex items-center gap-2 mt-1">
                <Slider
                  value={[velocityControl.angular.z]}
                  onValueChange={([value]) =>
                    setVelocityControl((prev) => ({
                      ...prev,
                      angular: { ...prev.angular, z: value },
                    }))
                  }
                  min={-3.14}
                  max={3.14}
                  step={0.1}
                  className="flex-1"
                />
                <span className="w-12 text-xs font-mono">
                  {velocityControl.angular.z.toFixed(1)}
                </span>
              </div>
            </div>

            <div className="flex gap-2">
              <Button
                onClick={handleSendVelocity}
                disabled={!config.isConnected || isMoving}
                className="flex-1 gap-2"
              >
                <Play className="h-4 w-4" />
                Send Velocity
              </Button>
              <Button
                variant="destructive"
                onClick={handleEmergencyStop}
                disabled={!config.isConnected}
                className="gap-2"
              >
                <Square className="h-4 w-4" />
                E-Stop
              </Button>
            </div>
          </div>
        </Card>

        {/* Navigation Control */}
        <Card className="p-6">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Navigation className="h-5 w-5 text-primary" />
            Navigation
          </h3>

          <div className="space-y-4">
            <div>
              <Label className="text-sm">Target X (m)</Label>
              <Input
                type="number"
                step="0.1"
                value={navigationGoal.x}
                onChange={(e) =>
                  setNavigationGoal((prev) => ({
                    ...prev,
                    x: parseFloat(e.target.value) || 0,
                  }))
                }
                className="mt-1"
              />
            </div>

            <div>
              <Label className="text-sm">Target Y (m)</Label>
              <Input
                type="number"
                step="0.1"
                value={navigationGoal.y}
                onChange={(e) =>
                  setNavigationGoal((prev) => ({
                    ...prev,
                    y: parseFloat(e.target.value) || 0,
                  }))
                }
                className="mt-1"
              />
            </div>

            <div>
              <Label className="text-sm">Target Yaw (rad)</Label>
              <Input
                type="number"
                step="0.1"
                value={navigationGoal.yaw}
                onChange={(e) =>
                  setNavigationGoal((prev) => ({
                    ...prev,
                    yaw: parseFloat(e.target.value) || 0,
                  }))
                }
                className="mt-1"
              />
            </div>

            <Button
              onClick={handleSendNavigationGoal}
              disabled={!config.isConnected}
              className="w-full gap-2"
            >
              <Navigation className="h-4 w-4" />
              Send Navigation Goal
            </Button>
          </div>
        </Card>

        {/* Real-time Data */}
        <Card className="p-6">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Zap className="h-5 w-5 text-primary" />
            Real-time Data
          </h3>

          <div className="space-y-4">
            {/* Odometry Data */}
            {odomData && (
              <div>
                <Label className="text-sm font-medium">Odometry</Label>
                <div className="mt-1 space-y-1 text-xs">
                  <div className="flex justify-between">
                    <span>Position:</span>
                    <span className="font-mono">
                      ({odomData.pose?.pose?.position?.x?.toFixed(2) || "0.00"},
                      {odomData.pose?.pose?.position?.y?.toFixed(2) || "0.00"})
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span>Linear Vel:</span>
                    <span className="font-mono">
                      {odomData.twist?.twist?.linear?.x?.toFixed(2) || "0.00"}{" "}
                      m/s
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span>Angular Vel:</span>
                    <span className="font-mono">
                      {odomData.twist?.twist?.angular?.z?.toFixed(2) || "0.00"}{" "}
                      rad/s
                    </span>
                  </div>
                </div>
              </div>
            )}

            {/* Battery Data */}
            {batteryData && (
              <div>
                <Label className="text-sm font-medium">Battery</Label>
                <div className="mt-1 space-y-1 text-xs">
                  <div className="flex justify-between">
                    <span>Voltage:</span>
                    <span className="font-mono">
                      {batteryData.voltage?.toFixed(1) || "0.0"}V
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span>Percentage:</span>
                    <span className="font-mono">
                      {batteryData.percentage?.toFixed(0) || "0"}%
                    </span>
                  </div>
                </div>
              </div>
            )}

            {/* Current Velocity */}
            {cmdVelData && (
              <div>
                <Label className="text-sm font-medium">Current Command</Label>
                <div className="mt-1 space-y-1 text-xs">
                  <div className="flex justify-between">
                    <span>Linear X:</span>
                    <span className="font-mono">
                      {cmdVelData.linear?.x?.toFixed(2) || "0.00"} m/s
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span>Angular Z:</span>
                    <span className="font-mono">
                      {cmdVelData.angular?.z?.toFixed(2) || "0.00"} rad/s
                    </span>
                  </div>
                </div>
              </div>
            )}

            {/* System Status */}
            {systemStatus && (
              <div>
                <Label className="text-sm font-medium">System Status</Label>
                <div className="mt-1 space-y-1 text-xs">
                  <div className="flex justify-between">
                    <span>Active Nodes:</span>
                    <span className="font-mono">
                      {systemStatus.nodes?.length || 0}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span>Topics:</span>
                    <span className="font-mono">
                      {systemStatus.topics?.length || 0}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span>Services:</span>
                    <span className="font-mono">
                      {systemStatus.services?.length || 0}
                    </span>
                  </div>
                </div>
              </div>
            )}

            {!config.isConnected && (
              <div className="flex items-center gap-2 text-yellow-600">
                <AlertTriangle className="h-4 w-4" />
                <span className="text-xs">
                  Connect to ROS to see real-time data
                </span>
              </div>
            )}
          </div>
        </Card>
      </div>
    </div>
  );
};

export default RobotControlExample;
