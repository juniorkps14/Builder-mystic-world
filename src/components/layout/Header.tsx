import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Separator } from "@/components/ui/separator";
import { useLanguage } from "@/contexts/LanguageContext";
import { useROSIntegration } from "@/services/ROSIntegrationService";
import { useToast } from "@/hooks/use-toast";
import {
  Activity,
  Bell,
  Power,
  Settings,
  Wifi,
  WifiOff,
  Clock,
  Cpu,
  HardDrive,
  Cable,
  CableIcon,
  Link,
  Unlink,
} from "lucide-react";

export function Header() {
  const { t } = useLanguage();
  const { isConnected, connect, disconnect, connectionStatus } =
    useROSIntegration();
  const { toast } = useToast();

  // Mock ROS system status
  const systemStatus = {
    roscore: true,
    nodes: 12,
    topics: 24,
    services: 8,
    uptime: "02:34:12",
    cpu: "34%",
    memory: "67%",
    network: true,
  };

  // ROS Bridge connection handlers
  const handleConnectRosBridge = async () => {
    try {
      await connect("ws://localhost:9090");
      toast({
        title: "ROS Bridge Connected",
        description: "Successfully connected to ROS Bridge server",
      });
    } catch (error) {
      toast({
        title: "Connection Failed",
        description: "Failed to connect to ROS Bridge server",
        variant: "destructive",
      });
    }
  };

  const handleDisconnectRosBridge = async () => {
    try {
      await disconnect();
      toast({
        title: "ROS Bridge Disconnected",
        description: "Disconnected from ROS Bridge server",
        variant: "destructive",
      });
    } catch (error) {
      console.error("Disconnect error:", error);
    }
  };

  return (
    <header className="sticky top-0 z-50 w-full border-b border-border/40 bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60">
      <div className="flex h-16 items-center justify-between px-6">
        {/* Left Section - Logo and Status */}
        <div className="flex items-center gap-6">
          <div className="flex items-center gap-3">
            <div className="relative">
              <Activity className="h-8 w-8 text-primary" />
              <div className="absolute -top-1 -right-1 w-3 h-3 bg-ros-success rounded-full animate-pulse-glow" />
            </div>
            <div>
              <h1 className="text-xl font-bold text-foreground">
                {t("app.name")}
              </h1>
              <p className="text-xs text-muted-foreground">
                {t("app.subtitle")}
              </p>
            </div>
          </div>

          <Separator orientation="vertical" className="h-8" />

          {/* System Status Indicators */}
          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2">
              <div
                className={`ros-status-indicator ${systemStatus.roscore ? "ros-status-active" : "ros-status-error"}`}
              />
              <span className="text-sm font-medium">ROScore</span>
            </div>

            <div className="flex items-center gap-2">
              {systemStatus.network ? (
                <Wifi className="h-4 w-4 text-ros-success" />
              ) : (
                <WifiOff className="h-4 w-4 text-ros-error" />
              )}
              <span className="text-sm">Network</span>
            </div>

            <Badge variant="secondary" className="gap-1">
              <Clock className="h-3 w-3" />
              {systemStatus.uptime}
            </Badge>
          </div>
        </div>

        {/* Right Section - System Metrics and Controls */}
        <div className="flex items-center gap-4">
          {/* System Metrics */}
          <div className="hidden md:flex items-center gap-4">
            <div className="flex items-center gap-2 text-sm">
              <Cpu className="h-4 w-4 text-primary" />
              <span>CPU: {systemStatus.cpu}</span>
            </div>

            <div className="flex items-center gap-2 text-sm">
              <HardDrive className="h-4 w-4 text-primary" />
              <span>MEM: {systemStatus.memory}</span>
            </div>

            {/* ROS Bridge Connection */}
            <div className="flex items-center gap-2">
              <div
                className={`w-2 h-2 rounded-full ${isConnected ? "bg-green-500" : "bg-red-500"}`}
              />
              <span className="text-xs text-muted-foreground">
                ROS Bridge: {isConnected ? "Connected" : "Disconnected"}
              </span>
              <Button
                variant={isConnected ? "destructive" : "default"}
                size="sm"
                onClick={
                  isConnected
                    ? handleDisconnectRosBridge
                    : handleConnectRosBridge
                }
                className="h-6 px-2 text-xs ml-1"
              >
                {isConnected ? (
                  <>
                    <Unlink className="h-3 w-3 mr-1" />
                    Disconnect
                  </>
                ) : (
                  <>
                    <Link className="h-3 w-3 mr-1" />
                    Connect
                  </>
                )}
              </Button>
            </div>

            <Separator orientation="vertical" className="h-8" />

            <div className="flex items-center gap-2 text-sm">
              <span className="text-muted-foreground">Nodes:</span>
              <Badge variant="outline">{systemStatus.nodes}</Badge>
            </div>

            <div className="flex items-center gap-2 text-sm">
              <span className="text-muted-foreground">Topics:</span>
              <Badge variant="outline">{systemStatus.topics}</Badge>
            </div>
          </div>

          <Separator orientation="vertical" className="h-8" />

          {/* Control Buttons */}
          <div className="flex items-center gap-2">
            <Button variant="ghost" size="icon">
              <Bell className="h-4 w-4" />
            </Button>

            <Button variant="ghost" size="icon">
              <Settings className="h-4 w-4" />
            </Button>

            <Button variant="destructive" size="icon">
              <Power className="h-4 w-4" />
            </Button>
          </div>
        </div>
      </div>
    </header>
  );
}
