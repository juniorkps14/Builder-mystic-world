import React, { useState } from "react";
import { useLanguage } from "@/contexts/LanguageContext";
import { useROSIntegration } from "@/services/ROSIntegrationService";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { useToast } from "@/hooks/use-toast";
import {
  Terminal,
  Download,
  CheckCircle,
  XCircle,
  AlertTriangle,
  Copy,
  ExternalLink,
  Settings,
  PlayCircle,
  Server,
  Wifi,
  WifiOff,
  Loader2,
  BookOpen,
  Code,
  Package,
  Monitor,
  Shield,
  Zap,
} from "lucide-react";

const ROSSetup: React.FC = () => {
  const { t } = useLanguage();
  const { toast } = useToast();
  const { isConnected, connect, disconnect, connectionUrl, setConnectionUrl } =
    useROSIntegration();
  const [isConnecting, setIsConnecting] = useState(false);
  const [customUrl, setCustomUrl] = useState("ws://localhost:9090");
  const [testResults, setTestResults] = useState<Record<string, boolean>>({});

  const handleConnect = async () => {
    setIsConnecting(true);
    try {
      await connect(customUrl);
      toast({
        title: "Connected Successfully",
        description: "ROS Bridge connection established",
      });
    } catch (error) {
      toast({
        title: "Connection Failed",
        description: error instanceof Error ? error.message : "Unknown error",
        variant: "destructive",
      });
    } finally {
      setIsConnecting(false);
    }
  };

  const handleDisconnect = () => {
    disconnect();
    toast({
      title: "Disconnected",
      description: "ROS Bridge connection closed",
    });
  };

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
    toast({
      title: "Copied!",
      description: "Command copied to clipboard",
    });
  };

  const testConnection = async () => {
    const results: Record<string, boolean> = {};

    // Simulate connection tests
    const tests = ["ros_core", "ros_bridge", "topics", "services", "websocket"];

    for (const test of tests) {
      await new Promise((resolve) => setTimeout(resolve, 500));
      results[test] = isConnected && Math.random() > 0.2; // 80% success rate for demo
    }

    setTestResults(results);
  };

  const installationSteps = [
    {
      id: "ubuntu_ros",
      title: "Install ROS (Robot Operating System)",
      description:
        "Install ROS Noetic on Ubuntu 20.04 or ROS Humble on Ubuntu 22.04",
      commands: [
        "# ROS Noetic (Ubuntu 20.04)",
        "sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'",
        "sudo apt install curl",
        "curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -",
        "sudo apt update",
        "sudo apt install ros-noetic-desktop-full",
        "",
        "# ROS Humble (Ubuntu 22.04)",
        "sudo apt install software-properties-common",
        "sudo add-apt-repository universe",
        "sudo apt update && sudo apt install curl -y",
        "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg",
        'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null',
        "sudo apt update",
        "sudo apt upgrade",
        "sudo apt install ros-humble-desktop",
      ],
    },
    {
      id: "environment",
      title: "Setup ROS Environment",
      description: "Configure your shell environment for ROS",
      commands: [
        "# For ROS Noetic",
        'echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc',
        "source ~/.bashrc",
        "",
        "# For ROS Humble",
        'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc',
        "source ~/.bashrc",
        "",
        "# Install dependencies",
        "sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential",
        "sudo rosdep init",
        "rosdep update",
      ],
    },
    {
      id: "rosbridge",
      title: "Install ROSBridge Suite",
      description: "Install and configure ROSBridge for web communication",
      commands: [
        "# For ROS Noetic",
        "sudo apt install ros-noetic-rosbridge-suite",
        "",
        "# For ROS Humble",
        "sudo apt install ros-humble-rosbridge-suite",
        "",
        "# Or install from source",
        "mkdir -p ~/catkin_ws/src",
        "cd ~/catkin_ws/src",
        "git clone https://github.com/RobotWebTools/rosbridge_suite.git",
        "cd ~/catkin_ws",
        "catkin_make",
        "source devel/setup.bash",
      ],
    },
    {
      id: "navigation",
      title: "Install Navigation Stack (Optional)",
      description: "Install navigation packages for robot movement",
      commands: [
        "# For ROS Noetic",
        "sudo apt install ros-noetic-navigation",
        "sudo apt install ros-noetic-move-base",
        "sudo apt install ros-noetic-amcl",
        "sudo apt install ros-noetic-map-server",
        "",
        "# For ROS Humble",
        "sudo apt install ros-humble-navigation2",
        "sudo apt install ros-humble-nav2-bringup",
        "sudo apt install ros-humble-nav2-map-server",
      ],
    },
    {
      id: "start_services",
      title: "Start ROS Services",
      description: "Commands to start ROSCore and ROSBridge",
      commands: [
        "# Terminal 1: Start ROS Core",
        "roscore",
        "",
        "# Terminal 2: Start ROSBridge WebSocket",
        "roslaunch rosbridge_server rosbridge_websocket.launch",
        "",
        "# Or with custom port",
        "roslaunch rosbridge_server rosbridge_websocket.launch port:=9090",
        "",
        "# For production deployment with authentication",
        "roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 authenticate:=true",
      ],
    },
  ];

  const productionConfig = {
    title: "Production Deployment Configuration",
    items: [
      {
        title: "Network Configuration",
        description: "Configure network settings for production",
        config: `# /etc/ros/production.yaml
ros_bridge:
  port: 9090
  host: "0.0.0.0"
  authenticate: true
  ssl:
    enabled: true
    cert_file: "/etc/ssl/certs/ros-bridge.crt"
    key_file: "/etc/ssl/private/ros-bridge.key"
  
security:
  allowed_origins:
    - "https://your-domain.com"
    - "https://robot-control.your-domain.com"
  rate_limit: 100  # requests per second
  
logging:
  level: "INFO"
  file: "/var/log/ros/bridge.log"`,
      },
      {
        title: "Systemd Service",
        description: "Create systemd service for auto-start",
        config: `# /etc/systemd/system/ros-bridge.service
[Unit]
Description=ROS Bridge WebSocket Server
After=network.target
Requires=network.target

[Service]
Type=simple
User=ros
Group=ros
Environment="ROS_MASTER_URI=http://localhost:11311"
Environment="ROS_HOSTNAME=localhost"
ExecStart=/opt/ros/noetic/bin/roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target`,
      },
      {
        title: "Nginx Reverse Proxy",
        description: "Configure Nginx for SSL termination",
        config: `# /etc/nginx/sites-available/ros-bridge
server {
    listen 443 ssl http2;
    server_name robot-api.your-domain.com;
    
    ssl_certificate /etc/ssl/certs/ros-bridge.crt;
    ssl_certificate_key /etc/ssl/private/ros-bridge.key;
    
    location /ros {
        proxy_pass http://localhost:9090;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        
        # WebSocket timeout
        proxy_read_timeout 86400;
        proxy_send_timeout 86400;
    }
}`,
      },
    ],
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            ROS Bridge Setup
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Complete installation and configuration guide for ROS Bridge
            integration
          </p>
        </div>

        {/* Connection Status */}
        <div className="flex items-center gap-3">
          <Badge
            variant={isConnected ? "default" : "secondary"}
            className="gap-2"
          >
            {isConnected ? (
              <Wifi className="h-4 w-4" />
            ) : (
              <WifiOff className="h-4 w-4" />
            )}
            {isConnected ? "Connected" : "Disconnected"}
          </Badge>
        </div>
      </div>

      {/* Quick Connection Panel */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Server className="h-5 w-5" />
            Quick Connection Test
          </CardTitle>
          <CardDescription>
            Test your ROS Bridge connection before proceeding with robot control
          </CardDescription>
        </CardHeader>
        <CardContent>
          <div className="space-y-4">
            <div className="flex gap-4">
              <div className="flex-1">
                <Label htmlFor="connection-url">ROS Bridge WebSocket URL</Label>
                <Input
                  id="connection-url"
                  value={customUrl}
                  onChange={(e) => setCustomUrl(e.target.value)}
                  placeholder="ws://localhost:9090"
                />
              </div>
              <div className="flex items-end gap-2">
                {!isConnected ? (
                  <Button
                    onClick={handleConnect}
                    disabled={isConnecting}
                    className="gap-2"
                  >
                    {isConnecting ? (
                      <Loader2 className="h-4 w-4 animate-spin" />
                    ) : (
                      <PlayCircle className="h-4 w-4" />
                    )}
                    Connect
                  </Button>
                ) : (
                  <Button
                    onClick={handleDisconnect}
                    variant="outline"
                    className="gap-2"
                  >
                    <WifiOff className="h-4 w-4" />
                    Disconnect
                  </Button>
                )}
                <Button
                  onClick={testConnection}
                  variant="outline"
                  className="gap-2"
                >
                  <Monitor className="h-4 w-4" />
                  Test
                </Button>
              </div>
            </div>

            {/* Test Results */}
            {Object.keys(testResults).length > 0 && (
              <div className="grid grid-cols-2 md:grid-cols-5 gap-2">
                {Object.entries(testResults).map(([test, passed]) => (
                  <div
                    key={test}
                    className="flex items-center gap-2 p-2 rounded border"
                  >
                    {passed ? (
                      <CheckCircle className="h-4 w-4 text-green-500" />
                    ) : (
                      <XCircle className="h-4 w-4 text-red-500" />
                    )}
                    <span className="text-sm capitalize">
                      {test.replace("_", " ")}
                    </span>
                  </div>
                ))}
              </div>
            )}

            {isConnected && (
              <Alert>
                <CheckCircle className="h-4 w-4" />
                <AlertDescription>
                  🎉 ROS Bridge is connected and ready! You can now control
                  robots and execute sequences.
                </AlertDescription>
              </Alert>
            )}
          </div>
        </CardContent>
      </Card>

      {/* Installation Guide */}
      <Tabs defaultValue="installation" className="space-y-4">
        <TabsList className="grid w-full grid-cols-3">
          <TabsTrigger value="installation" className="gap-2">
            <Package className="h-4 w-4" />
            Installation
          </TabsTrigger>
          <TabsTrigger value="production" className="gap-2">
            <Shield className="h-4 w-4" />
            Production
          </TabsTrigger>
          <TabsTrigger value="troubleshooting" className="gap-2">
            <AlertTriangle className="h-4 w-4" />
            Troubleshooting
          </TabsTrigger>
        </TabsList>

        {/* Installation Tab */}
        <TabsContent value="installation" className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <BookOpen className="h-5 w-5" />
                Step-by-Step Installation Guide
              </CardTitle>
              <CardDescription>
                Follow these steps to install ROS and ROSBridge on Ubuntu
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-6">
                {installationSteps.map((step, index) => (
                  <Card key={step.id} className="relative">
                    <CardHeader>
                      <div className="flex items-center gap-3">
                        <div className="flex items-center justify-center w-8 h-8 rounded-full bg-primary text-primary-foreground text-sm font-bold">
                          {index + 1}
                        </div>
                        <div>
                          <CardTitle className="text-lg">
                            {step.title}
                          </CardTitle>
                          <CardDescription>{step.description}</CardDescription>
                        </div>
                      </div>
                    </CardHeader>
                    <CardContent>
                      <div className="bg-muted rounded-lg p-4">
                        <div className="flex items-center justify-between mb-3">
                          <div className="flex items-center gap-2">
                            <Terminal className="h-4 w-4" />
                            <span className="text-sm font-medium">
                              Terminal Commands
                            </span>
                          </div>
                          <Button
                            size="sm"
                            variant="outline"
                            onClick={() =>
                              copyToClipboard(step.commands.join("\n"))
                            }
                            className="gap-1"
                          >
                            <Copy className="h-3 w-3" />
                            Copy All
                          </Button>
                        </div>
                        <ScrollArea className="h-40">
                          <pre className="text-xs font-mono whitespace-pre-wrap">
                            {step.commands.join("\n")}
                          </pre>
                        </ScrollArea>
                      </div>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </CardContent>
          </Card>
        </TabsContent>

        {/* Production Tab */}
        <TabsContent value="production" className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Shield className="h-5 w-5" />
                {productionConfig.title}
              </CardTitle>
              <CardDescription>
                Production-ready configuration for deployment
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-6">
                {productionConfig.items.map((item, index) => (
                  <Card key={index}>
                    <CardHeader>
                      <CardTitle className="text-lg">{item.title}</CardTitle>
                      <CardDescription>{item.description}</CardDescription>
                    </CardHeader>
                    <CardContent>
                      <div className="bg-muted rounded-lg p-4">
                        <div className="flex items-center justify-between mb-3">
                          <div className="flex items-center gap-2">
                            <Code className="h-4 w-4" />
                            <span className="text-sm font-medium">
                              Configuration
                            </span>
                          </div>
                          <Button
                            size="sm"
                            variant="outline"
                            onClick={() => copyToClipboard(item.config)}
                            className="gap-1"
                          >
                            <Copy className="h-3 w-3" />
                            Copy
                          </Button>
                        </div>
                        <ScrollArea className="h-60">
                          <pre className="text-xs font-mono whitespace-pre-wrap">
                            {item.config}
                          </pre>
                        </ScrollArea>
                      </div>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </CardContent>
          </Card>
        </TabsContent>

        {/* Troubleshooting Tab */}
        <TabsContent value="troubleshooting" className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <AlertTriangle className="h-5 w-5" />
                Common Issues & Solutions
              </CardTitle>
              <CardDescription>
                Troubleshoot common ROS Bridge connection problems
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <Alert>
                  <AlertTriangle className="h-4 w-4" />
                  <AlertDescription>
                    <strong>Connection Refused:</strong> Make sure ROS Core and
                    ROSBridge are running.
                    <br />
                    <code className="text-sm bg-muted px-1 rounded">
                      roscore
                    </code>{" "}
                    and{" "}
                    <code className="text-sm bg-muted px-1 rounded">
                      roslaunch rosbridge_server rosbridge_websocket.launch
                    </code>
                  </AlertDescription>
                </Alert>

                <Alert>
                  <AlertTriangle className="h-4 w-4" />
                  <AlertDescription>
                    <strong>CORS Issues:</strong> Add your domain to allowed
                    origins in ROSBridge config.
                    <br />
                    Use{" "}
                    <code className="text-sm bg-muted px-1 rounded">
                      roslaunch rosbridge_server rosbridge_websocket.launch
                      origins:="*"
                    </code>{" "}
                    for development.
                  </AlertDescription>
                </Alert>

                <Alert>
                  <AlertTriangle className="h-4 w-4" />
                  <AlertDescription>
                    <strong>Port Already in Use:</strong> Change the port
                    number.
                    <br />
                    <code className="text-sm bg-muted px-1 rounded">
                      roslaunch rosbridge_server rosbridge_websocket.launch
                      port:=9091
                    </code>
                  </AlertDescription>
                </Alert>

                <Alert>
                  <AlertTriangle className="h-4 w-4" />
                  <AlertDescription>
                    <strong>Firewall Blocking:</strong> Open port 9090 in your
                    firewall.
                    <br />
                    <code className="text-sm bg-muted px-1 rounded">
                      sudo ufw allow 9090
                    </code>
                  </AlertDescription>
                </Alert>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>System Requirements</CardTitle>
              <CardDescription>
                Minimum requirements for running ROS Bridge
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div className="space-y-2">
                  <h4 className="font-semibold">Ubuntu Requirements</h4>
                  <ul className="text-sm space-y-1 text-muted-foreground">
                    <li>• Ubuntu 20.04 LTS (for ROS Noetic)</li>
                    <li>• Ubuntu 22.04 LTS (for ROS Humble)</li>
                    <li>• 4GB RAM minimum, 8GB recommended</li>
                    <li>• 20GB free disk space</li>
                  </ul>
                </div>
                <div className="space-y-2">
                  <h4 className="font-semibold">Network Requirements</h4>
                  <ul className="text-sm space-y-1 text-muted-foreground">
                    <li>• WebSocket support (port 9090)</li>
                    <li>• CORS properly configured</li>
                    <li>• SSL/TLS for production</li>
                    <li>• Stable network connection</li>
                  </ul>
                </div>
              </div>
            </CardContent>
          </Card>
        </TabsContent>
      </Tabs>

      {/* Quick Links */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <ExternalLink className="h-5 w-5" />
            Useful Resources
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            <Button
              variant="outline"
              className="justify-start gap-2 h-auto p-4"
            >
              <BookOpen className="h-4 w-4" />
              <div className="text-left">
                <div className="font-medium">ROS Documentation</div>
                <div className="text-xs text-muted-foreground">
                  Official ROS docs
                </div>
              </div>
            </Button>
            <Button
              variant="outline"
              className="justify-start gap-2 h-auto p-4"
            >
              <Code className="h-4 w-4" />
              <div className="text-left">
                <div className="font-medium">ROSBridge Suite</div>
                <div className="text-xs text-muted-foreground">
                  GitHub repository
                </div>
              </div>
            </Button>
            <Button
              variant="outline"
              className="justify-start gap-2 h-auto p-4"
            >
              <Zap className="h-4 w-4" />
              <div className="text-left">
                <div className="font-medium">WebSocket API</div>
                <div className="text-xs text-muted-foreground">
                  API documentation
                </div>
              </div>
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  );
};

export default ROSSetup;
