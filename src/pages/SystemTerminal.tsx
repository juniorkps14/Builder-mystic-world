import React, { useState, useEffect, useRef, useCallback } from "react";
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
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useToast } from "@/hooks/use-toast";
import {
  Terminal,
  Play,
  Pause,
  Square,
  Trash2,
  Download,
  Upload,
  Power,
  Settings,
  RefreshCw,
  Copy,
  Save,
  AlertCircle,
  CheckCircle,
  Clock,
  Zap,
  Cpu,
  HardDrive,
  Wifi,
  Database,
  Activity,
} from "lucide-react";

interface CommandHistory {
  id: string;
  command: string;
  output: string;
  timestamp: Date;
  status: "success" | "error" | "running";
  executionTime?: number;
}

interface SystemStatus {
  cpu: number;
  memory: number;
  disk: number;
  temperature: number;
  processes: number;
  uptime: string;
}

const SystemTerminal: React.FC = () => {
  const { t } = useLanguage();
  const { toast } = useToast();
  const terminalRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  const [currentCommand, setCurrentCommand] = useState("");
  const [commandHistory, setCommandHistory] = useState<CommandHistory[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [isExecuting, setIsExecuting] = useState(false);
  const [systemStatus, setSystemStatus] = useState<SystemStatus>({
    cpu: 45,
    memory: 67,
    disk: 34,
    temperature: 42,
    processes: 156,
    uptime: "2d 14h 32m",
  });

  const { isConnected, publish, subscribe, callService } = useROSIntegration();

  // Predefined commands for quick access
  const quickCommands = [
    {
      category: "ROS Commands",
      commands: [
        { name: "List Topics", cmd: "rostopic list" },
        { name: "List Nodes", cmd: "rosnode list" },
        { name: "ROS Info", cmd: "rosnode info" },
        { name: "Kill Node", cmd: "rosnode kill" },
      ],
    },
    {
      category: "System Commands",
      commands: [
        { name: "System Info", cmd: "uname -a" },
        { name: "CPU Info", cmd: "lscpu" },
        { name: "Memory Info", cmd: "free -h" },
        { name: "Disk Usage", cmd: "df -h" },
        { name: "Process List", cmd: "ps aux" },
        { name: "Network Status", cmd: "ifconfig" },
      ],
    },
    {
      category: "Docker Commands",
      commands: [
        { name: "List Containers", cmd: "docker ps" },
        { name: "List Images", cmd: "docker images" },
        { name: "System Info", cmd: "docker info" },
        { name: "Container Stats", cmd: "docker stats" },
      ],
    },
    {
      category: "Service Control",
      commands: [
        { name: "Start ROS Core", cmd: "roscore" },
        { name: "Launch Robot", cmd: "roslaunch robot main.launch" },
        { name: "Stop All", cmd: "pkill -f ros" },
        { name: "Restart Network", cmd: "sudo systemctl restart networking" },
      ],
    },
  ];

  // Simulate system command execution
  const executeCommand = useCallback(
    async (command: string) => {
      if (!command.trim()) return;

      const commandId = Date.now().toString();
      const startTime = Date.now();

      const newCommand: CommandHistory = {
        id: commandId,
        command: command.trim(),
        output: "",
        timestamp: new Date(),
        status: "running",
      };

      setCommandHistory((prev) => [...prev, newCommand]);
      setIsExecuting(true);

      try {
        // Simulate command execution with different responses
        let output = "";
        let status: "success" | "error" = "success";

        // ROS Commands
        if (command.includes("rostopic list")) {
          output = `/cmd_vel
/scan
/odom
/tf
/tf_static
/map
/amcl_pose
/move_base/goal
/move_base/status
/joint_states
/imu/data`;
        } else if (command.includes("rosnode list")) {
          output = `/rosout
/move_base
/amcl
/map_server
/robot_state_publisher
/joint_state_publisher
/laser_scan_matcher
/tf2_ros`;
        } else if (command.includes("rosnode info")) {
          output = `Node [/move_base]
Publications:
 * /cmd_vel [geometry_msgs/Twist]
 * /move_base/feedback [move_base_msgs/MoveBaseActionFeedback]
 * /move_base/status [actionlib_msgs/GoalStatusArray]

Subscriptions:
 * /scan [sensor_msgs/LaserScan]
 * /tf [tf2_msgs/TFMessage]
 * /map [nav_msgs/OccupancyGrid]

Services:
 * /move_base/clear_costmaps
 * /move_base/make_plan`;
        }
        // System Commands
        else if (command.includes("uname -a")) {
          output = `Linux dino-robot 5.15.0-78-generic #85-Ubuntu SMP Fri Jul 7 15:25:09 UTC 2023 x86_64 x86_64 x86_64 GNU/Linux`;
        } else if (command.includes("free -h")) {
          output = `              total        used        free      shared  buff/cache   available
Mem:           7.7G        2.1G        3.2G        234M        2.4G        5.1G
Swap:          2.0G          0B        2.0G`;
        } else if (command.includes("df -h")) {
          output = `Filesystem      Size  Used Avail Use% Mounted on
/dev/sda1       458G  156G  279G  36% /
tmpfs           3.9G     0  3.9G   0% /dev/shm
/dev/sda2       477M  124M  329M  28% /boot`;
        } else if (command.includes("ps aux")) {
          output = `USER       PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root         1  0.0  0.1 168648 11584 ?        Ss   Jul07   0:04 /sbin/init
dino      1234  2.1  5.2 2847364 407896 ?      Sl   Jul07  45:23 /opt/ros/noetic/bin/roscore
dino      1456  1.8  3.4 1234567 267890 ?      Sl   Jul07  32:15 /opt/ros/noetic/bin/roslaunch robot main.launch`;
        } else if (command.includes("docker ps")) {
          output = `CONTAINER ID   IMAGE                    COMMAND                  CREATED          STATUS          PORTS                                       NAMES
a1b2c3d4e5f6   ros:noetic-desktop      "/ros_entrypoint.sh …"   2 hours ago      Up 2 hours      11311/tcp                                   ros_master
f6e5d4c3b2a1   dino/robot:latest       "/entrypoint.sh"         2 hours ago      Up 2 hours      0.0.0.0:8080->8080/tcp                     robot_control`;
        }
        // ROS Launch Commands
        else if (command.includes("roscore")) {
          output = `... logging to /home/dino/.ros/log/12345678-abcd-1234-5678-123456789abc/roslaunch-dino-robot-1234.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:11311/
ros_comm version 1.15.14

SUMMARY
========
PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.14

NODES

auto-starting new master
process[master]: started with pid [1234]
ROS_MASTER_URI=http://localhost:11311/

setting /run_id to 12345678-abcd-1234-5678-123456789abc
process[rosout-1]: started with pid [1235]
started core service [/rosout]`;
        } else if (command.includes("roslaunch")) {
          output = `... logging to /home/dino/.ros/log/12345678-abcd-1234-5678-123456789abc/roslaunch-dino-robot-1456.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:11311/

SUMMARY
========
PARAMETERS
 * /robot/base_frame: base_link
 * /robot/odom_frame: odom
 * /robot/map_frame: map

NODES
  /
    move_base (move_base/move_base)
    amcl (amcl/amcl)
    map_server (map_server/map_server)

auto-starting new master
process[move_base-1]: started with pid [1456]
process[amcl-2]: started with pid [1457]
process[map_server-3]: started with pid [1458]`;
        } else if (command.includes("pkill")) {
          output = `Stopping all ROS processes...
Terminated: roscore (PID: 1234)
Terminated: roslaunch (PID: 1456)
Terminated: move_base (PID: 1457)
All ROS processes stopped successfully.`;
        }
        // Error simulation
        else if (command.includes("sudo") && !command.includes("systemctl")) {
          output = `sudo: ${command}: command not found`;
          status = "error";
        } else if (command.includes("rm -rf")) {
          output = `Error: Dangerous command blocked for safety`;
          status = "error";
        } else {
          // Default response for unknown commands
          output = `Command executed: ${command}
Output: [Simulated command execution]
Status: Completed successfully`;
        }

        // Simulate execution delay
        await new Promise((resolve) =>
          setTimeout(resolve, 1000 + Math.random() * 2000),
        );

        const executionTime = Date.now() - startTime;

        setCommandHistory((prev) =>
          prev.map((cmd) =>
            cmd.id === commandId
              ? { ...cmd, output, status, executionTime }
              : cmd,
          ),
        );

        if (status === "success") {
          toast({
            title: "Command Executed",
            description: `${command} completed successfully`,
          });
        } else {
          toast({
            title: "Command Failed",
            description: `${command} failed to execute`,
            variant: "destructive",
          });
        }
      } catch (error) {
        setCommandHistory((prev) =>
          prev.map((cmd) =>
            cmd.id === commandId
              ? {
                  ...cmd,
                  output: `Error: ${error}`,
                  status: "error" as const,
                  executionTime: Date.now() - startTime,
                }
              : cmd,
          ),
        );
      } finally {
        setIsExecuting(false);
      }
    },
    [toast],
  );

  // Handle command input
  const handleCommandSubmit = useCallback(
    (e: React.FormEvent) => {
      e.preventDefault();
      if (currentCommand.trim() && !isExecuting) {
        executeCommand(currentCommand);
        setCurrentCommand("");
        setHistoryIndex(-1);
      }
    },
    [currentCommand, isExecuting, executeCommand],
  );

  // Handle keyboard navigation
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      if (e.key === "ArrowUp") {
        e.preventDefault();
        const newIndex = Math.min(historyIndex + 1, commandHistory.length - 1);
        if (newIndex >= 0) {
          setHistoryIndex(newIndex);
          setCurrentCommand(
            commandHistory[commandHistory.length - 1 - newIndex].command,
          );
        }
      } else if (e.key === "ArrowDown") {
        e.preventDefault();
        const newIndex = Math.max(historyIndex - 1, -1);
        setHistoryIndex(newIndex);
        if (newIndex === -1) {
          setCurrentCommand("");
        } else {
          setCurrentCommand(
            commandHistory[commandHistory.length - 1 - newIndex].command,
          );
        }
      }
    },
    [historyIndex, commandHistory],
  );

  // Auto-scroll to bottom
  useEffect(() => {
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [commandHistory]);

  // Update system status
  useEffect(() => {
    const interval = setInterval(() => {
      setSystemStatus((prev) => ({
        ...prev,
        cpu: Math.max(20, Math.min(80, prev.cpu + (Math.random() - 0.5) * 10)),
        memory: Math.max(
          30,
          Math.min(90, prev.memory + (Math.random() - 0.5) * 5),
        ),
        temperature: Math.max(
          35,
          Math.min(60, prev.temperature + (Math.random() - 0.5) * 3),
        ),
      }));
    }, 3000);

    return () => clearInterval(interval);
  }, []);

  const clearHistory = () => {
    setCommandHistory([]);
  };

  const copyCommand = (command: string) => {
    navigator.clipboard.writeText(command);
    toast({
      title: "Copied",
      description: "Command copied to clipboard",
    });
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            {t("terminal.title")}
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            {t("terminal.subtitle")}
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge
            variant={isConnected ? "default" : "secondary"}
            className="gap-2"
          >
            <div
              className={`w-2 h-2 rounded-full ${isConnected ? "bg-green-400" : "bg-gray-400"}`}
            />
            {isConnected ? "System Connected" : "System Disconnected"}
          </Badge>

          <Button variant="outline" onClick={clearHistory}>
            <Trash2 className="h-4 w-4 mr-2" />
            Clear
          </Button>
        </div>
      </div>

      {/* System Status Cards */}
      <div className="grid grid-cols-1 md:grid-cols-6 gap-4">
        <Card className="glass-effect">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-blue-500/10 rounded-lg">
                <Cpu className="h-5 w-5 text-blue-500" />
              </div>
              <div>
                <div className="text-sm text-muted-foreground">CPU</div>
                <div className="text-lg font-semibold">{systemStatus.cpu}%</div>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="glass-effect">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-green-500/10 rounded-lg">
                <HardDrive className="h-5 w-5 text-green-500" />
              </div>
              <div>
                <div className="text-sm text-muted-foreground">Memory</div>
                <div className="text-lg font-semibold">
                  {systemStatus.memory}%
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="glass-effect">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-purple-500/10 rounded-lg">
                <Database className="h-5 w-5 text-purple-500" />
              </div>
              <div>
                <div className="text-sm text-muted-foreground">Disk</div>
                <div className="text-lg font-semibold">
                  {systemStatus.disk}%
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="glass-effect">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-orange-500/10 rounded-lg">
                <Zap className="h-5 w-5 text-orange-500" />
              </div>
              <div>
                <div className="text-sm text-muted-foreground">Temp</div>
                <div className="text-lg font-semibold">
                  {systemStatus.temperature}°C
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="glass-effect">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-indigo-500/10 rounded-lg">
                <Activity className="h-5 w-5 text-indigo-500" />
              </div>
              <div>
                <div className="text-sm text-muted-foreground">Processes</div>
                <div className="text-lg font-semibold">
                  {systemStatus.processes}
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="glass-effect">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-emerald-500/10 rounded-lg">
                <Clock className="h-5 w-5 text-emerald-500" />
              </div>
              <div>
                <div className="text-sm text-muted-foreground">Uptime</div>
                <div className="text-lg font-semibold">
                  {systemStatus.uptime}
                </div>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Quick Commands Panel */}
        <Card className="lg:col-span-1 glass-effect">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Zap className="h-5 w-5" />
              Quick Commands
            </CardTitle>
            <CardDescription>Execute common system commands</CardDescription>
          </CardHeader>
          <CardContent>
            <ScrollArea className="h-[600px]">
              <div className="space-y-4">
                {quickCommands.map((category) => (
                  <div key={category.category}>
                    <h4 className="font-medium text-sm mb-2 text-muted-foreground">
                      {category.category}
                    </h4>
                    <div className="space-y-1">
                      {category.commands.map((cmd) => (
                        <Button
                          key={cmd.name}
                          variant="ghost"
                          size="sm"
                          className="w-full justify-start h-auto p-2 text-left"
                          onClick={() => setCurrentCommand(cmd.cmd)}
                        >
                          <div>
                            <div className="font-medium text-xs">
                              {cmd.name}
                            </div>
                            <div className="text-xs text-muted-foreground font-mono">
                              {cmd.cmd}
                            </div>
                          </div>
                        </Button>
                      ))}
                    </div>
                  </div>
                ))}
              </div>
            </ScrollArea>
          </CardContent>
        </Card>

        {/* Terminal Panel */}
        <Card className="lg:col-span-3 glass-effect">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Terminal className="h-5 w-5" />
              System Terminal
            </CardTitle>
            <CardDescription>
              Execute system commands directly on the robot
            </CardDescription>
          </CardHeader>
          <CardContent>
            {/* Terminal Output */}
            <div className="mb-4">
              <ScrollArea className="h-[500px] bg-gray-900 rounded-lg p-4 font-mono text-sm">
                <div ref={terminalRef} className="space-y-2">
                  {commandHistory.map((entry) => (
                    <div key={entry.id} className="space-y-1">
                      <div className="flex items-center gap-2">
                        <span className="text-green-400">dino@robot:~$</span>
                        <span className="text-white">{entry.command}</span>
                        <div className="flex items-center gap-1 ml-auto">
                          {entry.status === "running" && (
                            <RefreshCw className="h-3 w-3 text-yellow-400 animate-spin" />
                          )}
                          {entry.status === "success" && (
                            <CheckCircle className="h-3 w-3 text-green-400" />
                          )}
                          {entry.status === "error" && (
                            <AlertCircle className="h-3 w-3 text-red-400" />
                          )}
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={() => copyCommand(entry.command)}
                            className="h-4 w-4 p-0 text-gray-400 hover:text-white"
                          >
                            <Copy className="h-3 w-3" />
                          </Button>
                        </div>
                      </div>
                      {entry.output && (
                        <pre className="text-gray-300 whitespace-pre-wrap pl-4 border-l-2 border-gray-700">
                          {entry.output}
                        </pre>
                      )}
                      {entry.executionTime && (
                        <div className="text-xs text-gray-500 pl-4">
                          Executed in {entry.executionTime}ms
                        </div>
                      )}
                    </div>
                  ))}
                  {isExecuting && (
                    <div className="flex items-center gap-2 text-yellow-400">
                      <RefreshCw className="h-4 w-4 animate-spin" />
                      <span>Executing command...</span>
                    </div>
                  )}
                </div>
              </ScrollArea>
            </div>

            {/* Command Input */}
            <form onSubmit={handleCommandSubmit} className="flex gap-2">
              <div className="flex-1 relative">
                <div className="absolute left-3 top-1/2 transform -translate-y-1/2 text-green-400 font-mono text-sm">
                  dino@robot:~$
                </div>
                <Input
                  ref={inputRef}
                  value={currentCommand}
                  onChange={(e) => setCurrentCommand(e.target.value)}
                  onKeyDown={handleKeyDown}
                  placeholder="Enter system command..."
                  className="pl-24 font-mono bg-gray-900 border-gray-700 text-white"
                  disabled={isExecuting}
                />
              </div>
              <Button
                type="submit"
                disabled={!currentCommand.trim() || isExecuting}
                className="gap-2"
              >
                {isExecuting ? (
                  <RefreshCw className="h-4 w-4 animate-spin" />
                ) : (
                  <Play className="h-4 w-4" />
                )}
                Execute
              </Button>
            </form>

            <div className="text-xs text-muted-foreground mt-2">
              Use ↑↓ arrows to navigate command history. Click quick commands to
              auto-fill.
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
};

export default SystemTerminal;
