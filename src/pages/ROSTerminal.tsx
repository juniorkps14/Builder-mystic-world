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
  Terminal,
  Play,
  Pause,
  Settings,
  Wifi,
  WifiOff,
  Send,
  History,
  Trash2,
  Download,
  RefreshCw,
  Zap,
  CheckCircle,
  AlertTriangle,
  Clock,
} from "lucide-react";

interface TerminalEntry {
  id: string;
  timestamp: Date;
  type: "command" | "response" | "error" | "info";
  content: string;
  source?: string;
}

interface ROSConnection {
  ip: string;
  port: number;
  isConnected: boolean;
  lastPing: Date | null;
  websocket: WebSocket | null;
}

const ROSTerminal = () => {
  const { t } = useLanguage();
  const scrollAreaRef = useRef<HTMLDivElement>(null);
  const commandInputRef = useRef<HTMLInputElement>(null);

  const [terminalHistory, setTerminalHistory] = useState<TerminalEntry[]>([
    {
      id: "init_1",
      timestamp: new Date(),
      type: "info",
      content: "ROS Terminal initialized. Type 'help' for available commands.",
    },
    {
      id: "init_2",
      timestamp: new Date(),
      type: "info",
      content: "Use 'connect' to establish RosBridge WebSocket connection.",
    },
  ]);

  const [currentCommand, setCurrentCommand] = useState("");
  const [commandHistory, setCommandHistory] = useState<string[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [isAutoScroll, setIsAutoScroll] = useState(true);

  // ROS Connection state
  const [rosConnection, setRosConnection] = useState<ROSConnection>({
    ip: "localhost",
    port: 9090,
    isConnected: false,
    lastPing: null,
    websocket: null,
  });

  // ROS Master Configuration
  const [rosMasterConfig, setRosMasterConfig] = useState({
    masterIP: "localhost",
    masterPort: 11311,
    bridgeIP: "localhost",
    bridgePort: 9090,
    autoConnect: false,
  });

  // Auto-scroll to bottom when new entries are added
  useEffect(() => {
    if (isAutoScroll && scrollAreaRef.current) {
      const scrollElement = scrollAreaRef.current.querySelector(
        "[data-radix-scroll-area-viewport]",
      );
      if (scrollElement) {
        scrollElement.scrollTop = scrollElement.scrollHeight;
      }
    }
  }, [terminalHistory, isAutoScroll]);

  // RosBridge WebSocket connection
  const connectToRosBridge = () => {
    try {
      const wsUrl = `ws://${rosConnection.ip}:${rosConnection.port}`;
      const ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        setRosConnection((prev) => ({
          ...prev,
          isConnected: true,
          websocket: ws,
          lastPing: new Date(),
        }));

        addTerminalEntry("info", `Connected to RosBridge at ${wsUrl}`);

        // Send initial ping
        ws.send(
          JSON.stringify({
            op: "call_service",
            service: "/rosapi/get_time",
          }),
        );
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          addTerminalEntry("response", JSON.stringify(data, null, 2));
          setRosConnection((prev) => ({ ...prev, lastPing: new Date() }));
        } catch (e) {
          addTerminalEntry("response", event.data);
        }
      };

      ws.onclose = () => {
        setRosConnection((prev) => ({
          ...prev,
          isConnected: false,
          websocket: null,
        }));
        addTerminalEntry("error", "RosBridge connection closed");
      };

      ws.onerror = (error) => {
        addTerminalEntry("error", `WebSocket error: ${error}`);
      };
    } catch (error) {
      addTerminalEntry("error", `Failed to connect: ${error}`);
    }
  };

  const disconnectFromRosBridge = () => {
    if (rosConnection.websocket) {
      rosConnection.websocket.close();
    }
  };

  const addTerminalEntry = (
    type: TerminalEntry["type"],
    content: string,
    source?: string,
  ) => {
    const entry: TerminalEntry = {
      id: `entry_${Date.now()}_${Math.random()}`,
      timestamp: new Date(),
      type,
      content,
      source,
    };
    setTerminalHistory((prev) => [...prev, entry]);
  };

  const executeCommand = (command: string) => {
    if (!command.trim()) return;

    // Add command to history
    addTerminalEntry("command", command);
    setCommandHistory((prev) => [command, ...prev.slice(0, 49)]);
    setCurrentCommand("");
    setHistoryIndex(-1);

    // Parse and execute command
    const parts = command.trim().split(" ");
    const cmd = parts[0].toLowerCase();
    const args = parts.slice(1);

    switch (cmd) {
      case "help":
        executeHelpCommand();
        break;
      case "connect":
        connectToRosBridge();
        break;
      case "disconnect":
        disconnectFromRosBridge();
        break;
      case "clear":
        setTerminalHistory([]);
        break;
      case "topics":
        executeTopicsCommand(args);
        break;
      case "nodes":
        executeNodesCommand();
        break;
      case "services":
        executeServicesCommand();
        break;
      case "pub":
        executePublishCommand(args);
        break;
      case "sub":
        executeSubscribeCommand(args);
        break;
      case "call":
        executeServiceCall(args);
        break;
      case "set":
        executeSetCommand(args);
        break;
      case "status":
        executeStatusCommand();
        break;
      default:
        addTerminalEntry("error", `Unknown command: ${cmd}`);
        addTerminalEntry("info", "Type 'help' for available commands");
    }
  };

  const executeHelpCommand = () => {
    const helpText = `Available Commands:
  help                  - Show this help message
  connect               - Connect to RosBridge WebSocket
  disconnect            - Disconnect from RosBridge
  clear                 - Clear terminal history
  status                - Show connection status
  
ROS Commands (requires connection):
  topics [list]         - List available topics
  nodes                 - List active nodes
  services              - List available services
  pub <topic> <msg>     - Publish message to topic
  sub <topic>           - Subscribe to topic
  call <service> <args> - Call ROS service
  set master <ip:port>  - Set ROS Master address
  
Examples:
  pub /cmd_vel '{"linear":{"x":0.5},"angular":{"z":0.1}}'
  sub /odom
  call /move_base/make_plan '{"start":{"pose":{"position":{"x":0}}}}'`;

    addTerminalEntry("info", helpText);
  };

  const executeTopicsCommand = (args: string[]) => {
    if (!rosConnection.isConnected) {
      addTerminalEntry("error", "Not connected to RosBridge");
      return;
    }

    const message = {
      op: "call_service",
      service: "/rosapi/topics",
    };

    rosConnection.websocket?.send(JSON.stringify(message));
    addTerminalEntry("info", "Fetching topics list...");
  };

  const executeNodesCommand = () => {
    if (!rosConnection.isConnected) {
      addTerminalEntry("error", "Not connected to RosBridge");
      return;
    }

    const message = {
      op: "call_service",
      service: "/rosapi/nodes",
    };

    rosConnection.websocket?.send(JSON.stringify(message));
    addTerminalEntry("info", "Fetching nodes list...");
  };

  const executeServicesCommand = () => {
    if (!rosConnection.isConnected) {
      addTerminalEntry("error", "Not connected to RosBridge");
      return;
    }

    const message = {
      op: "call_service",
      service: "/rosapi/services",
    };

    rosConnection.websocket?.send(JSON.stringify(message));
    addTerminalEntry("info", "Fetching services list...");
  };

  const executePublishCommand = (args: string[]) => {
    if (!rosConnection.isConnected) {
      addTerminalEntry("error", "Not connected to RosBridge");
      return;
    }

    if (args.length < 2) {
      addTerminalEntry("error", "Usage: pub <topic> <message_json>");
      return;
    }

    try {
      const topic = args[0];
      const messageData = JSON.parse(args.slice(1).join(" "));

      const message = {
        op: "publish",
        topic: topic,
        msg: messageData,
      };

      rosConnection.websocket?.send(JSON.stringify(message));
      addTerminalEntry("info", `Published to ${topic}`);
    } catch (error) {
      addTerminalEntry("error", `Invalid JSON message: ${error}`);
    }
  };

  const executeSubscribeCommand = (args: string[]) => {
    if (!rosConnection.isConnected) {
      addTerminalEntry("error", "Not connected to RosBridge");
      return;
    }

    if (args.length < 1) {
      addTerminalEntry("error", "Usage: sub <topic>");
      return;
    }

    const topic = args[0];
    const message = {
      op: "subscribe",
      topic: topic,
    };

    rosConnection.websocket?.send(JSON.stringify(message));
    addTerminalEntry("info", `Subscribed to ${topic}`);
  };

  const executeServiceCall = (args: string[]) => {
    if (!rosConnection.isConnected) {
      addTerminalEntry("error", "Not connected to RosBridge");
      return;
    }

    if (args.length < 1) {
      addTerminalEntry("error", "Usage: call <service> [args_json]");
      return;
    }

    try {
      const service = args[0];
      const serviceArgs =
        args.length > 1 ? JSON.parse(args.slice(1).join(" ")) : {};

      const message = {
        op: "call_service",
        service: service,
        args: serviceArgs,
      };

      rosConnection.websocket?.send(JSON.stringify(message));
      addTerminalEntry("info", `Calling service ${service}...`);
    } catch (error) {
      addTerminalEntry("error", `Invalid service args: ${error}`);
    }
  };

  const executeSetCommand = (args: string[]) => {
    if (args.length < 2) {
      addTerminalEntry("error", "Usage: set master <ip:port>");
      return;
    }

    if (args[0] === "master") {
      const [ip, port] = args[1].split(":");
      setRosMasterConfig((prev) => ({
        ...prev,
        masterIP: ip || "localhost",
        masterPort: parseInt(port) || 11311,
      }));
      addTerminalEntry("info", `ROS Master set to ${ip}:${port}`);
    }
  };

  const executeStatusCommand = () => {
    const status = `Connection Status:
  RosBridge: ${rosConnection.isConnected ? "Connected" : "Disconnected"}
  Address: ${rosConnection.ip}:${rosConnection.port}
  Last Ping: ${rosConnection.lastPing?.toLocaleTimeString() || "Never"}
  
ROS Master Configuration:
  IP: ${rosMasterConfig.masterIP}
  Port: ${rosMasterConfig.masterPort}
  Bridge: ${rosMasterConfig.bridgeIP}:${rosMasterConfig.bridgePort}`;

    addTerminalEntry("info", status);
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "Enter") {
      executeCommand(currentCommand);
    } else if (e.key === "ArrowUp") {
      e.preventDefault();
      if (historyIndex < commandHistory.length - 1) {
        const newIndex = historyIndex + 1;
        setHistoryIndex(newIndex);
        setCurrentCommand(commandHistory[newIndex]);
      }
    } else if (e.key === "ArrowDown") {
      e.preventDefault();
      if (historyIndex > 0) {
        const newIndex = historyIndex - 1;
        setHistoryIndex(newIndex);
        setCurrentCommand(commandHistory[newIndex]);
      } else if (historyIndex === 0) {
        setHistoryIndex(-1);
        setCurrentCommand("");
      }
    }
  };

  const getEntryColor = (type: string) => {
    switch (type) {
      case "command":
        return "text-blue-400";
      case "response":
        return "text-green-400";
      case "error":
        return "text-red-400";
      case "info":
        return "text-yellow-400";
      default:
        return "text-white";
    }
  };

  const getEntryPrefix = (type: string) => {
    switch (type) {
      case "command":
        return "$ ";
      case "response":
        return "→ ";
      case "error":
        return "✗ ";
      case "info":
        return "ℹ ";
      default:
        return "";
    }
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Terminal className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            ROS Terminal
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Interactive ROS command line with RosBridge WebSocket integration
          </p>
        </div>

        <div className="flex gap-2">
          <Button
            variant={rosConnection.isConnected ? "default" : "outline"}
            onClick={
              rosConnection.isConnected
                ? disconnectFromRosBridge
                : connectToRosBridge
            }
            className="gap-2"
          >
            {rosConnection.isConnected ? (
              <Wifi className="h-4 w-4" />
            ) : (
              <WifiOff className="h-4 w-4" />
            )}
            {rosConnection.isConnected ? "Connected" : "Connect"}
          </Button>
          <Button variant="outline" className="gap-2">
            <Settings className="h-4 w-4" />
            Settings
          </Button>
        </div>
      </div>

      {/* ROS Master Configuration */}
      <Card className="p-4">
        <h3 className="text-lg font-light mb-4 flex items-center gap-2">
          <Zap className="h-5 w-5 text-primary" />
          ROS Configuration
        </h3>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
          <div>
            <Label className="text-sm">ROS Master IP</Label>
            <Input
              value={rosMasterConfig.masterIP}
              onChange={(e) =>
                setRosMasterConfig((prev) => ({
                  ...prev,
                  masterIP: e.target.value,
                }))
              }
              placeholder="localhost"
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">ROS Master Port</Label>
            <Input
              type="number"
              value={rosMasterConfig.masterPort}
              onChange={(e) =>
                setRosMasterConfig((prev) => ({
                  ...prev,
                  masterPort: parseInt(e.target.value) || 11311,
                }))
              }
              placeholder="11311"
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">RosBridge IP</Label>
            <Input
              value={rosMasterConfig.bridgeIP}
              onChange={(e) => {
                setRosMasterConfig((prev) => ({
                  ...prev,
                  bridgeIP: e.target.value,
                }));
                setRosConnection((prev) => ({ ...prev, ip: e.target.value }));
              }}
              placeholder="localhost"
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">RosBridge Port</Label>
            <Input
              type="number"
              value={rosMasterConfig.bridgePort}
              onChange={(e) => {
                const port = parseInt(e.target.value) || 9090;
                setRosMasterConfig((prev) => ({
                  ...prev,
                  bridgePort: port,
                }));
                setRosConnection((prev) => ({ ...prev, port }));
              }}
              placeholder="9090"
              className="mt-1"
            />
          </div>
        </div>

        <div className="flex items-center justify-between mt-4">
          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2">
              <div
                className={`w-3 h-3 rounded-full ${rosConnection.isConnected ? "bg-green-500 animate-pulse" : "bg-red-500"}`}
              />
              <span className="text-sm">
                {rosConnection.isConnected ? "Connected" : "Disconnected"}
              </span>
            </div>
            {rosConnection.lastPing && (
              <span className="text-xs text-muted-foreground">
                Last ping: {rosConnection.lastPing.toLocaleTimeString()}
              </span>
            )}
          </div>

          <div className="flex items-center gap-2">
            <Switch checked={isAutoScroll} onCheckedChange={setIsAutoScroll} />
            <Label className="text-sm">Auto Scroll</Label>
          </div>
        </div>
      </Card>

      {/* Terminal */}
      <Card className="p-4">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-light flex items-center gap-2">
            <Terminal className="h-5 w-5 text-primary" />
            Terminal Output
          </h3>
          <div className="flex gap-2">
            <Button
              size="sm"
              variant="outline"
              onClick={() => setTerminalHistory([])}
              className="gap-1"
            >
              <Trash2 className="h-3 w-3" />
              Clear
            </Button>
            <Button size="sm" variant="outline" className="gap-1">
              <Download className="h-3 w-3" />
              Export
            </Button>
          </div>
        </div>

        {/* Terminal Display */}
        <div className="bg-black rounded-lg p-4 font-mono text-sm">
          <ScrollArea className="h-96" ref={scrollAreaRef}>
            <div className="space-y-1">
              {terminalHistory.map((entry) => (
                <div
                  key={entry.id}
                  className={`${getEntryColor(entry.type)} whitespace-pre-wrap`}
                >
                  <span className="text-gray-500 text-xs">
                    [{entry.timestamp.toLocaleTimeString()}]
                  </span>{" "}
                  <span className={getEntryColor(entry.type)}>
                    {getEntryPrefix(entry.type)}
                    {entry.content}
                  </span>
                </div>
              ))}
            </div>
          </ScrollArea>

          {/* Command Input */}
          <div className="flex items-center gap-2 mt-4 pt-4 border-t border-gray-700">
            <span className="text-green-400">$</span>
            <Input
              ref={commandInputRef}
              value={currentCommand}
              onChange={(e) => setCurrentCommand(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Enter ROS command (type 'help' for commands)"
              className="bg-transparent border-none text-white placeholder-gray-500 focus:ring-0 font-mono"
            />
            <Button
              size="sm"
              onClick={() => executeCommand(currentCommand)}
              disabled={!currentCommand.trim()}
              className="gap-1"
            >
              <Send className="h-3 w-3" />
              Send
            </Button>
          </div>
        </div>
      </Card>

      {/* Command History */}
      <Card className="p-4">
        <h3 className="text-lg font-light mb-4 flex items-center gap-2">
          <History className="h-5 w-5 text-primary" />
          Command History ({commandHistory.length})
        </h3>

        <ScrollArea className="h-32">
          <div className="space-y-1">
            {commandHistory.map((cmd, index) => (
              <div
                key={index}
                className="flex items-center gap-2 p-2 hover:bg-muted/50 rounded cursor-pointer"
                onClick={() => setCurrentCommand(cmd)}
              >
                <span className="text-xs text-muted-foreground w-8">
                  {index + 1}
                </span>
                <span className="font-mono text-sm flex-1">{cmd}</span>
                <Button
                  size="sm"
                  variant="ghost"
                  onClick={(e) => {
                    e.stopPropagation();
                    executeCommand(cmd);
                  }}
                  className="gap-1"
                >
                  <Play className="h-3 w-3" />
                </Button>
              </div>
            ))}
          </div>
        </ScrollArea>
      </Card>
    </div>
  );
};

export default ROSTerminal;
