import React, { useState, useRef, useEffect, useCallback } from "react";
import { usePersistentState, usePersistentStore } from "@/hooks/use-persistence";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { ScrollArea } from "@/components/ui/scroll-area";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Plus,
  X,
  Terminal as TerminalIcon,
  Settings,
  Download,
  Copy,
  Trash2,
  History,
  Zap,
  AlertCircle,
  CheckCircle,
  MinusCircle,
  FolderOpen,
  Clock,
  Wifi,
  WifiOff,
} from "lucide-react";

interface TerminalLine {
  id: string;
  type: "command" | "output" | "error" | "system";
  content: string;
  timestamp: Date;
  directory?: string;
}

interface TerminalTab {
  id: string;
  name: string;
  type: "bash" | "ros" | "python" | "ssh";
  lines: TerminalLine[];
  history: string[];
  historyIndex: number;
  currentDirectory: string;
  isRunning: boolean;
  environment: Record<string, string>;
  pid?: number;
}

export default function Terminal() {
  // Persistent terminal state
  const { state: tabs, setState: setTabs } = usePersistentState<TerminalTab[]>({
    key: "terminal-tabs",
    defaultValue: [
      {
        id: "1",
        name: "bash",
        type: "bash",
        lines: [
          {
            id: "1",
            type: "system",
            content:
              "Welcome to Ubuntu 20.04.6 LTS (GNU/Linux 5.15.0-91-generic x86_64)",
            timestamp: new Date(),
          },
          {
            id: "2",
            type: "system",
            content: "Last login: " + new Date().toLocaleString(),
            timestamp: new Date(),
          },
          {
            id: "3",
            type: "system",
            content: 'Type "help" for available commands',
            timestamp: new Date(),
          },
        ],
        history: [],
        historyIndex: -1,
        currentDirectory: "/home/robot",
        isRunning: false,
        environment: {
          HOME: "/home/robot",
          USER: "robot",
          SHELL: "/bin/bash",
          PATH: "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
          ROS_DISTRO: "noetic",
        },
      },
    ],
    autoSave: true,
    autoSaveDelay: 2000,
  });

  const { store: terminalPrefs, updateField: updateTerminalPref } = usePersistentStore(
    "terminal-preferences",
    {
      activeTabId: "1",
      currentCommand: "",
      isConnected: true,
      fontSize: 14,
      theme: "dark",
    }
  );

  const { activeTabId, currentCommand, isConnected, fontSize, theme } = terminalPrefs;

  const commandInputRef = useRef<HTMLInputElement>(null);
  const terminalRef = useRef<HTMLDivElement>(null);

  const activeTab = tabs.find((tab) => tab.id === activeTabId);

  const terminalTypes = [
    { value: "bash", label: "Bash", icon: "💻", color: "text-green-400" },
    { value: "ros", label: "ROS", icon: "🤖", color: "text-blue-400" },
    { value: "python", label: "Python", icon: "🐍", color: "text-yellow-400" },
    { value: "ssh", label: "SSH", icon: "🔐", color: "text-red-400" },
  ];

  // Auto-scroll to bottom when new content is added
  useEffect(() => {
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [tabs, activeTabId]);

  // Focus input when tab changes
  useEffect(() => {
    if (commandInputRef.current) {
      commandInputRef.current.focus();
    }
  }, [activeTabId]);

  const addTab = useCallback(
    (type: "bash" | "ros" | "python" | "ssh" = "bash") => {
      const newId = Date.now().toString();
      const typeConfig = terminalTypes.find((t) => t.value === type);

      const welcomeMessages: Record<string, TerminalLine[]> = {
        bash: [
          {
            id: newId + "_1",
            type: "system",
            content: `${typeConfig?.icon} Starting new bash session...`,
            timestamp: new Date(),
          },
        ],
        ros: [
          {
            id: newId + "_1",
            type: "system",
            content: "🤖 ROS Noetic environment loaded",
            timestamp: new Date(),
          },
          {
            id: newId + "_2",
            type: "system",
            content: "ROS_MASTER_URI=http://localhost:11311",
            timestamp: new Date(),
          },
        ],
        python: [
          {
            id: newId + "_1",
            type: "system",
            content: "Python 3.8.10 (default, Nov 22 2023, 10:22:35)",
            timestamp: new Date(),
          },
          {
            id: newId + "_2",
            type: "system",
            content:
              'Type "help", "copyright", "credits" or "license" for more information.',
            timestamp: new Date(),
          },
        ],
        ssh: [
          {
            id: newId + "_1",
            type: "system",
            content: "🔐 SSH connection ready",
            timestamp: new Date(),
          },
        ],
      };

      const newTab: TerminalTab = {
        id: newId,
        name: `${type}-${tabs.length + 1}`,
        type,
        lines: welcomeMessages[type] || welcomeMessages.bash,
        history: [],
        historyIndex: -1,
        currentDirectory: type === "ros" ? "/opt/ros/noetic" : "/home/robot",
        isRunning: false,
        environment: {
          HOME: "/home/robot",
          USER: "robot",
          SHELL: type === "python" ? "/usr/bin/python3" : "/bin/bash",
          PATH: "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
          ...(type === "ros" && {
            ROS_DISTRO: "noetic",
            ROS_MASTER_URI: "http://localhost:11311",
          }),
        },
      };

      setTabs((prev) => [...prev, newTab]);
      updateTerminalPref("activeTabId", newId);
    },
    [tabs.length],
  );

  const closeTab = useCallback(
    (tabId: string) => {
      if (tabs.length <= 1) return;

      const newTabs = tabs.filter((tab) => tab.id !== tabId);
      setTabs(newTabs);

      if (activeTabId === tabId) {
        updateTerminalPref("activeTabId", newTabs[0]?.id || "");
      }
    },
    [tabs, activeTabId],
  );

  const executeCommand = useCallback(
    async (command: string) => {
      if (!command.trim() || !activeTab) return;

      const commandId = Date.now().toString();

      // Add command to history
      const updatedTab = {
        ...activeTab,
        history: [...activeTab.history, command],
        historyIndex: -1,
        isRunning: true,
      };

      // Add command line to output
      const commandLine: TerminalLine = {
        id: commandId,
        type: "command",
        content: `${getPrompt(activeTab)}${command}`,
        timestamp: new Date(),
        directory: activeTab.currentDirectory,
      };

      updatedTab.lines = [...updatedTab.lines, commandLine];

      // Update tabs
      setTabs((prev) =>
        prev.map((tab) => (tab.id === activeTabId ? updatedTab : tab)),
      );

      // Simulate command execution
      setTimeout(
        async () => {
          const outputLines = await simulateCommandExecution(
            command,
            activeTab,
          );

          setTabs((prev) =>
            prev.map((tab) => {
              if (tab.id === activeTabId) {
                return {
                  ...tab,
                  lines: [...tab.lines, ...outputLines],
                  isRunning: false,
                  currentDirectory: updateDirectory(
                    command,
                    tab.currentDirectory,
                  ),
                };
              }
              return tab;
            }),
          );
        },
        Math.random() * 800 + 200,
      ); // Random delay 200-1000ms

      setCurrentCommand("");
    },
    [activeTab, activeTabId],
  );

  const simulateCommandExecution = async (
    command: string,
    tab: TerminalTab,
  ): Promise<TerminalLine[]> => {
    const cmd = command.trim().toLowerCase();
    const timestamp = new Date();

    if (cmd === "clear") {
      // Clear will be handled separately
      return [];
    }

    if (cmd === "help") {
      return [
        {
          id: `${timestamp.getTime()}_1`,
          type: "output",
          content: "Available commands:",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_2`,
          type: "output",
          content: "  ls, cd, pwd, ps, top, htop, nano, vim",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_3`,
          type: "output",
          content: "  roscore, rosrun, roslaunch (ROS terminal)",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_4`,
          type: "output",
          content: "  clear, history, exit",
          timestamp,
        },
      ];
    }

    if (cmd === "history") {
      return tab.history.map((histCmd, index) => ({
        id: `${timestamp.getTime()}_${index}`,
        type: "output",
        content: `${index + 1}: ${histCmd}`,
        timestamp,
      }));
    }

    if (cmd === "exit") {
      return [
        {
          id: `${timestamp.getTime()}`,
          type: "system",
          content: "Session terminated. Close tab or start new session.",
          timestamp,
        },
      ];
    }

    // Type-specific commands
    switch (tab.type) {
      case "ros":
        return simulateRosCommand(cmd, timestamp);
      case "python":
        return simulatePythonCommand(cmd, timestamp);
      case "ssh":
        return simulateSSHCommand(cmd, timestamp);
      default:
        return simulateBashCommand(cmd, timestamp);
    }
  };

  const simulateBashCommand = (
    command: string,
    timestamp: Date,
  ): TerminalLine[] => {
    if (command.startsWith("ls")) {
      return [
        {
          id: `${timestamp.getTime()}_1`,
          type: "output",
          content:
            "catkin_ws/  Documents/  Downloads/  Pictures/  robot_config.yaml",
          timestamp,
        },
      ];
    }

    if (command === "pwd") {
      return [
        {
          id: `${timestamp.getTime()}`,
          type: "output",
          content: activeTab?.currentDirectory || "/home/robot",
          timestamp,
        },
      ];
    }

    if (command.startsWith("ps")) {
      return [
        {
          id: `${timestamp.getTime()}_1`,
          type: "output",
          content: "PID  TTY          TIME CMD",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_2`,
          type: "output",
          content: "1234 pts/0    00:00:01 bash",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_3`,
          type: "output",
          content: "1235 pts/0    00:00:02 roscore",
          timestamp,
        },
      ];
    }

    if (command.startsWith("cd ")) {
      const dir = command.substring(3).trim();
      if (dir === "..") {
        return []; // Directory change handled in updateDirectory
      }
      return [
        {
          id: `${timestamp.getTime()}`,
          type: "output",
          content: `Changed directory to: ${dir}`,
          timestamp,
        },
      ];
    }

    // Default output for unknown commands
    return [
      {
        id: `${timestamp.getTime()}`,
        type: "error",
        content: `bash: ${command}: command not found`,
        timestamp,
      },
    ];
  };

  const simulateRosCommand = (
    command: string,
    timestamp: Date,
  ): TerminalLine[] => {
    if (command === "roscore") {
      return [
        {
          id: `${timestamp.getTime()}_1`,
          type: "output",
          content: "Starting roscore...",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_2`,
          type: "output",
          content: "ROS Master started at http://localhost:11311",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_3`,
          type: "output",
          content: "Parameter server started",
          timestamp,
        },
      ];
    }

    if (command.startsWith("rostopic list")) {
      return [
        {
          id: `${timestamp.getTime()}_1`,
          type: "output",
          content: "/camera/image_raw",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_2`,
          type: "output",
          content: "/cmd_vel",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_3`,
          type: "output",
          content: "/odom",
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_4`,
          type: "output",
          content: "/scan",
          timestamp,
        },
      ];
    }

    return simulateBashCommand(command, timestamp);
  };

  const simulatePythonCommand = (
    command: string,
    timestamp: Date,
  ): TerminalLine[] => {
    if (command.startsWith("import")) {
      return [
        {
          id: `${timestamp.getTime()}`,
          type: "output",
          content: ">>> " + command,
          timestamp,
        },
      ];
    }

    if (command.startsWith("print")) {
      const match = command.match(/print\s*\(\s*["'](.*)["']\s*\)/);
      const output = match?.[1] || "Hello World";
      return [
        {
          id: `${timestamp.getTime()}_1`,
          type: "output",
          content: ">>> " + command,
          timestamp,
        },
        {
          id: `${timestamp.getTime()}_2`,
          type: "output",
          content: output,
          timestamp,
        },
      ];
    }

    return [
      {
        id: `${timestamp.getTime()}_1`,
        type: "output",
        content: ">>> " + command,
        timestamp,
      },
      {
        id: `${timestamp.getTime()}_2`,
        type: "output",
        content: "Command executed",
        timestamp,
      },
    ];
  };

  const simulateSSHCommand = (
    command: string,
    timestamp: Date,
  ): TerminalLine[] => {
    return [
      {
        id: `${timestamp.getTime()}`,
        type: "output",
        content: `[SSH] ${command}`,
        timestamp,
      },
    ];
  };

  const updateDirectory = (command: string, currentDir: string): string => {
    if (command.startsWith("cd ")) {
      const dir = command.substring(3).trim();
      if (dir === "..") {
        const parts = currentDir.split("/");
        parts.pop();
        return parts.length > 1 ? parts.join("/") : "/";
      } else if (dir.startsWith("/")) {
        return dir;
      } else {
        return `${currentDir}/${dir}`.replace("//", "/");
      }
    }
    return currentDir;
  };

  const getPrompt = (tab: TerminalTab): string => {
    const user = tab.environment.USER || "robot";
    const hostname = "robot-desktop";
    const shortDir = tab.currentDirectory.replace("/home/robot", "~");

    switch (tab.type) {
      case "python":
        return ">>> ";
      case "ros":
        return `[ROS] ${user}@${hostname}:${shortDir}$ `;
      case "ssh":
        return `[SSH] ${user}@remote:${shortDir}$ `;
      default:
        return `${user}@${hostname}:${shortDir}$ `;
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (!activeTab) return;

    if (e.key === "Enter") {
      if (currentCommand === "clear") {
        clearTerminal();
        return;
      }
      executeCommand(currentCommand);
    } else if (e.key === "ArrowUp") {
      e.preventDefault();
      navigateHistory("up");
    } else if (e.key === "ArrowDown") {
      e.preventDefault();
      navigateHistory("down");
    } else if (e.key === "Tab") {
      e.preventDefault();
      // Simple tab completion could be added here
    } else if (e.ctrlKey && e.key === "c") {
      e.preventDefault();
      interruptCommand();
    } else if (e.ctrlKey && e.key === "l") {
      e.preventDefault();
      clearTerminal();
    }
  };

  const navigateHistory = (direction: "up" | "down") => {
    if (!activeTab) return;

    const history = activeTab.history;
    if (history.length === 0) return;

    let newIndex = activeTab.historyIndex;

    if (direction === "up") {
      newIndex =
        newIndex === -1 ? history.length - 1 : Math.max(0, newIndex - 1);
    } else {
      newIndex = newIndex === -1 ? -1 : newIndex + 1;
      if (newIndex >= history.length) newIndex = -1;
    }

    setTabs((prev) =>
      prev.map((tab) =>
        tab.id === activeTabId ? { ...tab, historyIndex: newIndex } : tab,
      ),
    );

    updateTerminalPref("currentCommand", newIndex === -1 ? "" : history[newIndex]);
  };

  const clearTerminal = () => {
    if (!activeTab) return;

    setTabs((prev) =>
      prev.map((tab) => {
        if (tab.id === activeTabId) {
          return {
            ...tab,
            lines: [
              {
                id: Date.now().toString(),
                type: "system",
                content: "Terminal cleared",
                timestamp: new Date(),
              },
            ],
          };
        }
        return tab;
      }),
    );
    setCurrentCommand("");
  };

  const interruptCommand = () => {
    if (!activeTab) return;

    setTabs((prev) =>
      prev.map((tab) => {
        if (tab.id === activeTabId) {
          return {
            ...tab,
            isRunning: false,
            lines: [
              ...tab.lines,
              {
                id: Date.now().toString(),
                type: "system",
                content: "^C",
                timestamp: new Date(),
              },
            ],
          };
        }
        return tab;
      }),
    );
    setCurrentCommand("");
  };

  const copyAllOutput = () => {
    if (!activeTab) return;

    const content = activeTab.lines.map((line) => line.content).join("\n");
    navigator.clipboard.writeText(content);
  };

  const downloadSession = () => {
    if (!activeTab) return;

    const content = activeTab.lines
      .map((line) => `[${line.timestamp.toLocaleTimeString()}] ${line.content}`)
      .join("\n");

    const blob = new Blob([content], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `terminal-${activeTab.name}-${new Date().toISOString().split("T")[0]}.txt`;
    a.click();
    URL.revokeObjectURL(url);
  };

  if (!activeTab) return null;

  return (
    <div className="flex flex-col h-screen bg-gray-900 text-green-400 font-mono">
      {/* Header Bar */}
      <div className="flex items-center justify-between bg-gray-800 px-4 py-2 border-b border-gray-700">
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2">
            <TerminalIcon className="h-5 w-5" />
            <span className="font-medium">Terminal</span>
          </div>

          <div className="flex items-center gap-2">
            <Badge className={isConnected ? "bg-green-600" : "bg-red-600"}>
              {isConnected ? (
                <Wifi className="h-3 w-3 mr-1" />
              ) : (
                <WifiOff className="h-3 w-3 mr-1" />
              )}
              {isConnected ? "Connected" : "Disconnected"}
            </Badge>

            <Badge className="bg-blue-600">
              <Clock className="h-3 w-3 mr-1" />
              {new Date().toLocaleTimeString()}
            </Badge>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <Select
            value={fontSize.toString()}
            onValueChange={(value) => setFontSize(parseInt(value))}
          >
            <SelectTrigger className="w-20 h-8 bg-gray-700 border-gray-600 text-green-400">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="12">12px</SelectItem>
              <SelectItem value="14">14px</SelectItem>
              <SelectItem value="16">16px</SelectItem>
              <SelectItem value="18">18px</SelectItem>
            </SelectContent>
          </Select>

          <Button
            variant="ghost"
            size="sm"
            onClick={copyAllOutput}
            className="h-8 px-3 text-green-400 hover:bg-gray-700"
          >
            <Copy className="h-4 w-4" />
          </Button>

          <Button
            variant="ghost"
            size="sm"
            onClick={downloadSession}
            className="h-8 px-3 text-green-400 hover:bg-gray-700"
          >
            <Download className="h-4 w-4" />
          </Button>
        </div>
      </div>

      {/* Tab Bar */}
      <div className="flex items-center bg-gray-800 border-b border-gray-700">
        <div className="flex items-center flex-1 overflow-x-auto">
          {tabs.map((tab) => {
            const typeConfig = terminalTypes.find((t) => t.value === tab.type);
            return (
              <div
                key={tab.id}
                className={`flex items-center gap-2 px-4 py-2 border-r border-gray-700 cursor-pointer min-w-0 ${
                  tab.id === activeTabId
                    ? "bg-gray-900 text-green-400 border-b-2 border-green-400"
                    : "text-gray-400 hover:bg-gray-700"
                }`}
                onClick={() => updateTerminalPref("activeTabId", tab.id)}
              >
                <span className="text-sm">{typeConfig?.icon}</span>
                <span className="text-sm truncate">{tab.name}</span>
                {tab.isRunning && (
                  <div className="w-2 h-2 bg-yellow-400 rounded-full animate-pulse" />
                )}
                {tabs.length > 1 && (
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      closeTab(tab.id);
                    }}
                    className="ml-2 p-1 hover:bg-red-600 rounded transition-colors"
                  >
                    <X className="h-3 w-3" />
                  </button>
                )}
              </div>
            );
          })}
        </div>

        <div className="flex items-center gap-1 px-2">
          <Select onValueChange={(value) => addTab(value as any)}>
            <SelectTrigger className="w-12 h-8 bg-gray-700 border-gray-600 text-green-400">
              <Plus className="h-4 w-4" />
            </SelectTrigger>
            <SelectContent>
              {terminalTypes.map((type) => (
                <SelectItem key={type.value} value={type.value}>
                  <span className="mr-2">{type.icon}</span>
                  {type.label}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Terminal Content */}
      <div className="flex-1 flex flex-col overflow-hidden">
        <div
          ref={terminalRef}
          className="flex-1 overflow-y-auto p-4 space-y-1"
          style={{ fontSize: `${fontSize}px` }}
        >
          {activeTab.lines.map((line) => (
            <div key={line.id} className="leading-relaxed">
              <span
                className={
                  line.type === "command"
                    ? "text-white"
                    : line.type === "error"
                      ? "text-red-400"
                      : line.type === "system"
                        ? "text-blue-400"
                        : "text-green-400"
                }
              >
                {line.content}
              </span>
            </div>
          ))}
        </div>

        {/* Command Input */}
        <div className="border-t border-gray-700 p-4">
          <div className="flex items-center gap-2">
            <span className="text-blue-400 text-sm select-none">
              {getPrompt(activeTab)}
            </span>
            <input
              ref={commandInputRef}
              value={currentCommand}
              onChange={(e) => updateTerminalPref("currentCommand", e.target.value)}
              onKeyDown={handleKeyDown}
              className="flex-1 bg-transparent border-none outline-none text-green-400 font-mono"
              style={{ fontSize: `${fontSize}px` }}
              placeholder={
                activeTab.isRunning ? "Command running..." : "Enter command..."
              }
              disabled={activeTab.isRunning}
              autoComplete="off"
              spellCheck={false}
            />
            {activeTab.isRunning && (
              <div className="flex items-center gap-2 text-yellow-400">
                <div className="w-2 h-2 bg-yellow-400 rounded-full animate-pulse" />
                <span className="text-xs">Running...</span>
              </div>
            )}
          </div>

          {/* Status Bar */}
          <div className="flex items-center justify-between mt-2 text-xs text-gray-500">
            <div className="flex items-center gap-4">
              <span>Session: {activeTab.name}</span>
              <span>Dir: {activeTab.currentDirectory}</span>
              <span>Commands: {activeTab.history.length}</span>
            </div>
            <div className="flex items-center gap-4">
              <span>Ctrl+C: Interrupt</span>
              <span>Ctrl+L: Clear</span>
              <span>↑/↓: History</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
