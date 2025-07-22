import React, { useState, useRef, useEffect } from "react";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import {
  Plus,
  X,
  Terminal as TerminalIcon,
  Play,
  Square,
  RotateCcw,
  Settings,
  Download,
  Copy,
  Trash2,
  History,
  ChevronDown,
  Zap,
  AlertCircle,
  CheckCircle,
} from "lucide-react";

interface TerminalTab {
  id: string;
  name: string;
  type: 'ros' | 'system' | 'python' | 'custom';
  isActive: boolean;
  output: string[];
  history: string[];
  currentDirectory: string;
  isRunning: boolean;
  lastCommand: string;
}

interface Command {
  input: string;
  output: string[];
  timestamp: Date;
  exitCode: number;
  duration: number;
}

export default function Terminal() {
  const [tabs, setTabs] = useState<TerminalTab[]>([
    {
      id: '1',
      name: 'ROS Terminal',
      type: 'ros',
      isActive: true,
      output: [
        '$ Welcome to ROS Terminal',
        '$ ROS Noetic environment loaded',
        '$ Use "roscore" to start ROS master',
        '$ Type "help" for available commands',
      ],
      history: [],
      currentDirectory: '/opt/ros/noetic',
      isRunning: false,
      lastCommand: '',
    }
  ]);
  
  const [activeTabId, setActiveTabId] = useState('1');
  const [command, setCommand] = useState('');
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [showCommandHistory, setShowCommandHistory] = useState(false);
  const [isExecuting, setIsExecuting] = useState(false);
  const commandInputRef = useRef<HTMLInputElement>(null);
  const outputEndRef = useRef<HTMLDivElement>(null);
  
  const terminalTypes = [
    { value: 'ros', label: 'ROS Terminal', icon: '🤖' },
    { value: 'system', label: 'System Shell', icon: '💻' },
    { value: 'python', label: 'Python REPL', icon: '🐍' },
    { value: 'custom', label: 'Custom', icon: '⚡' },
  ];

  const commonCommands = {
    ros: [
      'roscore',
      'rosrun',
      'roslaunch',
      'rostopic list',
      'rostopic echo',
      'rosnode list',
      'rosservice list',
      'rosparam list',
      'rosbag record',
      'rosbag play',
    ],
    system: [
      'ls -la',
      'cd',
      'pwd',
      'ps aux',
      'top',
      'htop',
      'df -h',
      'free -h',
      'sudo systemctl status',
      'journalctl -f',
    ],
    python: [
      'import rospy',
      'import numpy as np',
      'import matplotlib.pyplot as plt',
      'help()',
      'dir()',
      'print("Hello World")',
      'exit()',
    ],
    custom: [
      'echo "Custom command"',
      'date',
      'whoami',
      'uname -a',
    ]
  };

  const activeTab = tabs.find(tab => tab.id === activeTabId);

  useEffect(() => {
    scrollToBottom();
  }, [tabs, activeTabId]);

  useEffect(() => {
    if (commandInputRef.current) {
      commandInputRef.current.focus();
    }
  }, [activeTabId]);

  const scrollToBottom = () => {
    if (outputEndRef.current) {
      outputEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  };

  const addTab = (type: 'ros' | 'system' | 'python' | 'custom' = 'system') => {
    const newId = Date.now().toString();
    const typeConfig = terminalTypes.find(t => t.value === type);
    
    const newTab: TerminalTab = {
      id: newId,
      name: `${typeConfig?.label || 'Terminal'} ${tabs.length + 1}`,
      type,
      isActive: false,
      output: [`$ Welcome to ${typeConfig?.label || 'Terminal'}`, `$ Ready for commands...`],
      history: [],
      currentDirectory: type === 'ros' ? '/opt/ros/noetic' : '/home/robot',
      isRunning: false,
      lastCommand: '',
    };

    setTabs(prev => [...prev, newTab]);
    setActiveTabId(newId);
  };

  const closeTab = (tabId: string) => {
    if (tabs.length <= 1) return; // Keep at least one tab
    
    const newTabs = tabs.filter(tab => tab.id !== tabId);
    setTabs(newTabs);
    
    if (activeTabId === tabId) {
      setActiveTabId(newTabs[0]?.id || '');
    }
  };

  const executeCommand = async (inputCommand: string) => {
    if (!inputCommand.trim() || !activeTab) return;

    setIsExecuting(true);
    const startTime = Date.now();

    // Add command to output
    const updatedTabs = tabs.map(tab => {
      if (tab.id === activeTabId) {
        return {
          ...tab,
          output: [...tab.output, `${tab.currentDirectory}$ ${inputCommand}`],
          history: [...tab.history, inputCommand],
          lastCommand: inputCommand,
          isRunning: true,
        };
      }
      return tab;
    });
    setTabs(updatedTabs);

    // Simulate command execution
    await new Promise(resolve => setTimeout(resolve, Math.random() * 1000 + 500));

    // Simulate command response based on type and command
    let output: string[] = [];
    let exitCode = 0;

    try {
      output = simulateCommandOutput(inputCommand, activeTab.type);
    } catch (error) {
      output = [`Error: ${error}`];
      exitCode = 1;
    }

    const endTime = Date.now();
    const duration = endTime - startTime;

    // Update tab with command output
    setTabs(prev => prev.map(tab => {
      if (tab.id === activeTabId) {
        return {
          ...tab,
          output: [...tab.output, ...output],
          isRunning: false,
        };
      }
      return tab;
    }));

    setCommand('');
    setHistoryIndex(-1);
    setIsExecuting(false);
  };

  const simulateCommandOutput = (cmd: string, type: string): string[] => {
    const command = cmd.trim().toLowerCase();
    
    if (command === 'help') {
      return [
        'Available commands:',
        '  help - Show this help message',
        '  clear - Clear terminal output',
        '  history - Show command history',
        '  exit - Close terminal',
        '',
        `${type.toUpperCase()} specific commands:`,
        ...commonCommands[type as keyof typeof commonCommands].map(c => `  ${c}`),
      ];
    }
    
    if (command === 'clear') {
      // Clear will be handled separately
      return [];
    }
    
    if (command === 'history') {
      return activeTab?.history.map((h, i) => `${i + 1}: ${h}`) || [];
    }
    
    if (command === 'exit') {
      return ['Terminal session ended.'];
    }

    // Type-specific command simulation
    switch (type) {
      case 'ros':
        return simulateRosCommand(command);
      case 'system':
        return simulateSystemCommand(command);
      case 'python':
        return simulatePythonCommand(command);
      default:
        return [`Command executed: ${cmd}`, 'Output: Success'];
    }
  };

  const simulateRosCommand = (command: string): string[] => {
    if (command === 'roscore') {
      return [
        'Starting roscore...',
        'ROS Master started at http://localhost:11311',
        'Parameter server started',
        'roscore started successfully',
      ];
    }
    
    if (command.startsWith('rostopic list')) {
      return [
        '/camera/image_raw',
        '/cmd_vel',
        '/odom',
        '/scan',
        '/tf',
        '/tf_static',
      ];
    }
    
    if (command.startsWith('rosnode list')) {
      return [
        '/camera_node',
        '/move_base',
        '/robot_state_publisher',
        '/rosout',
      ];
    }
    
    return [`ROS command executed: ${command}`, 'Status: OK'];
  };

  const simulateSystemCommand = (command: string): string[] => {
    if (command.startsWith('ls')) {
      return [
        'Documents/',
        'Downloads/',
        'catkin_ws/',
        'robot_config.yaml',
        'setup.bash',
      ];
    }
    
    if (command === 'pwd') {
      return ['/home/robot'];
    }
    
    if (command.startsWith('ps')) {
      return [
        'PID  COMMAND',
        '1234 roscore',
        '1235 robot_node',
        '1236 camera_node',
      ];
    }
    
    return [`System command executed: ${command}`, 'Exit code: 0'];
  };

  const simulatePythonCommand = (command: string): string[] => {
    if (command.startsWith('import')) {
      return [`Module imported successfully`];
    }
    
    if (command.startsWith('print')) {
      const match = command.match(/print\s*\(\s*["'](.*)["']\s*\)/);
      return [match?.[1] || 'Hello World'];
    }
    
    if (command === 'help()') {
      return [
        'Welcome to Python help utility!',
        'Type "help", "copyright", "credits" or "license" for more information.',
      ];
    }
    
    return [`>>> ${command}`, 'Command executed'];
  };

  const clearTerminal = () => {
    if (!activeTab) return;
    
    setTabs(prev => prev.map(tab => {
      if (tab.id === activeTabId) {
        return {
          ...tab,
          output: [`$ Terminal cleared`, `$ Ready for commands...`],
        };
      }
      return tab;
    }));
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      if (command === 'clear') {
        clearTerminal();
        setCommand('');
        return;
      }
      executeCommand(command);
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      if (activeTab && activeTab.history.length > 0) {
        const newIndex = historyIndex >= 0 ? 
          Math.max(0, historyIndex - 1) : 
          activeTab.history.length - 1;
        setHistoryIndex(newIndex);
        setCommand(activeTab.history[newIndex] || '');
      }
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      if (activeTab && historyIndex >= 0) {
        const newIndex = historyIndex < activeTab.history.length - 1 ? 
          historyIndex + 1 : 
          -1;
        setHistoryIndex(newIndex);
        setCommand(newIndex >= 0 ? activeTab.history[newIndex] || '' : '');
      }
    } else if (e.key === 'Tab') {
      e.preventDefault();
      // Simple tab completion
      if (activeTab) {
        const availableCommands = commonCommands[activeTab.type as keyof typeof commonCommands];
        const matches = availableCommands.filter(cmd => 
          cmd.toLowerCase().startsWith(command.toLowerCase())
        );
        if (matches.length === 1) {
          setCommand(matches[0]);
        }
      }
    }
  };

  const downloadOutput = () => {
    if (!activeTab) return;
    
    const content = activeTab.output.join('\n');
    const blob = new Blob([content], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `terminal-${activeTab.name}-${new Date().toISOString().split('T')[0]}.txt`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const copyOutput = () => {
    if (!activeTab) return;
    
    const content = activeTab.output.join('\n');
    navigator.clipboard.writeText(content);
  };

  if (!activeTab) return null;

  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="flex items-center justify-between mb-6">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            Terminal System
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Multi-tab terminal interface for ROS and system commands
          </p>
        </div>
        
        <div className="flex items-center gap-3">
          <Select 
            defaultValue="system"
            onValueChange={(value) => addTab(value as any)}
          >
            <SelectTrigger className="w-48">
              <Plus className="h-4 w-4 mr-2" />
              <SelectValue placeholder="Add Terminal" />
            </SelectTrigger>
            <SelectContent>
              {terminalTypes.map(type => (
                <SelectItem key={type.value} value={type.value}>
                  <span className="mr-2">{type.icon}</span>
                  {type.label}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          
          <Button variant="outline" onClick={downloadOutput} className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
          
          <Button variant="outline" onClick={copyOutput} className="gap-2">
            <Copy className="h-4 w-4" />
            Copy All
          </Button>
          
          <Button variant="outline" onClick={clearTerminal} className="gap-2">
            <Trash2 className="h-4 w-4" />
            Clear
          </Button>
        </div>
      </div>

      <Card className="p-0 bg-gray-900 text-green-400 font-mono overflow-hidden">
        {/* Terminal Tabs */}
        <div className="flex items-center bg-gray-800 border-b border-gray-700 px-4 py-2">
          <div className="flex items-center space-x-1 flex-1">
            {tabs.map(tab => (
              <div
                key={tab.id}
                className={`flex items-center gap-2 px-3 py-2 rounded-t-lg cursor-pointer ${
                  tab.id === activeTabId 
                    ? 'bg-gray-900 text-green-400' 
                    : 'bg-gray-700 text-gray-400 hover:bg-gray-600'
                }`}
                onClick={() => setActiveTabId(tab.id)}
              >
                <TerminalIcon className="h-4 w-4" />
                <span className="text-sm">{tab.name}</span>
                {tab.isRunning && (
                  <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse" />
                )}
                {tabs.length > 1 && (
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      closeTab(tab.id);
                    }}
                    className="ml-2 p-1 hover:bg-red-600 rounded"
                  >
                    <X className="h-3 w-3" />
                  </button>
                )}
              </div>
            ))}
          </div>
          
          <div className="flex items-center gap-2">
            <Badge variant="outline" className="text-xs bg-gray-700 text-gray-300">
              {terminalTypes.find(t => t.value === activeTab.type)?.icon} {activeTab.type.toUpperCase()}
            </Badge>
            {activeTab.isRunning && (
              <Badge className="text-xs bg-green-600">
                <Zap className="h-3 w-3 mr-1" />
                Running
              </Badge>
            )}
          </div>
        </div>

        {/* Terminal Output */}
        <div className="h-96">
          <ScrollArea className="h-full p-4">
            <div className="space-y-1">
              {activeTab.output.map((line, index) => (
                <div key={index} className="text-sm leading-relaxed">
                  {line.startsWith('$') ? (
                    <span className="text-blue-400">{line}</span>
                  ) : line.includes('Error') || line.includes('error') ? (
                    <span className="text-red-400">{line}</span>
                  ) : line.includes('Success') || line.includes('OK') ? (
                    <span className="text-green-400">{line}</span>
                  ) : (
                    <span className="text-gray-300">{line}</span>
                  )}
                </div>
              ))}
              <div ref={outputEndRef} />
            </div>
          </ScrollArea>
        </div>

        {/* Command Input */}
        <div className="border-t border-gray-700 p-4">
          <div className="flex items-center gap-2">
            <span className="text-blue-400 text-sm">
              {activeTab.currentDirectory}$
            </span>
            <Input
              ref={commandInputRef}
              value={command}
              onChange={(e) => setCommand(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Type command and press Enter..."
              className="flex-1 bg-transparent border-none text-green-400 placeholder-gray-500 focus:ring-0 focus:border-none p-0"
              disabled={isExecuting}
            />
            {isExecuting && (
              <div className="flex items-center gap-2 text-yellow-400">
                <div className="w-2 h-2 bg-yellow-400 rounded-full animate-pulse" />
                <span className="text-xs">Executing...</span>
              </div>
            )}
          </div>
          
          {/* Quick Commands */}
          <div className="mt-3 flex flex-wrap gap-2">
            {commonCommands[activeTab.type as keyof typeof commonCommands].slice(0, 6).map((cmd, index) => (
              <Button
                key={index}
                variant="outline"
                size="sm"
                onClick={() => setCommand(cmd)}
                className="text-xs bg-gray-800 border-gray-600 text-gray-300 hover:bg-gray-700"
                disabled={isExecuting}
              >
                {cmd}
              </Button>
            ))}
            <Button
              variant="outline"
              size="sm"
              onClick={() => setShowCommandHistory(!showCommandHistory)}
              className="text-xs bg-gray-800 border-gray-600 text-gray-300 hover:bg-gray-700 gap-1"
            >
              <History className="h-3 w-3" />
              History ({activeTab.history.length})
            </Button>
          </div>
          
          {/* Command History */}
          {showCommandHistory && activeTab.history.length > 0 && (
            <div className="mt-3 p-3 bg-gray-800 rounded-lg max-h-32 overflow-y-auto">
              <h4 className="text-xs text-gray-400 mb-2">Command History:</h4>
              <div className="space-y-1">
                {activeTab.history.slice(-10).map((histCmd, index) => (
                  <button
                    key={index}
                    onClick={() => {
                      setCommand(histCmd);
                      setShowCommandHistory(false);
                    }}
                    className="block w-full text-left text-xs text-gray-300 hover:text-green-400 hover:bg-gray-700 px-2 py-1 rounded"
                  >
                    {histCmd}
                  </button>
                ))}
              </div>
            </div>
          )}
        </div>
      </Card>

      {/* Status Bar */}
      <div className="mt-4 flex items-center justify-between text-sm text-gray-500">
        <div className="flex items-center gap-4">
          <span>Active: {activeTab.name}</span>
          <span>Type: {activeTab.type.toUpperCase()}</span>
          <span>Commands: {activeTab.history.length}</span>
        </div>
        <div className="flex items-center gap-4">
          <span>Press ↑/↓ for history, Tab for completion</span>
          <Badge className="bg-green-100 text-green-700">
            <CheckCircle className="h-3 w-3 mr-1" />
            Ready
          </Badge>
        </div>
      </div>
    </div>
  );
}
