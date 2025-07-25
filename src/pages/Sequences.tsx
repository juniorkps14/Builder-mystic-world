import React, { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
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
import { Label } from "@/components/ui/label";
import {
  usePersistentState,
  usePersistentStore,
} from "@/hooks/use-persistence";
import {
  Play,
  Pause,
  Square,
  RotateCcw,
  Plus,
  Settings,
  Clock,
  CheckCircle2,
  XCircle,
  AlertTriangle,
  Activity,
  Timer,
  Download,
  Upload,
  Save,
  Copy,
  Trash2,
  FileText,
  BarChart3,
  Zap,
  Edit,
  PlayCircle,
  PauseCircle,
  StopCircle,
  ChevronRight,
  MoreVertical,
  Search,
  Filter,
  SortAsc,
  FolderOpen,
  User,
  Calendar,
  Target,
} from "lucide-react";

interface Task {
  id: string;
  name: string;
  type: "navigation" | "manipulation" | "sensor" | "ai" | "custom";
  status: "idle" | "running" | "completed" | "failed" | "paused";
  duration: number;
  priority: "low" | "medium" | "high" | "critical";
  description: string;
  parameters: Record<string, any>;
  progress?: number;
  estimatedTime?: number;
  lastExecuted?: Date;
  successRate?: number;
  dependencies?: string[];
}

interface Sequence {
  id: string;
  name: string;
  description: string;
  status: "idle" | "running" | "completed" | "failed" | "paused";
  taskCount: number;
  completedTasks: number;
  totalDuration: number;
  lastExecuted?: Date;
  successRate: number;
  tags: string[];
}

export default function Sequences() {
  // Persistent state
  const { store: sequencePrefs, updateField } = usePersistentStore(
    "sequences-preferences",
    {
      selectedTaskId: null as string | null,
      selectedSequenceId: null as string | null,
      viewMode: "overview" as "overview" | "detailed",
      filterStatus: "all" as string,
      sortBy: "priority" as string,
    },
  );

  // State for task management
  const [isTaskDialogOpen, setIsTaskDialogOpen] = useState(false);
  const [isSequenceDialogOpen, setIsSequenceDialogOpen] = useState(false);
  const [editingTask, setEditingTask] = useState<Task | null>(null);
  const [newTask, setNewTask] = useState<Partial<Task>>({
    name: "",
    type: "navigation",
    priority: "medium",
    description: "",
    parameters: {},
  });

  const [sequences, setSequences] = useState<Sequence[]>([
    {
      id: "seq_1",
      name: "Security Patrol Route",
      description:
        "Comprehensive security patrol covering all designated checkpoints",
      status: "idle",
      taskCount: 5,
      completedTasks: 5,
      totalDuration: 12.5,
      lastExecuted: new Date(Date.now() - 3600000),
      successRate: 98.2,
      tags: ["security", "patrol", "autonomous"],
    },
    {
      id: "seq_2",
      name: "Maintenance Inspection",
      description: "Automated maintenance and system health verification",
      status: "running",
      taskCount: 8,
      completedTasks: 3,
      totalDuration: 25.0,
      successRate: 94.7,
      tags: ["maintenance", "inspection", "diagnostics"],
    },
    {
      id: "seq_3",
      name: "Delivery Protocol",
      description: "End-to-end delivery sequence with obstacle avoidance",
      status: "completed",
      taskCount: 6,
      completedTasks: 6,
      totalDuration: 8.3,
      lastExecuted: new Date(Date.now() - 1800000),
      successRate: 96.1,
      tags: ["delivery", "navigation", "pickup"],
    },
  ]);

  const [tasks, setTasks] = useState<Task[]>([
    {
      id: "task_1",
      name: "Navigate to Checkpoint Alpha",
      type: "navigation",
      status: "completed",
      duration: 180,
      priority: "high",
      description:
        "Move to security checkpoint Alpha using optimal path planning",
      parameters: { target: "checkpoint_alpha", speed: 0.8, precision: "high" },
      progress: 100,
      lastExecuted: new Date(Date.now() - 3600000),
      successRate: 99.1,
    },
    {
      id: "task_2",
      name: "Scan Environment",
      type: "sensor",
      status: "idle",
      duration: 45,
      priority: "medium",
      description: "360-degree environmental scan using LIDAR and cameras",
      parameters: { scan_type: "full", resolution: "high", duration: 30 },
      successRate: 97.8,
    },
    {
      id: "task_3",
      name: "Object Recognition",
      type: "ai",
      status: "idle",
      duration: 90,
      priority: "high",
      description: "AI-powered object detection and anomaly identification",
      parameters: {
        model: "yolo_v8",
        confidence: 0.85,
        classes: ["person", "vehicle"],
      },
      successRate: 94.3,
    },
    {
      id: "task_4",
      name: "Pick and Place Operation",
      type: "manipulation",
      status: "running",
      duration: 240,
      priority: "critical",
      description: "Precise manipulation task with force feedback",
      parameters: { object: "package_01", destination: "drop_zone_b" },
      progress: 65,
      estimatedTime: 84,
      successRate: 91.7,
    },
    {
      id: "task_5",
      name: "Status Report",
      type: "custom",
      status: "idle",
      duration: 15,
      priority: "low",
      description: "Generate and transmit status report to control center",
      parameters: { format: "json", include_metrics: true },
      successRate: 99.9,
    },
  ]);

  const [executionStatus, setExecutionStatus] = useState({
    isRunning: false,
    currentSequence: null as string | null,
    totalProgress: 45,
    estimatedTimeRemaining: 127,
    tasksCompleted: 3,
    totalTasks: 8,
  });

  const [logs, setLogs] = useState([
    {
      time: "14:32:15",
      level: "INFO",
      message: "Task 'Navigate to Checkpoint Alpha' completed successfully",
    },
    {
      time: "14:31:42",
      level: "INFO",
      message: "Starting environmental scan procedure",
    },
    {
      time: "14:31:15",
      level: "WARN",
      message: "Obstacle detected, recalculating path",
    },
    {
      time: "14:30:58",
      level: "INFO",
      message: "Sequence 'Security Patrol Route' initiated",
    },
  ]);

  const selectedTask = tasks.find((t) => t.id === sequencePrefs.selectedTaskId);
  const selectedSequence = sequences.find(
    (s) => s.id === sequencePrefs.selectedSequenceId,
  );

  // Task management functions
  const handleCreateTask = () => {
    if (!newTask.name) return;

    const task: Task = {
      id: `task_${Date.now()}`,
      name: newTask.name,
      type: newTask.type as Task["type"],
      status: "idle",
      duration: 60,
      priority: newTask.priority as Task["priority"],
      description: newTask.description || "",
      parameters: newTask.parameters || {},
      successRate: 0,
    };

    setTasks([...tasks, task]);
    setNewTask({
      name: "",
      type: "navigation",
      priority: "medium",
      description: "",
      parameters: {},
    });
    setIsTaskDialogOpen(false);
  };

  const handleEditTask = (task: Task) => {
    setEditingTask(task);
    setNewTask(task);
    setIsTaskDialogOpen(true);
  };

  const handleUpdateTask = () => {
    if (!editingTask || !newTask.name) return;

    setTasks(
      tasks.map((t) => (t.id === editingTask.id ? { ...t, ...newTask } : t)),
    );

    setEditingTask(null);
    setNewTask({
      name: "",
      type: "navigation",
      priority: "medium",
      description: "",
      parameters: {},
    });
    setIsTaskDialogOpen(false);
  };

  const handleDeleteTask = (taskId: string) => {
    setTasks(tasks.filter((t) => t.id !== taskId));
    if (sequencePrefs.selectedTaskId === taskId) {
      updateField("selectedTaskId", null);
    }
  };

  const handleExecuteTask = (taskId: string) => {
    setTasks(
      tasks.map((t) =>
        t.id === taskId ? { ...t, status: "running" as const, progress: 0 } : t,
      ),
    );
  };

  const handleRunSequence = (sequenceId: string) => {
    updateField("selectedSequenceId", sequenceId);
    setSequences(
      sequences.map((s) =>
        s.id === sequenceId ? { ...s, status: "running" as const } : s,
      ),
    );
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "completed":
        return "bg-emerald-500/10 text-emerald-700 border-emerald-200";
      case "running":
        return "bg-blue-500/10 text-blue-700 border-blue-200";
      case "failed":
        return "bg-red-500/10 text-red-700 border-red-200";
      case "paused":
        return "bg-yellow-500/10 text-yellow-700 border-yellow-200";
      default:
        return "bg-white/5 border border-white/100/10 text-slate-300 border-gray-200";
    }
  };

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case "critical":
        return "bg-red-500";
      case "high":
        return "bg-orange-500";
      case "medium":
        return "bg-yellow-500";
      default:
        return "bg-gray-400";
    }
  };

  const TypeIcon = ({ type }: { type: string }) => {
    switch (type) {
      case "navigation":
        return <div className="w-3 h-3 rounded-full bg-blue-500" />;
      case "manipulation":
        return <div className="w-3 h-3 rounded-full bg-purple-500" />;
      case "sensor":
        return <div className="w-3 h-3 rounded-full bg-green-500" />;
      case "ai":
        return <div className="w-3 h-3 rounded-full bg-pink-500" />;
      default:
        return (
          <div className="w-3 h-3 rounded-full bg-white/5 border border-white/100" />
        );
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Tesla-inspired Header with Glass Effect */}
      <div className="mb-8">
        <div className="bg-white/5 backdrop-blur-xl border border-white/10 rounded-3xl p-6 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="space-y-2">
              <h1 className="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
                Robot Sequences
              </h1>
              <p className="text-slate-400 font-light">
                Advanced task orchestration and autonomous execution control
              </p>
            </div>
            <div className="flex items-center gap-3">
              <Button className="bg-white/10 hover:bg-white/20 border border-white/20 backdrop-blur-sm transition-all duration-300">
                <Upload className="h-4 w-4 mr-2" />
                Import
              </Button>
              <Button className="bg-white/10 hover:bg-white/20 border border-white/20 backdrop-blur-sm transition-all duration-300">
                <Download className="h-4 w-4 mr-2" />
                Export
              </Button>
              <Button className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600 text-white shadow-lg transition-all duration-300">
                <Save className="h-4 w-4 mr-2" />
                Save All
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* Status Cards and Controls Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
        {/* Status Cards */}
        <div className="lg:col-span-2">
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
            {/* Active Sequences Card */}
            <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-4 hover:bg-white/10 transition-all duration-300 shadow-xl">
              <div className="text-center">
                <div className="h-10 w-10 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
                  <PlayCircle className="h-5 w-5 text-blue-400" />
                </div>
                <p className="text-slate-400 text-xs font-medium">
                  Active Sequences
                </p>
                <p className="text-xl font-light text-white mt-1">
                  {sequences.filter((s) => s.status === "running").length}
                </p>
              </div>
            </Card>

            {/* Execution Status Card */}
            <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-4 hover:bg-white/10 transition-all duration-300 shadow-xl">
              <div className="text-center">
                <div className="h-10 w-10 bg-gradient-to-br from-emerald-500/20 to-green-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
                  <Activity className="h-5 w-5 text-emerald-400" />
                </div>
                <p className="text-slate-400 text-xs font-medium">Progress</p>
                <p className="text-xl font-light text-white mt-1">
                  {executionStatus.totalProgress}%
                </p>
              </div>
            </Card>

            {/* Success Rate Card */}
            <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-4 hover:bg-white/10 transition-all duration-300 shadow-xl">
              <div className="text-center">
                <div className="h-10 w-10 bg-gradient-to-br from-emerald-500/20 to-teal-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
                  <CheckCircle2 className="h-5 w-5 text-emerald-400" />
                </div>
                <p className="text-slate-400 text-xs font-medium">
                  Success Rate
                </p>
                <p className="text-xl font-light text-white mt-1">96.2%</p>
              </div>
            </Card>

            {/* Active Tasks Card */}
            <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-4 hover:bg-white/10 transition-all duration-300 shadow-xl">
              <div className="text-center">
                <div className="h-10 w-10 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
                  <Timer className="h-5 w-5 text-purple-400" />
                </div>
                <p className="text-slate-400 text-xs font-medium">
                  Active Tasks
                </p>
                <p className="text-xl font-light text-white mt-1">
                  {tasks.filter((t) => t.status === "running").length}
                </p>
              </div>
            </Card>
          </div>

          {/* Sequence Selection */}
          <Card className="bg-white/5 backdrop-blur-xl border border-white/10 p-6 shadow-xl">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-lg font-light text-white">
                Available Sequences
              </h3>
              <Button
                onClick={() => setIsSequenceDialogOpen(true)}
                className="bg-white/10 hover:bg-white/20 border border-white/20"
                size="sm"
              >
                <Plus className="h-4 w-4 mr-2" />
                New Sequence
              </Button>
            </div>
            <div className="grid grid-cols-1 gap-3">
              {sequences.map((sequence) => (
                <div
                  key={sequence.id}
                  onClick={() => updateField("selectedSequenceId", sequence.id)}
                  className={`p-4 rounded-lg border transition-all duration-200 cursor-pointer ${
                    sequencePrefs.selectedSequenceId === sequence.id
                      ? "bg-blue-500/10 border-blue-400"
                      : "bg-white/5 border-white/10 hover:bg-white/10"
                  }`}
                >
                  <div className="flex items-center justify-between">
                    <div className="flex-1">
                      <div className="flex items-center gap-3">
                        <FolderOpen className="h-4 w-4 text-blue-400" />
                        <h4 className="font-medium text-white">
                          {sequence.name}
                        </h4>
                        <Badge
                          className={`text-xs ${getStatusColor(sequence.status)}`}
                        >
                          {sequence.status}
                        </Badge>
                      </div>
                      <p className="text-sm text-slate-400 mt-1">
                        {sequence.description}
                      </p>
                      <div className="flex items-center gap-4 mt-2 text-xs text-slate-400">
                        <span>{sequence.taskCount} tasks</span>
                        <span>{sequence.successRate}% success</span>
                        {sequence.lastExecuted && (
                          <span>
                            Last: {sequence.lastExecuted.toLocaleTimeString()}
                          </span>
                        )}
                      </div>
                    </div>
                    <Button
                      onClick={(e) => {
                        e.stopPropagation();
                        handleRunSequence(sequence.id);
                      }}
                      className="bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600"
                      size="sm"
                    >
                      <Play className="h-4 w-4" />
                    </Button>
                  </div>
                </div>
              ))}
            </div>
          </Card>
        </div>

        {/* Execution Logs - Moved from bottom to top section */}
        <div className="lg:col-span-1">
          <Card className="bg-white/5 backdrop-blur-xl border border-white/10 shadow-xl h-full">
            <div className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-light text-white">
                  Execution Logs
                </h3>
                <Button
                  size="sm"
                  className="bg-white/10 hover:bg-white/20 border border-white/20"
                >
                  <FileText className="h-4 w-4" />
                </Button>
              </div>

              <ScrollArea className="h-[400px]">
                <div className="space-y-2">
                  {logs.map((log, index) => (
                    <div
                      key={index}
                      className="flex gap-3 p-2 rounded-lg hover:bg-white/5 transition-colors"
                    >
                      <span className="text-xs font-mono text-slate-400 w-16 flex-shrink-0">
                        {log.time}
                      </span>
                      <span
                        className={`text-xs font-medium w-12 flex-shrink-0 ${
                          log.level === "INFO"
                            ? "text-blue-400"
                            : log.level === "WARN"
                              ? "text-yellow-400"
                              : "text-red-400"
                        }`}
                      >
                        {log.level}
                      </span>
                      <span className="text-xs text-slate-300 flex-1">
                        {log.message}
                      </span>
                    </div>
                  ))}
                </div>
              </ScrollArea>

              {/* Current execution status */}
              {executionStatus.isRunning && (
                <div className="mt-4 p-3 bg-blue-500/10 border border-blue-400/30 rounded-lg">
                  <div className="flex items-center gap-2 mb-2">
                    <Activity className="h-4 w-4 text-blue-400" />
                    <span className="text-sm font-medium text-blue-400">
                      Executing Sequence
                    </span>
                  </div>
                  <Progress
                    value={executionStatus.totalProgress}
                    className="h-1 bg-slate-700 mb-2"
                  />
                  <div className="flex justify-between text-xs text-slate-400">
                    <span>
                      {executionStatus.tasksCompleted}/
                      {executionStatus.totalTasks} tasks
                    </span>
                    <span>
                      {executionStatus.estimatedTimeRemaining}s remaining
                    </span>
                  </div>
                </div>
              )}
            </div>
          </Card>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Task List - POS Style */}
        <div className="lg:col-span-2">
          <Card className="bg-white/5 backdrop-blur-xl border border-white/10 shadow-2xl">
            <div className="p-6 border-b border-white/10">
              <div className="flex items-center justify-between">
                <h2 className="text-xl font-light text-white">
                  Task Management
                </h2>
                <div className="flex items-center gap-3">
                  <div className="relative">
                    <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-slate-400" />
                    <Input
                      placeholder="Search tasks..."
                      className="pl-10 bg-white/5 border-white/20 text-white placeholder-slate-400 focus:border-blue-400"
                    />
                  </div>
                  <Button
                    size="sm"
                    className="bg-white/10 hover:bg-white/20 border border-white/20"
                  >
                    <Filter className="h-4 w-4" />
                  </Button>
                  <Button
                    onClick={() => {
                      setEditingTask(null);
                      setNewTask({
                        name: "",
                        type: "navigation",
                        priority: "medium",
                        description: "",
                        parameters: {},
                      });
                      setIsTaskDialogOpen(true);
                    }}
                    className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600"
                    size="sm"
                  >
                    <Plus className="h-4 w-4 mr-2" />
                    Add Task
                  </Button>
                </div>
              </div>
            </div>

            <ScrollArea className="h-[500px]">
              <div className="divide-y divide-white/10">
                {tasks.map((task, index) => (
                  <div
                    key={task.id}
                    onClick={() => updateField("selectedTaskId", task.id)}
                    className={`p-4 cursor-pointer transition-all duration-200 hover:bg-white/5 ${
                      sequencePrefs.selectedTaskId === task.id
                        ? "bg-blue-500/10 border-l-4 border-blue-400"
                        : ""
                    }`}
                  >
                    <div className="flex items-center gap-4">
                      {/* Line Number */}
                      <div className="w-8 h-8 rounded-full bg-slate-700 flex items-center justify-center text-sm font-mono text-slate-300">
                        {String(index + 1).padStart(2, "0")}
                      </div>

                      {/* Task Type Indicator */}
                      <TypeIcon type={task.type} />

                      {/* Priority Bar */}
                      <div
                        className={`w-1 h-8 rounded-full ${getPriorityColor(task.priority)}`}
                      />

                      {/* Task Info */}
                      <div className="flex-1 min-w-0">
                        <div className="flex items-center gap-2">
                          <h3 className="font-medium text-white truncate">
                            {task.name}
                          </h3>
                          <Badge
                            className={`text-xs ${getStatusColor(task.status)}`}
                          >
                            {task.status}
                          </Badge>
                        </div>
                        <p className="text-sm text-slate-400 truncate mt-1">
                          {task.description}
                        </p>

                        {task.status === "running" && task.progress && (
                          <div className="mt-2">
                            <Progress
                              value={task.progress}
                              className="h-1 bg-slate-700"
                            />
                          </div>
                        )}
                      </div>

                      {/* Duration & Success Rate */}
                      <div className="text-right">
                        <p className="text-sm font-mono text-white">
                          {task.duration}s
                        </p>
                        {task.successRate && (
                          <p className="text-xs text-slate-400">
                            {task.successRate}% success
                          </p>
                        )}
                      </div>

                      {/* Action Buttons */}
                      <div className="flex items-center gap-2">
                        {task.status === "running" ? (
                          <Button
                            size="sm"
                            variant="outline"
                            className="border-yellow-500/50 text-yellow-400 hover:bg-yellow-500/10"
                          >
                            <PauseCircle className="h-4 w-4" />
                          </Button>
                        ) : task.status === "paused" ? (
                          <Button
                            size="sm"
                            variant="outline"
                            className="border-blue-500/50 text-blue-400 hover:bg-blue-500/10"
                          >
                            <PlayCircle className="h-4 w-4" />
                          </Button>
                        ) : (
                          <Button
                            size="sm"
                            variant="outline"
                            className="border-emerald-500/50 text-emerald-400 hover:bg-emerald-500/10"
                            onClick={(e) => {
                              e.stopPropagation();
                              handleExecuteTask(task.id);
                            }}
                          >
                            <Play className="h-4 w-4" />
                          </Button>
                        )}
                        <Button
                          size="sm"
                          variant="outline"
                          className="border-blue-500/50 text-blue-400 hover:bg-blue-500/10"
                          onClick={(e) => {
                            e.stopPropagation();
                            handleEditTask(task);
                          }}
                        >
                          <Edit className="h-4 w-4" />
                        </Button>
                        <Button
                          size="sm"
                          variant="outline"
                          className="border-red-500/50 text-red-400 hover:bg-red-500/10"
                          onClick={(e) => {
                            e.stopPropagation();
                            handleDeleteTask(task.id);
                          }}
                        >
                          <Trash2 className="h-4 w-4" />
                        </Button>
                        <ChevronRight className="h-4 w-4 text-slate-400" />
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </ScrollArea>
          </Card>
        </div>

        {/* Task Details & Logs */}
        <div className="space-y-6">
          {/* Selected Task Details */}
          {selectedTask && (
            <Card className="bg-white/5 backdrop-blur-xl border border-white/10 shadow-2xl">
              <div className="p-6">
                <div className="flex items-center justify-between mb-4">
                  <h3 className="text-lg font-light text-white">
                    Task Details
                  </h3>
                  <Button
                    size="sm"
                    className="bg-white/10 hover:bg-white/20 border border-white/20"
                  >
                    <Edit className="h-4 w-4" />
                  </Button>
                </div>

                <div className="space-y-4">
                  <div>
                    <p className="text-sm font-medium text-slate-400">Name</p>
                    <p className="text-white">{selectedTask.name}</p>
                  </div>

                  <div>
                    <p className="text-sm font-medium text-slate-400">Type</p>
                    <div className="flex items-center gap-2 mt-1">
                      <TypeIcon type={selectedTask.type} />
                      <span className="text-white capitalize">
                        {selectedTask.type}
                      </span>
                    </div>
                  </div>

                  <div>
                    <p className="text-sm font-medium text-slate-400">Status</p>
                    <Badge
                      className={`mt-1 ${getStatusColor(selectedTask.status)}`}
                    >
                      {selectedTask.status}
                    </Badge>
                  </div>

                  <div>
                    <p className="text-sm font-medium text-slate-400">
                      Description
                    </p>
                    <p className="text-sm text-slate-300 mt-1">
                      {selectedTask.description}
                    </p>
                  </div>

                  {selectedTask.progress && (
                    <div>
                      <p className="text-sm font-medium text-slate-400">
                        Progress
                      </p>
                      <div className="mt-2">
                        <Progress
                          value={selectedTask.progress}
                          className="h-2 bg-slate-700"
                        />
                        <p className="text-xs text-slate-400 mt-1">
                          {selectedTask.progress}% complete
                        </p>
                      </div>
                    </div>
                  )}

                  <div className="pt-4 border-t border-white/10">
                    <div className="flex gap-2">
                      <Button className="flex-1 bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600">
                        <Play className="h-4 w-4 mr-2" />
                        Execute
                      </Button>
                      <Button
                        variant="outline"
                        className="border-white/20 text-white hover:bg-white/10"
                      >
                        <Settings className="h-4 w-4" />
                      </Button>
                    </div>
                  </div>
                </div>
              </div>
            </Card>
          )}
        </div>
      </div>

      {/* Task Dialog */}
      <Dialog open={isTaskDialogOpen} onOpenChange={setIsTaskDialogOpen}>
        <DialogContent className="bg-slate-900 border border-white/20 text-white max-w-md">
          <DialogHeader>
            <DialogTitle className="text-xl font-light">
              {editingTask ? "Edit Task" : "Create New Task"}
            </DialogTitle>
            <DialogDescription className="text-slate-400">
              {editingTask
                ? "Modify the task parameters below."
                : "Define a new task for your sequence."}
            </DialogDescription>
          </DialogHeader>

          <div className="space-y-4">
            <div>
              <Label htmlFor="task-name" className="text-slate-300">
                Task Name
              </Label>
              <Input
                id="task-name"
                value={newTask.name}
                onChange={(e) =>
                  setNewTask({ ...newTask, name: e.target.value })
                }
                className="bg-white/5 border-white/20 text-white mt-1"
                placeholder="Enter task name..."
              />
            </div>

            <div>
              <Label htmlFor="task-type" className="text-slate-300">
                Type
              </Label>
              <Select
                value={newTask.type}
                onValueChange={(value) =>
                  setNewTask({ ...newTask, type: value as Task["type"] })
                }
              >
                <SelectTrigger className="bg-white/5 border-white/20 text-white mt-1">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent className="bg-slate-900 border-white/20">
                  <SelectItem value="navigation">Navigation</SelectItem>
                  <SelectItem value="manipulation">Manipulation</SelectItem>
                  <SelectItem value="sensor">Sensor</SelectItem>
                  <SelectItem value="ai">AI Processing</SelectItem>
                  <SelectItem value="custom">Custom</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div>
              <Label htmlFor="task-priority" className="text-slate-300">
                Priority
              </Label>
              <Select
                value={newTask.priority}
                onValueChange={(value) =>
                  setNewTask({
                    ...newTask,
                    priority: value as Task["priority"],
                  })
                }
              >
                <SelectTrigger className="bg-white/5 border-white/20 text-white mt-1">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent className="bg-slate-900 border-white/20">
                  <SelectItem value="low">Low</SelectItem>
                  <SelectItem value="medium">Medium</SelectItem>
                  <SelectItem value="high">High</SelectItem>
                  <SelectItem value="critical">Critical</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div>
              <Label htmlFor="task-description" className="text-slate-300">
                Description
              </Label>
              <Textarea
                id="task-description"
                value={newTask.description}
                onChange={(e) =>
                  setNewTask({ ...newTask, description: e.target.value })
                }
                className="bg-white/5 border-white/20 text-white mt-1"
                placeholder="Describe the task..."
                rows={3}
              />
            </div>
          </div>

          <DialogFooter className="gap-2">
            <Button
              variant="outline"
              onClick={() => setIsTaskDialogOpen(false)}
              className="border-white/20 text-white hover:bg-white/10"
            >
              Cancel
            </Button>
            <Button
              onClick={editingTask ? handleUpdateTask : handleCreateTask}
              className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600"
            >
              {editingTask ? "Update Task" : "Create Task"}
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {/* Sequence Dialog */}
      <Dialog
        open={isSequenceDialogOpen}
        onOpenChange={setIsSequenceDialogOpen}
      >
        <DialogContent className="bg-slate-900 border border-white/20 text-white max-w-md">
          <DialogHeader>
            <DialogTitle className="text-xl font-light">
              Create New Sequence
            </DialogTitle>
            <DialogDescription className="text-slate-400">
              Create a new task sequence for automated execution.
            </DialogDescription>
          </DialogHeader>

          <div className="space-y-4">
            <div>
              <Label htmlFor="sequence-name" className="text-slate-300">
                Sequence Name
              </Label>
              <Input
                id="sequence-name"
                className="bg-white/5 border-white/20 text-white mt-1"
                placeholder="Enter sequence name..."
              />
            </div>

            <div>
              <Label htmlFor="sequence-description" className="text-slate-300">
                Description
              </Label>
              <Textarea
                id="sequence-description"
                className="bg-white/5 border-white/20 text-white mt-1"
                placeholder="Describe the sequence..."
                rows={3}
              />
            </div>
          </div>

          <DialogFooter className="gap-2">
            <Button
              variant="outline"
              onClick={() => setIsSequenceDialogOpen(false)}
              className="border-white/20 text-white hover:bg-white/10"
            >
              Cancel
            </Button>
            <Button
              onClick={() => setIsSequenceDialogOpen(false)}
              className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600"
            >
              Create Sequence
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}
