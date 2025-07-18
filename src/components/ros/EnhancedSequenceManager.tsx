import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { TaskCardWithSubtasks } from "./TaskCardWithSubtasks";
import { SequenceEditor, Sequence } from "./SequenceEditor";
import { TaskParameterEditor } from "./TaskParameterEditor";
import { FeatureManager } from "./FeatureManager";
import { Task } from "./TaskCard";
import {
  Play,
  Pause,
  Square,
  RotateCcw,
  Plus,
  List,
  Settings,
  Clock,
  CheckCircle2,
  XCircle,
  AlertTriangle,
  Activity,
  Layers,
  Timer,
  Download,
  Upload,
  Save,
  FolderOpen,
  Copy,
  Trash2,
  FileText,
  BarChart3,
  Zap,
  Shield,
  Globe,
  Smartphone,
  Eye,
  EyeOff,
  Maximize2,
  Minimize2,
  Table as TableIcon,
  Grid3X3,
  MoreHorizontal,
  Edit,
  PlayCircle,
  PauseCircle,
  StopCircle,
} from "lucide-react";

export function EnhancedSequenceManager() {
  const [activeSequence, setActiveSequence] = useState<Sequence | null>(null);
  const [sequences, setSequences] = useState<Sequence[]>([]);
  const [tasks, setTasks] = useState<Task[]>([]);
  const [isEditing, setIsEditing] = useState(false);
  const [executionLogs, setExecutionLogs] = useState<string[]>([]);
  const [selectedTask, setSelectedTask] = useState<Task | null>(null);
  const [showTaskEditor, setShowTaskEditor] = useState(false);
  const [showFeatureManager, setShowFeatureManager] = useState(false);
  const [showImportDialog, setShowImportDialog] = useState(false);
  const [showExportDialog, setShowExportDialog] = useState(false);
  const [viewMode, setViewMode] = useState<
    "overview" | "execution" | "editing"
  >("overview");
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);
  const [focusedCard, setFocusedCard] = useState<string | null>(null);
  const [taskViewMode, setTaskViewMode] = useState<"cards" | "table">("table");

  // Performance metrics
  const [metrics, setMetrics] = useState({
    totalExecutions: 125,
    successRate: 94.4,
    averageTime: 245,
    totalRuntime: 45.2,
    lastExecution: new Date(Date.now() - 3600000),
  });

  // Enhanced mock data with new task types
  useEffect(() => {
    const mockSequences: Sequence[] = [
      {
        id: "security_patrol_v2",
        name: "Advanced Security Patrol",
        description: "AI-powered security patrol with anomaly detection",
        isParallel: false,
        maxConcurrency: 1,
        executionMode: {
          type: "sequential",
          maxConcurrency: 1,
          waitForFeedback: false,
          globalFeedbackTimeout: 30,
          failureHandling: "retry",
          retryCount: 2,
        },
        status: "idle",
        createdAt: new Date(Date.now() - 86400000),
        updatedAt: new Date(Date.now() - 3600000),
        tags: ["security", "ai", "patrol", "vision"],
        taskIds: ["nav_checkpoint_a", "ai_anomaly_scan", "voice_report"],
      },
      {
        id: "maintenance_routine",
        name: "Predictive Maintenance Check",
        description: "Comprehensive system health and maintenance assessment",
        isParallel: true,
        maxConcurrency: 3,
        executionMode: {
          type: "parallel",
          maxConcurrency: 3,
          waitForFeedback: true,
          globalFeedbackTimeout: 60,
          failureHandling: "continue",
          retryCount: 1,
        },
        status: "completed",
        createdAt: new Date(Date.now() - 172800000),
        updatedAt: new Date(),
        tags: ["maintenance", "diagnostics", "health"],
        taskIds: ["system_diagnostics", "sensor_calibration", "battery_check"],
      },
    ];

    const mockTasks: Task[] = [
      {
        id: "nav_checkpoint_a",
        name: "Navigate to Security Point Alpha",
        type: "movement",
        description:
          "Move to designated security checkpoint with obstacle avoidance",
        parameters: {
          position: { x: 5.2, y: 3.1, z: 0 },
          orientation: { roll: 0, pitch: 0, yaw: 0 },
          speed: 1.5,
          precision: 0.05,
          moveType: "linear",
          coordinateFrame: "map",
          pathPlanning: true,
          obstacleAvoidance: true,
        },
        timeout: 120,
        retries: 2,
        waitForFeedback: false,
        feedbackTimeout: 30,
        status: "completed",
        progress: 100,
        duration: 45,
        startTime: new Date(Date.now() - 300000),
        endTime: new Date(Date.now() - 255000),
        dependencies: [],
        hasSubtasks: false,
        subtasks: [],
        subtaskExecutionMode: "sequential",
        maxSubtaskConcurrency: 2,
        subtaskWaitForFeedback: false,
      },
      {
        id: "ai_anomaly_scan",
        name: "AI-Powered Anomaly Detection",
        type: "vision",
        description:
          "Scan environment for security anomalies using computer vision",
        parameters: {
          camera: "security_camera",
          processingType: "object_detection",
          targetObject: "person,vehicle,suspicious_object",
          confidence: 0.85,
          saveImage: true,
          imageFormat: "jpg",
          resolution: "1920x1080",
          roi: { x: 0, y: 0, width: 100, height: 100 },
        },
        timeout: 60,
        retries: 1,
        waitForFeedback: true,
        feedbackTimeout: 120,
        status: "running",
        progress: 65,
        duration: 39,
        startTime: new Date(Date.now() - 39000),
        dependencies: ["nav_checkpoint_a"],
        hasSubtasks: true,
        subtasks: [
          {
            id: "capture_image",
            name: "Capture Security Image",
            type: "sensor_reading",
            description: "Take high-resolution security photograph",
            parameters: { camera: "front", quality: "high" },
            timeout: 10,
            retries: 1,
            waitForFeedback: false,
            feedbackTimeout: 30,
            status: "completed",
            progress: 100,
            duration: 3,
          },
          {
            id: "analyze_threats",
            name: "Analyze Potential Threats",
            type: "ai_processing",
            description: "Process image for threat detection",
            parameters: { model: "security_v3", confidence: 0.9 },
            timeout: 30,
            retries: 2,
            waitForFeedback: false,
            feedbackTimeout: 30,
            status: "running",
            progress: 65,
            duration: 20,
          },
        ],
        subtaskExecutionMode: "sequential",
        maxSubtaskConcurrency: 2,
        subtaskWaitForFeedback: false,
      },
      {
        id: "voice_report",
        name: "Voice Status Report",
        type: "voice_command",
        description: "Generate and deliver voice status report",
        parameters: {
          language: "en-US",
          reportType: "security_summary",
          voiceModel: "professional",
          destination: "control_room",
        },
        timeout: 30,
        retries: 1,
        waitForFeedback: false,
        feedbackTimeout: 30,
        status: "pending",
        progress: 0,
        duration: 0,
        dependencies: ["ai_anomaly_scan"],
        hasSubtasks: false,
        subtasks: [],
        subtaskExecutionMode: "sequential",
        maxSubtaskConcurrency: 2,
        subtaskWaitForFeedback: false,
      },
      {
        id: "system_diagnostics",
        name: "Comprehensive System Diagnostics",
        type: "maintenance",
        description: "Run full system health check and diagnostics",
        parameters: {
          checkType: "diagnostics",
          component: "all",
          severity: "routine",
          autoFix: true,
          reportGeneration: true,
          scheduleNext: 30,
        },
        timeout: 300,
        retries: 1,
        waitForFeedback: true,
        feedbackTimeout: 180,
        status: "completed",
        progress: 100,
        duration: 150,
        dependencies: [],
        hasSubtasks: false,
        subtasks: [],
        subtaskExecutionMode: "parallel",
        maxSubtaskConcurrency: 3,
        subtaskWaitForFeedback: false,
      },
      {
        id: "sensor_calibration",
        name: "Multi-Sensor Calibration",
        type: "sensor_reading",
        description: "Calibrate all sensors for optimal performance",
        parameters: {
          sensorType: "all",
          duration: 60,
          samplingRate: 100,
          autoCalibrate: true,
          saveCalibration: true,
        },
        timeout: 180,
        retries: 2,
        waitForFeedback: false,
        feedbackTimeout: 30,
        status: "running",
        progress: 40,
        duration: 72,
        dependencies: [],
        hasSubtasks: false,
        subtasks: [],
        subtaskExecutionMode: "parallel",
        maxSubtaskConcurrency: 4,
        subtaskWaitForFeedback: false,
      },
      {
        id: "battery_check",
        name: "Battery Health Assessment",
        type: "safety_check",
        description: "Analyze battery performance and health",
        parameters: {
          checkType: "battery",
          threshold: 20,
          action: "warn",
          reportHealth: true,
          optimizePower: true,
        },
        timeout: 60,
        retries: 1,
        waitForFeedback: false,
        feedbackTimeout: 30,
        status: "pending",
        progress: 0,
        duration: 0,
        dependencies: [],
        hasSubtasks: false,
        subtasks: [],
        subtaskExecutionMode: "sequential",
        maxSubtaskConcurrency: 2,
        subtaskWaitForFeedback: false,
      },
    ];

    setSequences(mockSequences);
    setTasks(mockTasks);
    setActiveSequence(mockSequences[0]);
  }, []);

  // File operations
  const exportSequence = (sequence: Sequence) => {
    const sequenceTasks = tasks.filter((task) =>
      sequence.taskIds.includes(task.id),
    );
    const exportData = {
      sequence,
      tasks: sequenceTasks,
      exportDate: new Date().toISOString(),
      version: "2.0.0",
    };

    const blob = new Blob([JSON.stringify(exportData, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${sequence.name.replace(/\s+/g, "_")}.json`;
    a.click();
    URL.revokeObjectURL(url);

    addLog(`Exported sequence: ${sequence.name}`);
  };

  const importSequence = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const importData = JSON.parse(e.target?.result as string);
        const { sequence, tasks: importedTasks } = importData;

        // Generate new IDs to avoid conflicts
        const newSequenceId = `${sequence.id}_imported_${Date.now()}`;
        const taskIdMapping: Record<string, string> = {};

        // Update task IDs
        const updatedTasks = importedTasks.map((task: Task) => {
          const newTaskId = `${task.id}_imported_${Date.now()}`;
          taskIdMapping[task.id] = newTaskId;
          return { ...task, id: newTaskId };
        });

        // Update sequence with new task IDs
        const updatedSequence = {
          ...sequence,
          id: newSequenceId,
          name: `${sequence.name} (Imported)`,
          taskIds: sequence.taskIds.map((id: string) => taskIdMapping[id]),
          createdAt: new Date(),
          updatedAt: new Date(),
        };

        setSequences((prev) => [...prev, updatedSequence]);
        setTasks((prev) => [...prev, ...updatedTasks]);
        setActiveSequence(updatedSequence);

        addLog(`Imported sequence: ${updatedSequence.name}`);
      } catch (error) {
        addLog(`Failed to import sequence: ${error}`);
      }
    };
    reader.readAsText(file);
  };

  const duplicateSequence = (sequence: Sequence) => {
    const sequenceTasks = tasks.filter((task) =>
      sequence.taskIds.includes(task.id),
    );
    const taskIdMapping: Record<string, string> = {};

    // Create new tasks with updated IDs
    const newTasks = sequenceTasks.map((task) => {
      const newTaskId = `${task.id}_copy_${Date.now()}`;
      taskIdMapping[task.id] = newTaskId;
      return {
        ...task,
        id: newTaskId,
        name: `${task.name} (Copy)`,
        status: "pending" as const,
        progress: 0,
        duration: 0,
        startTime: undefined,
        endTime: undefined,
      };
    });

    // Create new sequence
    const newSequence: Sequence = {
      ...sequence,
      id: `${sequence.id}_copy_${Date.now()}`,
      name: `${sequence.name} (Copy)`,
      taskIds: sequence.taskIds.map((id) => taskIdMapping[id]),
      status: "idle",
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    setSequences((prev) => [...prev, newSequence]);
    setTasks((prev) => [...prev, ...newTasks]);
    addLog(`Duplicated sequence: ${newSequence.name}`);
  };

  const getSequenceTasks = (sequence: Sequence | null) => {
    if (!sequence) return [];
    return tasks.filter((task) => sequence.taskIds.includes(task.id));
  };

  const getSequenceProgress = (sequence: Sequence) => {
    const sequenceTasks = getSequenceTasks(sequence);
    if (sequenceTasks.length === 0) return 0;

    const totalProgress = sequenceTasks.reduce(
      (sum, task) => sum + task.progress,
      0,
    );
    return Math.round(totalProgress / sequenceTasks.length);
  };

  const getSequenceStatus = (sequence: Sequence) => {
    const sequenceTasks = getSequenceTasks(sequence);
    if (sequenceTasks.some((task) => task.status === "failed")) return "failed";
    if (sequenceTasks.some((task) => task.status === "running"))
      return "running";
    if (sequenceTasks.some((task) => task.status === "paused")) return "paused";
    if (sequenceTasks.every((task) => task.status === "completed"))
      return "completed";
    return "idle";
  };

  const handleSequenceAction = (action: string, sequenceId: string) => {
    console.log(`${action} sequence: ${sequenceId}`);
    if (action === "start") {
      setViewMode("execution");
      setFocusedCard("execution-status");
    }
    addLog(`${action.toUpperCase()} sequence: ${sequenceId}`);
  };

  const handleTaskAction = (action: string, taskId: string) => {
    console.log(`${action} task: ${taskId}`);
    if (action === "edit") {
      setViewMode("editing");
      setFocusedCard(taskId);
    }
    addLog(`${action.toUpperCase()} task: ${taskId}`);
  };

  const addLog = (message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    setExecutionLogs((prev) =>
      [`[${timestamp}] ${message}`, ...prev].slice(0, 100),
    );
  };

  const handleUpdateTask = (updatedTask: Task) => {
    setTasks((prev) =>
      prev.map((task) => (task.id === updatedTask.id ? updatedTask : task)),
    );
  };

  const handleDeleteTask = (taskId: string) => {
    setTasks((prev) => prev.filter((task) => task.id !== taskId));
    if (activeSequence) {
      setActiveSequence({
        ...activeSequence,
        taskIds: activeSequence.taskIds.filter((id) => id !== taskId),
      });
    }
    addLog(`Deleted task: ${taskId}`);
  };

  const handleDuplicateTask = (task: Task) => {
    const newTask = {
      ...task,
      id: `${task.id}_copy_${Date.now()}`,
      name: `${task.name} (Copy)`,
      status: "pending" as const,
      progress: 0,
      duration: 0,
      startTime: undefined,
      endTime: undefined,
    };
    setTasks((prev) => [...prev, newTask]);
    if (activeSequence) {
      setActiveSequence({
        ...activeSequence,
        taskIds: [...activeSequence.taskIds, newTask.id],
      });
    }
    addLog(`Duplicated task: ${task.name}`);
  };

  const createNewTask = () => {
    const newTask: Task = {
      id: `task_${Date.now()}`,
      name: "New Movement Task",
      type: "movement",
      description: "Move robot to a specified position",
      parameters: {
        movementMode: "auto_nav",
        position: { x: 0, y: 0, z: 0 },
        orientation: { roll: 0, pitch: 0, yaw: 0 },
        relativeDistance: { x: 0, y: 0, z: 0 },
        targetFrame: "map",
        speed: 1.0,
        acceleration: 0.5,
        precision: 0.01,
        moveType: "linear",
        coordinateFrame: "base_link",
        pathPlanning: true,
        obstacleAvoidance: true,
        useCostmap: true,
        costmapLayer: "global_costmap",
        plannerType: "navfn",
        recoveryBehavior: true,
        goalTolerance: { xy: 0.1, yaw: 0.1 },
        enableConditions: false,
        conditions: [],
        onSuccess: "continue",
        onFailure: "retry",
      },
      timeout: 60,
      retries: 1,
      waitForFeedback: false,
      feedbackTimeout: 30,
      status: "pending",
      progress: 0,
      duration: 0,
      dependencies: [],
      hasSubtasks: false,
      subtasks: [],
      subtaskExecutionMode: "sequential",
      maxSubtaskConcurrency: 2,
      subtaskWaitForFeedback: false,
    };

    setTasks((prev) => [...prev, newTask]);
    if (activeSequence) {
      setActiveSequence({
        ...activeSequence,
        taskIds: [...activeSequence.taskIds, newTask.id],
      });
    }
    // Auto-open parameter editor for new tasks
    setSelectedTask(newTask);
    setShowTaskEditor(true);
    setViewMode("editing");
    setFocusedCard(newTask.id);
    addLog(`Created new task: ${newTask.name} - Opening parameter editor`);
  };

  const currentTasks = getSequenceTasks(activeSequence);
  const runningTasks = currentTasks.filter((task) => task.status === "running");
  const pendingTasks = currentTasks.filter((task) => task.status === "pending");
  const completedTasks = currentTasks.filter(
    (task) => task.status === "completed",
  );

  // Task status utilities
  const getStatusColor = (status: string) => {
    switch (status) {
      case "running":
        return "text-blue-600 bg-blue-50";
      case "completed":
        return "text-green-600 bg-green-50";
      case "failed":
        return "text-red-600 bg-red-50";
      case "pending":
        return "text-orange-600 bg-orange-50";
      case "paused":
        return "text-yellow-600 bg-yellow-50";
      default:
        return "text-gray-600 bg-gray-50";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "running":
        return <PlayCircle className="h-4 w-4" />;
      case "completed":
        return <CheckCircle2 className="h-4 w-4" />;
      case "failed":
        return <XCircle className="h-4 w-4" />;
      case "pending":
        return <Clock className="h-4 w-4" />;
      case "paused":
        return <PauseCircle className="h-4 w-4" />;
      default:
        return <AlertTriangle className="h-4 w-4" />;
    }
  };

  const getTypeColor = (type: string) => {
    switch (type) {
      case "movement":
        return "bg-blue-100 text-blue-800";
      case "sensor_reading":
        return "bg-green-100 text-green-800";
      case "vision":
        return "bg-purple-100 text-purple-800";
      case "voice_command":
        return "bg-pink-100 text-pink-800";
      case "maintenance":
        return "bg-orange-100 text-orange-800";
      case "safety_check":
        return "bg-red-100 text-red-800";
      default:
        return "bg-gray-100 text-gray-800";
    }
  };

  // Smart layout based on view mode and context
  const getLayoutClass = () => {
    switch (viewMode) {
      case "execution":
        return "grid-cols-1 lg:grid-cols-4"; // Focus on execution status
      case "editing":
        return "grid-cols-1 lg:grid-cols-3"; // Balance for editing
      default:
        return "grid-cols-1 lg:grid-cols-5"; // Overview mode
    }
  };

  // Get card emphasis based on current focus
  const getCardClass = (cardId: string) => {
    const baseClass = "transition-all duration-300 ";
    if (focusedCard === cardId) {
      return baseClass + "ring-2 ring-primary shadow-lg scale-[1.02]";
    }
    return baseClass + "hover:shadow-md";
  };

  return (
    <div className="space-y-6">
      {/* Enhanced Header with Mode Switcher */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <div>
            <h2 className="text-2xl font-bold">
              {activeSequence?.name || "Sequence Manager"}
            </h2>
            <p className="text-muted-foreground">
              {activeSequence?.description || "Select a sequence to begin"}
            </p>
          </div>

          {/* View Mode Switcher */}
          <div className="flex items-center gap-2">
            <Badge variant="outline">Mode:</Badge>
            <div className="flex rounded-lg border">
              <Button
                variant={viewMode === "overview" ? "default" : "ghost"}
                size="sm"
                onClick={() => {
                  setViewMode("overview");
                  setFocusedCard(null);
                }}
                className="rounded-r-none"
              >
                <Eye className="h-4 w-4 mr-1" />
                Overview
              </Button>
              <Button
                variant={viewMode === "execution" ? "default" : "ghost"}
                size="sm"
                onClick={() => {
                  setViewMode("execution");
                  setFocusedCard("execution-status");
                }}
                className="rounded-none border-x"
              >
                <Play className="h-4 w-4 mr-1" />
                Execution
              </Button>
              <Button
                variant={viewMode === "editing" ? "default" : "ghost"}
                size="sm"
                onClick={() => {
                  setViewMode("editing");
                  setFocusedCard(selectedTask?.id || "task-editor");
                }}
                className="rounded-l-none"
              >
                <Settings className="h-4 w-4 mr-1" />
                Editing
              </Button>
            </div>
          </div>
        </div>

        {/* Header Actions */}
        <div className="flex items-center gap-2">
          {/* Task View Mode Switcher */}
          <div className="flex rounded-lg border">
            <Button
              variant={taskViewMode === "table" ? "default" : "ghost"}
              size="sm"
              onClick={() => setTaskViewMode("table")}
              className="rounded-r-none gap-1"
            >
              <TableIcon className="h-4 w-4" />
              Table
            </Button>
            <Button
              variant={taskViewMode === "cards" ? "default" : "ghost"}
              size="sm"
              onClick={() => setTaskViewMode("cards")}
              className="rounded-l-none border-l gap-1"
            >
              <Grid3X3 className="h-4 w-4" />
              Cards
            </Button>
          </div>

          <Separator orientation="vertical" className="h-6" />

          <Button
            variant="outline"
            size="sm"
            onClick={() => setSidebarCollapsed(!sidebarCollapsed)}
          >
            {sidebarCollapsed ? (
              <Maximize2 className="h-4 w-4" />
            ) : (
              <Minimize2 className="h-4 w-4" />
            )}
          </Button>
          <Button onClick={createNewTask} className="gap-2">
            <Plus className="h-4 w-4" />
            Add Task
          </Button>
        </div>
      </div>

      {/* Smart Layout Grid */}
      <div className={`grid gap-6 ${getLayoutClass()}`}>
        {/* Sidebar - Sequences List */}
        {!sidebarCollapsed && (
          <Card
            className={`${getCardClass("sequences-list")} ${viewMode === "overview" ? "lg:col-span-1" : "lg:col-span-1"}`}
          >
            <div className="p-4 border-b">
              <div className="flex items-center justify-between">
                <h3 className="font-semibold">Sequences</h3>
                <Badge variant="secondary">{sequences.length}</Badge>
              </div>
            </div>
            <ScrollArea className="h-[500px]">
              <div className="p-4 space-y-3">
                {sequences.map((sequence) => (
                  <div
                    key={sequence.id}
                    className={`p-3 rounded-lg border cursor-pointer transition-all duration-200 hover:bg-accent/50 ${
                      activeSequence?.id === sequence.id
                        ? "border-primary bg-primary/10"
                        : "border-border"
                    }`}
                    onClick={() => {
                      setActiveSequence(sequence);
                      setFocusedCard("sequence-details");
                    }}
                  >
                    <div className="flex items-center justify-between mb-2">
                      <span className="font-medium text-sm">
                        {sequence.name}
                      </span>
                      <Badge
                        variant={
                          getSequenceStatus(sequence) === "running"
                            ? "default"
                            : "outline"
                        }
                        className="text-xs"
                      >
                        {getSequenceStatus(sequence)}
                      </Badge>
                    </div>
                    <p className="text-xs text-muted-foreground mb-2">
                      {sequence.description}
                    </p>
                    <div className="space-y-1">
                      <Progress
                        value={getSequenceProgress(sequence)}
                        className="h-1"
                      />
                      <div className="flex justify-between text-xs text-muted-foreground">
                        <span>{getSequenceTasks(sequence).length} tasks</span>
                        <span>{getSequenceProgress(sequence)}%</span>
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </ScrollArea>
          </Card>
        )}

        {/* Main Content Area */}
        {activeSequence && (
          <>
            {/* Execution Status Card - Prominent in execution mode */}
            {(viewMode === "execution" || viewMode === "overview") && (
              <Card
                className={`${getCardClass("execution-status")} ${
                  viewMode === "execution" ? "lg:col-span-2" : "lg:col-span-2"
                }`}
              >
                <div className="p-6">
                  <div className="flex items-center justify-between mb-6">
                    <div>
                      <h3 className="text-lg font-semibold flex items-center gap-2">
                        <Activity className="h-5 w-5" />
                        Execution Status
                      </h3>
                      <p className="text-muted-foreground">
                        {activeSequence.name}
                      </p>
                    </div>
                    <Badge
                      variant={
                        getSequenceStatus(activeSequence) === "running"
                          ? "default"
                          : "outline"
                      }
                      className="gap-2"
                    >
                      <div
                        className={`w-2 h-2 rounded-full ${
                          getSequenceStatus(activeSequence) === "running"
                            ? "bg-green-400 animate-pulse"
                            : "bg-gray-400"
                        }`}
                      />
                      {getSequenceStatus(activeSequence).toUpperCase()}
                    </Badge>
                  </div>

                  {/* Performance Metrics */}
                  <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
                    <div className="text-center p-3 rounded-lg bg-muted/50">
                      <div className="text-xl font-bold text-green-600">
                        {completedTasks.length}
                      </div>
                      <div className="text-xs text-muted-foreground">
                        Completed
                      </div>
                    </div>
                    <div className="text-center p-3 rounded-lg bg-muted/50">
                      <div className="text-xl font-bold text-blue-600">
                        {runningTasks.length}
                      </div>
                      <div className="text-xs text-muted-foreground">
                        Running
                      </div>
                    </div>
                    <div className="text-center p-3 rounded-lg bg-muted/50">
                      <div className="text-xl font-bold text-orange-600">
                        {pendingTasks.length}
                      </div>
                      <div className="text-xs text-muted-foreground">
                        Pending
                      </div>
                    </div>
                    <div className="text-center p-3 rounded-lg bg-muted/50">
                      <div className="text-xl font-bold">
                        {currentTasks.length}
                      </div>
                      <div className="text-xs text-muted-foreground">Total</div>
                    </div>
                  </div>

                  {/* Overall Progress */}
                  <div className="space-y-2 mb-6">
                    <div className="flex justify-between text-sm">
                      <span>Overall Progress</span>
                      <span>{getSequenceProgress(activeSequence)}%</span>
                    </div>
                    <Progress
                      value={getSequenceProgress(activeSequence)}
                      className="h-3"
                    />
                  </div>

                  {/* Control Buttons */}
                  <div className="flex flex-wrap gap-2">
                    <Button
                      onClick={() =>
                        handleSequenceAction("start", activeSequence.id)
                      }
                      className="gap-2"
                      disabled={getSequenceStatus(activeSequence) === "running"}
                    >
                      <Play className="h-4 w-4" />
                      Start
                    </Button>
                    <Button
                      variant="outline"
                      onClick={() =>
                        handleSequenceAction("pause", activeSequence.id)
                      }
                      className="gap-2"
                    >
                      <Pause className="h-4 w-4" />
                      Pause
                    </Button>
                    <Button
                      variant="outline"
                      onClick={() =>
                        handleSequenceAction("stop", activeSequence.id)
                      }
                      className="gap-2"
                    >
                      <Square className="h-4 w-4" />
                      Stop
                    </Button>
                    <Button
                      variant="outline"
                      onClick={() =>
                        handleSequenceAction("restart", activeSequence.id)
                      }
                      className="gap-2"
                    >
                      <RotateCcw className="h-4 w-4" />
                      Restart
                    </Button>
                  </div>
                </div>
              </Card>
            )}

            {/* Task List - Table or Cards View */}
            <Card
              className={`${getCardClass("task-list")} ${
                viewMode === "execution"
                  ? "lg:col-span-2"
                  : viewMode === "editing"
                    ? "lg:col-span-2"
                    : "lg:col-span-2"
              }`}
            >
              <div className="p-4 border-b">
                <div className="flex items-center justify-between">
                  <h3 className="font-semibold">Tasks</h3>
                  <div className="flex items-center gap-2">
                    <Badge variant="outline">{currentTasks.length} tasks</Badge>
                    <Button size="sm" onClick={createNewTask} className="gap-1">
                      <Plus className="h-3 w-3" />
                      Add
                    </Button>
                  </div>
                </div>
              </div>

              {currentTasks.length === 0 ? (
                <div className="p-8 text-center">
                  <List className="h-12 w-12 text-muted-foreground mx-auto mb-3" />
                  <p className="text-muted-foreground mb-4">
                    No tasks in this sequence
                  </p>
                  <Button onClick={createNewTask} className="gap-2">
                    <Plus className="h-4 w-4" />
                    Add First Task
                  </Button>
                </div>
              ) : taskViewMode === "table" ? (
                /* Table View - POS Style */
                <div className="overflow-hidden">
                  <Table>
                    <TableHeader>
                      <TableRow className="bg-muted/50">
                        <TableHead className="w-[50px] text-center">
                          #
                        </TableHead>
                        <TableHead className="min-w-[200px]">
                          Task Name
                        </TableHead>
                        <TableHead className="w-[100px]">Type</TableHead>
                        <TableHead className="w-[100px] text-center">
                          Status
                        </TableHead>
                        <TableHead className="w-[120px] text-center">
                          Progress
                        </TableHead>
                        <TableHead className="w-[80px] text-center">
                          Duration
                        </TableHead>
                        <TableHead className="w-[100px]">
                          Dependencies
                        </TableHead>
                        <TableHead className="w-[120px] text-center">
                          Actions
                        </TableHead>
                      </TableRow>
                    </TableHeader>
                  </Table>
                  <ScrollArea className="h-[500px]">
                    <Table>
                      <TableBody>
                        {currentTasks.map((task, index) => (
                          <TableRow
                            key={task.id}
                            className={`cursor-pointer transition-colors hover:bg-accent/50 ${
                              focusedCard === task.id
                                ? "bg-primary/5 border-l-4 border-l-primary"
                                : ""
                            }`}
                            onClick={() => setFocusedCard(task.id)}
                          >
                            {/* Index */}
                            <TableCell className="text-center font-medium">
                              <div className="flex items-center justify-center w-8 h-8 rounded-full bg-primary/10 text-primary text-sm font-bold">
                                {index + 1}
                              </div>
                            </TableCell>

                            {/* Task Name & Description */}
                            <TableCell>
                              <div>
                                <div className="font-medium text-sm">
                                  {task.name}
                                </div>
                                <div className="text-xs text-muted-foreground line-clamp-2">
                                  {task.description}
                                </div>
                                {task.hasSubtasks && (
                                  <Badge
                                    variant="secondary"
                                    className="text-xs mt-1"
                                  >
                                    {task.subtasks?.length || 0} subtasks
                                  </Badge>
                                )}
                              </div>
                            </TableCell>

                            {/* Type */}
                            <TableCell>
                              <Badge
                                variant="outline"
                                className={`text-xs ${getTypeColor(task.type)}`}
                              >
                                {task.type.replace("_", " ")}
                              </Badge>
                            </TableCell>

                            {/* Status */}
                            <TableCell className="text-center">
                              <div
                                className={`inline-flex items-center gap-1 px-2 py-1 rounded-full text-xs font-medium ${getStatusColor(task.status)}`}
                              >
                                {getStatusIcon(task.status)}
                                {task.status}
                              </div>
                            </TableCell>

                            {/* Progress */}
                            <TableCell className="text-center">
                              <div className="w-full">
                                <div className="flex justify-between text-xs mb-1">
                                  <span>{task.progress}%</span>
                                </div>
                                <Progress
                                  value={task.progress}
                                  className="h-2"
                                />
                              </div>
                            </TableCell>

                            {/* Duration */}
                            <TableCell className="text-center">
                              <div className="text-sm">
                                <div>{task.duration}s</div>
                                <div className="text-xs text-muted-foreground">
                                  / {task.timeout}s
                                </div>
                              </div>
                            </TableCell>

                            {/* Dependencies */}
                            <TableCell>
                              {task.dependencies &&
                              task.dependencies.length > 0 ? (
                                <div className="text-xs">
                                  <Badge variant="outline" className="text-xs">
                                    {task.dependencies.length} deps
                                  </Badge>
                                </div>
                              ) : (
                                <span className="text-xs text-muted-foreground">
                                  None
                                </span>
                              )}
                            </TableCell>

                            {/* Actions */}
                            <TableCell className="text-center">
                              <div className="flex items-center justify-center gap-1">
                                <Button
                                  size="sm"
                                  variant="ghost"
                                  onClick={(e) => {
                                    e.stopPropagation();
                                    handleTaskAction("start", task.id);
                                  }}
                                  className="h-7 w-7 p-0"
                                  disabled={task.status === "running"}
                                >
                                  <PlayCircle className="h-3 w-3" />
                                </Button>
                                <Button
                                  size="sm"
                                  variant="ghost"
                                  onClick={(e) => {
                                    e.stopPropagation();
                                    setSelectedTask(task);
                                    setShowTaskEditor(true);
                                    setViewMode("editing");
                                    setFocusedCard(task.id);
                                  }}
                                  className="h-7 w-7 p-0"
                                >
                                  <Edit className="h-3 w-3" />
                                </Button>
                                <Button
                                  size="sm"
                                  variant="ghost"
                                  onClick={(e) => {
                                    e.stopPropagation();
                                    handleDuplicateTask(task);
                                  }}
                                  className="h-7 w-7 p-0"
                                >
                                  <Copy className="h-3 w-3" />
                                </Button>
                                <Button
                                  size="sm"
                                  variant="ghost"
                                  onClick={(e) => {
                                    e.stopPropagation();
                                    handleDeleteTask(task.id);
                                  }}
                                  className="h-7 w-7 p-0 text-destructive hover:text-destructive"
                                >
                                  <Trash2 className="h-3 w-3" />
                                </Button>
                              </div>
                            </TableCell>
                          </TableRow>
                        ))}
                      </TableBody>
                    </Table>
                  </ScrollArea>
                </div>
              ) : (
                /* Cards View - Original */
                <ScrollArea className="h-[500px]">
                  <div className="p-4 space-y-3">
                    {currentTasks.map((task, index) => (
                      <div
                        key={task.id}
                        className={`relative group ${getCardClass(task.id)}`}
                        onClick={() => setFocusedCard(task.id)}
                      >
                        <TaskCardWithSubtasks
                          task={task}
                          index={index}
                          isSequential={!activeSequence?.isParallel}
                          canStart={true}
                          onUpdate={handleUpdateTask}
                          onDelete={handleDeleteTask}
                          onDuplicate={handleDuplicateTask}
                          onStart={(taskId) =>
                            handleTaskAction("start", taskId)
                          }
                          onPause={(taskId) =>
                            handleTaskAction("pause", taskId)
                          }
                          onStop={(taskId) => handleTaskAction("stop", taskId)}
                        />
                        {/* Quick Edit Button */}
                        <div className="absolute top-2 right-2 opacity-0 group-hover:opacity-100 transition-opacity">
                          <Button
                            size="sm"
                            variant="outline"
                            onClick={(e) => {
                              e.stopPropagation();
                              setSelectedTask(task);
                              setShowTaskEditor(true);
                              setViewMode("editing");
                              setFocusedCard(task.id);
                            }}
                            className="h-7 w-7 p-0"
                          >
                            <Settings className="h-3 w-3" />
                          </Button>
                        </div>
                      </div>
                    ))}
                  </div>
                </ScrollArea>
              )}
            </Card>

            {/* Right Sidebar - Context Aware */}
            {(viewMode === "overview" || viewMode === "execution") && (
              <Card className={`${getCardClass("sidebar-info")} lg:col-span-1`}>
                <Tabs defaultValue="info" className="h-full">
                  <div className="p-4 border-b">
                    <TabsList className="grid w-full grid-cols-2">
                      <TabsTrigger value="info">Info</TabsTrigger>
                      <TabsTrigger value="logs">Logs</TabsTrigger>
                    </TabsList>
                  </div>

                  <TabsContent value="info" className="p-4 mt-0">
                    <div className="space-y-4">
                      <div>
                        <h4 className="font-semibold mb-2">Sequence Details</h4>
                        <div className="space-y-2 text-sm">
                          <div className="flex justify-between">
                            <span>Type:</span>
                            <Badge variant="outline">
                              {activeSequence.isParallel
                                ? "Parallel"
                                : "Sequential"}
                            </Badge>
                          </div>
                          <div className="flex justify-between">
                            <span>Created:</span>
                            <span>
                              {activeSequence.createdAt.toLocaleDateString()}
                            </span>
                          </div>
                          <div className="flex justify-between">
                            <span>Updated:</span>
                            <span>
                              {activeSequence.updatedAt.toLocaleDateString()}
                            </span>
                          </div>
                        </div>
                      </div>

                      <Separator />

                      <div>
                        <h4 className="font-semibold mb-2">Tags</h4>
                        <div className="flex flex-wrap gap-1">
                          {activeSequence.tags.map((tag, idx) => (
                            <Badge
                              key={idx}
                              variant="secondary"
                              className="text-xs"
                            >
                              {tag}
                            </Badge>
                          ))}
                        </div>
                      </div>

                      <Separator />

                      <div>
                        <h4 className="font-semibold mb-2">Actions</h4>
                        <div className="space-y-2">
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => exportSequence(activeSequence)}
                            className="w-full gap-2"
                          >
                            <Download className="h-3 w-3" />
                            Export
                          </Button>
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => duplicateSequence(activeSequence)}
                            className="w-full gap-2"
                          >
                            <Copy className="h-3 w-3" />
                            Duplicate
                          </Button>
                        </div>
                      </div>
                    </div>
                  </TabsContent>

                  <TabsContent value="logs" className="p-4 mt-0">
                    <ScrollArea className="h-[400px]">
                      <div className="space-y-2">
                        {executionLogs.length === 0 ? (
                          <p className="text-muted-foreground text-center py-8 text-sm">
                            No logs yet
                          </p>
                        ) : (
                          executionLogs.slice(0, 20).map((log, index) => (
                            <div
                              key={index}
                              className="text-xs font-mono p-2 rounded bg-muted/50"
                            >
                              {log}
                            </div>
                          ))
                        )}
                      </div>
                    </ScrollArea>
                  </TabsContent>
                </Tabs>
              </Card>
            )}
          </>
        )}

        {/* Empty State */}
        {!activeSequence && (
          <Card className="lg:col-span-full p-12 text-center">
            <Activity className="h-16 w-16 text-muted-foreground mx-auto mb-4" />
            <h3 className="text-xl font-semibold mb-2">Select a Sequence</h3>
            <p className="text-muted-foreground mb-6">
              Choose a sequence from the sidebar to view details and manage
              tasks
            </p>
            <Button onClick={() => setIsEditing(true)} className="gap-2">
              <Plus className="h-4 w-4" />
              Create New Sequence
            </Button>
          </Card>
        )}
      </div>

      {/* Sequence Editor Modal */}
      {isEditing && (
        <Dialog open={isEditing} onOpenChange={setIsEditing}>
          <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
            <SequenceEditor
              sequence={activeSequence || undefined}
              onSave={(sequence) => {
                if (activeSequence) {
                  setSequences((prev) =>
                    prev.map((s) => (s.id === sequence.id ? sequence : s)),
                  );
                  setActiveSequence(sequence);
                } else {
                  setSequences((prev) => [...prev, sequence]);
                  setActiveSequence(sequence);
                }
                setIsEditing(false);
                addLog(`Saved sequence: ${sequence.name}`);
              }}
              onCancel={() => setIsEditing(false)}
            />
          </DialogContent>
        </Dialog>
      )}

      {/* Task Parameter Editor */}
      {selectedTask && (
        <TaskParameterEditor
          task={selectedTask}
          isOpen={showTaskEditor}
          onClose={() => {
            setShowTaskEditor(false);
            setSelectedTask(null);
            setFocusedCard(null);
          }}
          onSave={handleUpdateTask}
        />
      )}

      {/* Feature Manager */}
      <Dialog open={showFeatureManager} onOpenChange={setShowFeatureManager}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Feature Management</DialogTitle>
            <DialogDescription>
              Enable and configure advanced robot features
            </DialogDescription>
          </DialogHeader>
          <FeatureManager />
        </DialogContent>
      </Dialog>
    </div>
  );
}
