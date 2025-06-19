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
    addLog(`${action.toUpperCase()} sequence: ${sequenceId}`);
  };

  const handleTaskAction = (action: string, taskId: string) => {
    console.log(`${action} task: ${taskId}`);
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
      name: "New Task",
      type: "movement",
      description: "Description for new task",
      parameters: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { roll: 0, pitch: 0, yaw: 0 },
        speed: 1.0,
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
    setSelectedTask(newTask);
    setShowTaskEditor(true);
  };

  const currentTasks = getSequenceTasks(activeSequence);
  const canStartNextTask = (task: Task, isSequential: boolean) => {
    if (!isSequential) return true;
    const taskIndex = currentTasks.findIndex((t) => t.id === task.id);
    if (taskIndex === 0) return true;
    return currentTasks[taskIndex - 1]?.status === "completed";
  };

  return (
    <div className="space-y-6">
      {/* Enhanced Header with Metrics */}
      {activeSequence && !isEditing && (
        <Card className="p-6">
          <div className="flex items-center justify-between mb-6">
            <div>
              <h2 className="text-2xl font-bold">{activeSequence.name}</h2>
              <p className="text-muted-foreground">
                {activeSequence.description}
              </p>
              <div className="flex items-center gap-2 mt-2">
                {activeSequence.tags.map((tag) => (
                  <Badge key={tag} variant="outline" className="text-xs">
                    {tag}
                  </Badge>
                ))}
              </div>
            </div>
            <div className="flex items-center gap-4">
              <Badge
                variant={activeSequence.isParallel ? "default" : "secondary"}
              >
                {activeSequence.isParallel ? "Parallel" : "Sequential"}
              </Badge>
              <Badge
                variant={
                  getSequenceStatus(activeSequence) === "running"
                    ? "default"
                    : "outline"
                }
                className="gap-1"
              >
                <Activity className="h-3 w-3" />
                {getSequenceStatus(activeSequence).toUpperCase()}
              </Badge>
            </div>
          </div>

          {/* Performance Metrics */}
          <div className="grid grid-cols-2 md:grid-cols-5 gap-4 mb-6">
            <div className="text-center">
              <div className="text-2xl font-bold">{currentTasks.length}</div>
              <div className="text-sm text-muted-foreground">Total Tasks</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-ros-success">
                {currentTasks.filter((t) => t.status === "completed").length}
              </div>
              <div className="text-sm text-muted-foreground">Completed</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-primary">
                {currentTasks.filter((t) => t.status === "running").length}
              </div>
              <div className="text-sm text-muted-foreground">Running</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-destructive">
                {currentTasks.filter((t) => t.status === "failed").length}
              </div>
              <div className="text-sm text-muted-foreground">Failed</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-500">
                {
                  currentTasks.filter((t) => t.status === "waiting_feedback")
                    .length
                }
              </div>
              <div className="text-sm text-muted-foreground">
                Waiting Feedback
              </div>
            </div>
          </div>

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

          {/* Enhanced Controls */}
          <div className="flex flex-wrap gap-2">
            <Button
              onClick={() => handleSequenceAction("start", activeSequence.id)}
              className="gap-2"
            >
              <Play className="h-4 w-4" />
              Start Sequence
            </Button>
            <Button
              variant="outline"
              onClick={() => handleSequenceAction("pause", activeSequence.id)}
              className="gap-2"
            >
              <Pause className="h-4 w-4" />
              Pause
            </Button>
            <Button
              variant="outline"
              onClick={() => handleSequenceAction("stop", activeSequence.id)}
              className="gap-2"
            >
              <Square className="h-4 w-4" />
              Stop
            </Button>
            <Button
              variant="outline"
              onClick={() => handleSequenceAction("restart", activeSequence.id)}
              className="gap-2"
            >
              <RotateCcw className="h-4 w-4" />
              Restart
            </Button>

            <Separator orientation="vertical" className="h-8" />

            <Button
              variant="outline"
              onClick={() => setIsEditing(true)}
              className="gap-2"
            >
              <Settings className="h-4 w-4" />
              Edit
            </Button>
            <Button onClick={createNewTask} className="gap-2">
              <Plus className="h-4 w-4" />
              Add Task
            </Button>

            <Separator orientation="vertical" className="h-8" />

            <Button
              variant="outline"
              onClick={() => exportSequence(activeSequence)}
              className="gap-2"
            >
              <Download className="h-4 w-4" />
              Export
            </Button>
            <Button
              variant="outline"
              onClick={() => duplicateSequence(activeSequence)}
              className="gap-2"
            >
              <Copy className="h-4 w-4" />
              Duplicate
            </Button>

            <Separator orientation="vertical" className="h-8" />

            <Button
              variant="outline"
              onClick={() => setShowFeatureManager(true)}
              className="gap-2"
            >
              <Zap className="h-4 w-4" />
              Features
            </Button>
          </div>
        </Card>
      )}

      {/* Enhanced Sequence Editor */}
      {isEditing ? (
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
      ) : (
        <Tabs defaultValue="tasks" className="space-y-4">
          <TabsList>
            <TabsTrigger value="tasks" className="gap-2">
              <List className="h-4 w-4" />
              Tasks
            </TabsTrigger>
            <TabsTrigger value="sequences" className="gap-2">
              <Layers className="h-4 w-4" />
              Library
            </TabsTrigger>
            <TabsTrigger value="analytics" className="gap-2">
              <BarChart3 className="h-4 w-4" />
              Analytics
            </TabsTrigger>
            <TabsTrigger value="logs" className="gap-2">
              <Activity className="h-4 w-4" />
              Logs
            </TabsTrigger>
          </TabsList>

          {/* Enhanced Task Management */}
          <TabsContent value="tasks" className="space-y-4">
            {currentTasks.length === 0 ? (
              <Card className="p-12 text-center">
                <List className="h-16 w-16 text-muted-foreground mx-auto mb-4" />
                <h3 className="text-xl font-semibold mb-2">No Tasks</h3>
                <p className="text-muted-foreground mb-4">
                  {activeSequence
                    ? "This sequence has no tasks yet."
                    : "Select a sequence to view tasks."}
                </p>
                <Button onClick={createNewTask} className="gap-2">
                  <Plus className="h-4 w-4" />
                  Add First Task
                </Button>
              </Card>
            ) : (
              <div className="space-y-4">
                {currentTasks.map((task, index) => (
                  <div key={task.id} className="relative">
                    <TaskCardWithSubtasks
                      task={task}
                      index={index}
                      isSequential={!activeSequence?.isParallel}
                      canStart={canStartNextTask(
                        task,
                        !activeSequence?.isParallel,
                      )}
                      onUpdate={handleUpdateTask}
                      onDelete={handleDeleteTask}
                      onDuplicate={handleDuplicateTask}
                      onStart={(taskId) => handleTaskAction("start", taskId)}
                      onPause={(taskId) => handleTaskAction("pause", taskId)}
                      onStop={(taskId) => handleTaskAction("stop", taskId)}
                    />
                    {/* Enhanced task controls */}
                    <div className="absolute top-2 right-2">
                      <Button
                        size="sm"
                        variant="ghost"
                        onClick={() => {
                          setSelectedTask(task);
                          setShowTaskEditor(true);
                        }}
                        className="opacity-0 group-hover:opacity-100 transition-opacity"
                      >
                        <Settings className="h-3 w-3" />
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </TabsContent>

          {/* Enhanced Sequence Library */}
          <TabsContent value="sequences" className="space-y-4">
            <div className="flex justify-between items-center">
              <div className="flex gap-2">
                <h3 className="text-lg font-semibold">Sequence Library</h3>
                <Badge variant="secondary">{sequences.length} sequences</Badge>
              </div>
              <div className="flex gap-2">
                <input
                  type="file"
                  accept=".json"
                  onChange={importSequence}
                  className="hidden"
                  id="import-sequence"
                />
                <Button
                  variant="outline"
                  onClick={() =>
                    document.getElementById("import-sequence")?.click()
                  }
                  className="gap-2"
                >
                  <Upload className="h-4 w-4" />
                  Import
                </Button>
                <Button onClick={() => setIsEditing(true)} className="gap-2">
                  <Plus className="h-4 w-4" />
                  Create Sequence
                </Button>
              </div>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
              {sequences.map((sequence) => (
                <Card
                  key={sequence.id}
                  className={`p-4 cursor-pointer transition-all hover:shadow-lg ${
                    activeSequence?.id === sequence.id
                      ? "ring-2 ring-primary"
                      : ""
                  }`}
                  onClick={() => setActiveSequence(sequence)}
                >
                  <div className="flex justify-between items-start mb-3">
                    <div className="flex-1">
                      <h4 className="font-semibold">{sequence.name}</h4>
                      <p className="text-sm text-muted-foreground line-clamp-2">
                        {sequence.description}
                      </p>
                    </div>
                    <div className="flex items-center gap-1">
                      <Button
                        size="sm"
                        variant="ghost"
                        onClick={(e) => {
                          e.stopPropagation();
                          exportSequence(sequence);
                        }}
                      >
                        <Download className="h-3 w-3" />
                      </Button>
                      <Button
                        size="sm"
                        variant="ghost"
                        onClick={(e) => {
                          e.stopPropagation();
                          duplicateSequence(sequence);
                        }}
                      >
                        <Copy className="h-3 w-3" />
                      </Button>
                    </div>
                  </div>

                  <div className="space-y-2">
                    <div className="flex justify-between text-sm">
                      <span>Progress</span>
                      <span>{getSequenceProgress(sequence)}%</span>
                    </div>
                    <Progress
                      value={getSequenceProgress(sequence)}
                      className="h-2"
                    />
                  </div>

                  <div className="flex justify-between items-center mt-3">
                    <div className="flex gap-1">
                      {sequence.tags.slice(0, 2).map((tag, idx) => (
                        <Badge key={idx} variant="outline" className="text-xs">
                          {tag}
                        </Badge>
                      ))}
                      {sequence.tags.length > 2 && (
                        <Badge variant="outline" className="text-xs">
                          +{sequence.tags.length - 2}
                        </Badge>
                      )}
                    </div>
                    <div className="flex items-center gap-2">
                      <Badge
                        variant={sequence.isParallel ? "default" : "secondary"}
                        className="text-xs"
                      >
                        {sequence.isParallel ? "Parallel" : "Sequential"}
                      </Badge>
                      <span className="text-sm text-muted-foreground">
                        {getSequenceTasks(sequence).length} tasks
                      </span>
                    </div>
                  </div>
                </Card>
              ))}
            </div>
          </TabsContent>

          {/* Analytics Dashboard */}
          <TabsContent value="analytics" className="space-y-4">
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
              <Card className="p-4">
                <div className="flex items-center gap-3">
                  <div className="p-2 rounded-lg bg-primary/10">
                    <BarChart3 className="h-5 w-5 text-primary" />
                  </div>
                  <div>
                    <p className="text-sm text-muted-foreground">
                      Total Executions
                    </p>
                    <p className="text-2xl font-bold">
                      {metrics.totalExecutions}
                    </p>
                  </div>
                </div>
              </Card>

              <Card className="p-4">
                <div className="flex items-center gap-3">
                  <div className="p-2 rounded-lg bg-ros-success/10">
                    <CheckCircle2 className="h-5 w-5 text-ros-success" />
                  </div>
                  <div>
                    <p className="text-sm text-muted-foreground">
                      Success Rate
                    </p>
                    <p className="text-2xl font-bold">{metrics.successRate}%</p>
                  </div>
                </div>
              </Card>

              <Card className="p-4">
                <div className="flex items-center gap-3">
                  <div className="p-2 rounded-lg bg-blue-500/10">
                    <Timer className="h-5 w-5 text-blue-500" />
                  </div>
                  <div>
                    <p className="text-sm text-muted-foreground">Avg Time</p>
                    <p className="text-2xl font-bold">{metrics.averageTime}s</p>
                  </div>
                </div>
              </Card>

              <Card className="p-4">
                <div className="flex items-center gap-3">
                  <div className="p-2 rounded-lg bg-accent/10">
                    <Clock className="h-5 w-5 text-accent" />
                  </div>
                  <div>
                    <p className="text-sm text-muted-foreground">
                      Total Runtime
                    </p>
                    <p className="text-2xl font-bold">
                      {metrics.totalRuntime}h
                    </p>
                  </div>
                </div>
              </Card>
            </div>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Performance Trends</h3>
              <div className="h-64 flex items-center justify-center bg-muted rounded">
                <p className="text-muted-foreground">
                  Performance charts would be displayed here
                </p>
              </div>
            </Card>
          </TabsContent>

          {/* Enhanced Execution Logs */}
          <TabsContent value="logs" className="space-y-4">
            <Card className="p-4">
              <div className="flex justify-between items-center mb-4">
                <h3 className="text-lg font-semibold">Execution Logs</h3>
                <div className="flex gap-2">
                  <Button variant="outline" size="sm">
                    <Download className="h-4 w-4 mr-2" />
                    Export Logs
                  </Button>
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => setExecutionLogs([])}
                  >
                    <Trash2 className="h-4 w-4 mr-2" />
                    Clear
                  </Button>
                </div>
              </div>
              <ScrollArea className="h-96">
                <div className="space-y-2">
                  {executionLogs.length === 0 ? (
                    <p className="text-muted-foreground text-center py-8">
                      No execution logs yet. Start a sequence to see logs here.
                    </p>
                  ) : (
                    executionLogs.map((log, index) => (
                      <div
                        key={index}
                        className="text-sm font-mono p-2 rounded bg-muted"
                      >
                        {log}
                      </div>
                    ))
                  )}
                </div>
              </ScrollArea>
            </Card>
          </TabsContent>
        </Tabs>
      )}

      {/* Task Parameter Editor */}
      {selectedTask && (
        <TaskParameterEditor
          task={selectedTask}
          isOpen={showTaskEditor}
          onClose={() => {
            setShowTaskEditor(false);
            setSelectedTask(null);
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
