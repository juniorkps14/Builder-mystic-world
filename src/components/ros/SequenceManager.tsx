import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { TaskCard, Task } from "./TaskCard";
import { SequenceEditor, Sequence } from "./SequenceEditor";
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
} from "lucide-react";

export function SequenceManager() {
  const [activeSequence, setActiveSequence] = useState<Sequence | null>(null);
  const [sequences, setSequences] = useState<Sequence[]>([]);
  const [tasks, setTasks] = useState<Task[]>([]);
  const [isEditing, setIsEditing] = useState(false);
  const [executionLogs, setExecutionLogs] = useState<string[]>([]);

  // Mock data
  useEffect(() => {
    const mockSequences: Sequence[] = [
      {
        id: "patrol_sequence",
        name: "Security Patrol",
        description: "Automated security patrol route with checkpoints",
        isParallel: false,
        maxConcurrency: 1,
        status: "idle",
        createdAt: new Date(Date.now() - 86400000),
        updatedAt: new Date(Date.now() - 3600000),
        tags: ["security", "patrol", "navigation"],
        taskIds: ["task_1", "task_2", "task_3", "task_4"],
      },
      {
        id: "inspection_sequence",
        name: "Equipment Inspection",
        description: "Multi-sensor inspection of critical equipment",
        isParallel: true,
        maxConcurrency: 3,
        status: "running",
        createdAt: new Date(Date.now() - 172800000),
        updatedAt: new Date(),
        tags: ["inspection", "sensors", "maintenance"],
        taskIds: ["task_5", "task_6", "task_7"],
      },
    ];

    const mockTasks: Task[] = [
      {
        id: "task_1",
        name: "Navigate to Checkpoint A",
        type: "movement",
        description: "Move to first security checkpoint",
        parameters: { x: 5.2, y: 3.1, theta: 0 },
        timeout: 120,
        retries: 2,
        status: "completed",
        progress: 100,
        duration: 45,
        startTime: new Date(Date.now() - 300000),
        endTime: new Date(Date.now() - 255000),
        dependencies: [],
      },
      {
        id: "task_2",
        name: "Capture Security Image",
        type: "sensor_reading",
        description: "Take photo for security record",
        parameters: { camera: "front", resolution: "1080p" },
        timeout: 30,
        retries: 1,
        status: "completed",
        progress: 100,
        duration: 8,
        dependencies: ["task_1"],
      },
      {
        id: "task_3",
        name: "Navigate to Checkpoint B",
        type: "movement",
        description: "Move to second security checkpoint",
        parameters: { x: -2.8, y: 4.5, theta: 1.57 },
        timeout: 120,
        retries: 2,
        status: "running",
        progress: 65,
        duration: 78,
        startTime: new Date(Date.now() - 78000),
        dependencies: ["task_2"],
      },
      {
        id: "task_4",
        name: "Return to Base",
        type: "movement",
        description: "Return to home position",
        parameters: { x: 0, y: 0, theta: 0 },
        timeout: 180,
        retries: 1,
        status: "pending",
        progress: 0,
        duration: 0,
        dependencies: ["task_3"],
      },
      {
        id: "task_5",
        name: "Temperature Check",
        type: "sensor_reading",
        description: "Monitor equipment temperature",
        parameters: { sensor: "thermal", threshold: 85 },
        timeout: 15,
        retries: 2,
        status: "running",
        progress: 30,
        duration: 5,
        dependencies: [],
      },
      {
        id: "task_6",
        name: "Vibration Analysis",
        type: "sensor_reading",
        description: "Analyze equipment vibration patterns",
        parameters: { sensor: "accelerometer", duration: 60 },
        timeout: 90,
        retries: 1,
        status: "running",
        progress: 75,
        duration: 45,
        dependencies: [],
      },
      {
        id: "task_7",
        name: "Generate Report",
        type: "custom_script",
        description: "Compile inspection report",
        parameters: { format: "pdf", include_images: true },
        timeout: 60,
        retries: 1,
        status: "pending",
        progress: 0,
        duration: 0,
        dependencies: ["task_5", "task_6"],
      },
    ];

    setSequences(mockSequences);
    setTasks(mockTasks);
    setActiveSequence(mockSequences[0]);
  }, []);

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
    // Add actual sequence control logic here
    addLog(`${action.toUpperCase()} sequence: ${sequenceId}`);
  };

  const handleTaskAction = (action: string, taskId: string) => {
    console.log(`${action} task: ${taskId}`);
    // Add actual task control logic here
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
    addLog(`Deleted task: ${taskId}`);
  };

  const handleDuplicateTask = (task: Task) => {
    const newTask = {
      ...task,
      id: `${task.id}_copy_${Date.now()}`,
      name: `${task.name} (Copy)`,
    };
    setTasks((prev) => [...prev, newTask]);
    addLog(`Duplicated task: ${task.name}`);
  };

  const createNewTask = () => {
    const newTask: Task = {
      id: `task_${Date.now()}`,
      name: "New Task",
      type: "custom_script",
      description: "Description for new task",
      parameters: {},
      timeout: 60,
      retries: 1,
      status: "pending",
      progress: 0,
      duration: 0,
      dependencies: [],
    };
    setTasks((prev) => [...prev, newTask]);
    if (activeSequence) {
      setActiveSequence({
        ...activeSequence,
        taskIds: [...activeSequence.taskIds, newTask.id],
      });
    }
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
      {/* Sequence Header */}
      {activeSequence && !isEditing && (
        <Card className="p-6">
          <div className="flex items-center justify-between mb-4">
            <div>
              <h2 className="text-2xl font-bold">{activeSequence.name}</h2>
              <p className="text-muted-foreground">
                {activeSequence.description}
              </p>
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

          <div className="grid grid-cols-1 md:grid-cols-4 gap-4 mb-6">
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

          <div className="flex gap-2">
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
          </div>
        </Card>
      )}

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
              Sequences
            </TabsTrigger>
            <TabsTrigger value="logs" className="gap-2">
              <Activity className="h-4 w-4" />
              Execution Logs
            </TabsTrigger>
          </TabsList>

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
                  <TaskCard
                    key={task.id}
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
                ))}
              </div>
            )}
          </TabsContent>

          <TabsContent value="sequences" className="space-y-4">
            <div className="flex justify-between items-center">
              <h3 className="text-lg font-semibold">All Sequences</h3>
              <Button onClick={() => setIsEditing(true)} className="gap-2">
                <Plus className="h-4 w-4" />
                Create Sequence
              </Button>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              {sequences.map((sequence) => (
                <Card
                  key={sequence.id}
                  className={`p-4 cursor-pointer transition-all ${
                    activeSequence?.id === sequence.id
                      ? "ring-2 ring-primary"
                      : ""
                  }`}
                  onClick={() => setActiveSequence(sequence)}
                >
                  <div className="flex justify-between items-start mb-3">
                    <div>
                      <h4 className="font-semibold">{sequence.name}</h4>
                      <p className="text-sm text-muted-foreground">
                        {sequence.description}
                      </p>
                    </div>
                    <Badge
                      variant={sequence.isParallel ? "default" : "secondary"}
                    >
                      {sequence.isParallel ? "Parallel" : "Sequential"}
                    </Badge>
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
                    <div className="text-sm text-muted-foreground">
                      {getSequenceTasks(sequence).length} tasks
                    </div>
                  </div>
                </Card>
              ))}
            </div>
          </TabsContent>

          <TabsContent value="logs" className="space-y-4">
            <Card className="p-4">
              <h3 className="text-lg font-semibold mb-4">Execution Logs</h3>
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
    </div>
  );
}
