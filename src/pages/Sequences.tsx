import React, { useState, useEffect } from "react";
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
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "@/components/ui/alert-dialog";
import { useToast } from "@/hooks/use-toast";
import {
  Plus,
  Play,
  Pause,
  Square,
  Trash2,
  Settings,
  Copy,
  Save,
  Download,
  Upload,
  List,
  CheckCircle,
  XCircle,
  Clock,
  AlertCircle,
  RotateCcw,
  Zap,
  Navigation,
} from "lucide-react";

interface Task {
  id: string;
  name: string;
  type: "movement" | "sensor" | "action" | "wait";
  description: string;
  duration: number; // in seconds
  status: "pending" | "running" | "completed" | "failed";
  progress: number; // 0-100
  parameters: Record<string, any>;
  createdAt: Date;
}

interface Sequence {
  id: string;
  name: string;
  description: string;
  tasks: Task[];
  status: "idle" | "running" | "paused" | "completed" | "failed";
  createdAt: Date;
  isParallel: boolean;
}

const Sequences: React.FC = () => {
  const { t } = useLanguage();
  const { toast } = useToast();
  const [sequences, setSequences] = useState<Sequence[]>([]);
  const [activeSequence, setActiveSequence] = useState<Sequence | null>(null);
  const [isCreateDialogOpen, setIsCreateDialogOpen] = useState(false);
  const [isTaskDialogOpen, setIsTaskDialogOpen] = useState(false);
  const [newSequenceName, setNewSequenceName] = useState("");
  const [newSequenceDescription, setNewSequenceDescription] = useState("");
  const [newTaskName, setNewTaskName] = useState("");
  const [newTaskType, setNewTaskType] = useState<Task["type"]>("movement");
  const [newTaskDescription, setNewTaskDescription] = useState("");
  const [newTaskDuration, setNewTaskDuration] = useState(10);

  const { isConnected, publish, subscribe } = useROSIntegration();

  // Task type options
  const taskTypes = [
    {
      value: "movement",
      label: "Movement",
      description: "Robot movement commands",
    },
    {
      value: "sensor",
      label: "Sensor Reading",
      description: "Read sensor data",
    },
    { value: "action", label: "Action", description: "Execute robot action" },
    { value: "wait", label: "Wait", description: "Wait for specified time" },
  ];

  // Create new sequence
  const createSequence = () => {
    if (!newSequenceName.trim()) return;

    const newSequence: Sequence = {
      id: `seq_${Date.now()}`,
      name: newSequenceName,
      description: newSequenceDescription,
      tasks: [],
      status: "idle",
      createdAt: new Date(),
      isParallel: false,
    };

    setSequences((prev) => [...prev, newSequence]);
    setActiveSequence(newSequence);
    setIsCreateDialogOpen(false);
    setNewSequenceName("");
    setNewSequenceDescription("");

    toast({
      title: "Sequence Created",
      description: `${newSequenceName} has been created successfully`,
    });
  };

  // Add task to active sequence
  const addTask = () => {
    if (!activeSequence || !newTaskName.trim()) return;

    const newTask: Task = {
      id: `task_${Date.now()}`,
      name: newTaskName,
      type: newTaskType,
      description: newTaskDescription,
      duration: newTaskDuration,
      status: "pending",
      progress: 0,
      parameters: {},
      createdAt: new Date(),
    };

    const updatedSequence = {
      ...activeSequence,
      tasks: [...activeSequence.tasks, newTask],
    };

    setSequences((prev) =>
      prev.map((seq) => (seq.id === activeSequence.id ? updatedSequence : seq)),
    );
    setActiveSequence(updatedSequence);
    setIsTaskDialogOpen(false);
    setNewTaskName("");
    setNewTaskDescription("");
    setNewTaskDuration(10);

    toast({
      title: "Task Added",
      description: `${newTaskName} has been added to the sequence`,
    });
  };

  // Delete single task
  const deleteTask = (taskId: string) => {
    if (!activeSequence) return;

    const updatedSequence = {
      ...activeSequence,
      tasks: activeSequence.tasks.filter((task) => task.id !== taskId),
    };

    setSequences((prev) =>
      prev.map((seq) => (seq.id === activeSequence.id ? updatedSequence : seq)),
    );
    setActiveSequence(updatedSequence);

    toast({
      title: "Task Deleted",
      description: "Task has been removed from the sequence",
      variant: "destructive",
    });
  };

  // Clear all tasks
  const clearAllTasks = () => {
    if (!activeSequence) return;

    const updatedSequence = {
      ...activeSequence,
      tasks: [],
    };

    setSequences((prev) =>
      prev.map((seq) => (seq.id === activeSequence.id ? updatedSequence : seq)),
    );
    setActiveSequence(updatedSequence);

    toast({
      title: "All Tasks Cleared",
      description: "All tasks have been removed from the sequence",
      variant: "destructive",
    });
  };

  // Delete sequence
  const deleteSequence = (sequenceId: string) => {
    setSequences((prev) => prev.filter((seq) => seq.id !== sequenceId));
    if (activeSequence?.id === sequenceId) {
      setActiveSequence(null);
    }

    toast({
      title: "Sequence Deleted",
      description: "Sequence has been deleted successfully",
      variant: "destructive",
    });
  };

  // Start sequence execution
  const startSequence = (sequence: Sequence) => {
    if (sequence.tasks.length === 0) {
      toast({
        title: "Cannot Start",
        description: "Sequence has no tasks to execute",
        variant: "destructive",
      });
      return;
    }

    // Update sequence status
    const updatedSequence = { ...sequence, status: "running" as const };
    setSequences((prev) =>
      prev.map((seq) => (seq.id === sequence.id ? updatedSequence : seq)),
    );
    setActiveSequence(updatedSequence);

    // Simulate task execution
    executeTasksSequentially(sequence.tasks);

    toast({
      title: "Sequence Started",
      description: `${sequence.name} is now executing`,
    });
  };

  // Simulate task execution
  const executeTasksSequentially = (tasks: Task[]) => {
    let currentTaskIndex = 0;

    const executeNextTask = () => {
      if (currentTaskIndex >= tasks.length) {
        // All tasks completed
        if (activeSequence) {
          const completedSequence = {
            ...activeSequence,
            status: "completed" as const,
          };
          setSequences((prev) =>
            prev.map((seq) =>
              seq.id === activeSequence.id ? completedSequence : seq,
            ),
          );
          setActiveSequence(completedSequence);
        }
        return;
      }

      const task = tasks[currentTaskIndex];
      const taskDuration = task.duration * 1000; // Convert to milliseconds

      // Start task
      updateTaskStatus(task.id, "running", 0);

      // Simulate progress
      const progressInterval = setInterval(() => {
        updateTaskProgress(task.id, (elapsed) => {
          const progress = Math.min(100, (elapsed / taskDuration) * 100);
          return progress;
        });
      }, 100);

      // Complete task after duration
      setTimeout(() => {
        clearInterval(progressInterval);
        updateTaskStatus(task.id, "completed", 100);
        currentTaskIndex++;
        setTimeout(executeNextTask, 500); // Small delay between tasks
      }, taskDuration);
    };

    executeNextTask();
  };

  // Update task status
  const updateTaskStatus = (
    taskId: string,
    status: Task["status"],
    progress: number,
  ) => {
    if (!activeSequence) return;

    const updatedTasks = activeSequence.tasks.map((task) =>
      task.id === taskId ? { ...task, status, progress } : task,
    );

    const updatedSequence = { ...activeSequence, tasks: updatedTasks };

    setSequences((prev) =>
      prev.map((seq) => (seq.id === activeSequence.id ? updatedSequence : seq)),
    );
    setActiveSequence(updatedSequence);
  };

  // Update task progress
  const updateTaskProgress = (
    taskId: string,
    progressCalculator: (elapsed: number) => number,
  ) => {
    if (!activeSequence) return;

    const task = activeSequence.tasks.find((t) => t.id === taskId);
    if (!task) return;

    const elapsed = Date.now() - task.createdAt.getTime();
    const progress = progressCalculator(elapsed);

    updateTaskStatus(taskId, task.status, progress);
  };

  // Get status icon
  const getStatusIcon = (status: Task["status"] | Sequence["status"]) => {
    switch (status) {
      case "pending":
        return <Clock className="h-4 w-4 text-gray-500" />;
      case "running":
        return <Zap className="h-4 w-4 text-blue-500 animate-pulse" />;
      case "completed":
        return <CheckCircle className="h-4 w-4 text-green-500" />;
      case "failed":
        return <XCircle className="h-4 w-4 text-red-500" />;
      default:
        return <AlertCircle className="h-4 w-4 text-gray-500" />;
    }
  };

  // Get status color
  const getStatusColor = (status: Task["status"] | Sequence["status"]) => {
    switch (status) {
      case "running":
        return "bg-blue-500";
      case "completed":
        return "bg-green-500";
      case "failed":
        return "bg-red-500";
      default:
        return "bg-gray-500";
    }
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            {t("sequence.title")}
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            {t("sequence.subtitle")}
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
            {isConnected ? t("common.connected") : t("common.disconnected")}
          </Badge>

          <Dialog
            open={isCreateDialogOpen}
            onOpenChange={setIsCreateDialogOpen}
          >
            <DialogTrigger asChild>
              <Button className="gap-2">
                <Plus className="h-4 w-4" />
                {t("sequence.createSequence")}
              </Button>
            </DialogTrigger>
            <DialogContent>
              <DialogHeader>
                <DialogTitle>Create New Sequence</DialogTitle>
                <DialogDescription>
                  Create a new task sequence for your robot
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div>
                  <Label htmlFor="sequenceName">Sequence Name</Label>
                  <Input
                    id="sequenceName"
                    value={newSequenceName}
                    onChange={(e) => setNewSequenceName(e.target.value)}
                    placeholder="Enter sequence name"
                  />
                </div>
                <div>
                  <Label htmlFor="sequenceDescription">Description</Label>
                  <Textarea
                    id="sequenceDescription"
                    value={newSequenceDescription}
                    onChange={(e) => setNewSequenceDescription(e.target.value)}
                    placeholder="Enter sequence description"
                  />
                </div>
              </div>
              <DialogFooter>
                <Button
                  variant="outline"
                  onClick={() => setIsCreateDialogOpen(false)}
                >
                  {t("common.cancel")}
                </Button>
                <Button
                  onClick={createSequence}
                  disabled={!newSequenceName.trim()}
                >
                  {t("common.create")}
                </Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
        </div>
      </div>

      {/* Main Content */}
      {sequences.length === 0 ? (
        // Empty State
        <Card className="p-12 text-center glass-effect">
          <List className="h-16 w-16 text-muted-foreground mx-auto mb-4" />
          <h3 className="text-2xl font-semibold mb-2">
            {t("sequence.noSequences")}
          </h3>
          <p className="text-muted-foreground mb-6 max-w-md mx-auto">
            Get started by creating your first task sequence. You can add
            movement commands, sensor readings, and custom actions.
          </p>
          <Dialog
            open={isCreateDialogOpen}
            onOpenChange={setIsCreateDialogOpen}
          >
            <DialogTrigger asChild>
              <Button size="lg" className="gap-2">
                <Plus className="h-5 w-5" />
                Create Your First Sequence
              </Button>
            </DialogTrigger>
            <DialogContent>
              <DialogHeader>
                <DialogTitle>Create New Sequence</DialogTitle>
                <DialogDescription>
                  Create a new task sequence for your robot
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div>
                  <Label htmlFor="sequenceName">Sequence Name</Label>
                  <Input
                    id="sequenceName"
                    value={newSequenceName}
                    onChange={(e) => setNewSequenceName(e.target.value)}
                    placeholder="Enter sequence name"
                  />
                </div>
                <div>
                  <Label htmlFor="sequenceDescription">Description</Label>
                  <Textarea
                    id="sequenceDescription"
                    value={newSequenceDescription}
                    onChange={(e) => setNewSequenceDescription(e.target.value)}
                    placeholder="Enter sequence description"
                  />
                </div>
              </div>
              <DialogFooter>
                <Button
                  variant="outline"
                  onClick={() => setIsCreateDialogOpen(false)}
                >
                  {t("common.cancel")}
                </Button>
                <Button
                  onClick={createSequence}
                  disabled={!newSequenceName.trim()}
                >
                  {t("common.create")}
                </Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
        </Card>
      ) : (
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Sequences List */}
          <Card className="glass-effect">
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <List className="h-5 w-5" />
                Sequences ({sequences.length})
              </CardTitle>
              <CardDescription>
                Manage your robot task sequences
              </CardDescription>
            </CardHeader>
            <CardContent>
              <ScrollArea className="h-[500px]">
                <div className="space-y-3">
                  {sequences.map((sequence) => (
                    <div
                      key={sequence.id}
                      className={`p-3 rounded-lg border cursor-pointer transition-all duration-200 hover:bg-accent/50 ${
                        activeSequence?.id === sequence.id
                          ? "border-primary bg-primary/10"
                          : "border-border"
                      }`}
                      onClick={() => setActiveSequence(sequence)}
                    >
                      <div className="flex items-center justify-between mb-2">
                        <div className="flex items-center gap-2">
                          {getStatusIcon(sequence.status)}
                          <span className="font-medium text-sm">
                            {sequence.name}
                          </span>
                        </div>
                        <div className="flex items-center gap-1">
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={(e) => {
                              e.stopPropagation();
                              startSequence(sequence);
                            }}
                            className="h-6 w-6 p-0"
                          >
                            <Play className="h-3 w-3" />
                          </Button>
                          <AlertDialog>
                            <AlertDialogTrigger asChild>
                              <Button
                                variant="ghost"
                                size="sm"
                                onClick={(e) => e.stopPropagation()}
                                className="h-6 w-6 p-0"
                              >
                                <Trash2 className="h-3 w-3" />
                              </Button>
                            </AlertDialogTrigger>
                            <AlertDialogContent>
                              <AlertDialogHeader>
                                <AlertDialogTitle>
                                  Delete Sequence
                                </AlertDialogTitle>
                                <AlertDialogDescription>
                                  Are you sure you want to delete "
                                  {sequence.name}"? This action cannot be
                                  undone.
                                </AlertDialogDescription>
                              </AlertDialogHeader>
                              <AlertDialogFooter>
                                <AlertDialogCancel>Cancel</AlertDialogCancel>
                                <AlertDialogAction
                                  onClick={() => deleteSequence(sequence.id)}
                                  className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                                >
                                  Delete
                                </AlertDialogAction>
                              </AlertDialogFooter>
                            </AlertDialogContent>
                          </AlertDialog>
                        </div>
                      </div>
                      <p className="text-xs text-muted-foreground mb-2">
                        {sequence.description}
                      </p>
                      <div className="flex items-center justify-between text-xs text-muted-foreground">
                        <span>{sequence.tasks.length} tasks</span>
                        <Badge variant="outline" className="text-xs">
                          {sequence.status}
                        </Badge>
                      </div>
                    </div>
                  ))}
                </div>
              </ScrollArea>
            </CardContent>
          </Card>

          {/* Task Management */}
          <div className="lg:col-span-2">
            {activeSequence ? (
              <Card className="glass-effect">
                <CardHeader>
                  <div className="flex items-center justify-between">
                    <div>
                      <CardTitle className="flex items-center gap-2">
                        {getStatusIcon(activeSequence.status)}
                        {activeSequence.name}
                      </CardTitle>
                      <CardDescription>
                        {activeSequence.description}
                      </CardDescription>
                    </div>
                    <div className="flex items-center gap-2">
                      <Badge variant="outline">
                        {activeSequence.tasks.length} tasks
                      </Badge>
                      <Button
                        variant="outline"
                        size="sm"
                        onClick={() => startSequence(activeSequence)}
                        disabled={activeSequence.tasks.length === 0}
                        className="gap-2"
                      >
                        <Play className="h-4 w-4" />
                        Start
                      </Button>
                    </div>
                  </div>
                </CardHeader>
                <CardContent>
                  {/* Task Controls */}
                  <div className="flex items-center justify-between mb-4">
                    <h4 className="font-medium">Tasks</h4>
                    <div className="flex items-center gap-2">
                      {activeSequence.tasks.length > 0 && (
                        <AlertDialog>
                          <AlertDialogTrigger asChild>
                            <Button
                              variant="outline"
                              size="sm"
                              className="gap-2"
                            >
                              <Trash2 className="h-4 w-4" />
                              Clear All
                            </Button>
                          </AlertDialogTrigger>
                          <AlertDialogContent>
                            <AlertDialogHeader>
                              <AlertDialogTitle>
                                Clear All Tasks
                              </AlertDialogTitle>
                              <AlertDialogDescription>
                                Are you sure you want to remove all tasks from
                                this sequence? This action cannot be undone.
                              </AlertDialogDescription>
                            </AlertDialogHeader>
                            <AlertDialogFooter>
                              <AlertDialogCancel>Cancel</AlertDialogCancel>
                              <AlertDialogAction
                                onClick={clearAllTasks}
                                className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                              >
                                Clear All
                              </AlertDialogAction>
                            </AlertDialogFooter>
                          </AlertDialogContent>
                        </AlertDialog>
                      )}

                      <Dialog
                        open={isTaskDialogOpen}
                        onOpenChange={setIsTaskDialogOpen}
                      >
                        <DialogTrigger asChild>
                          <Button size="sm" className="gap-2">
                            <Plus className="h-4 w-4" />
                            Add Task
                          </Button>
                        </DialogTrigger>
                        <DialogContent>
                          <DialogHeader>
                            <DialogTitle>Add New Task</DialogTitle>
                            <DialogDescription>
                              Add a new task to the sequence
                            </DialogDescription>
                          </DialogHeader>
                          <div className="space-y-4">
                            <div>
                              <Label htmlFor="taskName">Task Name</Label>
                              <Input
                                id="taskName"
                                value={newTaskName}
                                onChange={(e) => setNewTaskName(e.target.value)}
                                placeholder="Enter task name"
                              />
                            </div>
                            <div>
                              <Label htmlFor="taskType">Task Type</Label>
                              <Select
                                value={newTaskType}
                                onValueChange={(value) =>
                                  setNewTaskType(value as Task["type"])
                                }
                              >
                                <SelectTrigger>
                                  <SelectValue />
                                </SelectTrigger>
                                <SelectContent>
                                  {taskTypes.map((type) => (
                                    <SelectItem
                                      key={type.value}
                                      value={type.value}
                                    >
                                      <div>
                                        <div className="font-medium">
                                          {type.label}
                                        </div>
                                        <div className="text-xs text-muted-foreground">
                                          {type.description}
                                        </div>
                                      </div>
                                    </SelectItem>
                                  ))}
                                </SelectContent>
                              </Select>
                            </div>
                            <div>
                              <Label htmlFor="taskDescription">
                                Description
                              </Label>
                              <Textarea
                                id="taskDescription"
                                value={newTaskDescription}
                                onChange={(e) =>
                                  setNewTaskDescription(e.target.value)
                                }
                                placeholder="Enter task description"
                              />
                            </div>
                            <div>
                              <Label htmlFor="taskDuration">
                                Duration (seconds)
                              </Label>
                              <Input
                                id="taskDuration"
                                type="number"
                                value={newTaskDuration}
                                onChange={(e) =>
                                  setNewTaskDuration(
                                    parseInt(e.target.value) || 10,
                                  )
                                }
                                min="1"
                                max="3600"
                              />
                            </div>
                          </div>
                          <DialogFooter>
                            <Button
                              variant="outline"
                              onClick={() => setIsTaskDialogOpen(false)}
                            >
                              Cancel
                            </Button>
                            <Button
                              onClick={addTask}
                              disabled={!newTaskName.trim()}
                            >
                              Add Task
                            </Button>
                          </DialogFooter>
                        </DialogContent>
                      </Dialog>
                    </div>
                  </div>

                  {/* Tasks List */}
                  <ScrollArea className="h-[400px]">
                    {activeSequence.tasks.length === 0 ? (
                      <div className="text-center py-8">
                        <Navigation className="h-12 w-12 text-muted-foreground mx-auto mb-3" />
                        <p className="text-muted-foreground mb-4">
                          {t("sequence.noTasks")}
                        </p>
                        <Dialog
                          open={isTaskDialogOpen}
                          onOpenChange={setIsTaskDialogOpen}
                        >
                          <DialogTrigger asChild>
                            <Button className="gap-2">
                              <Plus className="h-4 w-4" />
                              {t("sequence.addFirstTask")}
                            </Button>
                          </DialogTrigger>
                        </Dialog>
                      </div>
                    ) : (
                      <div className="space-y-3">
                        {activeSequence.tasks.map((task, index) => (
                          <div key={task.id} className="border rounded-lg p-4">
                            <div className="flex items-center justify-between mb-2">
                              <div className="flex items-center gap-3">
                                <div className="flex items-center justify-center w-6 h-6 rounded-full bg-primary/10 text-primary text-sm font-medium">
                                  {index + 1}
                                </div>
                                <div>
                                  <h5 className="font-medium">{task.name}</h5>
                                  <p className="text-xs text-muted-foreground">
                                    {task.description}
                                  </p>
                                </div>
                              </div>
                              <div className="flex items-center gap-2">
                                <Badge variant="outline" className="text-xs">
                                  {task.type}
                                </Badge>
                                <div className="flex items-center gap-1">
                                  {getStatusIcon(task.status)}
                                  <span className="text-xs">{task.status}</span>
                                </div>
                                <AlertDialog>
                                  <AlertDialogTrigger asChild>
                                    <Button
                                      variant="ghost"
                                      size="sm"
                                      className="h-6 w-6 p-0"
                                    >
                                      <Trash2 className="h-3 w-3" />
                                    </Button>
                                  </AlertDialogTrigger>
                                  <AlertDialogContent>
                                    <AlertDialogHeader>
                                      <AlertDialogTitle>
                                        Delete Task
                                      </AlertDialogTitle>
                                      <AlertDialogDescription>
                                        Are you sure you want to delete "
                                        {task.name}"? This action cannot be
                                        undone.
                                      </AlertDialogDescription>
                                    </AlertDialogHeader>
                                    <AlertDialogFooter>
                                      <AlertDialogCancel>
                                        Cancel
                                      </AlertDialogCancel>
                                      <AlertDialogAction
                                        onClick={() => deleteTask(task.id)}
                                        className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                                      >
                                        Delete
                                      </AlertDialogAction>
                                    </AlertDialogFooter>
                                  </AlertDialogContent>
                                </AlertDialog>
                              </div>
                            </div>

                            {/* Progress Bar */}
                            {task.status === "running" && (
                              <div className="mt-3">
                                <div className="flex justify-between text-xs mb-1">
                                  <span>Progress</span>
                                  <span>{task.progress.toFixed(0)}%</span>
                                </div>
                                <div className="w-full bg-muted rounded-full h-2">
                                  <div
                                    className={`h-2 rounded-full transition-all duration-300 ${getStatusColor(task.status)}`}
                                    style={{ width: `${task.progress}%` }}
                                  />
                                </div>
                              </div>
                            )}

                            <div className="flex items-center justify-between mt-3 text-xs text-muted-foreground">
                              <span>Duration: {task.duration}s</span>
                              <span>
                                Created: {task.createdAt.toLocaleTimeString()}
                              </span>
                            </div>
                          </div>
                        ))}
                      </div>
                    )}
                  </ScrollArea>
                </CardContent>
              </Card>
            ) : (
              <Card className="p-12 text-center glass-effect">
                <List className="h-16 w-16 text-muted-foreground mx-auto mb-4" />
                <h3 className="text-xl font-semibold mb-2">
                  Select a Sequence
                </h3>
                <p className="text-muted-foreground">
                  Choose a sequence from the left panel to view and manage its
                  tasks
                </p>
              </Card>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default Sequences;
