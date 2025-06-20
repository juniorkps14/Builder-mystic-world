import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { Switch } from "@/components/ui/switch";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Separator } from "@/components/ui/separator";
import {
  GripVertical,
  Play,
  Pause,
  Square,
  CheckCircle2,
  XCircle,
  Clock,
  Settings,
  Trash2,
  Copy,
  AlertTriangle,
  Timer,
  Plus,
  ChevronDown,
  ChevronRight,
  Layers,
  ArrowRight,
  Zap,
} from "lucide-react";
import { cn } from "@/lib/utils";

export interface Subtask {
  id: string;
  name: string;
  type: string;
  description: string;
  parameters: Record<string, any>;
  timeout: number;
  retries: number;
  waitForFeedback: boolean;
  feedbackTimeout: number;
  status:
    | "pending"
    | "running"
    | "completed"
    | "failed"
    | "paused"
    | "waiting_feedback";
  progress: number;
  duration: number;
  startTime?: Date;
  endTime?: Date;
  errorMessage?: string;
}

export interface Task {
  id: string;
  name: string;
  type: string;
  description: string;
  parameters: Record<string, any>;
  timeout: number;
  retries: number;
  waitForFeedback: boolean;
  feedbackTimeout: number;
  status:
    | "pending"
    | "running"
    | "completed"
    | "failed"
    | "paused"
    | "waiting_feedback";
  progress: number;
  duration: number;
  startTime?: Date;
  endTime?: Date;
  errorMessage?: string;
  dependencies: string[];
  // Subtask management
  hasSubtasks: boolean;
  subtasks: Subtask[];
  subtaskExecutionMode: "sequential" | "parallel";
  maxSubtaskConcurrency: number;
  subtaskWaitForFeedback: boolean;
}

interface TaskCardProps {
  task: Task;
  index: number;
  isSequential: boolean;
  canStart: boolean;
  onUpdate: (task: Task) => void;
  onDelete: (taskId: string) => void;
  onDuplicate: (task: Task) => void;
  onStart: (taskId: string) => void;
  onPause: (taskId: string) => void;
  onStop: (taskId: string) => void;
  onSubtaskUpdate: (taskId: string, subtask: Subtask) => void;
  onSubtaskDelete: (taskId: string, subtaskId: string) => void;
  onSubtaskAdd: (taskId: string) => void;
  onSubtaskStart: (taskId: string, subtaskId: string) => void;
  onSubtaskPause: (taskId: string, subtaskId: string) => void;
  onSubtaskStop: (taskId: string, subtaskId: string) => void;
  dragHandleProps?: any;
}

export function TaskCard({
  task,
  index,
  isSequential,
  canStart,
  onUpdate,
  onDelete,
  onDuplicate,
  onStart,
  onPause,
  onStop,
  onSubtaskUpdate,
  onSubtaskDelete,
  onSubtaskAdd,
  onSubtaskStart,
  onSubtaskPause,
  onSubtaskStop,
  dragHandleProps,
}: TaskCardProps) {
  const [isEditing, setIsEditing] = useState(false);
  const [editedTask, setEditedTask] = useState(task);
  const [showSubtasks, setShowSubtasks] = useState(true);

  const taskTypeNames = {
    movement: "Robot Movement",
    manipulation: "Arm Manipulation",
    vision: "Vision Processing",
    sensor_reading: "Sensor Reading",
    ai_processing: "AI Processing",
    safety_check: "Safety Check",
    communication: "Communication",
    voice_command: "Voice Command",
    data_logging: "Data Logging",
    maintenance: "Maintenance",
    // Legacy types for backwards compatibility
    ros_service: "ROS Service Call",
    ros_action: "ROS Action",
    custom_script: "Custom Script",
    wait: "Wait/Delay",
  };

  const statusColors = {
    pending: "bg-muted text-muted-foreground",
    running: "bg-primary text-primary-foreground",
    completed: "bg-ros-success text-white",
    failed: "bg-destructive text-destructive-foreground",
    paused: "bg-ros-warning text-black",
    waiting_feedback: "bg-blue-500 text-white",
  };

  const statusIcons = {
    pending: Clock,
    running: Play,
    completed: CheckCircle2,
    failed: XCircle,
    paused: Pause,
    waiting_feedback: AlertTriangle,
  };

  const StatusIcon = statusIcons[task.status];

  const handleSave = () => {
    onUpdate(editedTask);
    setIsEditing(false);
  };

  const handleCancel = () => {
    setEditedTask(task);
    setIsEditing(false);
  };

  const formatDuration = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, "0")}`;
  };

  return (
    <Card
      className={cn(
        "transition-all duration-200 border-l-4",
        task.status === "running" && "border-l-primary shadow-lg",
        task.status === "completed" && "border-l-ros-success",
        task.status === "failed" && "border-l-destructive",
        task.status === "paused" && "border-l-ros-warning",
        task.status === "pending" && "border-l-muted-foreground",
      )}
    >
      <div className="p-4">
        {/* Header */}
        <div className="flex items-center justify-between mb-4">
          <div className="flex items-center gap-3">
            <div
              {...dragHandleProps}
              className="cursor-grab active:cursor-grabbing"
            >
              <GripVertical className="h-4 w-4 text-muted-foreground" />
            </div>

            <div className="flex items-center gap-2">
              <Badge variant="outline" className="text-xs">
                #{index + 1}
              </Badge>
              <Badge className={statusColors[task.status]}>
                <StatusIcon className="h-3 w-3 mr-1" />
                {task.status.toUpperCase()}
              </Badge>
            </div>
          </div>

          <div className="flex items-center gap-2">
            {/* Control Buttons */}
            {task.status === "pending" && canStart && (
              <Button
                size="sm"
                onClick={() => onStart(task.id)}
                className="gap-1"
              >
                <Play className="h-3 w-3" />
                Start
              </Button>
            )}

            {task.status === "running" && (
              <>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => onPause(task.id)}
                  className="gap-1"
                >
                  <Pause className="h-3 w-3" />
                  Pause
                </Button>
                <Button
                  size="sm"
                  variant="destructive"
                  onClick={() => onStop(task.id)}
                  className="gap-1"
                >
                  <Square className="h-3 w-3" />
                  Stop
                </Button>
              </>
            )}

            {task.status === "paused" && (
              <Button
                size="sm"
                onClick={() => onStart(task.id)}
                className="gap-1"
              >
                <Play className="h-3 w-3" />
                Resume
              </Button>
            )}

            {task.status === "waiting_feedback" && (
              <>
                <Button
                  size="sm"
                  onClick={() => onStart(task.id)}
                  className="gap-1"
                >
                  <CheckCircle2 className="h-3 w-3" />
                  Confirm
                </Button>
                <Button
                  size="sm"
                  variant="destructive"
                  onClick={() => onStop(task.id)}
                  className="gap-1"
                >
                  <XCircle className="h-3 w-3" />
                  Reject
                </Button>
              </>
            )}

            {/* Action Buttons */}
            <Button
              size="sm"
              variant="ghost"
              onClick={() => setIsEditing(!isEditing)}
              title="Quick Edit (Basic)"
            >
              <Settings className="h-4 w-4" />
            </Button>
            <Button
              size="sm"
              variant="ghost"
              onClick={() => onDuplicate(task)}
              title="Duplicate Task"
            >
              <Copy className="h-4 w-4" />
            </Button>
            <Button
              size="sm"
              variant="ghost"
              onClick={() => onDelete(task.id)}
              title="Delete Task"
            >
              <Trash2 className="h-4 w-4" />
            </Button>
          </div>
        </div>

        {!isEditing ? (
          /* View Mode */
          <div className="space-y-3">
            <div>
              <h3 className="font-semibold text-lg">{task.name}</h3>
              <p className="text-sm text-muted-foreground">
                {task.description}
              </p>
            </div>

            <div className="grid grid-cols-2 md:grid-cols-4 gap-3 text-sm">
              <div>
                <span className="text-muted-foreground">Type:</span>
                <p className="font-medium">
                  {taskTypeNames[task.type as keyof typeof taskTypeNames] ||
                    task.type}
                </p>
              </div>
              <div>
                <span className="text-muted-foreground">Timeout:</span>
                <p className="font-mono">{task.timeout}s</p>
              </div>
              <div>
                <span className="text-muted-foreground">Retries:</span>
                <p className="font-mono">{task.retries}</p>
              </div>
              <div>
                <span className="text-muted-foreground">Duration:</span>
                <p className="font-mono">{formatDuration(task.duration)}</p>
              </div>
            </div>

            <div className="grid grid-cols-2 gap-3 text-sm mt-3">
              <div className="flex items-center gap-2">
                <span className="text-muted-foreground">Feedback:</span>
                <Badge
                  variant={task.waitForFeedback ? "default" : "outline"}
                  className="text-xs"
                >
                  {task.waitForFeedback ? "Required" : "Auto"}
                </Badge>
              </div>
              {task.waitForFeedback && (
                <div>
                  <span className="text-muted-foreground">
                    Feedback Timeout:
                  </span>
                  <p className="font-mono">{task.feedbackTimeout}s</p>
                </div>
              )}
            </div>

            {/* Progress Bar */}
            {task.status === "running" && (
              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span>Progress</span>
                  <span>{task.progress}%</span>
                </div>
                <Progress value={task.progress} className="h-2" />
              </div>
            )}

            {/* Error Message */}
            {task.status === "failed" && task.errorMessage && (
              <div className="p-3 rounded-lg bg-destructive/10 border border-destructive/20">
                <div className="flex items-center gap-2 text-destructive mb-1">
                  <AlertTriangle className="h-4 w-4" />
                  <span className="font-medium">Error</span>
                </div>
                <p className="text-sm text-destructive/90">
                  {task.errorMessage}
                </p>
              </div>
            )}

            {/* Dependencies */}
            {task.dependencies.length > 0 && (
              <div>
                <span className="text-sm text-muted-foreground">
                  Dependencies:
                </span>
                <div className="flex flex-wrap gap-1 mt-1">
                  {task.dependencies.map((dep, idx) => (
                    <Badge key={idx} variant="outline" className="text-xs">
                      {dep}
                    </Badge>
                  ))}
                </div>
              </div>
            )}
          </div>
        ) : (
          /* Edit Mode */
          <div className="space-y-4">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div>
                <Label htmlFor="name">Task Name</Label>
                <Input
                  id="name"
                  value={editedTask.name}
                  onChange={(e) =>
                    setEditedTask({ ...editedTask, name: e.target.value })
                  }
                />
              </div>
              <div>
                <Label htmlFor="type">Task Type</Label>
                <Select
                  value={editedTask.type}
                  onValueChange={(value) =>
                    setEditedTask({ ...editedTask, type: value })
                  }
                >
                  <SelectTrigger>
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="movement">🚗 Robot Movement</SelectItem>
                    <SelectItem value="manipulation">
                      🦾 Arm Manipulation
                    </SelectItem>
                    <SelectItem value="vision">👁️ Vision Processing</SelectItem>
                    <SelectItem value="sensor_reading">
                      📡 Sensor Reading
                    </SelectItem>
                    <SelectItem value="ai_processing">
                      🧠 AI Processing
                    </SelectItem>
                    <SelectItem value="safety_check">
                      🛡️ Safety Check
                    </SelectItem>
                    <SelectItem value="communication">
                      📡 Communication
                    </SelectItem>
                    <SelectItem value="voice_command">
                      🎤 Voice Command
                    </SelectItem>
                    <SelectItem value="data_logging">
                      📊 Data Logging
                    </SelectItem>
                    <SelectItem value="maintenance">🔧 Maintenance</SelectItem>
                  </SelectContent>
                </Select>
              </div>
            </div>

            <div>
              <Label htmlFor="description">Description</Label>
              <Textarea
                id="description"
                value={editedTask.description}
                onChange={(e) =>
                  setEditedTask({ ...editedTask, description: e.target.value })
                }
                rows={2}
              />
            </div>

            <div className="grid grid-cols-2 gap-4">
              <div>
                <Label htmlFor="timeout">Timeout (seconds)</Label>
                <Input
                  id="timeout"
                  type="number"
                  value={editedTask.timeout}
                  onChange={(e) =>
                    setEditedTask({
                      ...editedTask,
                      timeout: parseInt(e.target.value) || 0,
                    })
                  }
                />
              </div>
              <div>
                <Label htmlFor="retries">Max Retries</Label>
                <Input
                  id="retries"
                  type="number"
                  value={editedTask.retries}
                  onChange={(e) =>
                    setEditedTask({
                      ...editedTask,
                      retries: parseInt(e.target.value) || 0,
                    })
                  }
                />
              </div>
            </div>

            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <div className="space-y-1">
                  <Label>Wait for Feedback</Label>
                  <p className="text-sm text-muted-foreground">
                    {editedTask.waitForFeedback
                      ? "Task will wait for confirmation before proceeding"
                      : "Task will complete automatically"}
                  </p>
                </div>
                <Switch
                  checked={editedTask.waitForFeedback}
                  onCheckedChange={(checked) =>
                    setEditedTask({ ...editedTask, waitForFeedback: checked })
                  }
                />
              </div>

              {editedTask.waitForFeedback && (
                <div>
                  <Label htmlFor="feedback-timeout">
                    Feedback Timeout (seconds)
                  </Label>
                  <Input
                    id="feedback-timeout"
                    type="number"
                    value={editedTask.feedbackTimeout}
                    onChange={(e) =>
                      setEditedTask({
                        ...editedTask,
                        feedbackTimeout: parseInt(e.target.value) || 30,
                      })
                    }
                  />
                </div>
              )}
            </div>

            <Separator />

            {/* Subtask Configuration */}
            <div className="space-y-4">
              <h4 className="font-medium">Subtask Configuration</h4>

              <div className="flex items-center justify-between">
                <div className="space-y-1">
                  <Label>Enable Subtasks</Label>
                  <p className="text-sm text-muted-foreground">
                    Break this task into smaller subtasks
                  </p>
                </div>
                <Switch
                  checked={editedTask.hasSubtasks}
                  onCheckedChange={(checked) =>
                    setEditedTask({ ...editedTask, hasSubtasks: checked })
                  }
                />
              </div>

              {editedTask.hasSubtasks && (
                <div className="space-y-4 p-4 rounded-lg bg-muted/50">
                  <div>
                    <Label>Subtask Execution Mode</Label>
                    <div className="grid grid-cols-2 gap-2 mt-2">
                      <Button
                        type="button"
                        variant={
                          editedTask.subtaskExecutionMode === "sequential"
                            ? "default"
                            : "outline"
                        }
                        onClick={() =>
                          setEditedTask({
                            ...editedTask,
                            subtaskExecutionMode: "sequential",
                          })
                        }
                        className="gap-2 justify-start"
                        size="sm"
                      >
                        <ArrowRight className="h-4 w-4" />
                        Sequential
                      </Button>
                      <Button
                        type="button"
                        variant={
                          editedTask.subtaskExecutionMode === "parallel"
                            ? "default"
                            : "outline"
                        }
                        onClick={() =>
                          setEditedTask({
                            ...editedTask,
                            subtaskExecutionMode: "parallel",
                          })
                        }
                        className="gap-2 justify-start"
                        size="sm"
                      >
                        <Zap className="h-4 w-4" />
                        Parallel
                      </Button>
                    </div>
                  </div>

                  {editedTask.subtaskExecutionMode === "parallel" && (
                    <div>
                      <Label htmlFor="max-subtask-concurrency">
                        Max Concurrent Subtasks:{" "}
                        {editedTask.maxSubtaskConcurrency}
                      </Label>
                      <Input
                        id="max-subtask-concurrency"
                        type="range"
                        min="1"
                        max="5"
                        value={editedTask.maxSubtaskConcurrency}
                        onChange={(e) =>
                          setEditedTask({
                            ...editedTask,
                            maxSubtaskConcurrency: parseInt(e.target.value),
                          })
                        }
                        className="mt-2"
                      />
                    </div>
                  )}

                  <div className="flex items-center justify-between">
                    <div className="space-y-1">
                      <Label>Subtasks Wait for Feedback (Default)</Label>
                      <p className="text-xs text-muted-foreground">
                        Default feedback setting for new subtasks
                      </p>
                    </div>
                    <Switch
                      checked={editedTask.subtaskWaitForFeedback}
                      onCheckedChange={(checked) =>
                        setEditedTask({
                          ...editedTask,
                          subtaskWaitForFeedback: checked,
                        })
                      }
                    />
                  </div>
                </div>
              )}
            </div>

            <Separator />

            <div className="flex justify-end gap-2">
              <Button variant="outline" onClick={handleCancel}>
                Cancel
              </Button>
              <Button onClick={handleSave}>Save Changes</Button>
            </div>
          </div>
        )}
      </div>
    </Card>
  );
}
