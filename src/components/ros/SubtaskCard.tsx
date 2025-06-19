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
  Minimize2,
} from "lucide-react";
import { cn } from "@/lib/utils";
import { Subtask } from "./TaskCard";

interface SubtaskCardProps {
  subtask: Subtask;
  index: number;
  isSequential: boolean;
  canStart: boolean;
  onUpdate: (subtask: Subtask) => void;
  onDelete: (subtaskId: string) => void;
  onDuplicate: (subtask: Subtask) => void;
  onStart: (subtaskId: string) => void;
  onPause: (subtaskId: string) => void;
  onStop: (subtaskId: string) => void;
}

export function SubtaskCard({
  subtask,
  index,
  isSequential,
  canStart,
  onUpdate,
  onDelete,
  onDuplicate,
  onStart,
  onPause,
  onStop,
}: SubtaskCardProps) {
  const [isEditing, setIsEditing] = useState(false);
  const [editedSubtask, setEditedSubtask] = useState(subtask);

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

  const StatusIcon = statusIcons[subtask.status];

  const handleSave = () => {
    onUpdate(editedSubtask);
    setIsEditing(false);
  };

  const handleCancel = () => {
    setEditedSubtask(subtask);
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
        "ml-6 transition-all duration-200 border-l-2 bg-muted/30",
        subtask.status === "running" && "border-l-primary shadow-md",
        subtask.status === "completed" && "border-l-ros-success",
        subtask.status === "failed" && "border-l-destructive",
        subtask.status === "paused" && "border-l-ros-warning",
        subtask.status === "pending" && "border-l-muted-foreground",
      )}
    >
      <div className="p-3">
        {/* Header */}
        <div className="flex items-center justify-between mb-3">
          <div className="flex items-center gap-2">
            <Minimize2 className="h-3 w-3 text-muted-foreground" />
            <Badge variant="outline" className="text-xs">
              #{index + 1}
            </Badge>
            <Badge className={cn(statusColors[subtask.status], "text-xs")}>
              <StatusIcon className="h-2 w-2 mr-1" />
              {subtask.status.replace("_", " ").toUpperCase()}
            </Badge>
          </div>

          <div className="flex items-center gap-1">
            {/* Control Buttons */}
            {subtask.status === "pending" && canStart && (
              <Button
                size="sm"
                onClick={() => onStart(subtask.id)}
                className="gap-1 h-6 px-2"
              >
                <Play className="h-2 w-2" />
                Start
              </Button>
            )}

            {subtask.status === "running" && (
              <>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => onPause(subtask.id)}
                  className="gap-1 h-6 px-2"
                >
                  <Pause className="h-2 w-2" />
                  Pause
                </Button>
                <Button
                  size="sm"
                  variant="destructive"
                  onClick={() => onStop(subtask.id)}
                  className="gap-1 h-6 px-2"
                >
                  <Square className="h-2 w-2" />
                  Stop
                </Button>
              </>
            )}

            {subtask.status === "paused" && (
              <Button
                size="sm"
                onClick={() => onStart(subtask.id)}
                className="gap-1 h-6 px-2"
              >
                <Play className="h-2 w-2" />
                Resume
              </Button>
            )}

            {subtask.status === "waiting_feedback" && (
              <>
                <Button
                  size="sm"
                  onClick={() => onStart(subtask.id)}
                  className="gap-1 h-6 px-2"
                >
                  <CheckCircle2 className="h-2 w-2" />
                  Confirm
                </Button>
                <Button
                  size="sm"
                  variant="destructive"
                  onClick={() => onStop(subtask.id)}
                  className="gap-1 h-6 px-2"
                >
                  <XCircle className="h-2 w-2" />
                  Reject
                </Button>
              </>
            )}

            {/* Action Buttons */}
            <Button
              size="sm"
              variant="ghost"
              onClick={() => setIsEditing(!isEditing)}
              className="h-6 w-6 p-0"
            >
              <Settings className="h-3 w-3" />
            </Button>
            <Button
              size="sm"
              variant="ghost"
              onClick={() => onDuplicate(subtask)}
              className="h-6 w-6 p-0"
            >
              <Copy className="h-3 w-3" />
            </Button>
            <Button
              size="sm"
              variant="ghost"
              onClick={() => onDelete(subtask.id)}
              className="h-6 w-6 p-0"
            >
              <Trash2 className="h-3 w-3" />
            </Button>
          </div>
        </div>

        {!isEditing ? (
          /* View Mode */
          <div className="space-y-2">
            <div>
              <h4 className="font-medium text-sm">{subtask.name}</h4>
              <p className="text-xs text-muted-foreground">
                {subtask.description}
              </p>
            </div>

            <div className="grid grid-cols-2 gap-2 text-xs">
              <div>
                <span className="text-muted-foreground">Type:</span>
                <p className="font-mono text-xs">{subtask.type}</p>
              </div>
              <div>
                <span className="text-muted-foreground">Duration:</span>
                <p className="font-mono text-xs">
                  {formatDuration(subtask.duration)}
                </p>
              </div>
            </div>

            {/* Progress Bar */}
            {subtask.status === "running" && (
              <div className="space-y-1">
                <div className="flex justify-between text-xs">
                  <span>Progress</span>
                  <span>{subtask.progress}%</span>
                </div>
                <Progress value={subtask.progress} className="h-1" />
              </div>
            )}

            {/* Error Message */}
            {subtask.status === "failed" && subtask.errorMessage && (
              <div className="p-2 rounded bg-destructive/10 border border-destructive/20">
                <div className="flex items-center gap-1 text-destructive mb-1">
                  <AlertTriangle className="h-3 w-3" />
                  <span className="font-medium text-xs">Error</span>
                </div>
                <p className="text-xs text-destructive/90">
                  {subtask.errorMessage}
                </p>
              </div>
            )}

            {/* Feedback Info */}
            <div className="flex items-center gap-2 text-xs">
              <span className="text-muted-foreground">Feedback:</span>
              <Badge
                variant={subtask.waitForFeedback ? "default" : "outline"}
                className="text-xs h-4"
              >
                {subtask.waitForFeedback ? "Required" : "Auto"}
              </Badge>
              {subtask.waitForFeedback && (
                <span className="font-mono text-muted-foreground">
                  {subtask.feedbackTimeout}s
                </span>
              )}
            </div>
          </div>
        ) : (
          /* Edit Mode */
          <div className="space-y-3">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
              <div>
                <Label htmlFor="subtask-name" className="text-xs">
                  Subtask Name
                </Label>
                <Input
                  id="subtask-name"
                  value={editedSubtask.name}
                  onChange={(e) =>
                    setEditedSubtask({ ...editedSubtask, name: e.target.value })
                  }
                  className="h-7"
                />
              </div>
              <div>
                <Label htmlFor="subtask-type" className="text-xs">
                  Subtask Type
                </Label>
                <Select
                  value={editedSubtask.type}
                  onValueChange={(value) =>
                    setEditedSubtask({ ...editedSubtask, type: value })
                  }
                >
                  <SelectTrigger className="h-7">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="ros_service">
                      ROS Service Call
                    </SelectItem>
                    <SelectItem value="ros_action">ROS Action</SelectItem>
                    <SelectItem value="movement">Robot Movement</SelectItem>
                    <SelectItem value="sensor_reading">
                      Sensor Reading
                    </SelectItem>
                    <SelectItem value="custom_script">Custom Script</SelectItem>
                    <SelectItem value="wait">Wait/Delay</SelectItem>
                  </SelectContent>
                </Select>
              </div>
            </div>

            <div>
              <Label htmlFor="subtask-description" className="text-xs">
                Description
              </Label>
              <Textarea
                id="subtask-description"
                value={editedSubtask.description}
                onChange={(e) =>
                  setEditedSubtask({
                    ...editedSubtask,
                    description: e.target.value,
                  })
                }
                rows={2}
                className="text-xs"
              />
            </div>

            <div className="grid grid-cols-2 gap-3">
              <div>
                <Label htmlFor="subtask-timeout" className="text-xs">
                  Timeout (seconds)
                </Label>
                <Input
                  id="subtask-timeout"
                  type="number"
                  value={editedSubtask.timeout}
                  onChange={(e) =>
                    setEditedSubtask({
                      ...editedSubtask,
                      timeout: parseInt(e.target.value) || 0,
                    })
                  }
                  className="h-7"
                />
              </div>
              <div>
                <Label htmlFor="subtask-retries" className="text-xs">
                  Max Retries
                </Label>
                <Input
                  id="subtask-retries"
                  type="number"
                  value={editedSubtask.retries}
                  onChange={(e) =>
                    setEditedSubtask({
                      ...editedSubtask,
                      retries: parseInt(e.target.value) || 0,
                    })
                  }
                  className="h-7"
                />
              </div>
            </div>

            <div className="flex items-center justify-between">
              <div className="space-y-1">
                <Label className="text-xs">Wait for Feedback</Label>
                <p className="text-xs text-muted-foreground">
                  Require manual confirmation
                </p>
              </div>
              <Switch
                checked={editedSubtask.waitForFeedback}
                onCheckedChange={(checked) =>
                  setEditedSubtask({
                    ...editedSubtask,
                    waitForFeedback: checked,
                  })
                }
              />
            </div>

            <Separator />

            <div className="flex justify-end gap-2">
              <Button variant="outline" onClick={handleCancel} size="sm">
                Cancel
              </Button>
              <Button onClick={handleSave} size="sm">
                Save
              </Button>
            </div>
          </div>
        )}
      </div>
    </Card>
  );
}
