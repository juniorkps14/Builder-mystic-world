import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { Switch } from "@/components/ui/switch";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { ExecutionModeSelector } from "./ExecutionModeSelector";
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
  Plus,
  Save,
  Upload,
  Download,
  FileText,
  Settings,
  Play,
  Pause,
  RotateCcw,
  Trash2,
} from "lucide-react";

export interface Sequence {
  id: string;
  name: string;
  description: string;
  isParallel: boolean;
  maxConcurrency: number;
  executionMode: {
    type: "sequential" | "parallel";
    maxConcurrency: number;
    waitForFeedback: boolean;
    globalFeedbackTimeout: number;
    failureHandling: "stop" | "continue" | "retry";
    retryCount: number;
  };
  status: "idle" | "running" | "paused" | "completed" | "failed";
  createdAt: Date;
  updatedAt: Date;
  tags: string[];
  taskIds: string[];
}

interface SequenceEditorProps {
  sequence?: Sequence;
  onSave: (sequence: Sequence) => void;
  onCancel: () => void;
}

export function SequenceEditor({
  sequence,
  onSave,
  onCancel,
}: SequenceEditorProps) {
  const [editedSequence, setEditedSequence] = useState<Sequence>(
    sequence || {
      id: `seq_${Date.now()}`,
      name: "",
      description: "",
      isParallel: false,
      maxConcurrency: 3,
      executionMode: {
        type: "sequential",
        maxConcurrency: 3,
        waitForFeedback: false,
        globalFeedbackTimeout: 30,
        failureHandling: "stop",
        retryCount: 1,
      },
      status: "idle",
      createdAt: new Date(),
      updatedAt: new Date(),
      tags: [],
      taskIds: [],
    },
  );

  const [newTag, setNewTag] = useState("");

  const handleSave = () => {
    onSave({
      ...editedSequence,
      updatedAt: new Date(),
    });
  };

  const addTag = () => {
    if (newTag.trim() && !editedSequence.tags.includes(newTag.trim())) {
      setEditedSequence({
        ...editedSequence,
        tags: [...editedSequence.tags, newTag.trim()],
      });
      setNewTag("");
    }
  };

  const removeTag = (tagToRemove: string) => {
    setEditedSequence({
      ...editedSequence,
      tags: editedSequence.tags.filter((tag) => tag !== tagToRemove),
    });
  };

  const taskTemplates = [
    {
      name: "Navigate to Position",
      type: "movement",
      description: "Move robot to specified coordinates",
    },
    {
      name: "Take Photo",
      type: "sensor_reading",
      description: "Capture image from camera",
    },
    {
      name: "Wait",
      type: "wait",
      description: "Pause execution for specified duration",
    },
    {
      name: "Check Battery",
      type: "ros_service",
      description: "Check robot battery level",
    },
    {
      name: "Custom Action",
      type: "ros_action",
      description: "Execute custom ROS action",
    },
  ];

  return (
    <div className="space-y-6">
      <Card className="p-6">
        <h3 className="text-lg font-semibold mb-4">
          {sequence ? "Edit Sequence" : "Create New Sequence"}
        </h3>

        <div className="space-y-4">
          {/* Basic Information */}
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div>
              <Label htmlFor="seq-name">Sequence Name</Label>
              <Input
                id="seq-name"
                value={editedSequence.name}
                onChange={(e) =>
                  setEditedSequence({ ...editedSequence, name: e.target.value })
                }
                placeholder="Enter sequence name..."
              />
            </div>
            <div>
              <Label htmlFor="seq-id">Sequence ID</Label>
              <Input
                id="seq-id"
                value={editedSequence.id}
                onChange={(e) =>
                  setEditedSequence({ ...editedSequence, id: e.target.value })
                }
                placeholder="unique_sequence_id"
              />
            </div>
          </div>

          <div>
            <Label htmlFor="seq-description">Description</Label>
            <Textarea
              id="seq-description"
              value={editedSequence.description}
              onChange={(e) =>
                setEditedSequence({
                  ...editedSequence,
                  description: e.target.value,
                })
              }
              placeholder="Describe what this sequence does..."
              rows={3}
            />
          </div>

          {/* Execution Settings */}
          <Separator />
          <h4 className="font-medium">Execution Settings</h4>

          <div className="flex items-center justify-between">
            <div className="space-y-1">
              <Label htmlFor="parallel-mode">Parallel Execution</Label>
              <p className="text-sm text-muted-foreground">
                {editedSequence.isParallel
                  ? "Tasks will run concurrently when possible"
                  : "Tasks will run sequentially, one after another"}
              </p>
            </div>
            <Switch
              id="parallel-mode"
              checked={editedSequence.isParallel}
              onCheckedChange={(checked) =>
                setEditedSequence({ ...editedSequence, isParallel: checked })
              }
            />
          </div>

          {editedSequence.isParallel && (
            <div>
              <Label htmlFor="max-concurrency">Max Concurrent Tasks</Label>
              <Input
                id="max-concurrency"
                type="number"
                min="1"
                max="10"
                value={editedSequence.maxConcurrency}
                onChange={(e) =>
                  setEditedSequence({
                    ...editedSequence,
                    maxConcurrency: parseInt(e.target.value) || 1,
                  })
                }
              />
            </div>
          )}

          {/* Tags */}
          <Separator />
          <div>
            <Label htmlFor="tags">Tags</Label>
            <div className="flex gap-2 mb-2">
              <Input
                id="tags"
                value={newTag}
                onChange={(e) => setNewTag(e.target.value)}
                placeholder="Add a tag..."
                onKeyPress={(e) => e.key === "Enter" && addTag()}
              />
              <Button onClick={addTag} size="sm">
                <Plus className="h-4 w-4" />
              </Button>
            </div>
            {editedSequence.tags.length > 0 && (
              <div className="flex flex-wrap gap-2">
                {editedSequence.tags.map((tag, index) => (
                  <Badge key={index} variant="secondary" className="gap-1">
                    {tag}
                    <button
                      onClick={() => removeTag(tag)}
                      className="ml-1 hover:text-destructive"
                    >
                      ×
                    </button>
                  </Badge>
                ))}
              </div>
            )}
          </div>

          {/* Actions */}
          <Separator />
          <div className="flex justify-between">
            <div className="flex gap-2">
              <Dialog>
                <DialogTrigger asChild>
                  <Button variant="outline" className="gap-2">
                    <FileText className="h-4 w-4" />
                    Templates
                  </Button>
                </DialogTrigger>
                <DialogContent>
                  <DialogHeader>
                    <DialogTitle>Task Templates</DialogTitle>
                    <DialogDescription>
                      Choose from predefined task templates to quickly add
                      common tasks.
                    </DialogDescription>
                  </DialogHeader>
                  <div className="space-y-2">
                    {taskTemplates.map((template, index) => (
                      <Card
                        key={index}
                        className="p-3 cursor-pointer hover:bg-accent"
                      >
                        <div className="flex justify-between items-start">
                          <div>
                            <h4 className="font-medium">{template.name}</h4>
                            <p className="text-sm text-muted-foreground">
                              {template.description}
                            </p>
                          </div>
                          <Badge variant="outline">{template.type}</Badge>
                        </div>
                      </Card>
                    ))}
                  </div>
                  <DialogFooter>
                    <Button variant="outline">Cancel</Button>
                    <Button>Add Selected</Button>
                  </DialogFooter>
                </DialogContent>
              </Dialog>

              <Button variant="outline" className="gap-2">
                <Upload className="h-4 w-4" />
                Import
              </Button>
              <Button variant="outline" className="gap-2">
                <Download className="h-4 w-4" />
                Export
              </Button>
            </div>

            <div className="flex gap-2">
              <Button variant="outline" onClick={onCancel}>
                Cancel
              </Button>
              <Button onClick={handleSave} className="gap-2">
                <Save className="h-4 w-4" />
                Save Sequence
              </Button>
            </div>
          </div>
        </div>
      </Card>

      <ExecutionModeSelector
        mode={editedSequence.executionMode}
        onModeChange={(mode) =>
          setEditedSequence({
            ...editedSequence,
            executionMode: mode,
            isParallel: mode.type === "parallel",
            maxConcurrency: mode.maxConcurrency,
          })
        }
      />
    </div>
  );
}
