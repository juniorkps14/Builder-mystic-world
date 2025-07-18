import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
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
  Play,
  Pause,
  Square,
  Plus,
  Trash2,
  Edit,
  Copy,
  List,
  Activity,
  CheckCircle,
  Clock,
  AlertCircle,
  Settings,
} from "lucide-react";

interface SimpleTask {
  id: string;
  name: string;
  type: "movement" | "sensor" | "action" | "wait";
  description: string;
  duration: number;
  status: "pending" | "running" | "completed" | "failed";
  progress: number;
}

interface SimpleSequence {
  id: string;
  name: string;
  description: string;
  tasks: SimpleTask[];
  status: "idle" | "running" | "paused" | "completed" | "failed";
  isRunning: boolean;
}

export function BasicSequenceManager() {
  const { toast } = useToast();
  const [sequences, setSequences] = useState<SimpleSequence[]>([]);
  const [selectedSequence, setSelectedSequence] =
    useState<SimpleSequence | null>(null);
  const [isCreateSequenceOpen, setIsCreateSequenceOpen] = useState(false);
  const [isCreateTaskOpen, setIsCreateTaskOpen] = useState(false);
  const [isEditTaskOpen, setIsEditTaskOpen] = useState(false);
  const [editingTask, setEditingTask] = useState<SimpleTask | null>(null);

  // Form states
  const [newSequenceName, setNewSequenceName] = useState("");
  const [newSequenceDescription, setNewSequenceDescription] = useState("");
  const [newTaskName, setNewTaskName] = useState("");
  const [newTaskType, setNewTaskType] =
    useState<SimpleTask["type"]>("movement");
  const [newTaskDescription, setNewTaskDescription] = useState("");
  const [newTaskDuration, setNewTaskDuration] = useState(10);

  // Mock data
  useEffect(() => {
    const mockSequences: SimpleSequence[] = [
      {
        id: "patrol_basic",
        name: "Basic Patrol",
        description: "Simple security patrol route",
        status: "idle",
        isRunning: false,
        tasks: [
          {
            id: "move_1",
            name: "Move to Point A",
            type: "movement",
            description: "Go to checkpoint A",
            duration: 30,
            status: "completed",
            progress: 100,
          },
          {
            id: "wait_1",
            name: "Wait 5 seconds",
            type: "wait",
            description: "Wait at checkpoint",
            duration: 5,
            status: "running",
            progress: 60,
          },
          {
            id: "move_2",
            name: "Return to Base",
            type: "movement",
            description: "Return to starting position",
            duration: 25,
            status: "pending",
            progress: 0,
          },
        ],
      },
      {
        id: "cleaning_basic",
        name: "Room Cleaning",
        description: "Basic room cleaning sequence",
        status: "idle",
        isRunning: false,
        tasks: [
          {
            id: "scan_1",
            name: "Scan Room",
            type: "sensor",
            description: "Scan room for obstacles",
            duration: 10,
            status: "completed",
            progress: 100,
          },
          {
            id: "clean_1",
            name: "Start Cleaning",
            type: "action",
            description: "Begin cleaning process",
            duration: 120,
            status: "pending",
            progress: 0,
          },
        ],
      },
    ];

    setSequences(mockSequences);
    setSelectedSequence(mockSequences[0]);
  }, []);

  // Create new sequence
  const createSequence = () => {
    if (!newSequenceName.trim()) return;

    const newSequence: SimpleSequence = {
      id: `seq_${Date.now()}`,
      name: newSequenceName,
      description: newSequenceDescription,
      tasks: [],
      status: "idle",
      isRunning: false,
    };

    setSequences((prev) => [...prev, newSequence]);
    setSelectedSequence(newSequence);
    setIsCreateSequenceOpen(false);
    setNewSequenceName("");
    setNewSequenceDescription("");

    toast({
      title: "✅ Sequence Created",
      description: `${newSequenceName} has been created`,
    });
  };

  // Create new task
  const createTask = () => {
    if (!selectedSequence || !newTaskName.trim()) return;

    const newTask: SimpleTask = {
      id: `task_${Date.now()}`,
      name: newTaskName,
      type: newTaskType,
      description: newTaskDescription,
      duration: newTaskDuration,
      status: "pending",
      progress: 0,
    };

    const updatedSequence = {
      ...selectedSequence,
      tasks: [...selectedSequence.tasks, newTask],
    };

    setSequences((prev) =>
      prev.map((seq) =>
        seq.id === selectedSequence.id ? updatedSequence : seq,
      ),
    );
    setSelectedSequence(updatedSequence);
    setIsCreateTaskOpen(false);
    setNewTaskName("");
    setNewTaskDescription("");
    setNewTaskDuration(10);

    toast({
      title: "✅ Task Added",
      description: `${newTaskName} has been added`,
    });
  };

  // Update task
  const updateTask = () => {
    if (!selectedSequence || !editingTask) return;

    const updatedSequence = {
      ...selectedSequence,
      tasks: selectedSequence.tasks.map((task) =>
        task.id === editingTask.id ? editingTask : task,
      ),
    };

    setSequences((prev) =>
      prev.map((seq) =>
        seq.id === selectedSequence.id ? updatedSequence : seq,
      ),
    );
    setSelectedSequence(updatedSequence);
    setIsEditTaskOpen(false);
    setEditingTask(null);

    toast({
      title: "✅ Task Updated",
      description: "Task has been updated",
    });
  };

  // Delete task
  const deleteTask = (taskId: string) => {
    if (!selectedSequence) return;

    const updatedSequence = {
      ...selectedSequence,
      tasks: selectedSequence.tasks.filter((task) => task.id !== taskId),
    };

    setSequences((prev) =>
      prev.map((seq) =>
        seq.id === selectedSequence.id ? updatedSequence : seq,
      ),
    );
    setSelectedSequence(updatedSequence);

    toast({
      title: "🗑️ Task Deleted",
      description: "Task has been removed",
      variant: "destructive",
    });
  };

  // Start sequence
  const startSequence = (sequence: SimpleSequence) => {
    if (sequence.tasks.length === 0) {
      toast({
        title: "❌ Cannot Start",
        description: "Add some tasks first",
        variant: "destructive",
      });
      return;
    }

    const updatedSequence = {
      ...sequence,
      status: "running" as const,
      isRunning: true,
    };
    setSequences((prev) =>
      prev.map((seq) => (seq.id === sequence.id ? updatedSequence : seq)),
    );
    setSelectedSequence(updatedSequence);

    toast({
      title: "🚀 Sequence Started",
      description: `${sequence.name} is now running`,
    });
  };

  // Stop sequence
  const stopSequence = (sequence: SimpleSequence) => {
    const updatedSequence = {
      ...sequence,
      status: "idle" as const,
      isRunning: false,
    };
    setSequences((prev) =>
      prev.map((seq) => (seq.id === sequence.id ? updatedSequence : seq)),
    );
    setSelectedSequence(updatedSequence);

    toast({
      title: "⏹️ Sequence Stopped",
      description: `${sequence.name} has been stopped`,
    });
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "running":
        return <Activity className="h-4 w-4 text-blue-500 animate-pulse" />;
      case "completed":
        return <CheckCircle className="h-4 w-4 text-green-500" />;
      case "failed":
        return <AlertCircle className="h-4 w-4 text-red-500" />;
      default:
        return <Clock className="h-4 w-4 text-gray-400" />;
    }
  };

  const taskTypes = [
    {
      value: "movement",
      label: "Movement",
      description: "Move robot to position",
    },
    { value: "sensor", label: "Sensor", description: "Read sensor data" },
    { value: "action", label: "Action", description: "Perform robot action" },
    { value: "wait", label: "Wait", description: "Wait for specified time" },
  ];

  return (
    <div className="max-w-7xl mx-auto p-6 space-y-6">
      {/* Simple Header */}
      <div className="text-center space-y-2">
        <h1 className="text-3xl font-bold">Robot Sequences</h1>
        <p className="text-muted-foreground">
          Create and run simple robot task sequences
        </p>
      </div>

      {/* Main Layout */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Sequences List */}
        <Card className="p-6">
          <div className="flex items-center justify-between mb-4">
            <h2 className="text-xl font-semibold">My Sequences</h2>
            <Dialog
              open={isCreateSequenceOpen}
              onOpenChange={setIsCreateSequenceOpen}
            >
              <DialogTrigger asChild>
                <Button className="gap-2">
                  <Plus className="h-4 w-4" />
                  New
                </Button>
              </DialogTrigger>
              <DialogContent>
                <DialogHeader>
                  <DialogTitle>Create New Sequence</DialogTitle>
                  <DialogDescription>
                    Create a new sequence of tasks for your robot
                  </DialogDescription>
                </DialogHeader>
                <div className="space-y-4">
                  <div>
                    <Label>Sequence Name</Label>
                    <Input
                      value={newSequenceName}
                      onChange={(e) => setNewSequenceName(e.target.value)}
                      placeholder="Enter sequence name"
                    />
                  </div>
                  <div>
                    <Label>Description</Label>
                    <Textarea
                      value={newSequenceDescription}
                      onChange={(e) =>
                        setNewSequenceDescription(e.target.value)
                      }
                      placeholder="What does this sequence do?"
                      rows={3}
                    />
                  </div>
                </div>
                <DialogFooter>
                  <Button
                    variant="outline"
                    onClick={() => setIsCreateSequenceOpen(false)}
                  >
                    Cancel
                  </Button>
                  <Button
                    onClick={createSequence}
                    disabled={!newSequenceName.trim()}
                  >
                    Create
                  </Button>
                </DialogFooter>
              </DialogContent>
            </Dialog>
          </div>

          <ScrollArea className="h-[500px]">
            <div className="space-y-3">
              {sequences.map((sequence) => (
                <div
                  key={sequence.id}
                  className={`p-4 rounded-lg border cursor-pointer transition-all ${
                    selectedSequence?.id === sequence.id
                      ? "border-blue-500 bg-blue-50"
                      : "border-gray-200 hover:border-gray-300"
                  }`}
                  onClick={() => setSelectedSequence(sequence)}
                >
                  <div className="flex items-center justify-between mb-2">
                    <h3 className="font-medium">{sequence.name}</h3>
                    <div className="flex items-center gap-2">
                      {sequence.isRunning ? (
                        <Button
                          size="sm"
                          variant="outline"
                          onClick={(e) => {
                            e.stopPropagation();
                            stopSequence(sequence);
                          }}
                        >
                          <Square className="h-3 w-3" />
                        </Button>
                      ) : (
                        <Button
                          size="sm"
                          onClick={(e) => {
                            e.stopPropagation();
                            startSequence(sequence);
                          }}
                        >
                          <Play className="h-3 w-3" />
                        </Button>
                      )}
                    </div>
                  </div>
                  <p className="text-sm text-muted-foreground mb-3">
                    {sequence.description}
                  </p>
                  <div className="flex items-center justify-between text-sm">
                    <span>{sequence.tasks.length} tasks</span>
                    <Badge
                      variant={sequence.isRunning ? "default" : "secondary"}
                    >
                      {sequence.isRunning ? "Running" : "Ready"}
                    </Badge>
                  </div>
                </div>
              ))}
            </div>
          </ScrollArea>
        </Card>

        {/* Tasks Management */}
        <div className="lg:col-span-2">
          {selectedSequence ? (
            <Card className="p-6">
              <div className="flex items-center justify-between mb-6">
                <div>
                  <h2 className="text-xl font-semibold">
                    {selectedSequence.name}
                  </h2>
                  <p className="text-muted-foreground">
                    {selectedSequence.description}
                  </p>
                </div>
                <Dialog
                  open={isCreateTaskOpen}
                  onOpenChange={setIsCreateTaskOpen}
                >
                  <DialogTrigger asChild>
                    <Button className="gap-2">
                      <Plus className="h-4 w-4" />
                      Add Task
                    </Button>
                  </DialogTrigger>
                  <DialogContent>
                    <DialogHeader>
                      <DialogTitle>Add New Task</DialogTitle>
                      <DialogDescription>
                        Add a new task to this sequence
                      </DialogDescription>
                    </DialogHeader>
                    <div className="space-y-4">
                      <div>
                        <Label>Task Name</Label>
                        <Input
                          value={newTaskName}
                          onChange={(e) => setNewTaskName(e.target.value)}
                          placeholder="Enter task name"
                        />
                      </div>
                      <div>
                        <Label>Task Type</Label>
                        <Select
                          value={newTaskType}
                          onValueChange={(value) =>
                            setNewTaskType(value as SimpleTask["type"])
                          }
                        >
                          <SelectTrigger>
                            <SelectValue />
                          </SelectTrigger>
                          <SelectContent>
                            {taskTypes.map((type) => (
                              <SelectItem key={type.value} value={type.value}>
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
                        <Label>Description</Label>
                        <Textarea
                          value={newTaskDescription}
                          onChange={(e) =>
                            setNewTaskDescription(e.target.value)
                          }
                          placeholder="What does this task do?"
                          rows={2}
                        />
                      </div>
                      <div>
                        <Label>Duration (seconds)</Label>
                        <Input
                          type="number"
                          value={newTaskDuration}
                          onChange={(e) =>
                            setNewTaskDuration(parseInt(e.target.value) || 10)
                          }
                          min="1"
                          max="3600"
                        />
                      </div>
                    </div>
                    <DialogFooter>
                      <Button
                        variant="outline"
                        onClick={() => setIsCreateTaskOpen(false)}
                      >
                        Cancel
                      </Button>
                      <Button
                        onClick={createTask}
                        disabled={!newTaskName.trim()}
                      >
                        Add Task
                      </Button>
                    </DialogFooter>
                  </DialogContent>
                </Dialog>
              </div>

              {selectedSequence.tasks.length === 0 ? (
                <div className="text-center py-12">
                  <List className="h-16 w-16 text-muted-foreground mx-auto mb-4" />
                  <h3 className="text-lg font-semibold mb-2">No Tasks Yet</h3>
                  <p className="text-muted-foreground mb-4">
                    Add your first task to get started
                  </p>
                  <Button
                    onClick={() => setIsCreateTaskOpen(true)}
                    className="gap-2"
                  >
                    <Plus className="h-4 w-4" />
                    Add First Task
                  </Button>
                </div>
              ) : (
                <div className="space-y-3">
                  {selectedSequence.tasks.map((task, index) => (
                    <div key={task.id} className="border rounded-lg p-4">
                      <div className="flex items-center justify-between mb-3">
                        <div className="flex items-center gap-3">
                          <div className="flex items-center justify-center w-8 h-8 rounded-full bg-blue-100 text-blue-600 text-sm font-bold">
                            {index + 1}
                          </div>
                          <div>
                            <h4 className="font-medium">{task.name}</h4>
                            <p className="text-sm text-muted-foreground">
                              {task.description}
                            </p>
                          </div>
                        </div>
                        <div className="flex items-center gap-2">
                          <Badge variant="outline" className="capitalize">
                            {task.type}
                          </Badge>
                          <div className="flex items-center gap-1">
                            {getStatusIcon(task.status)}
                            <span className="text-sm capitalize">
                              {task.status}
                            </span>
                          </div>
                          <Button
                            size="sm"
                            variant="ghost"
                            onClick={() => {
                              setEditingTask(task);
                              setIsEditTaskOpen(true);
                            }}
                          >
                            <Edit className="h-4 w-4" />
                          </Button>
                          <AlertDialog>
                            <AlertDialogTrigger asChild>
                              <Button size="sm" variant="ghost">
                                <Trash2 className="h-4 w-4" />
                              </Button>
                            </AlertDialogTrigger>
                            <AlertDialogContent>
                              <AlertDialogHeader>
                                <AlertDialogTitle>Delete Task</AlertDialogTitle>
                                <AlertDialogDescription>
                                  Are you sure you want to delete "{task.name}"?
                                </AlertDialogDescription>
                              </AlertDialogHeader>
                              <AlertDialogFooter>
                                <AlertDialogCancel>Cancel</AlertDialogCancel>
                                <AlertDialogAction
                                  onClick={() => deleteTask(task.id)}
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
                        <div className="space-y-2">
                          <div className="flex justify-between text-sm">
                            <span>Progress</span>
                            <span>{task.progress}%</span>
                          </div>
                          <Progress value={task.progress} className="h-2" />
                        </div>
                      )}

                      <div className="flex items-center justify-between mt-3 text-sm text-muted-foreground">
                        <span>Duration: {task.duration} seconds</span>
                      </div>
                    </div>
                  ))}
                </div>
              )}
            </Card>
          ) : (
            <Card className="p-12 text-center">
              <Activity className="h-16 w-16 text-muted-foreground mx-auto mb-4" />
              <h3 className="text-xl font-semibold mb-2">Select a Sequence</h3>
              <p className="text-muted-foreground">
                Choose a sequence from the left to view and manage its tasks
              </p>
            </Card>
          )}
        </div>
      </div>

      {/* Edit Task Dialog */}
      {editingTask && (
        <Dialog open={isEditTaskOpen} onOpenChange={setIsEditTaskOpen}>
          <DialogContent>
            <DialogHeader>
              <DialogTitle>Edit Task</DialogTitle>
              <DialogDescription>Update task details</DialogDescription>
            </DialogHeader>
            <div className="space-y-4">
              <div>
                <Label>Task Name</Label>
                <Input
                  value={editingTask.name}
                  onChange={(e) =>
                    setEditingTask({ ...editingTask, name: e.target.value })
                  }
                />
              </div>
              <div>
                <Label>Task Type</Label>
                <Select
                  value={editingTask.type}
                  onValueChange={(value) =>
                    setEditingTask({
                      ...editingTask,
                      type: value as SimpleTask["type"],
                    })
                  }
                >
                  <SelectTrigger>
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    {taskTypes.map((type) => (
                      <SelectItem key={type.value} value={type.value}>
                        {type.label}
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
              <div>
                <Label>Description</Label>
                <Textarea
                  value={editingTask.description}
                  onChange={(e) =>
                    setEditingTask({
                      ...editingTask,
                      description: e.target.value,
                    })
                  }
                  rows={2}
                />
              </div>
              <div>
                <Label>Duration (seconds)</Label>
                <Input
                  type="number"
                  value={editingTask.duration}
                  onChange={(e) =>
                    setEditingTask({
                      ...editingTask,
                      duration: parseInt(e.target.value) || 10,
                    })
                  }
                  min="1"
                  max="3600"
                />
              </div>
            </div>
            <DialogFooter>
              <Button
                variant="outline"
                onClick={() => setIsEditTaskOpen(false)}
              >
                Cancel
              </Button>
              <Button onClick={updateTask}>Save Changes</Button>
            </DialogFooter>
          </DialogContent>
        </Dialog>
      )}
    </div>
  );
}
