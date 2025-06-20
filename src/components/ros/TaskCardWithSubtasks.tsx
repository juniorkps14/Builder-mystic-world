import { useState } from "react";
import { TaskCard, Task, Subtask } from "./TaskCard";
import { SubtaskCard } from "./SubtaskCard";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import {
  Collapsible,
  CollapsibleContent,
  CollapsibleTrigger,
} from "@/components/ui/collapsible";
import {
  ChevronDown,
  ChevronRight,
  Plus,
  Layers,
  ArrowRight,
  Zap,
} from "lucide-react";

interface TaskCardWithSubtasksProps {
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
  dragHandleProps?: any;
}

export function TaskCardWithSubtasks(props: TaskCardWithSubtasksProps) {
  const [showSubtasks, setShowSubtasks] = useState(true);

  const handleSubtaskUpdate = (taskId: string, updatedSubtask: Subtask) => {
    const updatedTask = {
      ...props.task,
      subtasks: props.task.subtasks.map((subtask) =>
        subtask.id === updatedSubtask.id ? updatedSubtask : subtask,
      ),
    };
    props.onUpdate(updatedTask);
  };

  const handleSubtaskDelete = (taskId: string, subtaskId: string) => {
    const updatedTask = {
      ...props.task,
      subtasks: props.task.subtasks.filter(
        (subtask) => subtask.id !== subtaskId,
      ),
    };
    props.onUpdate(updatedTask);
  };

  const handleSubtaskAdd = (taskId: string) => {
    const newSubtask: Subtask = {
      id: `subtask_${Date.now()}`,
      name: "New Subtask",
      type: "movement",
      description: "Description for new subtask",
      parameters: {},
      timeout: 30,
      retries: 1,
      waitForFeedback: props.task.subtaskWaitForFeedback,
      feedbackTimeout: 30,
      status: "pending",
      progress: 0,
      duration: 0,
    };

    const updatedTask = {
      ...props.task,
      subtasks: [...props.task.subtasks, newSubtask],
    };
    props.onUpdate(updatedTask);
  };

  const handleSubtaskStart = (taskId: string, subtaskId: string) => {
    console.log(`Start subtask: ${subtaskId} in task: ${taskId}`);
    // Implement subtask start logic
  };

  const handleSubtaskPause = (taskId: string, subtaskId: string) => {
    console.log(`Pause subtask: ${subtaskId} in task: ${taskId}`);
    // Implement subtask pause logic
  };

  const handleSubtaskStop = (taskId: string, subtaskId: string) => {
    console.log(`Stop subtask: ${subtaskId} in task: ${taskId}`);
    // Implement subtask stop logic
  };

  const handleSubtaskDuplicate = (subtask: Subtask) => {
    const duplicatedSubtask: Subtask = {
      ...subtask,
      id: `${subtask.id}_copy_${Date.now()}`,
      name: `${subtask.name} (Copy)`,
      status: "pending",
      progress: 0,
      duration: 0,
      startTime: undefined,
      endTime: undefined,
      errorMessage: undefined,
    };

    const updatedTask = {
      ...props.task,
      subtasks: [...props.task.subtasks, duplicatedSubtask],
    };
    props.onUpdate(updatedTask);
  };

  const canStartSubtask = (subtask: Subtask, isSequential: boolean) => {
    if (!isSequential) return true;
    const subtaskIndex = props.task.subtasks.findIndex(
      (s) => s.id === subtask.id,
    );
    if (subtaskIndex === 0) return true;
    return props.task.subtasks[subtaskIndex - 1]?.status === "completed";
  };

  const getSubtaskProgress = () => {
    if (props.task.subtasks.length === 0) return 0;
    const totalProgress = props.task.subtasks.reduce(
      (sum, subtask) => sum + subtask.progress,
      0,
    );
    return Math.round(totalProgress / props.task.subtasks.length);
  };

  return (
    <div className="space-y-2">
      {/* Main Task Card */}
      <TaskCard
        {...props}
        onSubtaskUpdate={handleSubtaskUpdate}
        onSubtaskDelete={handleSubtaskDelete}
        onSubtaskAdd={handleSubtaskAdd}
        onSubtaskStart={handleSubtaskStart}
        onSubtaskPause={handleSubtaskPause}
        onSubtaskStop={handleSubtaskStop}
      />

      {/* Subtasks Section */}
      {props.task.hasSubtasks && (
        <div className="ml-8">
          <Collapsible open={showSubtasks} onOpenChange={setShowSubtasks}>
            <CollapsibleTrigger asChild>
              <Button
                variant="ghost"
                size="sm"
                className="w-full justify-start gap-2 text-muted-foreground hover:text-foreground"
              >
                {showSubtasks ? (
                  <ChevronDown className="h-4 w-4" />
                ) : (
                  <ChevronRight className="h-4 w-4" />
                )}
                <Layers className="h-4 w-4" />
                <span>Subtasks ({props.task.subtasks.length})</span>
                {props.task.subtaskExecutionMode === "parallel" ? (
                  <Badge variant="outline" className="gap-1">
                    <Zap className="h-3 w-3" />
                    Parallel
                  </Badge>
                ) : (
                  <Badge variant="outline" className="gap-1">
                    <ArrowRight className="h-3 w-3" />
                    Sequential
                  </Badge>
                )}
                {props.task.subtasks.length > 0 && (
                  <Badge variant="secondary" className="ml-auto">
                    {getSubtaskProgress()}%
                  </Badge>
                )}
              </Button>
            </CollapsibleTrigger>

            <CollapsibleContent className="space-y-2 mt-2">
              {props.task.subtasks.length === 0 ? (
                <div className="text-center py-6 text-muted-foreground">
                  <Layers className="h-8 w-8 mx-auto mb-2 opacity-50" />
                  <p className="text-sm">No subtasks yet</p>
                  <Button
                    size="sm"
                    onClick={() => handleSubtaskAdd(props.task.id)}
                    className="gap-2 mt-2"
                  >
                    <Plus className="h-3 w-3" />
                    Add First Subtask
                  </Button>
                </div>
              ) : (
                <>
                  {props.task.subtasks.map((subtask, subtaskIndex) => (
                    <SubtaskCard
                      key={subtask.id}
                      subtask={subtask}
                      index={subtaskIndex}
                      isSequential={
                        props.task.subtaskExecutionMode === "sequential"
                      }
                      canStart={canStartSubtask(
                        subtask,
                        props.task.subtaskExecutionMode === "sequential",
                      )}
                      onUpdate={(updatedSubtask) =>
                        handleSubtaskUpdate(props.task.id, updatedSubtask)
                      }
                      onDelete={(subtaskId) =>
                        handleSubtaskDelete(props.task.id, subtaskId)
                      }
                      onDuplicate={handleSubtaskDuplicate}
                      onStart={(subtaskId) =>
                        handleSubtaskStart(props.task.id, subtaskId)
                      }
                      onPause={(subtaskId) =>
                        handleSubtaskPause(props.task.id, subtaskId)
                      }
                      onStop={(subtaskId) =>
                        handleSubtaskStop(props.task.id, subtaskId)
                      }
                    />
                  ))}

                  <div className="flex justify-center">
                    <Button
                      size="sm"
                      variant="outline"
                      onClick={() => handleSubtaskAdd(props.task.id)}
                      className="gap-2"
                    >
                      <Plus className="h-3 w-3" />
                      Add Subtask
                    </Button>
                  </div>
                </>
              )}
            </CollapsibleContent>
          </Collapsible>
        </div>
      )}
    </div>
  );
}
