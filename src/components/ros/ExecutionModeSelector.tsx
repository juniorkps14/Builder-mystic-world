import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Label } from "@/components/ui/label";
import { RadioGroup, RadioGroupItem } from "@/components/ui/radio-group";
import { Switch } from "@/components/ui/switch";
import { Input } from "@/components/ui/input";
import { Separator } from "@/components/ui/separator";
import {
  ArrowRight,
  Zap,
  Users,
  Clock,
  AlertTriangle,
  CheckCircle,
  Settings,
} from "lucide-react";

interface ExecutionMode {
  type: "sequential" | "parallel";
  maxConcurrency: number;
  waitForFeedback: boolean;
  globalFeedbackTimeout: number;
  failureHandling: "stop" | "continue" | "retry";
  retryCount: number;
}

interface ExecutionModeSelectorProps {
  mode: ExecutionMode;
  onModeChange: (mode: ExecutionMode) => void;
}

export function ExecutionModeSelector({
  mode,
  onModeChange,
}: ExecutionModeSelectorProps) {
  const updateMode = (updates: Partial<ExecutionMode>) => {
    onModeChange({ ...mode, ...updates });
  };

  return (
    <Card className="p-6">
      <div className="flex items-center gap-3 mb-6">
        <Settings className="h-6 w-6 text-primary" />
        <h3 className="text-xl font-semibold">Execution Settings</h3>
      </div>

      <div className="space-y-6">
        {/* Execution Type */}
        <div>
          <Label className="text-base font-semibold mb-4 block">
            Execution Mode
          </Label>
          <RadioGroup
            value={mode.type}
            onValueChange={(value: "sequential" | "parallel") =>
              updateMode({ type: value })
            }
            className="space-y-4"
          >
            <div className="flex items-start space-x-3">
              <RadioGroupItem value="sequential" id="sequential" />
              <div className="flex-1">
                <label
                  htmlFor="sequential"
                  className="flex items-center gap-2 font-medium cursor-pointer"
                >
                  <ArrowRight className="h-4 w-4 text-primary" />
                  Sequential Execution
                </label>
                <p className="text-sm text-muted-foreground mt-1">
                  Tasks run one after another in order. Next task starts only
                  when the previous one completes.
                </p>
                <div className="flex items-center gap-2 mt-2">
                  <Badge variant="outline" className="text-xs">
                    Predictable
                  </Badge>
                  <Badge variant="outline" className="text-xs">
                    Slower
                  </Badge>
                  <Badge variant="outline" className="text-xs">
                    Dependencies
                  </Badge>
                </div>
              </div>
            </div>

            <div className="flex items-start space-x-3">
              <RadioGroupItem value="parallel" id="parallel" />
              <div className="flex-1">
                <label
                  htmlFor="parallel"
                  className="flex items-center gap-2 font-medium cursor-pointer"
                >
                  <Zap className="h-4 w-4 text-accent" />
                  Parallel Execution
                </label>
                <p className="text-sm text-muted-foreground mt-1">
                  Multiple tasks can run simultaneously. Faster execution but
                  requires careful dependency management.
                </p>
                <div className="flex items-center gap-2 mt-2">
                  <Badge variant="outline" className="text-xs">
                    Faster
                  </Badge>
                  <Badge variant="outline" className="text-xs">
                    Concurrent
                  </Badge>
                  <Badge variant="outline" className="text-xs">
                    Resource Aware
                  </Badge>
                </div>
              </div>
            </div>
          </RadioGroup>
        </div>

        {/* Parallel Settings */}
        {mode.type === "parallel" && (
          <div className="space-y-4 p-4 rounded-lg bg-accent/10 border border-accent/20">
            <div className="flex items-center gap-2">
              <Users className="h-4 w-4 text-accent" />
              <Label className="font-semibold">Parallel Configuration</Label>
            </div>
            <div>
              <Label htmlFor="max-concurrency" className="text-sm">
                Maximum Concurrent Tasks: {mode.maxConcurrency}
              </Label>
              <Input
                id="max-concurrency"
                type="range"
                min="1"
                max="10"
                value={mode.maxConcurrency}
                onChange={(e) =>
                  updateMode({ maxConcurrency: parseInt(e.target.value) })
                }
                className="mt-2"
              />
              <p className="text-xs text-muted-foreground mt-1">
                Higher values allow more tasks to run simultaneously but may
                strain system resources.
              </p>
            </div>
          </div>
        )}

        <Separator />

        {/* Feedback Settings */}
        <div className="space-y-4">
          <div className="flex items-center gap-2">
            <AlertTriangle className="h-4 w-4 text-ros-warning" />
            <Label className="font-semibold">Feedback Control</Label>
          </div>

          <div className="flex items-center justify-between">
            <div className="space-y-1">
              <Label>Global Feedback Requirement</Label>
              <p className="text-sm text-muted-foreground">
                {mode.waitForFeedback
                  ? "All tasks will wait for manual confirmation by default"
                  : "Tasks will complete automatically unless individually configured"}
              </p>
            </div>
            <Switch
              checked={mode.waitForFeedback}
              onCheckedChange={(checked) =>
                updateMode({ waitForFeedback: checked })
              }
            />
          </div>

          {mode.waitForFeedback && (
            <div>
              <Label htmlFor="global-feedback-timeout">
                Global Feedback Timeout (seconds)
              </Label>
              <Input
                id="global-feedback-timeout"
                type="number"
                min="10"
                max="600"
                value={mode.globalFeedbackTimeout}
                onChange={(e) =>
                  updateMode({
                    globalFeedbackTimeout: parseInt(e.target.value) || 30,
                  })
                }
                className="mt-1"
              />
              <p className="text-xs text-muted-foreground mt-1">
                How long to wait for user feedback before timing out. Individual
                tasks can override this setting.
              </p>
            </div>
          )}
        </div>

        <Separator />

        {/* Failure Handling */}
        <div className="space-y-4">
          <div className="flex items-center gap-2">
            <AlertTriangle className="h-4 w-4 text-destructive" />
            <Label className="font-semibold">Failure Handling</Label>
          </div>

          <RadioGroup
            value={mode.failureHandling}
            onValueChange={(value: "stop" | "continue" | "retry") =>
              updateMode({ failureHandling: value })
            }
            className="space-y-3"
          >
            <div className="flex items-center space-x-2">
              <RadioGroupItem value="stop" id="stop" />
              <label htmlFor="stop" className="text-sm cursor-pointer">
                <strong>Stop on Failure</strong> - Halt entire sequence when any
                task fails
              </label>
            </div>
            <div className="flex items-center space-x-2">
              <RadioGroupItem value="continue" id="continue" />
              <label htmlFor="continue" className="text-sm cursor-pointer">
                <strong>Continue on Failure</strong> - Skip failed tasks and
                continue with remaining tasks
              </label>
            </div>
            <div className="flex items-center space-x-2">
              <RadioGroupItem value="retry" id="retry" />
              <label htmlFor="retry" className="text-sm cursor-pointer">
                <strong>Retry on Failure</strong> - Automatically retry failed
                tasks before continuing
              </label>
            </div>
          </RadioGroup>

          {mode.failureHandling === "retry" && (
            <div className="ml-6">
              <Label htmlFor="retry-count">Max Retry Attempts</Label>
              <Input
                id="retry-count"
                type="number"
                min="1"
                max="5"
                value={mode.retryCount}
                onChange={(e) =>
                  updateMode({ retryCount: parseInt(e.target.value) || 1 })
                }
                className="mt-1 w-20"
              />
            </div>
          )}
        </div>

        {/* Summary */}
        <div className="p-4 rounded-lg bg-muted/30">
          <div className="flex items-center gap-2 mb-3">
            <CheckCircle className="h-4 w-4 text-ros-success" />
            <Label className="font-semibold">Configuration Summary</Label>
          </div>
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span>Execution Mode:</span>
              <Badge variant={mode.type === "parallel" ? "default" : "outline"}>
                {mode.type === "parallel"
                  ? `Parallel (${mode.maxConcurrency} max)`
                  : "Sequential"}
              </Badge>
            </div>
            <div className="flex justify-between">
              <span>Feedback Required:</span>
              <Badge variant={mode.waitForFeedback ? "destructive" : "outline"}>
                {mode.waitForFeedback ? "Yes" : "No"}
              </Badge>
            </div>
            {mode.waitForFeedback && (
              <div className="flex justify-between">
                <span>Feedback Timeout:</span>
                <span className="font-mono">{mode.globalFeedbackTimeout}s</span>
              </div>
            )}
            <div className="flex justify-between">
              <span>On Failure:</span>
              <Badge variant="outline">{mode.failureHandling}</Badge>
            </div>
          </div>
        </div>
      </div>
    </Card>
  );
}
