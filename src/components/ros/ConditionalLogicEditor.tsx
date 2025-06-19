import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Plus,
  Trash2,
  Code,
  GitBranch,
  ArrowRight,
  CheckCircle,
  XCircle,
  AlertTriangle,
  RotateCw,
} from "lucide-react";

interface Condition {
  id: string;
  type: "if" | "else_if" | "else" | "switch" | "case";
  condition: string;
  operator: "==" | "!=" | ">" | "<" | ">=" | "<=" | "contains" | "exists";
  value: string;
  action: string;
  actionParams?: Record<string, any>;
}

interface ConditionalLogicEditorProps {
  conditions: Condition[];
  onChange: (conditions: Condition[]) => void;
  availableVariables: string[];
  availableActions: string[];
}

export function ConditionalLogicEditor({
  conditions,
  onChange,
  availableVariables,
  availableActions,
}: ConditionalLogicEditorProps) {
  const [selectedCondition, setSelectedCondition] = useState<string | null>(
    null,
  );

  const addCondition = (type: Condition["type"]) => {
    const newCondition: Condition = {
      id: `condition_${Date.now()}`,
      type,
      condition: "",
      operator: "==",
      value: "",
      action: "continue",
    };

    onChange([...conditions, newCondition]);
  };

  const updateCondition = (id: string, updates: Partial<Condition>) => {
    onChange(
      conditions.map((cond) =>
        cond.id === id ? { ...cond, ...updates } : cond,
      ),
    );
  };

  const removeCondition = (id: string) => {
    onChange(conditions.filter((cond) => cond.id !== id));
  };

  const getConditionIcon = (type: Condition["type"]) => {
    switch (type) {
      case "if":
      case "else_if":
        return GitBranch;
      case "else":
        return ArrowRight;
      case "switch":
        return Code;
      case "case":
        return CheckCircle;
      default:
        return GitBranch;
    }
  };

  const getActionIcon = (action: string) => {
    switch (action) {
      case "continue":
        return CheckCircle;
      case "retry":
        return RotateCw;
      case "abort":
        return XCircle;
      case "wait":
        return AlertTriangle;
      default:
        return ArrowRight;
    }
  };

  return (
    <Card className="p-4">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-2">
          <Code className="h-5 w-5 text-primary" />
          <h3 className="font-semibold">Conditional Logic</h3>
        </div>
        <div className="flex gap-2">
          <Button
            size="sm"
            variant="outline"
            onClick={() => addCondition("if")}
            className="gap-1"
          >
            <Plus className="h-3 w-3" />
            If
          </Button>
          <Button
            size="sm"
            variant="outline"
            onClick={() => addCondition("switch")}
            className="gap-1"
          >
            <Plus className="h-3 w-3" />
            Switch
          </Button>
        </div>
      </div>

      {conditions.length === 0 ? (
        <div className="text-center py-8 text-muted-foreground">
          <Code className="h-12 w-12 mx-auto mb-3 opacity-50" />
          <p className="text-sm">No conditions defined</p>
          <p className="text-xs">
            Add IF or SWITCH statements to control task flow
          </p>
        </div>
      ) : (
        <div className="space-y-3">
          {conditions.map((condition, index) => {
            const Icon = getConditionIcon(condition.type);
            const ActionIcon = getActionIcon(condition.action);

            return (
              <Card
                key={condition.id}
                className={`p-3 transition-all cursor-pointer ${
                  selectedCondition === condition.id
                    ? "ring-2 ring-primary"
                    : "hover:bg-accent"
                }`}
                onClick={() => setSelectedCondition(condition.id)}
              >
                <div className="flex items-center gap-3">
                  <div className="flex items-center gap-2 min-w-0 flex-1">
                    <Icon className="h-4 w-4 text-primary flex-shrink-0" />
                    <Badge
                      variant={
                        condition.type === "else" ? "secondary" : "outline"
                      }
                      className="text-xs flex-shrink-0"
                    >
                      {condition.type.toUpperCase()}
                    </Badge>

                    {condition.type !== "else" && (
                      <>
                        <Input
                          value={condition.condition}
                          onChange={(e) =>
                            updateCondition(condition.id, {
                              condition: e.target.value,
                            })
                          }
                          placeholder="Variable or expression"
                          className="h-7 text-xs flex-1 min-w-0"
                          onClick={(e) => e.stopPropagation()}
                        />

                        <Select
                          value={condition.operator}
                          onValueChange={(value) =>
                            updateCondition(condition.id, {
                              operator: value as Condition["operator"],
                            })
                          }
                        >
                          <SelectTrigger className="h-7 w-16 text-xs">
                            <SelectValue />
                          </SelectTrigger>
                          <SelectContent>
                            <SelectItem value="==">=</SelectItem>
                            <SelectItem value="!=">!=</SelectItem>
                            <SelectItem value=">">&gt;</SelectItem>
                            <SelectItem value="<">&lt;</SelectItem>
                            <SelectItem value=">=">&gt;=</SelectItem>
                            <SelectItem value="<=">&lt;=</SelectItem>
                            <SelectItem value="contains">contains</SelectItem>
                            <SelectItem value="exists">exists</SelectItem>
                          </SelectContent>
                        </Select>

                        <Input
                          value={condition.value}
                          onChange={(e) =>
                            updateCondition(condition.id, {
                              value: e.target.value,
                            })
                          }
                          placeholder="Value"
                          className="h-7 text-xs w-24"
                          onClick={(e) => e.stopPropagation()}
                        />
                      </>
                    )}

                    <ArrowRight className="h-3 w-3 text-muted-foreground flex-shrink-0" />

                    <div className="flex items-center gap-1">
                      <ActionIcon className="h-3 w-3 text-accent flex-shrink-0" />
                      <Select
                        value={condition.action}
                        onValueChange={(value) =>
                          updateCondition(condition.id, { action: value })
                        }
                      >
                        <SelectTrigger className="h-7 w-24 text-xs">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          {availableActions.map((action) => (
                            <SelectItem key={action} value={action}>
                              {action}
                            </SelectItem>
                          ))}
                        </SelectContent>
                      </Select>
                    </div>
                  </div>

                  <div className="flex items-center gap-1">
                    {index > 0 &&
                      conditions[index - 1].type === "if" &&
                      condition.type === "if" && (
                        <Button
                          size="sm"
                          variant="ghost"
                          onClick={(e) => {
                            e.stopPropagation();
                            updateCondition(condition.id, { type: "else_if" });
                          }}
                          className="h-6 px-2 text-xs"
                        >
                          →ELIF
                        </Button>
                      )}

                    {condition.type === "if" && (
                      <Button
                        size="sm"
                        variant="ghost"
                        onClick={(e) => {
                          e.stopPropagation();
                          addCondition("else");
                        }}
                        className="h-6 px-2 text-xs"
                      >
                        +ELSE
                      </Button>
                    )}

                    <Button
                      size="sm"
                      variant="ghost"
                      onClick={(e) => {
                        e.stopPropagation();
                        removeCondition(condition.id);
                      }}
                      className="h-6 w-6 p-0"
                    >
                      <Trash2 className="h-3 w-3" />
                    </Button>
                  </div>
                </div>

                {/* Expanded view */}
                {selectedCondition === condition.id && (
                  <div className="mt-3 pt-3 border-t space-y-3">
                    <div className="grid grid-cols-2 gap-3">
                      <div>
                        <Label className="text-xs">Available Variables</Label>
                        <div className="flex flex-wrap gap-1 mt-1">
                          {availableVariables.map((variable) => (
                            <Badge
                              key={variable}
                              variant="outline"
                              className="text-xs cursor-pointer hover:bg-accent"
                              onClick={() =>
                                updateCondition(condition.id, {
                                  condition: variable,
                                })
                              }
                            >
                              {variable}
                            </Badge>
                          ))}
                        </div>
                      </div>

                      <div>
                        <Label className="text-xs">Quick Values</Label>
                        <div className="flex flex-wrap gap-1 mt-1">
                          {["true", "false", "0", "1", "null", "empty"].map(
                            (value) => (
                              <Badge
                                key={value}
                                variant="outline"
                                className="text-xs cursor-pointer hover:bg-accent"
                                onClick={() =>
                                  updateCondition(condition.id, { value })
                                }
                              >
                                {value}
                              </Badge>
                            ),
                          )}
                        </div>
                      </div>
                    </div>

                    <div>
                      <Label className="text-xs">Condition Preview</Label>
                      <div className="mt-1 p-2 bg-muted rounded text-xs font-mono">
                        {condition.type === "else"
                          ? "ELSE"
                          : `${condition.type.toUpperCase()} (${condition.condition} ${condition.operator} ${condition.value})`}{" "}
                        → {condition.action.toUpperCase()}
                      </div>
                    </div>
                  </div>
                )}
              </Card>
            );
          })}
        </div>
      )}

      {conditions.length > 0 && (
        <div className="mt-4 p-3 bg-muted/50 rounded">
          <Label className="text-xs font-semibold">Logic Flow Summary</Label>
          <div className="mt-2 text-xs font-mono space-y-1">
            {conditions.map((condition, index) => (
              <div key={condition.id} className="flex items-center gap-2">
                <span className="text-muted-foreground">{index + 1}.</span>
                <span>
                  {condition.type === "else"
                    ? "ELSE"
                    : `${condition.type.toUpperCase()}`}
                </span>
                {condition.type !== "else" && (
                  <span className="text-primary">
                    ({condition.condition} {condition.operator}{" "}
                    {condition.value})
                  </span>
                )}
                <ArrowRight className="h-3 w-3" />
                <span className="text-accent">{condition.action}</span>
              </div>
            ))}
          </div>
        </div>
      )}
    </Card>
  );
}
