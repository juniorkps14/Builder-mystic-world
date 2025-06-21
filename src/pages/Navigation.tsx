import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Switch } from "@/components/ui/switch";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Map,
  Navigation as NavigationIcon,
  Target,
  Route,
  MapPin,
  Compass,
  Play,
  Pause,
  Square,
  RotateCcw,
  Zap,
  Settings,
  AlertTriangle,
  CheckCircle,
  Clock,
  Crosshair,
  Move,
} from "lucide-react";

interface NavigationGoal {
  id: string;
  name: string;
  x: number;
  y: number;
  yaw: number;
  status: "pending" | "active" | "succeeded" | "failed" | "cancelled";
  timestamp: Date;
}

interface PathPoint {
  x: number;
  y: number;
  yaw: number;
}

const Navigation = () => {
  const { t } = useLanguage();

  const [currentGoal, setCurrentGoal] = useState<NavigationGoal | null>(null);
  const [navigationStatus, setNavigationStatus] = useState<
    "idle" | "planning" | "executing" | "error"
  >("idle");
  const [currentPosition, setCurrentPosition] = useState({
    x: 0,
    y: 0,
    yaw: 0,
  });
  const [goalPosition, setGoalPosition] = useState({ x: 5, y: 3, yaw: 0 });
  const [plannerType, setPlannerType] = useState("global");
  const [autonomousMode, setAutonomousMode] = useState(false);
  const [costmapEnabled, setCostmapEnabled] = useState(true);

  const [recentGoals, setRecentGoals] = useState<NavigationGoal[]>([
    {
      id: "goal_1",
      name: "Charging Station",
      x: 10.5,
      y: 2.3,
      yaw: 1.57,
      status: "succeeded",
      timestamp: new Date(Date.now() - 300000),
    },
    {
      id: "goal_2",
      name: "Inspection Point A",
      x: 8.2,
      y: 6.1,
      yaw: 0,
      status: "succeeded",
      timestamp: new Date(Date.now() - 600000),
    },
    {
      id: "goal_3",
      name: "Loading Zone",
      x: 3.7,
      y: 4.5,
      yaw: -1.57,
      status: "failed",
      timestamp: new Date(Date.now() - 900000),
    },
  ]);

  const [plannerStats, setPlannerStats] = useState({
    totalGoals: 47,
    successRate: 94.2,
    avgPlanTime: 0.85,
    avgExecutionTime: 23.4,
  });

  // Simulate real-time updates
  useEffect(() => {
    const interval = setInterval(() => {
      // Simulate robot position updates
      setCurrentPosition((prev) => ({
        x: prev.x + (Math.random() - 0.5) * 0.02,
        y: prev.y + (Math.random() - 0.5) * 0.02,
        yaw: prev.yaw + (Math.random() - 0.5) * 0.05,
      }));
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const handleStartNavigation = () => {
    const newGoal: NavigationGoal = {
      id: `goal_${Date.now()}`,
      name: `Goal ${goalPosition.x.toFixed(1)}, ${goalPosition.y.toFixed(1)}`,
      x: goalPosition.x,
      y: goalPosition.y,
      yaw: goalPosition.yaw,
      status: "active",
      timestamp: new Date(),
    };

    setCurrentGoal(newGoal);
    setNavigationStatus("planning");

    // Simulate planning and execution
    setTimeout(() => {
      setNavigationStatus("executing");
    }, 2000);

    setTimeout(() => {
      setNavigationStatus("idle");
      setCurrentGoal(null);
      setRecentGoals((prev) => [newGoal, ...prev.slice(0, 4)]);
    }, 8000);
  };

  const handleCancelNavigation = () => {
    if (currentGoal) {
      setCurrentGoal({ ...currentGoal, status: "cancelled" });
      setNavigationStatus("idle");
      setTimeout(() => setCurrentGoal(null), 1000);
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "succeeded":
        return "text-green-500";
      case "failed":
        return "text-red-500";
      case "active":
        return "text-blue-500";
      case "cancelled":
        return "text-yellow-500";
      default:
        return "text-muted-foreground";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "succeeded":
        return CheckCircle;
      case "failed":
        return AlertTriangle;
      case "active":
        return Clock;
      case "cancelled":
        return Square;
      default:
        return Clock;
    }
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="space-y-2">
        <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
          <div className="p-2 rounded-lg bg-primary/10 float-animation">
            <NavigationIcon className="h-6 w-6 md:h-8 w-8 text-primary" />
          </div>
          {t("nav.navigation")}
        </h1>
        <p className="text-muted-foreground text-sm md:text-base">
          ROS Navigation Stack - Path Planning & Autonomous Navigation
        </p>
      </div>

      {/* Navigation Controls */}
      <Card className="p-4 md:p-6 glass-effect">
        <div className="flex flex-col lg:flex-row gap-6">
          {/* Goal Setting */}
          <div className="flex-1 space-y-4">
            <h3 className="text-lg font-light flex items-center gap-2">
              <Target className="h-5 w-5 text-primary" />
              Navigation Goal
            </h3>

            <div className="grid grid-cols-1 sm:grid-cols-3 gap-4">
              <div className="space-y-2">
                <Label className="text-sm">X Position (m)</Label>
                <Input
                  type="number"
                  step="0.1"
                  value={goalPosition.x}
                  onChange={(e) =>
                    setGoalPosition((prev) => ({
                      ...prev,
                      x: parseFloat(e.target.value) || 0,
                    }))
                  }
                  className="font-mono"
                />
              </div>
              <div className="space-y-2">
                <Label className="text-sm">Y Position (m)</Label>
                <Input
                  type="number"
                  step="0.1"
                  value={goalPosition.y}
                  onChange={(e) =>
                    setGoalPosition((prev) => ({
                      ...prev,
                      y: parseFloat(e.target.value) || 0,
                    }))
                  }
                  className="font-mono"
                />
              </div>
              <div className="space-y-2">
                <Label className="text-sm">Yaw Angle (rad)</Label>
                <Input
                  type="number"
                  step="0.1"
                  value={goalPosition.yaw}
                  onChange={(e) =>
                    setGoalPosition((prev) => ({
                      ...prev,
                      yaw: parseFloat(e.target.value) || 0,
                    }))
                  }
                  className="font-mono"
                />
              </div>
            </div>

            <div className="flex flex-col sm:flex-row gap-3">
              <Button
                onClick={handleStartNavigation}
                disabled={navigationStatus !== "idle"}
                className="flex-1 gap-2 hover-lift"
              >
                <Play className="h-4 w-4" />
                Start Navigation
              </Button>
              <Button
                variant="outline"
                onClick={handleCancelNavigation}
                disabled={navigationStatus === "idle"}
                className="flex-1 gap-2"
              >
                <Square className="h-4 w-4" />
                Cancel Goal
              </Button>
            </div>
          </div>

          {/* Status Panel */}
          <div className="lg:w-80 space-y-4">
            <h3 className="text-lg font-light flex items-center gap-2">
              <Compass className="h-5 w-5 text-primary" />
              Status
            </h3>

            <div className="space-y-3">
              <div className="flex justify-between items-center">
                <span className="text-sm text-muted-foreground">
                  Navigation State:
                </span>
                <Badge
                  variant={
                    navigationStatus === "idle" ? "secondary" : "default"
                  }
                >
                  {navigationStatus.toUpperCase()}
                </Badge>
              </div>

              <div className="space-y-2">
                <Label className="text-sm">Current Position</Label>
                <div className="grid grid-cols-3 gap-2 text-xs font-mono">
                  <div className="bg-muted p-2 rounded text-center">
                    <div className="text-muted-foreground">X</div>
                    <div>{currentPosition.x.toFixed(2)}</div>
                  </div>
                  <div className="bg-muted p-2 rounded text-center">
                    <div className="text-muted-foreground">Y</div>
                    <div>{currentPosition.y.toFixed(2)}</div>
                  </div>
                  <div className="bg-muted p-2 rounded text-center">
                    <div className="text-muted-foreground">θ</div>
                    <div>{currentPosition.yaw.toFixed(2)}</div>
                  </div>
                </div>
              </div>

              {currentGoal && (
                <div className="space-y-2">
                  <Label className="text-sm">Active Goal</Label>
                  <div className="bg-primary/5 p-3 rounded border border-primary/20">
                    <div className="font-mono text-sm">{currentGoal.name}</div>
                    <div className="text-xs text-muted-foreground mt-1">
                      Target: ({currentGoal.x}, {currentGoal.y},{" "}
                      {currentGoal.yaw.toFixed(2)})
                    </div>
                  </div>
                </div>
              )}
            </div>
          </div>
        </div>
      </Card>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Map Visualization */}
        <Card className="lg:col-span-2 p-4 md:p-6">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <Map className="h-5 w-5 text-primary" />
            Navigation Map
          </h3>

          <div className="relative bg-gradient-to-br from-slate-50 to-slate-100 rounded-lg border-2 border-dashed border-muted-foreground/20 aspect-[4/3] overflow-hidden">
            {/* Grid Background */}
            <div className="absolute inset-0 opacity-20">
              <svg className="w-full h-full">
                <defs>
                  <pattern
                    id="grid"
                    width="20"
                    height="20"
                    patternUnits="userSpaceOnUse"
                  >
                    <path
                      d="M 20 0 L 0 0 0 20"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="0.5"
                    />
                  </pattern>
                </defs>
                <rect width="100%" height="100%" fill="url(#grid)" />
              </svg>
            </div>

            {/* Robot Position */}
            <div
              className="absolute w-4 h-4 bg-blue-500 rounded-full border-2 border-white shadow-lg transform -translate-x-2 -translate-y-2 glow-blue"
              style={{
                left: `${(currentPosition.x + 10) * 3}%`,
                top: `${(10 - currentPosition.y) * 3}%`,
              }}
            >
              <div className="absolute inset-0 bg-blue-400 rounded-full animate-ping opacity-75" />
            </div>

            {/* Goal Position */}
            <div
              className="absolute w-4 h-4 bg-green-500 rounded-full border-2 border-white shadow-lg transform -translate-x-2 -translate-y-2"
              style={{
                left: `${(goalPosition.x + 10) * 3}%`,
                top: `${(10 - goalPosition.y) * 3}%`,
              }}
            >
              <Target className="w-3 h-3 text-white absolute top-0.5 left-0.5" />
            </div>

            {/* Sample Obstacles */}
            <div
              className="absolute w-8 h-8 bg-red-300 rounded opacity-60"
              style={{ left: "30%", top: "40%" }}
            />
            <div
              className="absolute w-6 h-12 bg-red-300 rounded opacity-60"
              style={{ left: "60%", top: "20%" }}
            />
            <div
              className="absolute w-12 h-6 bg-red-300 rounded opacity-60"
              style={{ left: "20%", top: "70%" }}
            />

            {/* Path Line */}
            {navigationStatus === "executing" && (
              <svg className="absolute inset-0 w-full h-full pointer-events-none">
                <line
                  x1={`${(currentPosition.x + 10) * 3}%`}
                  y1={`${(10 - currentPosition.y) * 3}%`}
                  x2={`${(goalPosition.x + 10) * 3}%`}
                  y2={`${(10 - goalPosition.y) * 3}%`}
                  stroke="rgb(0, 172, 230)"
                  strokeWidth="2"
                  strokeDasharray="5,5"
                  className="animate-pulse"
                />
              </svg>
            )}

            {/* Legend */}
            <div className="absolute bottom-2 left-2 bg-white/90 backdrop-blur-sm p-2 rounded text-xs space-y-1">
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 bg-blue-500 rounded-full" />
                <span>Robot</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 bg-green-500 rounded-full" />
                <span>Goal</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 bg-red-300 rounded" />
                <span>Obstacles</span>
              </div>
            </div>
          </div>
        </Card>

        {/* Side Panels */}
        <div className="space-y-6">
          {/* Navigation Settings */}
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Settings className="h-5 w-5 text-primary" />
              Settings
            </h3>

            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <Label className="text-sm">Autonomous Mode</Label>
                <Switch
                  checked={autonomousMode}
                  onCheckedChange={setAutonomousMode}
                />
              </div>

              <div className="flex items-center justify-between">
                <Label className="text-sm">Costmap Enabled</Label>
                <Switch
                  checked={costmapEnabled}
                  onCheckedChange={setCostmapEnabled}
                />
              </div>

              <div className="space-y-2">
                <Label className="text-sm">Planner Type</Label>
                <div className="grid grid-cols-2 gap-2">
                  <Button
                    variant={plannerType === "global" ? "default" : "outline"}
                    size="sm"
                    onClick={() => setPlannerType("global")}
                  >
                    Global
                  </Button>
                  <Button
                    variant={plannerType === "local" ? "default" : "outline"}
                    size="sm"
                    onClick={() => setPlannerType("local")}
                  >
                    Local
                  </Button>
                </div>
              </div>
            </div>
          </Card>

          {/* Statistics */}
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Route className="h-5 w-5 text-primary" />
              Statistics
            </h3>

            <div className="space-y-3">
              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">
                  Total Goals:
                </span>
                <Badge variant="outline">{plannerStats.totalGoals}</Badge>
              </div>

              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-muted-foreground">Success Rate:</span>
                  <span className="font-mono">{plannerStats.successRate}%</span>
                </div>
                <Progress value={plannerStats.successRate} className="h-2" />
              </div>

              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">
                  Avg Plan Time:
                </span>
                <span className="text-sm font-mono">
                  {plannerStats.avgPlanTime}s
                </span>
              </div>

              <div className="flex justify-between">
                <span className="text-sm text-muted-foreground">
                  Avg Exec Time:
                </span>
                <span className="text-sm font-mono">
                  {plannerStats.avgExecutionTime}s
                </span>
              </div>
            </div>
          </Card>
        </div>
      </div>

      {/* Recent Goals */}
      <Card className="p-4 md:p-6">
        <h3 className="text-lg font-light mb-4 flex items-center gap-2">
          <MapPin className="h-5 w-5 text-primary" />
          Recent Navigation Goals
        </h3>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {recentGoals.map((goal, index) => {
            const StatusIcon = getStatusIcon(goal.status);

            return (
              <div
                key={goal.id}
                className={`p-4 rounded-lg border bg-gradient-to-br from-background to-muted/20 hover-lift stagger-item`}
                style={{ animationDelay: `${index * 0.1}s` }}
              >
                <div className="flex items-start justify-between mb-2">
                  <h4 className="font-medium text-sm">{goal.name}</h4>
                  <StatusIcon
                    className={`h-4 w-4 ${getStatusColor(goal.status)}`}
                  />
                </div>

                <div className="space-y-1 text-xs text-muted-foreground">
                  <div className="flex justify-between">
                    <span>Position:</span>
                    <span className="font-mono">
                      ({goal.x}, {goal.y})
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span>Orientation:</span>
                    <span className="font-mono">{goal.yaw.toFixed(2)} rad</span>
                  </div>
                  <div className="flex justify-between">
                    <span>Time:</span>
                    <span>{goal.timestamp.toLocaleTimeString()}</span>
                  </div>
                </div>

                <div className="mt-3 flex gap-2">
                  <Button
                    size="sm"
                    variant="outline"
                    className="flex-1 text-xs"
                    onClick={() =>
                      setGoalPosition({ x: goal.x, y: goal.y, yaw: goal.yaw })
                    }
                  >
                    <Crosshair className="h-3 w-3 mr-1" />
                    Set as Goal
                  </Button>
                </div>
              </div>
            );
          })}
        </div>
      </Card>
    </div>
  );
};

export default Navigation;
