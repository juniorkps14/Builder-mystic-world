import React, { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Navigation as NavigationIcon,
  Target,
  MapPin,
  Play,
  Pause,
  Square,
  RotateCcw,
  Compass,
  Route,
  Flag,
  AlertTriangle,
  CheckCircle,
  Clock,
  TrendingUp,
  Settings,
  Zap,
  Activity,
  Crosshair,
} from "lucide-react";

interface Goal {
  id: string;
  name: string;
  x: number;
  y: number;
  theta: number;
  status: "pending" | "active" | "completed" | "failed";
  timestamp: Date;
}

interface NavigationStats {
  totalGoals: number;
  completedGoals: number;
  failedGoals: number;
  averageTime: number;
  successRate: number;
}

export default function Navigation() {
  const [currentPosition, setCurrentPosition] = useState({
    x: 2.45,
    y: 1.23,
    theta: 0.785, // 45 degrees in radians
  });

  const [goalPosition, setGoalPosition] = useState({
    x: 5.0,
    y: 3.0,
    theta: 1.57, // 90 degrees in radians
  });

  const [navigationStatus, setNavigationStatus] = useState<"idle" | "navigating" | "completed" | "failed">("idle");
  const [progress, setProgress] = useState(0);
  const [estimatedTime, setEstimatedTime] = useState(120);

  const [recentGoals, setRecentGoals] = useState<Goal[]>([
    {
      id: "goal_1",
      name: "Checkpoint Alpha",
      x: 3.2,
      y: 1.8,
      theta: 0,
      status: "completed",
      timestamp: new Date(Date.now() - 300000),
    },
    {
      id: "goal_2", 
      name: "Loading Zone",
      x: 6.5,
      y: 4.2,
      theta: 1.57,
      status: "completed",
      timestamp: new Date(Date.now() - 600000),
    },
    {
      id: "goal_3",
      name: "Inspection Point",
      x: 1.0,
      y: 5.0,
      theta: 3.14,
      status: "failed",
      timestamp: new Date(Date.now() - 900000),
    },
  ]);

  const [stats, setStats] = useState<NavigationStats>({
    totalGoals: 45,
    completedGoals: 41,
    failedGoals: 4,
    averageTime: 132,
    successRate: 91.1,
  });

  // Simulate navigation progress
  useEffect(() => {
    if (navigationStatus === "navigating") {
      const interval = setInterval(() => {
        setProgress(prev => {
          if (prev >= 100) {
            setNavigationStatus("completed");
            return 100;
          }
          setEstimatedTime(prev => Math.max(0, prev - 2));
          return prev + 2;
        });
      }, 1000);
      return () => clearInterval(interval);
    }
  }, [navigationStatus]);

  const startNavigation = () => {
    setNavigationStatus("navigating");
    setProgress(0);
    setEstimatedTime(120);
  };

  const cancelNavigation = () => {
    setNavigationStatus("idle");
    setProgress(0);
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "navigating":
        return "from-blue-500 to-cyan-500";
      case "completed":
        return "from-emerald-500 to-teal-500";
      case "failed":
        return "from-red-500 to-pink-500";
      default:
        return "from-gray-500 to-slate-500";
    }
  };

  const getGoalStatusColor = (status: string) => {
    switch (status) {
      case "completed":
        return "bg-emerald-500/20 text-emerald-300 border-emerald-500/30";
      case "failed":
        return "bg-red-500/20 text-red-300 border-red-500/30";
      case "active":
        return "bg-blue-500/20 text-blue-300 border-blue-500/30";
      default:
        return "bg-gray-500/20 text-gray-300 border-gray-500/30";
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Tesla-inspired Header */}
      <div className="mb-8">
        <div className="bg-white/10 backdrop-blur-xl border border-white/20 rounded-3xl p-6 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="space-y-2">
              <h1 className="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
                Navigation System
              </h1>
              <p className="text-slate-300 font-light">
                Autonomous navigation control and path planning
              </p>
            </div>
            
            <div className="flex items-center gap-3">
              <Badge className={`bg-gradient-to-r ${getStatusColor(navigationStatus)} text-white border-0 px-4 py-2`}>
                {navigationStatus.charAt(0).toUpperCase() + navigationStatus.slice(1)}
              </Badge>
            </div>
          </div>
        </div>
      </div>

      {/* Navigation Stats */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Total Goals</p>
              <p className="text-2xl font-light text-white mt-1">{stats.totalGoals}</p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center">
              <Target className="h-6 w-6 text-blue-400" />
            </div>
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Success Rate</p>
              <p className="text-2xl font-light text-white mt-1">{stats.successRate}%</p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-emerald-500/20 to-teal-500/20 rounded-xl flex items-center justify-center">
              <TrendingUp className="h-6 w-6 text-emerald-400" />
            </div>
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Avg Time</p>
              <p className="text-2xl font-light text-white mt-1">{stats.averageTime}s</p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center">
              <Clock className="h-6 w-6 text-purple-400" />
            </div>
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Active Goals</p>
              <p className="text-2xl font-light text-white mt-1">
                {navigationStatus === "navigating" ? 1 : 0}
              </p>
            </div>
            <div className="h-12 w-12 bg-gradient-to-br from-orange-500/20 to-red-500/20 rounded-xl flex items-center justify-center">
              <Activity className="h-6 w-6 text-orange-400" />
            </div>
          </div>
        </Card>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Navigation Control */}
        <div className="lg:col-span-2 space-y-6">
          {/* Current Navigation */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-2xl">
            <div className="p-6">
              <h2 className="text-xl font-light text-white mb-6">Navigation Control</h2>
              
              {navigationStatus === "navigating" && (
                <div className="mb-6 p-4 bg-blue-500/20 border border-blue-500/30 rounded-xl">
                  <div className="flex items-center justify-between mb-3">
                    <span className="text-blue-300 font-medium">Navigation in Progress</span>
                    <span className="text-blue-300 text-sm">{progress.toFixed(1)}%</span>
                  </div>
                  <Progress value={progress} className="h-3 bg-slate-700 mb-3" />
                  <div className="flex justify-between text-sm text-blue-300">
                    <span>ETA: {Math.floor(estimatedTime / 60)}:{(estimatedTime % 60).toString().padStart(2, '0')}</span>
                    <span>Distance remaining: {((100 - progress) * 0.1).toFixed(1)}m</span>
                  </div>
                </div>
              )}

              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                {/* Current Position */}
                <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                  <h3 className="text-lg font-medium text-white mb-4 flex items-center gap-2">
                    <MapPin className="h-5 w-5 text-green-400" />
                    Current Position
                  </h3>
                  <div className="space-y-3">
                    <div>
                      <Label className="text-slate-400 text-sm">X Position</Label>
                      <p className="text-white font-mono text-lg">{currentPosition.x.toFixed(2)}m</p>
                    </div>
                    <div>
                      <Label className="text-slate-400 text-sm">Y Position</Label>
                      <p className="text-white font-mono text-lg">{currentPosition.y.toFixed(2)}m</p>
                    </div>
                    <div>
                      <Label className="text-slate-400 text-sm">Orientation</Label>
                      <p className="text-white font-mono text-lg">{(currentPosition.theta * 180 / Math.PI).toFixed(1)}°</p>
                    </div>
                  </div>
                </div>

                {/* Goal Position */}
                <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                  <h3 className="text-lg font-medium text-white mb-4 flex items-center gap-2">
                    <Crosshair className="h-5 w-5 text-blue-400" />
                    Goal Position
                  </h3>
                  <div className="space-y-3">
                    <div>
                      <Label className="text-slate-400 text-sm">X Goal</Label>
                      <Input
                        type="number"
                        value={goalPosition.x}
                        onChange={(e) => setGoalPosition(prev => ({ ...prev, x: parseFloat(e.target.value) || 0 }))}
                        className="bg-white/10 border-white/20 text-white"
                        step="0.1"
                      />
                    </div>
                    <div>
                      <Label className="text-slate-400 text-sm">Y Goal</Label>
                      <Input
                        type="number"
                        value={goalPosition.y}
                        onChange={(e) => setGoalPosition(prev => ({ ...prev, y: parseFloat(e.target.value) || 0 }))}
                        className="bg-white/10 border-white/20 text-white"
                        step="0.1"
                      />
                    </div>
                    <div>
                      <Label className="text-slate-400 text-sm">Orientation (°)</Label>
                      <Input
                        type="number"
                        value={(goalPosition.theta * 180 / Math.PI).toFixed(1)}
                        onChange={(e) => setGoalPosition(prev => ({ ...prev, theta: (parseFloat(e.target.value) || 0) * Math.PI / 180 }))}
                        className="bg-white/10 border-white/20 text-white"
                        step="1"
                      />
                    </div>
                  </div>
                </div>
              </div>

              {/* Control Buttons */}
              <div className="flex gap-4 mt-6">
                {navigationStatus === "idle" ? (
                  <Button
                    onClick={startNavigation}
                    className="flex-1 bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600 text-white shadow-lg"
                  >
                    <Play className="h-4 w-4 mr-2" />
                    Start Navigation
                  </Button>
                ) : navigationStatus === "navigating" ? (
                  <Button
                    onClick={cancelNavigation}
                    className="flex-1 bg-gradient-to-r from-red-500 to-pink-500 hover:from-red-600 hover:to-pink-600 text-white shadow-lg"
                  >
                    <Square className="h-4 w-4 mr-2" />
                    Cancel Navigation
                  </Button>
                ) : (
                  <Button
                    onClick={() => setNavigationStatus("idle")}
                    className="flex-1 bg-white/10 hover:bg-white/20 border border-white/20 text-white"
                  >
                    <RotateCcw className="h-4 w-4 mr-2" />
                    Reset
                  </Button>
                )}
                
                <Button className="bg-white/10 hover:bg-white/20 border border-white/20 text-white">
                  <Settings className="h-4 w-4 mr-2" />
                  Settings
                </Button>
              </div>
            </div>
          </Card>

          {/* Map Visualization Placeholder */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-2xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">Navigation Map</h3>
              <div className="aspect-video bg-gradient-to-br from-slate-800 to-slate-900 rounded-xl border border-white/10 flex items-center justify-center">
                <div className="text-center">
                  <Compass className="h-16 w-16 text-blue-400 mx-auto mb-4" />
                  <p className="text-slate-300">Interactive Navigation Map</p>
                  <p className="text-slate-400 text-sm mt-2">Real-time position and path visualization</p>
                </div>
              </div>
            </div>
          </Card>
        </div>

        {/* Recent Goals & Status */}
        <div className="space-y-6">
          {/* Recent Goals */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">Recent Goals</h3>
              <div className="space-y-3">
                {recentGoals.map((goal) => (
                  <div key={goal.id} className="bg-white/5 rounded-lg p-3 border border-white/10">
                    <div className="flex items-center justify-between mb-2">
                      <span className="font-medium text-white text-sm">{goal.name}</span>
                      <Badge className={getGoalStatusColor(goal.status)}>
                        {goal.status}
                      </Badge>
                    </div>
                    <div className="text-xs text-slate-400 space-y-1">
                      <p>Position: ({goal.x.toFixed(1)}, {goal.y.toFixed(1)})</p>
                      <p>Time: {goal.timestamp.toLocaleTimeString()}</p>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </Card>

          {/* Quick Actions */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">Quick Actions</h3>
              <div className="space-y-3">
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <Flag className="h-4 w-4 mr-2" />
                  Save Current Position
                </Button>
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <Route className="h-4 w-4 mr-2" />
                  Plan Path
                </Button>
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <RotateCcw className="h-4 w-4 mr-2" />
                  Return Home
                </Button>
              </div>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
}
