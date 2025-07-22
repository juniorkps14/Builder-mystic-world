import React, { useState } from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import { Slider } from "@/components/ui/slider";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Bot,
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  RotateCcw,
  RotateCw,
  Square,
  Play,
  Pause,
  Home,
  MapPin,
  Zap,
  Shield,
  Activity,
  Target,
  Navigation,
  Compass,
  Save,
  SkipForward,
  SkipBack,
  MoveUpRight,
  MoveUpLeft,
  MoveDownRight,
  MoveDownLeft,
} from "lucide-react";
import "../styles/flat-vector-theme.css";

export default function HolonomicRobotControl() {
  const [robotStatus, setRobotStatus] = useState<"idle" | "moving" | "stopped">(
    "idle",
  );
  const [linearSpeed, setLinearSpeed] = useState([50]);
  const [angularSpeed, setAngularSpeed] = useState([30]);
  const [strafeSpeed, setStrafeSpeed] = useState([40]);
  const [position, setPosition] = useState({ x: 0, y: 0, rotation: 0 });
  const [velocity, setVelocity] = useState({
    linear_x: 0,
    linear_y: 0,
    angular_z: 0,
  });

  // Holonomic movement controls
  const holonomicButtons = [
    // Diagonal movements
    {
      icon: MoveUpLeft,
      label: "↖",
      action: "move_diagonal_up_left",
      position: "col-start-1 row-start-1",
      color: "bg-purple-500",
    },
    {
      icon: ArrowUp,
      label: "↑",
      action: "move_forward",
      position: "col-start-2 row-start-1",
      color: "bg-blue-500",
    },
    {
      icon: MoveUpRight,
      label: "↗",
      action: "move_diagonal_up_right",
      position: "col-start-3 row-start-1",
      color: "bg-purple-500",
    },
    // Lateral movements
    {
      icon: ArrowLeft,
      label: "←",
      action: "strafe_left",
      position: "col-start-1 row-start-2",
      color: "bg-green-500",
    },
    {
      icon: Square,
      label: "STOP",
      action: "stop",
      position: "col-start-2 row-start-2",
      color: "bg-red-500",
    },
    {
      icon: ArrowRight,
      label: "→",
      action: "strafe_right",
      position: "col-start-3 row-start-2",
      color: "bg-green-500",
    },
    // Diagonal movements
    {
      icon: MoveDownLeft,
      label: "↙",
      action: "move_diagonal_down_left",
      position: "col-start-1 row-start-3",
      color: "bg-purple-500",
    },
    {
      icon: ArrowDown,
      label: "↓",
      action: "move_backward",
      position: "col-start-2 row-start-3",
      color: "bg-blue-500",
    },
    {
      icon: MoveDownRight,
      label: "↘",
      action: "move_diagonal_down_right",
      position: "col-start-3 row-start-3",
      color: "bg-purple-500",
    },
    // Rotation
    {
      icon: RotateCcw,
      label: "⟲",
      action: "rotate_left",
      position: "col-start-1 row-start-4",
      color: "bg-orange-500",
    },
    {
      icon: RotateCw,
      label: "⟳",
      action: "rotate_right",
      position: "col-start-3 row-start-4",
      color: "bg-orange-500",
    },
  ];

  const quickActions = [
    {
      title: "Go Home",
      icon: Home,
      color: "bg-blue-500",
      description: "Return to base position (0,0,0)",
    },
    {
      title: "Emergency Stop",
      icon: Shield,
      color: "bg-red-500",
      description: "Stop all movement immediately",
    },
    {
      title: "Hold Position",
      icon: Target,
      color: "bg-yellow-500",
      description: "Lock current position",
    },
    {
      title: "Free Movement",
      icon: Navigation,
      color: "bg-green-500",
      description: "Enable free movement mode",
    },
  ];

  const handleHolonomicMovement = (action: string) => {
    setRobotStatus("moving");

    const linear_speed = linearSpeed[0] / 100;
    const strafe_speed = strafeSpeed[0] / 100;
    const angular_speed = angularSpeed[0] / 100;

    let newVelocity = { linear_x: 0, linear_y: 0, angular_z: 0 };

    switch (action) {
      case "move_forward":
        newVelocity = { linear_x: linear_speed, linear_y: 0, angular_z: 0 };
        setPosition((prev) => ({ ...prev, y: prev.y + 0.1 }));
        break;
      case "move_backward":
        newVelocity = { linear_x: -linear_speed, linear_y: 0, angular_z: 0 };
        setPosition((prev) => ({ ...prev, y: prev.y - 0.1 }));
        break;
      case "strafe_left":
        newVelocity = { linear_x: 0, linear_y: strafe_speed, angular_z: 0 };
        setPosition((prev) => ({ ...prev, x: prev.x - 0.1 }));
        break;
      case "strafe_right":
        newVelocity = { linear_x: 0, linear_y: -strafe_speed, angular_z: 0 };
        setPosition((prev) => ({ ...prev, x: prev.x + 0.1 }));
        break;
      case "move_diagonal_up_left":
        newVelocity = {
          linear_x: linear_speed * 0.7,
          linear_y: strafe_speed * 0.7,
          angular_z: 0,
        };
        setPosition((prev) => ({
          ...prev,
          x: prev.x - 0.07,
          y: prev.y + 0.07,
        }));
        break;
      case "move_diagonal_up_right":
        newVelocity = {
          linear_x: linear_speed * 0.7,
          linear_y: -strafe_speed * 0.7,
          angular_z: 0,
        };
        setPosition((prev) => ({
          ...prev,
          x: prev.x + 0.07,
          y: prev.y + 0.07,
        }));
        break;
      case "move_diagonal_down_left":
        newVelocity = {
          linear_x: -linear_speed * 0.7,
          linear_y: strafe_speed * 0.7,
          angular_z: 0,
        };
        setPosition((prev) => ({
          ...prev,
          x: prev.x - 0.07,
          y: prev.y - 0.07,
        }));
        break;
      case "move_diagonal_down_right":
        newVelocity = {
          linear_x: -linear_speed * 0.7,
          linear_y: -strafe_speed * 0.7,
          angular_z: 0,
        };
        setPosition((prev) => ({
          ...prev,
          x: prev.x + 0.07,
          y: prev.y - 0.07,
        }));
        break;
      case "rotate_left":
        newVelocity = { linear_x: 0, linear_y: 0, angular_z: angular_speed };
        setPosition((prev) => ({
          ...prev,
          rotation: (prev.rotation + 15) % 360,
        }));
        break;
      case "rotate_right":
        newVelocity = { linear_x: 0, linear_y: 0, angular_z: -angular_speed };
        setPosition((prev) => ({
          ...prev,
          rotation: (prev.rotation - 15) % 360,
        }));
        break;
      case "stop":
        newVelocity = { linear_x: 0, linear_y: 0, angular_z: 0 };
        setRobotStatus("stopped");
        break;
    }

    setVelocity(newVelocity);
    console.log(`Holonomic movement: ${action}`, newVelocity);

    // Auto return to idle after movement
    if (action !== "stop") {
      setTimeout(() => setRobotStatus("idle"), 1000);
    }
  };

  const handleApply = () => {
    console.log("Applying holonomic control settings...");
    // TODO: Save settings to JSON and apply
  };

  const getStatusColor = () => {
    switch (robotStatus) {
      case "moving":
        return "bg-blue-100 text-blue-700 border-blue-200";
      case "stopped":
        return "bg-red-100 text-red-700 border-red-200";
      default:
        return "bg-gray-100 text-gray-700 border-gray-200";
    }
  };

  const getStatusIcon = () => {
    switch (robotStatus) {
      case "moving":
        return <Activity className="w-5 h-5 animate-pulse" />;
      case "stopped":
        return <Square className="w-5 h-5" />;
      default:
        return <Pause className="w-5 h-5" />;
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="flat-title">Holonomic Robot Control 🚀</h1>
          <p className="flat-subtitle">
            Advanced omni-directional robot movement control
          </p>
        </div>
        <Button onClick={handleApply} className="gap-2">
          <Save className="h-4 w-4" />
          Apply Settings
        </Button>
      </div>

      {/* Status Section */}
      <div className="flat-grid flat-grid-4 mb-8">
        {/* Robot Status */}
        <Card className="flat-card">
          <div className="flex items-center gap-3 mb-4">
            <div className="w-12 h-12 bg-gradient-to-br from-blue-500 to-purple-600 rounded-xl flex items-center justify-center">
              <Bot className="w-6 h-6 text-white" />
            </div>
            <div>
              <h3 className="font-semibold text-gray-900">Robot Status</h3>
              <p className="text-sm text-gray-500">Holonomic drive</p>
            </div>
          </div>
          <div
            className={`flex items-center gap-2 px-4 py-3 rounded-lg border-2 ${getStatusColor()}`}
          >
            {getStatusIcon()}
            <span className="font-medium capitalize">{robotStatus}</span>
          </div>
        </Card>

        {/* Position */}
        <Card className="flat-card">
          <div className="flex items-center gap-3 mb-4">
            <div className="w-12 h-12 bg-gradient-to-br from-green-500 to-teal-600 rounded-xl flex items-center justify-center">
              <MapPin className="w-6 h-6 text-white" />
            </div>
            <div>
              <h3 className="font-semibold text-gray-900">Position</h3>
              <p className="text-sm text-gray-500">X, Y coordinates</p>
            </div>
          </div>
          <div className="space-y-2">
            <div className="flex justify-between">
              <span className="text-sm text-slate-300">X:</span>
              <span className="text-sm font-medium">
                {position.x.toFixed(2)}m
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-sm text-slate-300">Y:</span>
              <span className="text-sm font-medium">
                {position.y.toFixed(2)}m
              </span>
            </div>
          </div>
        </Card>

        {/* Velocity */}
        <Card className="flat-card">
          <div className="flex items-center gap-3 mb-4">
            <div className="w-12 h-12 bg-gradient-to-br from-orange-500 to-red-600 rounded-xl flex items-center justify-center">
              <Zap className="w-6 h-6 text-white" />
            </div>
            <div>
              <h3 className="font-semibold text-gray-900">Velocity</h3>
              <p className="text-sm text-gray-500">Current speed</p>
            </div>
          </div>
          <div className="space-y-1 text-xs">
            <div className="flex justify-between">
              <span>Linear X:</span>
              <span className="font-mono">{velocity.linear_x.toFixed(2)}</span>
            </div>
            <div className="flex justify-between">
              <span>Linear Y:</span>
              <span className="font-mono">{velocity.linear_y.toFixed(2)}</span>
            </div>
            <div className="flex justify-between">
              <span>Angular Z:</span>
              <span className="font-mono">{velocity.angular_z.toFixed(2)}</span>
            </div>
          </div>
        </Card>

        {/* Rotation */}
        <Card className="flat-card">
          <div className="flex items-center gap-3 mb-4">
            <div className="w-12 h-12 bg-gradient-to-br from-purple-500 to-pink-600 rounded-xl flex items-center justify-center">
              <Compass className="w-6 h-6 text-white" />
            </div>
            <div>
              <h3 className="font-semibold text-gray-900">Orientation</h3>
              <p className="text-sm text-gray-500">Current angle</p>
            </div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">
              {position.rotation}°
            </div>
            <div className="w-16 h-16 mx-auto mt-2 border-2 border-white/20 rounded-full flex items-center justify-center">
              <div
                className="w-1 h-6 bg-blue-500 rounded-full"
                style={{
                  transform: `rotate(${position.rotation}deg)`,
                  transformOrigin: "center bottom",
                }}
              ></div>
            </div>
          </div>
        </Card>
      </div>

      {/* Control Panel */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-8 mb-8">
        {/* Holonomic Movement Controls */}
        <Card className="flat-card">
          <h3 className="text-lg font-semibold text-gray-900 mb-6">
            Holonomic Movement Controls
          </h3>

          {/* Speed Controls */}
          <div className="grid grid-cols-3 gap-4 mb-6">
            {/* Linear Speed */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <label className="text-sm font-medium text-gray-700">
                  Linear
                </label>
                <Badge className="flat-badge-primary">{linearSpeed[0]}%</Badge>
              </div>
              <Slider
                value={linearSpeed}
                onValueChange={setLinearSpeed}
                max={100}
                step={5}
                className="w-full"
              />
            </div>

            {/* Strafe Speed */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <label className="text-sm font-medium text-gray-700">
                  Strafe
                </label>
                <Badge className="flat-badge-success">{strafeSpeed[0]}%</Badge>
              </div>
              <Slider
                value={strafeSpeed}
                onValueChange={setStrafeSpeed}
                max={100}
                step={5}
                className="w-full"
              />
            </div>

            {/* Angular Speed */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <label className="text-sm font-medium text-gray-700">
                  Angular
                </label>
                <Badge className="flat-badge-warning">{angularSpeed[0]}%</Badge>
              </div>
              <Slider
                value={angularSpeed}
                onValueChange={setAngularSpeed}
                max={100}
                step={5}
                className="w-full"
              />
            </div>
          </div>

          {/* Holonomic Direction Pad */}
          <div className="grid grid-cols-3 gap-2 max-w-sm mx-auto">
            {holonomicButtons.map((button, index) => {
              const IconComponent = button.icon;
              return (
                <Button
                  key={index}
                  onClick={() => handleHolonomicMovement(button.action)}
                  className={`flat-button ${button.position} h-14 w-14 ${button.color} text-white hover:opacity-90 flex flex-col gap-1 text-lg`}
                  title={button.action.replace(/_/g, " ")}
                >
                  <span className="text-xl">{button.label}</span>
                </Button>
              );
            })}
          </div>

          {/* Manual Velocity Input */}
          <div className="mt-6 pt-6 border-t">
            <h4 className="font-semibold text-gray-900 mb-4">
              Manual Velocity Control
            </h4>
            <div className="grid grid-cols-3 gap-4">
              <div>
                <Label>Linear X (m/s)</Label>
                <Input
                  type="number"
                  step="0.1"
                  value={velocity.linear_x}
                  onChange={(e) =>
                    setVelocity((prev) => ({
                      ...prev,
                      linear_x: parseFloat(e.target.value) || 0,
                    }))
                  }
                />
              </div>
              <div>
                <Label>Linear Y (m/s)</Label>
                <Input
                  type="number"
                  step="0.1"
                  value={velocity.linear_y}
                  onChange={(e) =>
                    setVelocity((prev) => ({
                      ...prev,
                      linear_y: parseFloat(e.target.value) || 0,
                    }))
                  }
                />
              </div>
              <div>
                <Label>Angular Z (rad/s)</Label>
                <Input
                  type="number"
                  step="0.1"
                  value={velocity.angular_z}
                  onChange={(e) =>
                    setVelocity((prev) => ({
                      ...prev,
                      angular_z: parseFloat(e.target.value) || 0,
                    }))
                  }
                />
              </div>
            </div>
          </div>
        </Card>

        {/* Quick Actions */}
        <Card className="flat-card">
          <h3 className="text-lg font-semibold text-gray-900 mb-6">
            Quick Actions
          </h3>
          <div className="space-y-4">
            {quickActions.map((action, index) => {
              const IconComponent = action.icon;
              return (
                <Button
                  key={index}
                  className={`flat-button w-full justify-start text-white ${action.color} hover:opacity-90 h-16`}
                  onClick={() => console.log(action.title)}
                >
                  <div className="flex items-center gap-4">
                    <IconComponent className="w-6 h-6" />
                    <div className="text-left">
                      <div className="font-semibold">{action.title}</div>
                      <div className="text-sm opacity-90">
                        {action.description}
                      </div>
                    </div>
                  </div>
                </Button>
              );
            })}
          </div>
        </Card>
      </div>

      {/* Robot Visualization */}
      <Card className="flat-card">
        <h3 className="text-lg font-semibold text-gray-900 mb-6">
          Holonomic Robot Visualization
        </h3>
        <div className="relative w-full h-96 bg-gray-100 rounded-lg overflow-hidden">
          {/* Grid Background */}
          <div
            className="absolute inset-0 opacity-20"
            style={{
              backgroundImage: `
                linear-gradient(rgba(0,0,0,0.1) 1px, transparent 1px),
                linear-gradient(90deg, rgba(0,0,0,0.1) 1px, transparent 1px)
              `,
              backgroundSize: "20px 20px",
            }}
          ></div>

          {/* Center Point */}
          <div className="absolute top-1/2 left-1/2 w-2 h-2 bg-red-500 rounded-full transform -translate-x-1/2 -translate-y-1/2"></div>

          {/* Robot with Direction Indicators */}
          <div
            className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 transition-all duration-300 ease-out"
            style={{
              transform: `translate(-50%, -50%) translate(${position.x * 20}px, ${-position.y * 20}px)`,
            }}
          >
            {/* Robot Body */}
            <div
              className="w-12 h-12 bg-blue-500 rounded-lg flex items-center justify-center"
              style={{
                transform: `rotate(${position.rotation}deg)`,
              }}
            >
              <Bot className="w-6 h-6 text-white" />
            </div>

            {/* Velocity Vectors */}
            {velocity.linear_x !== 0 && (
              <div
                className="absolute top-1/2 left-1/2 w-1 bg-red-400 transform-gpu -translate-x-1/2 -translate-y-1/2"
                style={{
                  height: `${Math.abs(velocity.linear_x) * 40}px`,
                  transform: `translate(-50%, -50%) rotate(${velocity.linear_x > 0 ? 0 : 180}deg)`,
                  transformOrigin: "center bottom",
                }}
              />
            )}
            {velocity.linear_y !== 0 && (
              <div
                className="absolute top-1/2 left-1/2 w-1 bg-green-400 transform-gpu -translate-x-1/2 -translate-y-1/2"
                style={{
                  height: `${Math.abs(velocity.linear_y) * 40}px`,
                  transform: `translate(-50%, -50%) rotate(${velocity.linear_y > 0 ? 90 : -90}deg)`,
                  transformOrigin: "center bottom",
                }}
              />
            )}
          </div>

          {/* Info Panels */}
          <div className="absolute top-4 left-4 bg-white/90 backdrop-blur-sm px-3 py-2 rounded-lg">
            <div className="text-sm font-medium text-gray-900">Position</div>
            <div className="text-xs text-slate-300">
              X: {position.x.toFixed(2)}m, Y: {position.y.toFixed(2)}m
            </div>
            <div className="text-xs text-slate-300">θ: {position.rotation}°</div>
          </div>

          <div className="absolute top-4 right-4 bg-white/90 backdrop-blur-sm px-3 py-2 rounded-lg">
            <div className="text-sm font-medium text-gray-900">Velocity</div>
            <div className="text-xs text-slate-300">
              Vx: {velocity.linear_x.toFixed(2)} m/s
            </div>
            <div className="text-xs text-slate-300">
              Vy: {velocity.linear_y.toFixed(2)} m/s
            </div>
            <div className="text-xs text-slate-300">
              ω: {velocity.angular_z.toFixed(2)} rad/s
            </div>
          </div>

          <div className="absolute bottom-4 left-4 bg-white/90 backdrop-blur-sm px-3 py-2 rounded-lg">
            <div className="flex items-center gap-2">
              {getStatusIcon()}
              <span className="text-sm font-medium capitalize">
                {robotStatus}
              </span>
            </div>
          </div>
        </div>
      </Card>
    </div>
  );
}
