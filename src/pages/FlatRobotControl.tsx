import React, { useState } from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import { Slider } from "@/components/ui/slider";
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
} from "lucide-react";
import "../styles/flat-vector-theme.css";

export default function FlatRobotControl() {
  const [robotStatus, setRobotStatus] = useState<"idle" | "moving" | "stopped">(
    "idle",
  );
  const [speed, setSpeed] = useState([50]);
  const [position, setPosition] = useState({ x: 0, y: 0, rotation: 0 });

  const movementButtons = [
    {
      icon: ArrowUp,
      label: "Forward",
      action: "forward",
      position: "col-start-2",
      color: "bg-blue-500",
    },
    {
      icon: ArrowLeft,
      label: "Left",
      action: "left",
      position: "col-start-1 row-start-2",
      color: "bg-green-500",
    },
    {
      icon: Square,
      label: "Stop",
      action: "stop",
      position: "col-start-2 row-start-2",
      color: "bg-red-500",
    },
    {
      icon: ArrowRight,
      label: "Right",
      action: "right",
      position: "col-start-3 row-start-2",
      color: "bg-green-500",
    },
    {
      icon: ArrowDown,
      label: "Backward",
      action: "backward",
      position: "col-start-2 row-start-3",
      color: "bg-blue-500",
    },
    {
      icon: RotateCcw,
      label: "Turn Left",
      action: "turn_left",
      position: "col-start-1 row-start-3",
      color: "bg-purple-500",
    },
    {
      icon: RotateCw,
      label: "Turn Right",
      action: "turn_right",
      position: "col-start-3 row-start-3",
      color: "bg-purple-500",
    },
  ];

  const quickActions = [
    {
      title: "Go Home",
      icon: Home,
      color: "bg-blue-500",
      description: "Return to base position",
    },
    {
      title: "Emergency Stop",
      icon: Shield,
      color: "bg-red-500",
      description: "Stop all movement immediately",
    },
    {
      title: "Auto Navigate",
      icon: Navigation,
      color: "bg-green-500",
      description: "Enable autonomous navigation",
    },
    {
      title: "Manual Mode",
      icon: Target,
      color: "bg-orange-500",
      description: "Switch to manual control",
    },
  ];

  const handleMovement = (action: string) => {
    setRobotStatus("moving");
    console.log(`Robot moving: ${action} at speed ${speed[0]}%`);

    // Update position for demo
    switch (action) {
      case "forward":
        setPosition((prev) => ({ ...prev, y: prev.y + 0.1 }));
        break;
      case "backward":
        setPosition((prev) => ({ ...prev, y: prev.y - 0.1 }));
        break;
      case "left":
        setPosition((prev) => ({ ...prev, x: prev.x - 0.1 }));
        break;
      case "right":
        setPosition((prev) => ({ ...prev, x: prev.x + 0.1 }));
        break;
      case "turn_left":
        setPosition((prev) => ({
          ...prev,
          rotation: (prev.rotation - 15) % 360,
        }));
        break;
      case "turn_right":
        setPosition((prev) => ({
          ...prev,
          rotation: (prev.rotation + 15) % 360,
        }));
        break;
      case "stop":
        setRobotStatus("stopped");
        break;
    }

    // Auto return to idle after movement
    if (action !== "stop") {
      setTimeout(() => setRobotStatus("idle"), 1000);
    }
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
      <div className="mb-8">
        <h1 className="flat-title">Robot Control 🤖</h1>
        <p className="flat-subtitle">
          Control your robot's movement and monitor its status
        </p>
      </div>

      {/* Status Section */}
      <div className="flat-grid flat-grid-3 mb-8">
        {/* Robot Status */}
        <Card className="flat-card">
          <div className="flex items-center gap-3 mb-4">
            <div className="w-12 h-12 bg-gradient-to-br from-blue-500 to-purple-600 rounded-xl flex items-center justify-center">
              <Bot className="w-6 h-6 text-white" />
            </div>
            <div>
              <h3 className="font-semibold text-gray-900">Robot Status</h3>
              <p className="text-sm text-gray-500">Current state</p>
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
              <span className="text-sm text-gray-600">X:</span>
              <span className="text-sm font-medium">
                {position.x.toFixed(2)}m
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-sm text-gray-600">Y:</span>
              <span className="text-sm font-medium">
                {position.y.toFixed(2)}m
              </span>
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
              <h3 className="font-semibold text-gray-900">Rotation</h3>
              <p className="text-sm text-gray-500">Current angle</p>
            </div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">
              {position.rotation}°
            </div>
            <div className="w-16 h-16 mx-auto mt-2 border-2 border-gray-300 rounded-full flex items-center justify-center">
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
        {/* Movement Controls */}
        <Card className="flat-card">
          <h3 className="text-lg font-semibold text-gray-900 mb-6">
            Movement Controls
          </h3>

          {/* Speed Control */}
          <div className="mb-6">
            <div className="flex items-center justify-between mb-3">
              <label className="text-sm font-medium text-gray-700">
                Speed Control
              </label>
              <Badge className="flat-badge-primary">{speed[0]}%</Badge>
            </div>
            <Slider
              value={speed}
              onValueChange={setSpeed}
              max={100}
              step={10}
              className="w-full"
            />
            <div className="flex justify-between text-xs text-gray-500 mt-1">
              <span>Slow</span>
              <span>Fast</span>
            </div>
          </div>

          {/* Direction Pad */}
          <div className="grid grid-cols-3 gap-3 max-w-xs mx-auto">
            {movementButtons.map((button, index) => {
              const IconComponent = button.icon;
              return (
                <Button
                  key={index}
                  onClick={() => handleMovement(button.action)}
                  className={`flat-button ${button.position} h-16 w-16 ${button.color} text-white hover:opacity-90 flex flex-col gap-1`}
                  title={button.label}
                >
                  <IconComponent className="w-6 h-6" />
                  <span className="text-xs">{button.label}</span>
                </Button>
              );
            })}
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
          Robot Visualization
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

          {/* Robot */}
          <div
            className="absolute top-1/2 left-1/2 w-12 h-12 bg-blue-500 rounded-lg flex items-center justify-center transform -translate-x-1/2 -translate-y-1/2 transition-all duration-300 ease-out"
            style={{
              transform: `translate(-50%, -50%) translate(${position.x * 20}px, ${-position.y * 20}px) rotate(${position.rotation}deg)`,
            }}
          >
            <Bot className="w-6 h-6 text-white" />
          </div>

          {/* Position Info */}
          <div className="absolute top-4 left-4 bg-white/90 backdrop-blur-sm px-3 py-2 rounded-lg">
            <div className="text-sm font-medium text-gray-900">Position</div>
            <div className="text-xs text-gray-600">
              X: {position.x.toFixed(2)}m, Y: {position.y.toFixed(2)}m
            </div>
            <div className="text-xs text-gray-600">
              Angle: {position.rotation}°
            </div>
          </div>

          {/* Status Info */}
          <div className="absolute top-4 right-4 bg-white/90 backdrop-blur-sm px-3 py-2 rounded-lg">
            <div className="flex items-center gap-2">
              {getStatusIcon()}
              <span className="text-sm font-medium capitalize">
                {robotStatus}
              </span>
            </div>
            <div className="text-xs text-gray-600">Speed: {speed[0]}%</div>
          </div>
        </div>
      </Card>
    </div>
  );
}
