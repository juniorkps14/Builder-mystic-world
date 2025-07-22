import React, { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Alert, AlertDescription } from "@/components/ui/alert";
import {
  Zap,
  Move3D,
  Hand,
  RotateCcw,
  Home,
  Square,
  Play,
  Pause,
  AlertTriangle,
  Settings,
  Target,
  Grip,
  Activity,
  Power,
  Shield,
  Gauge,
  Clock,
  CheckCircle,
} from "lucide-react";

export default function RoboticArm() {
  // Joint states (6-DOF arm)
  const [joints, setJoints] = useState({
    base: [0], // Base rotation
    shoulder: [0], // Shoulder
    elbow: [0], // Elbow
    wrist1: [0], // Wrist 1
    wrist2: [0], // Wrist 2
    wrist3: [0], // Wrist 3
  });

  // End effector state
  const [endEffector, setEndEffector] = useState({
    x: [250], // X position (mm)
    y: [0], // Y position (mm)
    z: [300], // Z position (mm)
    roll: [0], // Roll (deg)
    pitch: [0], // Pitch (deg)
    yaw: [0], // Yaw (deg)
  });

  // Gripper state
  const [gripper, setGripper] = useState({
    position: [50], // Gripper opening (0-100%)
    force: [30], // Gripping force (0-100%)
  });

  // Arm status
  const [armStatus, setArmStatus] = useState({
    power: true,
    homed: true,
    moving: false,
    error: false,
    mode: "manual", // manual, auto, teach
  });

  // Preset positions
  const presetPositions = [
    {
      name: "Home Position",
      icon: Home,
      joints: {
        base: 0,
        shoulder: 0,
        elbow: 0,
        wrist1: 0,
        wrist2: 0,
        wrist3: 0,
      },
      description: "Safe home position",
    },
    {
      name: "Pick Position",
      icon: Hand,
      joints: {
        base: 45,
        shoulder: -30,
        elbow: 90,
        wrist1: -60,
        wrist2: 0,
        wrist3: 0,
      },
      description: "Optimal picking position",
    },
    {
      name: "Place Position",
      icon: Target,
      joints: {
        base: -45,
        shoulder: -45,
        elbow: 120,
        wrist1: -75,
        wrist2: 0,
        wrist3: 0,
      },
      description: "Standard placing position",
    },
    {
      name: "Inspection",
      icon: Activity,
      joints: {
        base: 0,
        shoulder: -90,
        elbow: 90,
        wrist1: 0,
        wrist2: 90,
        wrist3: 0,
      },
      description: "Inspection orientation",
    },
  ];

  const updateJoint = (jointName: string, value: number[]) => {
    setJoints((prev) => ({ ...prev, [jointName]: value }));
  };

  const updateEndEffector = (axis: string, value: number[]) => {
    setEndEffector((prev) => ({ ...prev, [axis]: value }));
  };

  const moveToPreset = (preset: any) => {
    setArmStatus((prev) => ({ ...prev, moving: true }));

    // Simulate movement
    setTimeout(() => {
      setJoints({
        base: [preset.joints.base],
        shoulder: [preset.joints.shoulder],
        elbow: [preset.joints.elbow],
        wrist1: [preset.joints.wrist1],
        wrist2: [preset.joints.wrist2],
        wrist3: [preset.joints.wrist3],
      });
      setArmStatus((prev) => ({ ...prev, moving: false }));
    }, 2000);
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "moving":
        return "from-blue-500 to-cyan-500";
      case "ready":
        return "from-emerald-500 to-teal-500";
      case "error":
        return "from-red-500 to-pink-500";
      default:
        return "from-gray-500 to-slate-500";
    }
  };

  const getCurrentStatus = () => {
    if (armStatus.error) return "error";
    if (armStatus.moving) return "moving";
    if (armStatus.power && armStatus.homed) return "ready";
    return "offline";
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Tesla-inspired Header */}
      <div className="mb-8">
        <div className="bg-white/10 backdrop-blur-xl border border-white/20 rounded-3xl p-6 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="space-y-2">
              <h1 className="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
                Robotic Arm Control
              </h1>
              <p className="text-slate-300 font-light">
                6-DOF robotic arm control with precision positioning and force
                feedback
              </p>
            </div>

            <div className="flex items-center gap-4">
              <Badge
                className={`bg-gradient-to-r ${getStatusColor(getCurrentStatus())} text-white border-0 px-4 py-2`}
              >
                {getCurrentStatus().charAt(0).toUpperCase() +
                  getCurrentStatus().slice(1)}
              </Badge>
              <Button
                onClick={() =>
                  setArmStatus((prev) => ({ ...prev, power: !prev.power }))
                }
                className={`${
                  armStatus.power
                    ? "bg-gradient-to-r from-red-500 to-pink-500 hover:from-red-600 hover:to-pink-600"
                    : "bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600"
                } text-white shadow-lg transition-all duration-300`}
              >
                <Power className="h-4 w-4 mr-2" />
                {armStatus.power ? "Power Off" : "Power On"}
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* System Status Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Power Status</p>
              <p className="text-xl font-light text-white mt-1">
                {armStatus.power ? "Online" : "Offline"}
              </p>
            </div>
            <div
              className={`h-12 w-12 rounded-xl flex items-center justify-center ${
                armStatus.power
                  ? "bg-emerald-500/20"
                  : "bg-white/5 border border-white/100/20"
              }`}
            >
              <Power
                className={`h-6 w-6 ${armStatus.power ? "text-emerald-400" : "text-gray-400"}`}
              />
            </div>
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Safety Status</p>
              <p className="text-xl font-light text-white mt-1">
                {armStatus.error ? "Alert" : "Safe"}
              </p>
            </div>
            <div
              className={`h-12 w-12 rounded-xl flex items-center justify-center ${
                armStatus.error ? "bg-red-500/20" : "bg-emerald-500/20"
              }`}
            >
              <Shield
                className={`h-6 w-6 ${armStatus.error ? "text-red-400" : "text-emerald-400"}`}
              />
            </div>
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Operation Mode</p>
              <p className="text-xl font-light text-white mt-1 capitalize">
                {armStatus.mode}
              </p>
            </div>
            <div className="h-12 w-12 bg-blue-500/20 rounded-xl flex items-center justify-center">
              <Gauge className="h-6 w-6 text-blue-400" />
            </div>
          </div>
        </Card>

        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-slate-400 text-sm">Home Status</p>
              <p className="text-xl font-light text-white mt-1">
                {armStatus.homed ? "Homed" : "Not Homed"}
              </p>
            </div>
            <div
              className={`h-12 w-12 rounded-xl flex items-center justify-center ${
                armStatus.homed ? "bg-emerald-500/20" : "bg-yellow-500/20"
              }`}
            >
              <CheckCircle
                className={`h-6 w-6 ${armStatus.homed ? "text-emerald-400" : "text-yellow-400"}`}
              />
            </div>
          </div>
        </Card>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-8">
        {/* Main Control Panel */}
        <div className="lg:col-span-3">
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-2xl">
            <div className="p-6">
              <Tabs defaultValue="joints" className="w-full">
                <TabsList className="grid w-full grid-cols-4 bg-white/10 border border-white/20">
                  <TabsTrigger
                    value="joints"
                    className="data-[state=active]:bg-white/20"
                  >
                    Joint Control
                  </TabsTrigger>
                  <TabsTrigger
                    value="cartesian"
                    className="data-[state=active]:bg-white/20"
                  >
                    Cartesian
                  </TabsTrigger>
                  <TabsTrigger
                    value="gripper"
                    className="data-[state=active]:bg-white/20"
                  >
                    Gripper
                  </TabsTrigger>
                  <TabsTrigger
                    value="teach"
                    className="data-[state=active]:bg-white/20"
                  >
                    Teach Mode
                  </TabsTrigger>
                </TabsList>

                {/* Joint Control */}
                <TabsContent value="joints" className="mt-6">
                  <div className="space-y-6">
                    <h3 className="text-lg font-light text-white">
                      Joint Position Control
                    </h3>

                    {Object.entries(joints).map(([jointName, value]) => (
                      <div
                        key={jointName}
                        className="bg-white/5 rounded-xl p-4 border border-white/10"
                      >
                        <div className="flex items-center justify-between mb-3">
                          <label className="text-white font-medium capitalize">
                            {jointName}
                          </label>
                          <Badge className="bg-white/20 text-white border border-white/30">
                            {value[0].toFixed(1)}°
                          </Badge>
                        </div>
                        <Slider
                          value={value}
                          onValueChange={(newValue) =>
                            updateJoint(jointName, newValue)
                          }
                          max={180}
                          min={-180}
                          step={1}
                          className="w-full"
                          disabled={!armStatus.power || armStatus.moving}
                        />
                        <div className="flex justify-between text-xs text-slate-400 mt-2">
                          <span>-180°</span>
                          <span>0°</span>
                          <span>180°</span>
                        </div>
                      </div>
                    ))}
                  </div>
                </TabsContent>

                {/* Cartesian Control */}
                <TabsContent value="cartesian" className="mt-6">
                  <div className="space-y-6">
                    <h3 className="text-lg font-light text-white">
                      Cartesian Position Control
                    </h3>

                    {/* Position Control */}
                    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                      {Object.entries(endEffector)
                        .slice(0, 3)
                        .map(([axis, value]) => (
                          <div
                            key={axis}
                            className="bg-white/5 rounded-xl p-4 border border-white/10"
                          >
                            <div className="flex items-center justify-between mb-3">
                              <label className="text-white font-medium uppercase">
                                {axis}
                              </label>
                              <Badge className="bg-white/20 text-white border border-white/30">
                                {value[0].toFixed(0)}mm
                              </Badge>
                            </div>
                            <Slider
                              value={value}
                              onValueChange={(newValue) =>
                                updateEndEffector(axis, newValue)
                              }
                              max={axis === "z" ? 600 : 400}
                              min={axis === "z" ? 50 : -400}
                              step={1}
                              className="w-full"
                              disabled={!armStatus.power || armStatus.moving}
                            />
                          </div>
                        ))}
                    </div>

                    {/* Orientation Control */}
                    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                      {Object.entries(endEffector)
                        .slice(3, 6)
                        .map(([axis, value]) => (
                          <div
                            key={axis}
                            className="bg-white/5 rounded-xl p-4 border border-white/10"
                          >
                            <div className="flex items-center justify-between mb-3">
                              <label className="text-white font-medium capitalize">
                                {axis}
                              </label>
                              <Badge className="bg-white/20 text-white border border-white/30">
                                {value[0].toFixed(1)}°
                              </Badge>
                            </div>
                            <Slider
                              value={value}
                              onValueChange={(newValue) =>
                                updateEndEffector(axis, newValue)
                              }
                              max={180}
                              min={-180}
                              step={1}
                              className="w-full"
                              disabled={!armStatus.power || armStatus.moving}
                            />
                          </div>
                        ))}
                    </div>
                  </div>
                </TabsContent>

                {/* Gripper Control */}
                <TabsContent value="gripper" className="mt-6">
                  <div className="space-y-6">
                    <h3 className="text-lg font-light text-white">
                      Gripper Control
                    </h3>

                    <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                      <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                        <div className="flex items-center justify-between mb-3">
                          <label className="text-white font-medium">
                            Gripper Position
                          </label>
                          <Badge className="bg-white/20 text-white border border-white/30">
                            {gripper.position[0]}%
                          </Badge>
                        </div>
                        <Slider
                          value={gripper.position}
                          onValueChange={(value) =>
                            setGripper((prev) => ({ ...prev, position: value }))
                          }
                          max={100}
                          min={0}
                          step={1}
                          className="w-full"
                          disabled={!armStatus.power}
                        />
                        <div className="flex justify-between text-xs text-slate-400 mt-2">
                          <span>Closed</span>
                          <span>Open</span>
                        </div>
                      </div>

                      <div className="bg-white/5 rounded-xl p-4 border border-white/10">
                        <div className="flex items-center justify-between mb-3">
                          <label className="text-white font-medium">
                            Gripping Force
                          </label>
                          <Badge className="bg-white/20 text-white border border-white/30">
                            {gripper.force[0]}%
                          </Badge>
                        </div>
                        <Slider
                          value={gripper.force}
                          onValueChange={(value) =>
                            setGripper((prev) => ({ ...prev, force: value }))
                          }
                          max={100}
                          min={0}
                          step={1}
                          className="w-full"
                          disabled={!armStatus.power}
                        />
                        <div className="flex justify-between text-xs text-slate-400 mt-2">
                          <span>Light</span>
                          <span>Maximum</span>
                        </div>
                      </div>
                    </div>

                    <div className="flex gap-4">
                      <Button className="bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600">
                        <Grip className="h-4 w-4 mr-2" />
                        Close Gripper
                      </Button>
                      <Button className="bg-white/10 hover:bg-white/20 border border-white/20">
                        <Hand className="h-4 w-4 mr-2" />
                        Open Gripper
                      </Button>
                    </div>
                  </div>
                </TabsContent>

                {/* Teach Mode */}
                <TabsContent value="teach" className="mt-6">
                  <div className="text-center py-12">
                    <Move3D className="h-16 w-16 text-blue-400 mx-auto mb-4" />
                    <h3 className="text-xl font-light text-white mb-2">
                      Teach Mode
                    </h3>
                    <p className="text-slate-300 mb-6">
                      Move the arm manually to record positions
                    </p>
                    <Button className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600">
                      <Settings className="h-4 w-4 mr-2" />
                      Enable Teach Mode
                    </Button>
                  </div>
                </TabsContent>
              </Tabs>
            </div>
          </Card>
        </div>

        {/* Preset Positions & Quick Actions */}
        <div className="space-y-6">
          {/* Preset Positions */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                Preset Positions
              </h3>
              <div className="space-y-3">
                {presetPositions.map((preset, index) => {
                  const IconComponent = preset.icon;
                  return (
                    <button
                      key={index}
                      onClick={() => moveToPreset(preset)}
                      disabled={!armStatus.power || armStatus.moving}
                      className="w-full bg-white/5 hover:bg-white/15 border border-white/10 rounded-lg p-4 text-left transition-all duration-300 disabled:opacity-50 disabled:cursor-not-allowed"
                    >
                      <div className="flex items-center gap-3">
                        <div className="p-2 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-lg border border-white/20">
                          <IconComponent className="h-5 w-5 text-blue-400" />
                        </div>
                        <div>
                          <p className="text-white font-medium text-sm">
                            {preset.name}
                          </p>
                          <p className="text-slate-400 text-xs">
                            {preset.description}
                          </p>
                        </div>
                      </div>
                    </button>
                  );
                })}
              </div>
            </div>
          </Card>

          {/* Quick Actions */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                Quick Actions
              </h3>
              <div className="space-y-3">
                <Button
                  className="w-full bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600"
                  disabled={!armStatus.power}
                  onClick={() => moveToPreset(presetPositions[0])}
                >
                  <Home className="h-4 w-4 mr-2" />
                  Go Home
                </Button>
                <Button
                  className="w-full bg-white/10 hover:bg-white/20 border border-white/20"
                  disabled={!armStatus.power}
                >
                  <RotateCcw className="h-4 w-4 mr-2" />
                  Reset Position
                </Button>
                <Button
                  className="w-full bg-gradient-to-r from-red-500 to-pink-500 hover:from-red-600 hover:to-pink-600"
                  disabled={!armStatus.power}
                >
                  <Square className="h-4 w-4 mr-2" />
                  Emergency Stop
                </Button>
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <Settings className="h-4 w-4 mr-2" />
                  Arm Settings
                </Button>
              </div>
            </div>
          </Card>

          {/* Movement Status */}
          {armStatus.moving && (
            <Card className="bg-blue-500/20 border border-blue-500/30 shadow-xl">
              <div className="p-6">
                <div className="flex items-center gap-3 mb-4">
                  <Activity className="h-5 w-5 text-blue-400 animate-pulse" />
                  <h3 className="text-lg font-light text-blue-300">
                    Moving to Position
                  </h3>
                </div>
                <Progress value={65} className="h-2 bg-blue-900/50" />
                <p className="text-blue-200 text-sm mt-2">
                  Estimated time: 1.5s remaining
                </p>
              </div>
            </Card>
          )}
        </div>
      </div>
    </div>
  );
}
