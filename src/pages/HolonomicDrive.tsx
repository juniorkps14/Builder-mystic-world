import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  RotateCw,
  RotateCcw,
  Move,
  Settings,
  Zap,
  Gauge,
  Circle,
  Square,
  Navigation,
  Activity,
} from "lucide-react";

const HolonomicDrive = () => {
  const { t } = useLanguage();

  // Velocity controls
  const [velocity, setVelocity] = useState({
    linear_x: [0], // Forward/Backward (m/s)
    linear_y: [0], // Left/Right strafe (m/s)
    angular_z: [0], // Rotation (rad/s)
  });

  // Wheel states for 4-wheel holonomic (mecanum or omni)
  const [wheels, setWheels] = useState({
    front_left: { speed: 0, current: 2.1, temperature: 35, status: "active" },
    front_right: { speed: 0, current: 1.9, temperature: 33, status: "active" },
    rear_left: { speed: 0, current: 2.3, temperature: 38, status: "active" },
    rear_right: { speed: 0, current: 2.0, temperature: 34, status: "active" },
  });

  // Kinematics parameters
  const [kinematics, setKinematics] = useState({
    wheel_radius: [0.1], // meters
    wheel_base: [0.5], // meters (distance between front and rear wheels)
    track_width: [0.4], // meters (distance between left and right wheels)
    max_linear_vel: [2.0], // m/s
    max_angular_vel: [3.14], // rad/s
  });

  const [isEmergencyStop, setIsEmergencyStop] = useState(false);
  const [driveMode, setDriveMode] = useState<"manual" | "auto">("manual");

  // Calculate individual wheel speeds based on holonomic kinematics
  const calculateWheelSpeeds = (vx: number, vy: number, vz: number) => {
    const wheelRadius = kinematics.wheel_radius[0];
    const wheelBase = kinematics.wheel_base[0];
    const trackWidth = kinematics.track_width[0];

    // Mecanum wheel kinematics
    const l = (wheelBase + trackWidth) / 2; // Half of the sum of wheelbase and track width

    const frontLeft = (vx - vy - l * vz) / wheelRadius;
    const frontRight = (vx + vy + l * vz) / wheelRadius;
    const rearLeft = (vx + vy - l * vz) / wheelRadius;
    const rearRight = (vx - vy + l * vz) / wheelRadius;

    return { frontLeft, frontRight, rearLeft, rearRight };
  };

  const handleVelocityChange = (axis: string, value: number[]) => {
    setVelocity((prev) => ({ ...prev, [axis]: value }));

    // Calculate wheel speeds
    const speeds = calculateWheelSpeeds(
      velocity.linear_x[0],
      velocity.linear_y[0],
      velocity.angular_z[0],
    );

    // Update wheel states
    setWheels((prev) => ({
      front_left: { ...prev.front_left, speed: speeds.frontLeft },
      front_right: { ...prev.front_right, speed: speeds.frontRight },
      rear_left: { ...prev.rear_left, speed: speeds.rearLeft },
      rear_right: { ...prev.rear_right, speed: speeds.rearRight },
    }));
  };

  const presetMoves = [
    { name: "Forward", vx: 0.5, vy: 0, vz: 0, icon: ArrowUp },
    { name: "Backward", vx: -0.5, vy: 0, vz: 0, icon: ArrowDown },
    { name: "Strafe Left", vx: 0, vy: 0.5, vz: 0, icon: ArrowLeft },
    { name: "Strafe Right", vx: 0, vy: -0.5, vz: 0, icon: ArrowRight },
    { name: "Rotate Left", vx: 0, vy: 0, vz: 1.0, icon: RotateCcw },
    { name: "Rotate Right", vx: 0, vy: 0, vz: -1.0, icon: RotateCw },
    { name: "Diagonal FL", vx: 0.35, vy: 0.35, vz: 0, icon: Move },
    { name: "Diagonal FR", vx: 0.35, vy: -0.35, vz: 0, icon: Move },
  ];

  const executePresetMove = (move: (typeof presetMoves)[0]) => {
    if (isEmergencyStop) return;

    setVelocity({
      linear_x: [move.vx],
      linear_y: [move.vy],
      angular_z: [move.vz],
    });
  };

  const stopAllMotion = () => {
    setVelocity({
      linear_x: [0],
      linear_y: [0],
      angular_z: [0],
    });
  };

  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            {t("holonomic.title")}
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            4-Wheel Holonomic Drive System with Omnidirectional Movement
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge className="bg-cyan-100 text-cyan-700">
            <Circle className="h-3 w-3 mr-1" />
            Omni-directional
          </Badge>
        </div>
      </div>

      <div className="space-y-6">

      {/* Emergency Stop & Mode */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <Button
            size="lg"
            variant={isEmergencyStop ? "default" : "destructive"}
            onClick={() => setIsEmergencyStop(!isEmergencyStop)}
            className="gap-2"
          >
            <Square className="h-5 w-5" />
            EMERGENCY STOP
          </Button>
          <Badge variant={driveMode === "manual" ? "default" : "secondary"}>
            {driveMode.toUpperCase()} MODE
          </Badge>
        </div>

        <div className="flex items-center gap-2">
          <Button
            variant={driveMode === "manual" ? "default" : "outline"}
            onClick={() => setDriveMode("manual")}
          >
            Manual
          </Button>
          <Button
            variant={driveMode === "auto" ? "default" : "outline"}
            onClick={() => setDriveMode("auto")}
          >
            Auto
          </Button>
        </div>
      </div>

      <Tabs defaultValue="velocity" className="space-y-4">
        <TabsList className="grid w-full grid-cols-4">
          <TabsTrigger value="velocity" className="gap-2">
            <Gauge className="h-4 w-4" />
            {t("holonomic.velocity")}
          </TabsTrigger>
          <TabsTrigger value="wheels" className="gap-2">
            <Circle className="h-4 w-4" />
            {t("holonomic.wheels")}
          </TabsTrigger>
          <TabsTrigger value="kinematics" className="gap-2">
            <Settings className="h-4 w-4" />
            {t("holonomic.kinematics")}
          </TabsTrigger>
          <TabsTrigger value="presets" className="gap-2">
            <Navigation className="h-4 w-4" />
            Presets
          </TabsTrigger>
        </TabsList>

        {/* Velocity Control */}
        <TabsContent value="velocity" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {/* Manual Velocity Control */}
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                {t("holonomic.velocity")} Control
              </h3>

              <div className="space-y-6">
                <div>
                  <label className="text-sm font-medium mb-2 block">
                    {t("holonomic.linear")} X: {velocity.linear_x[0].toFixed(2)}{" "}
                    m/s
                  </label>
                  <Slider
                    value={velocity.linear_x}
                    onValueChange={(value) =>
                      handleVelocityChange("linear_x", value)
                    }
                    min={-kinematics.max_linear_vel[0]}
                    max={kinematics.max_linear_vel[0]}
                    step={0.01}
                    disabled={isEmergencyStop}
                  />
                  <div className="flex justify-between text-xs text-muted-foreground mt-1">
                    <span>Backward</span>
                    <span>Forward</span>
                  </div>
                </div>

                <div>
                  <label className="text-sm font-medium mb-2 block">
                    {t("holonomic.strafe")} Y: {velocity.linear_y[0].toFixed(2)}{" "}
                    m/s
                  </label>
                  <Slider
                    value={velocity.linear_y}
                    onValueChange={(value) =>
                      handleVelocityChange("linear_y", value)
                    }
                    min={-kinematics.max_linear_vel[0]}
                    max={kinematics.max_linear_vel[0]}
                    step={0.01}
                    disabled={isEmergencyStop}
                  />
                  <div className="flex justify-between text-xs text-muted-foreground mt-1">
                    <span>Right</span>
                    <span>Left</span>
                  </div>
                </div>

                <div>
                  <label className="text-sm font-medium mb-2 block">
                    {t("holonomic.angular")} Z:{" "}
                    {velocity.angular_z[0].toFixed(2)} rad/s
                  </label>
                  <Slider
                    value={velocity.angular_z}
                    onValueChange={(value) =>
                      handleVelocityChange("angular_z", value)
                    }
                    min={-kinematics.max_angular_vel[0]}
                    max={kinematics.max_angular_vel[0]}
                    step={0.01}
                    disabled={isEmergencyStop}
                  />
                  <div className="flex justify-between text-xs text-muted-foreground mt-1">
                    <span>CCW</span>
                    <span>CW</span>
                  </div>
                </div>

                <Separator />

                <div className="flex gap-2">
                  <Button
                    onClick={stopAllMotion}
                    variant="destructive"
                    className="gap-2"
                    disabled={isEmergencyStop}
                  >
                    <Square className="h-4 w-4" />
                    STOP
                  </Button>
                  <Button variant="outline" disabled={isEmergencyStop}>
                    Coast
                  </Button>
                  <Button variant="outline" disabled={isEmergencyStop}>
                    Brake
                  </Button>
                </div>
              </div>
            </Card>

            {/* Direction Pad */}
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Direction Control</h3>

              <div className="grid grid-cols-3 gap-3 mb-6">
                {/* Top row */}
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[6])} // Diagonal FL
                >
                  ↖
                </Button>
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[0])} // Forward
                >
                  <ArrowUp className="h-5 w-5" />
                </Button>
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[7])} // Diagonal FR
                >
                  ↗
                </Button>

                {/* Middle row */}
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[2])} // Strafe Left
                >
                  <ArrowLeft className="h-5 w-5" />
                </Button>
                <Button
                  variant="destructive"
                  className="aspect-square"
                  onClick={stopAllMotion}
                  disabled={isEmergencyStop}
                >
                  <Square className="h-5 w-5" />
                </Button>
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[3])} // Strafe Right
                >
                  <ArrowRight className="h-5 w-5" />
                </Button>

                {/* Bottom row */}
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                >
                  ↙
                </Button>
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[1])} // Backward
                >
                  <ArrowDown className="h-5 w-5" />
                </Button>
                <Button
                  variant="outline"
                  className="aspect-square"
                  disabled={isEmergencyStop}
                >
                  ↘
                </Button>
              </div>

              {/* Rotation Controls */}
              <div className="flex justify-center gap-3">
                <Button
                  variant="outline"
                  className="gap-2"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[4])} // Rotate Left
                >
                  <RotateCcw className="h-4 w-4" />
                  Rotate Left
                </Button>
                <Button
                  variant="outline"
                  className="gap-2"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(presetMoves[5])} // Rotate Right
                >
                  <RotateCw className="h-4 w-4" />
                  Rotate Right
                </Button>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Wheel Status */}
        <TabsContent value="wheels" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            {Object.entries(wheels).map(([wheelName, wheel]) => (
              <Card key={wheelName} className="p-4">
                <div className="flex items-center justify-between mb-3">
                  <h4 className="font-medium capitalize">
                    {wheelName.replace("_", " ")}
                  </h4>
                  <Badge
                    variant={
                      wheel.status === "active" ? "default" : "destructive"
                    }
                  >
                    {wheel.status}
                  </Badge>
                </div>

                <div className="space-y-3">
                  <div className="flex justify-between text-sm">
                    <span>Speed:</span>
                    <span className="font-mono">
                      {wheel.speed.toFixed(2)} rad/s
                    </span>
                  </div>

                  <div className="space-y-1">
                    <div className="flex justify-between text-sm">
                      <span>Current:</span>
                      <span className="font-mono">{wheel.current} A</span>
                    </div>
                    <Progress
                      value={(wheel.current / 5) * 100}
                      className="h-2"
                    />
                  </div>

                  <div className="space-y-1">
                    <div className="flex justify-between text-sm">
                      <span>Temperature:</span>
                      <span className="font-mono">{wheel.temperature}°C</span>
                    </div>
                    <Progress
                      value={(wheel.temperature / 80) * 100}
                      className="h-2"
                    />
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        {/* Kinematics Configuration */}
        <TabsContent value="kinematics" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">
              {t("holonomic.kinematics")} Parameters
            </h3>

            <div className="space-y-6">
              <div>
                <label className="text-sm font-medium mb-2 block">
                  Wheel Radius: {kinematics.wheel_radius[0].toFixed(3)} m
                </label>
                <Slider
                  value={kinematics.wheel_radius}
                  onValueChange={(value) =>
                    setKinematics((prev) => ({ ...prev, wheel_radius: value }))
                  }
                  min={0.05}
                  max={0.2}
                  step={0.001}
                />
              </div>

              <div>
                <label className="text-sm font-medium mb-2 block">
                  Wheel Base: {kinematics.wheel_base[0].toFixed(3)} m
                </label>
                <Slider
                  value={kinematics.wheel_base}
                  onValueChange={(value) =>
                    setKinematics((prev) => ({ ...prev, wheel_base: value }))
                  }
                  min={0.2}
                  max={1.0}
                  step={0.001}
                />
              </div>

              <div>
                <label className="text-sm font-medium mb-2 block">
                  Track Width: {kinematics.track_width[0].toFixed(3)} m
                </label>
                <Slider
                  value={kinematics.track_width}
                  onValueChange={(value) =>
                    setKinematics((prev) => ({ ...prev, track_width: value }))
                  }
                  min={0.2}
                  max={1.0}
                  step={0.001}
                />
              </div>

              <Separator />

              <div>
                <label className="text-sm font-medium mb-2 block">
                  Max Linear Velocity: {kinematics.max_linear_vel[0].toFixed(2)}{" "}
                  m/s
                </label>
                <Slider
                  value={kinematics.max_linear_vel}
                  onValueChange={(value) =>
                    setKinematics((prev) => ({
                      ...prev,
                      max_linear_vel: value,
                    }))
                  }
                  min={0.1}
                  max={5.0}
                  step={0.01}
                />
              </div>

              <div>
                <label className="text-sm font-medium mb-2 block">
                  Max Angular Velocity:{" "}
                  {kinematics.max_angular_vel[0].toFixed(2)} rad/s
                </label>
                <Slider
                  value={kinematics.max_angular_vel}
                  onValueChange={(value) =>
                    setKinematics((prev) => ({
                      ...prev,
                      max_angular_vel: value,
                    }))
                  }
                  min={0.1}
                  max={6.28}
                  step={0.01}
                />
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Preset Movements */}
        <TabsContent value="presets" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">Preset Movements</h3>

            <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
              {presetMoves.map((move, index) => (
                <Button
                  key={index}
                  variant="outline"
                  className="aspect-square flex flex-col gap-2"
                  disabled={isEmergencyStop}
                  onClick={() => executePresetMove(move)}
                >
                  <move.icon className="h-6 w-6" />
                  <span className="text-xs">{move.name}</span>
                </Button>
              ))}
            </div>
          </Card>
        </TabsContent>
      </Tabs>
      </div>
    </div>
  );
};

export default HolonomicDrive;
