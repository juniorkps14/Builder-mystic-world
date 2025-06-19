import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { useLanguage } from "@/contexts/LanguageContext";
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
} from "lucide-react";

const RoboticArm = () => {
  const { t } = useLanguage();

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
    position: [50], // 0-100%
    force: [30], // Force limit
  });

  const [armStatus, setArmStatus] = useState("idle");
  const [isEmergencyStop, setIsEmergencyStop] = useState(false);

  const jointLimits = {
    base: { min: -180, max: 180, unit: "°" },
    shoulder: { min: -90, max: 90, unit: "°" },
    elbow: { min: -150, max: 150, unit: "°" },
    wrist1: { min: -180, max: 180, unit: "°" },
    wrist2: { min: -90, max: 90, unit: "°" },
    wrist3: { min: -180, max: 180, unit: "°" },
  };

  const presetPositions = [
    { name: "Home", joints: [0, 0, 0, 0, 0, 0] },
    { name: "Rest", joints: [0, -90, 90, 0, 0, 0] },
    { name: "Pick", joints: [45, -45, 60, 0, -15, 0] },
    { name: "Place", joints: [-45, -30, 45, 0, -15, 0] },
    { name: "Vertical", joints: [0, 0, -90, 0, 90, 0] },
  ];

  const handleJointChange = (joint: string, value: number[]) => {
    setJoints((prev) => ({ ...prev, [joint]: value }));
  };

  const handleEndEffectorChange = (axis: string, value: number[]) => {
    setEndEffector((prev) => ({ ...prev, [axis]: value }));
  };

  const goToPreset = (preset: (typeof presetPositions)[0]) => {
    const newJoints = {
      base: [preset.joints[0]],
      shoulder: [preset.joints[1]],
      elbow: [preset.joints[2]],
      wrist1: [preset.joints[3]],
      wrist2: [preset.joints[4]],
      wrist3: [preset.joints[5]],
    };
    setJoints(newJoints);
    setArmStatus("moving");
  };

  const handleEmergencyStop = () => {
    setIsEmergencyStop(!isEmergencyStop);
    if (!isEmergencyStop) {
      setArmStatus("emergency_stop");
    } else {
      setArmStatus("idle");
    }
  };

  return (
    <div className="space-y-6">
      <div className="mb-6">
        <h1 className="text-3xl font-bold">{t("arm.title")}</h1>
        <p className="text-muted-foreground">
          6-DOF Robotic Arm Control with End Effector Management
        </p>
      </div>

      {/* Emergency Stop & Status */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <Button
            size="lg"
            variant={isEmergencyStop ? "default" : "destructive"}
            onClick={handleEmergencyStop}
            className="gap-2"
          >
            <AlertTriangle className="h-5 w-5" />
            {t("arm.emergency")}
          </Button>
          <Badge variant={armStatus === "idle" ? "secondary" : "default"}>
            {armStatus.toUpperCase()}
          </Badge>
        </div>

        <div className="flex items-center gap-2">
          <Button variant="outline" className="gap-2">
            <Home className="h-4 w-4" />
            {t("arm.home")}
          </Button>
          <Button variant="outline" className="gap-2">
            <Settings className="h-4 w-4" />
            Calibrate
          </Button>
        </div>
      </div>

      {isEmergencyStop && (
        <Alert className="border-destructive">
          <AlertTriangle className="h-4 w-4" />
          <AlertDescription>
            Emergency stop is active. All arm movements are disabled.
          </AlertDescription>
        </Alert>
      )}

      <Tabs defaultValue="joints" className="space-y-4">
        <TabsList className="grid w-full grid-cols-4">
          <TabsTrigger value="joints" className="gap-2">
            <Zap className="h-4 w-4" />
            {t("arm.joints")}
          </TabsTrigger>
          <TabsTrigger value="cartesian" className="gap-2">
            <Move3D className="h-4 w-4" />
            Cartesian
          </TabsTrigger>
          <TabsTrigger value="gripper" className="gap-2">
            <Hand className="h-4 w-4" />
            {t("arm.gripper")}
          </TabsTrigger>
          <TabsTrigger value="presets" className="gap-2">
            <Target className="h-4 w-4" />
            Presets
          </TabsTrigger>
        </TabsList>

        {/* Joint Control */}
        <TabsContent value="joints" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">
              {t("arm.joints")} Control
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              {Object.entries(joints).map(([jointName, value]) => {
                const limits =
                  jointLimits[jointName as keyof typeof jointLimits];
                return (
                  <div key={jointName} className="space-y-3">
                    <div className="flex justify-between items-center">
                      <label className="font-medium capitalize">
                        {jointName}: {value[0].toFixed(1)}
                        {limits.unit}
                      </label>
                      <Badge variant="outline" className="text-xs">
                        {limits.min}° to {limits.max}°
                      </Badge>
                    </div>
                    <Slider
                      value={value}
                      onValueChange={(newValue) =>
                        handleJointChange(jointName, newValue)
                      }
                      min={limits.min}
                      max={limits.max}
                      step={1}
                      disabled={isEmergencyStop}
                      className="w-full"
                    />
                    <div className="flex justify-between text-xs text-muted-foreground">
                      <span>{limits.min}°</span>
                      <span>{limits.max}°</span>
                    </div>
                  </div>
                );
              })}
            </div>

            <Separator className="my-6" />

            <div className="flex gap-2">
              <Button disabled={isEmergencyStop} className="gap-2">
                <Play className="h-4 w-4" />
                Execute Motion
              </Button>
              <Button
                variant="outline"
                disabled={isEmergencyStop}
                className="gap-2"
              >
                <Pause className="h-4 w-4" />
                Pause
              </Button>
              <Button
                variant="outline"
                disabled={isEmergencyStop}
                className="gap-2"
              >
                <Square className="h-4 w-4" />
                Stop
              </Button>
            </div>
          </Card>
        </TabsContent>

        {/* Cartesian Control */}
        <TabsContent value="cartesian" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">
              {t("arm.endEffector")} Control
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              {/* Position Control */}
              <div className="space-y-4">
                <h4 className="font-medium">{t("arm.position")} (mm)</h4>

                <div className="space-y-3">
                  <div>
                    <label className="text-sm">X: {endEffector.x[0]}mm</label>
                    <Slider
                      value={endEffector.x}
                      onValueChange={(value) =>
                        handleEndEffectorChange("x", value)
                      }
                      min={-500}
                      max={500}
                      step={1}
                      disabled={isEmergencyStop}
                    />
                  </div>

                  <div>
                    <label className="text-sm">Y: {endEffector.y[0]}mm</label>
                    <Slider
                      value={endEffector.y}
                      onValueChange={(value) =>
                        handleEndEffectorChange("y", value)
                      }
                      min={-500}
                      max={500}
                      step={1}
                      disabled={isEmergencyStop}
                    />
                  </div>

                  <div>
                    <label className="text-sm">Z: {endEffector.z[0]}mm</label>
                    <Slider
                      value={endEffector.z}
                      onValueChange={(value) =>
                        handleEndEffectorChange("z", value)
                      }
                      min={0}
                      max={600}
                      step={1}
                      disabled={isEmergencyStop}
                    />
                  </div>
                </div>
              </div>

              {/* Orientation Control */}
              <div className="space-y-4">
                <h4 className="font-medium">{t("arm.orientation")} (°)</h4>

                <div className="space-y-3">
                  <div>
                    <label className="text-sm">
                      Roll: {endEffector.roll[0]}°
                    </label>
                    <Slider
                      value={endEffector.roll}
                      onValueChange={(value) =>
                        handleEndEffectorChange("roll", value)
                      }
                      min={-180}
                      max={180}
                      step={1}
                      disabled={isEmergencyStop}
                    />
                  </div>

                  <div>
                    <label className="text-sm">
                      Pitch: {endEffector.pitch[0]}°
                    </label>
                    <Slider
                      value={endEffector.pitch}
                      onValueChange={(value) =>
                        handleEndEffectorChange("pitch", value)
                      }
                      min={-180}
                      max={180}
                      step={1}
                      disabled={isEmergencyStop}
                    />
                  </div>

                  <div>
                    <label className="text-sm">
                      Yaw: {endEffector.yaw[0]}°
                    </label>
                    <Slider
                      value={endEffector.yaw}
                      onValueChange={(value) =>
                        handleEndEffectorChange("yaw", value)
                      }
                      min={-180}
                      max={180}
                      step={1}
                      disabled={isEmergencyStop}
                    />
                  </div>
                </div>
              </div>
            </div>

            <Separator className="my-6" />

            <div className="flex gap-2">
              <Button disabled={isEmergencyStop} className="gap-2">
                <Target className="h-4 w-4" />
                Move to Target
              </Button>
              <Button variant="outline" disabled={isEmergencyStop}>
                Plan Path
              </Button>
            </div>
          </Card>
        </TabsContent>

        {/* Gripper Control */}
        <TabsContent value="gripper" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">
              {t("arm.gripper")} Control
            </h3>

            <div className="space-y-6">
              <div>
                <label className="text-sm font-medium mb-2 block">
                  Gripper Position: {gripper.position[0]}%
                </label>
                <Slider
                  value={gripper.position}
                  onValueChange={(value) =>
                    setGripper((prev) => ({ ...prev, position: value }))
                  }
                  min={0}
                  max={100}
                  step={1}
                  disabled={isEmergencyStop}
                />
                <div className="flex justify-between text-xs text-muted-foreground mt-1">
                  <span>Closed</span>
                  <span>Open</span>
                </div>
              </div>

              <div>
                <label className="text-sm font-medium mb-2 block">
                  Grip Force: {gripper.force[0]}%
                </label>
                <Slider
                  value={gripper.force}
                  onValueChange={(value) =>
                    setGripper((prev) => ({ ...prev, force: value }))
                  }
                  min={0}
                  max={100}
                  step={1}
                  disabled={isEmergencyStop}
                />
              </div>

              <Separator />

              <div className="flex gap-2">
                <Button disabled={isEmergencyStop} className="gap-2">
                  <Grip className="h-4 w-4" />
                  Close Gripper
                </Button>
                <Button
                  variant="outline"
                  disabled={isEmergencyStop}
                  className="gap-2"
                >
                  <Hand className="h-4 w-4" />
                  Open Gripper
                </Button>
                <Button variant="outline" disabled={isEmergencyStop}>
                  Stop Gripper
                </Button>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Preset Positions */}
        <TabsContent value="presets" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">
              {t("arm.preset")} Positions
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
              {presetPositions.map((preset, index) => (
                <Card
                  key={index}
                  className="p-4 cursor-pointer hover:bg-accent transition-colors"
                >
                  <div className="space-y-3">
                    <div className="flex justify-between items-center">
                      <h4 className="font-medium">{preset.name}</h4>
                      <Badge variant="outline">Preset</Badge>
                    </div>

                    <div className="text-xs space-y-1">
                      {preset.joints.map((angle, jointIndex) => (
                        <div key={jointIndex} className="flex justify-between">
                          <span>J{jointIndex + 1}:</span>
                          <span>{angle}°</span>
                        </div>
                      ))}
                    </div>

                    <Button
                      size="sm"
                      className="w-full"
                      disabled={isEmergencyStop}
                      onClick={() => goToPreset(preset)}
                    >
                      Go to Position
                    </Button>
                  </div>
                </Card>
              ))}
            </div>

            <Separator className="my-6" />

            <div className="flex gap-2">
              <Button variant="outline" disabled={isEmergencyStop}>
                Save Current Position
              </Button>
              <Button variant="outline">Load from File</Button>
              <Button variant="outline">Export Positions</Button>
            </div>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
};

export default RoboticArm;
