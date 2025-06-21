import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { Switch } from "@/components/ui/switch";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Circle,
  Settings,
  Zap,
  ArrowRight,
  RotateCw,
  Move,
  Navigation,
  Gauge,
} from "lucide-react";

interface WheelConfig {
  type: "diff_drive" | "ackermann" | "mecanum" | "swerve_drive";
  wheelCount: number;
  driveType: "pwm_both" | "pwm_dir";
  wheelDiameter: number;
  wheelBase: number;
  trackWidth: number;
  maxSpeed: number;
  gearRatio: number;
  encoderPPR: number;
  wheels: {
    [key: string]: {
      motorPin: number;
      directionPin?: number;
      encoderA: number;
      encoderB: number;
      enabled: boolean;
      inverted: boolean;
    };
  };
}

const WheelConfiguration = () => {
  const { t } = useLanguage();

  const [wheelConfig, setWheelConfig] = useState<WheelConfig>({
    type: "diff_drive",
    wheelCount: 4,
    driveType: "pwm_dir",
    wheelDiameter: 0.1,
    wheelBase: 0.5,
    trackWidth: 0.4,
    maxSpeed: 2.0,
    gearRatio: 50.0,
    encoderPPR: 1024,
    wheels: {
      front_left: {
        motorPin: 3,
        directionPin: 4,
        encoderA: 2,
        encoderB: 5,
        enabled: true,
        inverted: false,
      },
      front_right: {
        motorPin: 6,
        directionPin: 7,
        encoderA: 8,
        encoderB: 9,
        enabled: true,
        inverted: true,
      },
      rear_left: {
        motorPin: 10,
        directionPin: 11,
        encoderA: 12,
        encoderB: 13,
        enabled: true,
        inverted: false,
      },
      rear_right: {
        motorPin: 14,
        directionPin: 15,
        encoderA: 16,
        encoderB: 17,
        enabled: true,
        inverted: true,
      },
    },
  });

  const driveTypeDescriptions = {
    diff_drive: {
      name: "Differential Drive",
      description: "Two independently driven wheels with caster wheels",
      icon: ArrowRight,
      wheels: ["left", "right"],
      features: ["Simple control", "Good maneuverability", "Cost effective"],
    },
    ackermann: {
      name: "Ackermann Steering",
      description: "Front wheel steering with rear wheel drive",
      icon: Navigation,
      wheels: ["front_left", "front_right", "rear_left", "rear_right"],
      features: ["Car-like steering", "Stable at speed", "Natural turning"],
    },
    mecanum: {
      name: "Mecanum Wheels",
      description: "Four mecanum wheels for omnidirectional movement",
      icon: Move,
      wheels: ["front_left", "front_right", "rear_left", "rear_right"],
      features: ["Omnidirectional", "Zero turn radius", "Complex control"],
    },
    swerve_drive: {
      name: "Swerve Drive",
      description: "Independent steering and drive for each wheel",
      icon: RotateCw,
      wheels: ["front_left", "front_right", "rear_left", "rear_right"],
      features: [
        "Full omnidirectional",
        "Independent wheel control",
        "Advanced",
      ],
    },
  };

  const updateWheelConfig = (field: keyof WheelConfig, value: any) => {
    setWheelConfig((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const updateWheelPin = (
    wheelName: string,
    pinType: string,
    value: number,
  ) => {
    setWheelConfig((prev) => ({
      ...prev,
      wheels: {
        ...prev.wheels,
        [wheelName]: {
          ...prev.wheels[wheelName],
          [pinType]: value,
        },
      },
    }));
  };

  const toggleWheel = (wheelName: string, enabled: boolean) => {
    setWheelConfig((prev) => ({
      ...prev,
      wheels: {
        ...prev.wheels,
        [wheelName]: {
          ...prev.wheels[wheelName],
          enabled,
        },
      },
    }));
  };

  const getWheelNames = () => {
    const config = driveTypeDescriptions[wheelConfig.type];
    return config.wheels;
  };

  const generateWheelConfiguration = () => {
    const wheelNames = getWheelNames();
    const newWheels: any = {};

    wheelNames.forEach((name, index) => {
      newWheels[name] = {
        motorPin: 3 + index * 4,
        directionPin:
          wheelConfig.driveType === "pwm_dir" ? 4 + index * 4 : undefined,
        encoderA: 2 + index * 4,
        encoderB: 5 + index * 4,
        enabled: true,
        inverted: name.includes("right"),
      };
    });

    setWheelConfig((prev) => ({
      ...prev,
      wheels: newWheels,
    }));
  };

  const currentDriveType = driveTypeDescriptions[wheelConfig.type];
  const IconComponent = currentDriveType.icon;

  return (
    <div className="space-y-6">
      {/* Drive Type Selection */}
      <Card className="p-6">
        <h3 className="text-lg font-light mb-4 flex items-center gap-2">
          <Circle className="h-5 w-5 text-primary" />
          Wheel Drive Configuration
        </h3>

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Drive Type Selection */}
          <div className="space-y-4">
            <div>
              <Label className="text-sm font-medium">Drive Type</Label>
              <Select
                value={wheelConfig.type}
                onValueChange={(value: any) => {
                  updateWheelConfig("type", value);
                  setTimeout(generateWheelConfiguration, 100);
                }}
              >
                <SelectTrigger className="mt-1">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  {Object.entries(driveTypeDescriptions).map(
                    ([key, config]) => (
                      <SelectItem key={key} value={key}>
                        <div className="flex items-center gap-2">
                          <config.icon className="h-4 w-4" />
                          {config.name}
                        </div>
                      </SelectItem>
                    ),
                  )}
                </SelectContent>
              </Select>
            </div>

            <div>
              <Label className="text-sm font-medium">Motor Control Type</Label>
              <Select
                value={wheelConfig.driveType}
                onValueChange={(value: any) => {
                  updateWheelConfig("driveType", value);
                  generateWheelConfiguration();
                }}
              >
                <SelectTrigger className="mt-1">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="pwm_dir">
                    <div className="flex flex-col">
                      <span>PWM + Direction</span>
                      <span className="text-xs text-muted-foreground">
                        Separate PWM and direction pins
                      </span>
                    </div>
                  </SelectItem>
                  <SelectItem value="pwm_both">
                    <div className="flex flex-col">
                      <span>Dual PWM</span>
                      <span className="text-xs text-muted-foreground">
                        Forward and reverse PWM pins
                      </span>
                    </div>
                  </SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div>
              <Label className="text-sm font-medium">Number of Wheels</Label>
              <Select
                value={wheelConfig.wheelCount.toString()}
                onValueChange={(value) => {
                  updateWheelConfig("wheelCount", parseInt(value));
                  generateWheelConfiguration();
                }}
              >
                <SelectTrigger className="mt-1">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="2">2 Wheels</SelectItem>
                  <SelectItem value="4">4 Wheels</SelectItem>
                  <SelectItem value="6">6 Wheels</SelectItem>
                  <SelectItem value="8">8 Wheels</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </div>

          {/* Drive Type Info */}
          <div className="space-y-4">
            <div className="p-4 border rounded-lg bg-gradient-to-br from-background to-primary/5">
              <div className="flex items-center gap-3 mb-3">
                <div className="p-2 rounded-lg bg-primary/10">
                  <IconComponent className="h-5 w-5 text-primary" />
                </div>
                <div>
                  <h4 className="font-medium">{currentDriveType.name}</h4>
                  <p className="text-sm text-muted-foreground">
                    {currentDriveType.description}
                  </p>
                </div>
              </div>

              <div className="space-y-2">
                <Label className="text-xs font-medium">Features:</Label>
                <div className="flex flex-wrap gap-1">
                  {currentDriveType.features.map((feature) => (
                    <Badge
                      key={feature}
                      variant="secondary"
                      className="text-xs"
                    >
                      {feature}
                    </Badge>
                  ))}
                </div>
              </div>

              <div className="mt-3">
                <Label className="text-xs font-medium">Wheels:</Label>
                <div className="flex flex-wrap gap-1 mt-1">
                  {getWheelNames().map((wheel) => (
                    <Badge key={wheel} variant="outline" className="text-xs">
                      {wheel.replace("_", " ")}
                    </Badge>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </div>
      </Card>

      {/* Physical Parameters */}
      <Card className="p-6">
        <h3 className="text-lg font-light mb-4 flex items-center gap-2">
          <Gauge className="h-5 w-5 text-primary" />
          Physical Parameters
        </h3>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          <div>
            <Label className="text-sm">Wheel Diameter (m)</Label>
            <Input
              type="number"
              step="0.001"
              value={wheelConfig.wheelDiameter}
              onChange={(e) =>
                updateWheelConfig("wheelDiameter", parseFloat(e.target.value))
              }
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">Wheel Base (m)</Label>
            <Input
              type="number"
              step="0.001"
              value={wheelConfig.wheelBase}
              onChange={(e) =>
                updateWheelConfig("wheelBase", parseFloat(e.target.value))
              }
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">Track Width (m)</Label>
            <Input
              type="number"
              step="0.001"
              value={wheelConfig.trackWidth}
              onChange={(e) =>
                updateWheelConfig("trackWidth", parseFloat(e.target.value))
              }
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">Max Speed (m/s)</Label>
            <Input
              type="number"
              step="0.1"
              value={wheelConfig.maxSpeed}
              onChange={(e) =>
                updateWheelConfig("maxSpeed", parseFloat(e.target.value))
              }
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">Gear Ratio</Label>
            <Input
              type="number"
              step="0.1"
              value={wheelConfig.gearRatio}
              onChange={(e) =>
                updateWheelConfig("gearRatio", parseFloat(e.target.value))
              }
              className="mt-1"
            />
          </div>

          <div>
            <Label className="text-sm">Encoder PPR</Label>
            <Input
              type="number"
              value={wheelConfig.encoderPPR}
              onChange={(e) =>
                updateWheelConfig("encoderPPR", parseInt(e.target.value))
              }
              className="mt-1"
            />
          </div>
        </div>
      </Card>

      {/* Individual Wheel Configuration */}
      <Card className="p-6">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-light flex items-center gap-2">
            <Settings className="h-5 w-5 text-primary" />
            Individual Wheel Configuration
          </h3>
          <Button
            onClick={generateWheelConfiguration}
            variant="outline"
            size="sm"
            className="gap-2"
          >
            <Zap className="h-4 w-4" />
            Auto Generate
          </Button>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-4">
          {getWheelNames().map((wheelName) => {
            const wheel = wheelConfig.wheels[wheelName];
            if (!wheel) return null;

            return (
              <div
                key={wheelName}
                className={`p-4 border rounded-lg space-y-3 transition-all ${
                  wheel.enabled
                    ? "bg-gradient-to-br from-background to-primary/5"
                    : "bg-muted/20"
                }`}
              >
                <div className="flex items-center justify-between">
                  <Label className="font-medium capitalize">
                    {wheelName.replace("_", " ")}
                  </Label>
                  <Switch
                    checked={wheel.enabled}
                    onCheckedChange={(enabled) =>
                      toggleWheel(wheelName, enabled)
                    }
                  />
                </div>

                <div className="space-y-2">
                  <div>
                    <Label className="text-xs text-muted-foreground">
                      Motor Pin
                    </Label>
                    <Input
                      type="number"
                      value={wheel.motorPin}
                      onChange={(e) =>
                        updateWheelPin(
                          wheelName,
                          "motorPin",
                          parseInt(e.target.value),
                        )
                      }
                      disabled={!wheel.enabled}
                      className="h-8 text-xs"
                    />
                  </div>

                  {wheelConfig.driveType === "pwm_dir" && (
                    <div>
                      <Label className="text-xs text-muted-foreground">
                        Direction Pin
                      </Label>
                      <Input
                        type="number"
                        value={wheel.directionPin || 0}
                        onChange={(e) =>
                          updateWheelPin(
                            wheelName,
                            "directionPin",
                            parseInt(e.target.value),
                          )
                        }
                        disabled={!wheel.enabled}
                        className="h-8 text-xs"
                      />
                    </div>
                  )}

                  <div className="grid grid-cols-2 gap-2">
                    <div>
                      <Label className="text-xs text-muted-foreground">
                        Encoder A
                      </Label>
                      <Input
                        type="number"
                        value={wheel.encoderA}
                        onChange={(e) =>
                          updateWheelPin(
                            wheelName,
                            "encoderA",
                            parseInt(e.target.value),
                          )
                        }
                        disabled={!wheel.enabled}
                        className="h-8 text-xs"
                      />
                    </div>
                    <div>
                      <Label className="text-xs text-muted-foreground">
                        Encoder B
                      </Label>
                      <Input
                        type="number"
                        value={wheel.encoderB}
                        onChange={(e) =>
                          updateWheelPin(
                            wheelName,
                            "encoderB",
                            parseInt(e.target.value),
                          )
                        }
                        disabled={!wheel.enabled}
                        className="h-8 text-xs"
                      />
                    </div>
                  </div>

                  <div className="flex items-center justify-between">
                    <Label className="text-xs text-muted-foreground">
                      Inverted
                    </Label>
                    <Switch
                      checked={wheel.inverted}
                      onCheckedChange={(inverted) =>
                        setWheelConfig((prev) => ({
                          ...prev,
                          wheels: {
                            ...prev.wheels,
                            [wheelName]: { ...wheel, inverted },
                          },
                        }))
                      }
                      disabled={!wheel.enabled}
                    />
                  </div>
                </div>

                <Badge
                  variant={wheel.enabled ? "default" : "secondary"}
                  className="w-full justify-center text-xs"
                >
                  {wheel.enabled ? "Active" : "Disabled"}
                </Badge>
              </div>
            );
          })}
        </div>
      </Card>

      {/* Configuration Summary */}
      <Card className="p-6 bg-gradient-to-br from-primary/5 to-primary/10">
        <h3 className="text-lg font-light mb-4">Configuration Summary</h3>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Drive Type:</span>
              <Badge variant="outline">{currentDriveType.name}</Badge>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Control Type:</span>
              <Badge variant="outline">
                {wheelConfig.driveType === "pwm_dir" ? "PWM + DIR" : "Dual PWM"}
              </Badge>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Active Wheels:</span>
              <Badge variant="outline">
                {
                  Object.values(wheelConfig.wheels).filter((w) => w.enabled)
                    .length
                }
              </Badge>
            </div>
          </div>

          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Max Speed:</span>
              <span className="font-mono">{wheelConfig.maxSpeed} m/s</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Wheel Diameter:</span>
              <span className="font-mono">{wheelConfig.wheelDiameter} m</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Track Width:</span>
              <span className="font-mono">{wheelConfig.trackWidth} m</span>
            </div>
          </div>
        </div>
      </Card>
    </div>
  );
};

export default WheelConfiguration;
