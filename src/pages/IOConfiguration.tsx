import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Switch } from "@/components/ui/switch";
import { ScrollArea } from "@/components/ui/scroll-area";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Settings,
  Zap,
  Cpu,
  Circle,
  Activity,
  Compass,
  Save,
  RefreshCw,
  Download,
  Upload,
  Gauge,
  Camera,
  Radar,
  Thermometer,
  AlertTriangle,
  CheckCircle,
} from "lucide-react";
import WheelConfiguration from "@/components/ros/WheelConfiguration";

const IOConfiguration = () => {
  const { t } = useLanguage();

  // Enhanced pin configurations with more categories
  const [pinConfigs, setPinConfigs] = useState({
    locomotion: {
      description: "Wheel Motors and Drive System",
      icon: Circle,
      pins: {
        front_left_pwm: {
          pin: 3,
          mode: "PWM",
          enabled: true,
          voltage: "12V",
          current: "2.5A",
        },
        front_left_dir: {
          pin: 4,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.1A",
        },
        front_right_pwm: {
          pin: 5,
          mode: "PWM",
          enabled: true,
          voltage: "12V",
          current: "2.5A",
        },
        front_right_dir: {
          pin: 6,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.1A",
        },
        rear_left_pwm: {
          pin: 9,
          mode: "PWM",
          enabled: true,
          voltage: "12V",
          current: "2.5A",
        },
        rear_left_dir: {
          pin: 10,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.1A",
        },
        rear_right_pwm: {
          pin: 11,
          mode: "PWM",
          enabled: true,
          voltage: "12V",
          current: "2.5A",
        },
        rear_right_dir: {
          pin: 12,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.1A",
        },
      },
    },
    manipulation: {
      description: "Robotic Arm and End Effector",
      icon: Activity,
      pins: {
        base_pwm: {
          pin: 13,
          mode: "PWM",
          enabled: true,
          voltage: "24V",
          current: "1.8A",
        },
        base_dir: {
          pin: 14,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.1A",
        },
        shoulder_pwm: {
          pin: 15,
          mode: "PWM",
          enabled: true,
          voltage: "24V",
          current: "1.5A",
        },
        shoulder_dir: {
          pin: 16,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.1A",
        },
        elbow_pwm: {
          pin: 17,
          mode: "PWM",
          enabled: true,
          voltage: "24V",
          current: "1.2A",
        },
        elbow_dir: {
          pin: 18,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.1A",
        },
        wrist_pwm: {
          pin: 19,
          mode: "PWM",
          enabled: true,
          voltage: "12V",
          current: "0.8A",
        },
        gripper_pwm: {
          pin: 20,
          mode: "PWM",
          enabled: true,
          voltage: "12V",
          current: "1.0A",
        },
        gripper_sensor: {
          pin: 21,
          mode: "ADC",
          enabled: true,
          voltage: "3.3V",
          current: "0.05A",
        },
      },
    },
    navigation_sensors: {
      description: "Position and Orientation Sensors",
      icon: Compass,
      pins: {
        imu_sda: {
          pin: 22,
          mode: "I2C",
          enabled: true,
          voltage: "3.3V",
          current: "0.02A",
        },
        imu_scl: {
          pin: 23,
          mode: "I2C",
          enabled: true,
          voltage: "3.3V",
          current: "0.02A",
        },
        gps_tx: {
          pin: 24,
          mode: "UART",
          enabled: true,
          voltage: "3.3V",
          current: "0.03A",
        },
        gps_rx: {
          pin: 25,
          mode: "UART",
          enabled: true,
          voltage: "3.3V",
          current: "0.03A",
        },
        magnetometer_sda: {
          pin: 26,
          mode: "I2C",
          enabled: false,
          voltage: "3.3V",
          current: "0.01A",
        },
        magnetometer_scl: {
          pin: 27,
          mode: "I2C",
          enabled: false,
          voltage: "3.3V",
          current: "0.01A",
        },
      },
    },
    proximity_sensors: {
      description: "Distance and Obstacle Detection",
      icon: Radar,
      pins: {
        lidar_tx: {
          pin: 28,
          mode: "UART",
          enabled: true,
          voltage: "5V",
          current: "0.8A",
        },
        lidar_rx: {
          pin: 29,
          mode: "UART",
          enabled: true,
          voltage: "5V",
          current: "0.8A",
        },
        ultrasonic_trig: {
          pin: 30,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.02A",
        },
        ultrasonic_echo: {
          pin: 31,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.02A",
        },
        front_ir: {
          pin: 32,
          mode: "ADC",
          enabled: true,
          voltage: "5V",
          current: "0.05A",
        },
        rear_ir: {
          pin: 33,
          mode: "ADC",
          enabled: true,
          voltage: "5V",
          current: "0.05A",
        },
      },
    },
    vision_system: {
      description: "Cameras and Vision Processing",
      icon: Camera,
      pins: {
        camera_power: {
          pin: 34,
          mode: "GPIO",
          enabled: true,
          voltage: "5V",
          current: "0.5A",
        },
        camera_i2c_sda: {
          pin: 35,
          mode: "I2C",
          enabled: true,
          voltage: "3.3V",
          current: "0.1A",
        },
        camera_i2c_scl: {
          pin: 36,
          mode: "I2C",
          enabled: true,
          voltage: "3.3V",
          current: "0.1A",
        },
        depth_camera_usb: {
          pin: 37,
          mode: "USB",
          enabled: false,
          voltage: "5V",
          current: "1.2A",
        },
        led_ring_pwm: {
          pin: 38,
          mode: "PWM",
          enabled: true,
          voltage: "12V",
          current: "0.3A",
        },
      },
    },
    environmental: {
      description: "Environmental Monitoring",
      icon: Thermometer,
      pins: {
        temp_sensor: {
          pin: 39,
          mode: "ADC",
          enabled: true,
          voltage: "3.3V",
          current: "0.01A",
        },
        humidity_sensor: {
          pin: 40,
          mode: "ADC",
          enabled: true,
          voltage: "3.3V",
          current: "0.01A",
        },
        pressure_sda: {
          pin: 41,
          mode: "I2C",
          enabled: false,
          voltage: "3.3V",
          current: "0.02A",
        },
        pressure_scl: {
          pin: 42,
          mode: "I2C",
          enabled: false,
          voltage: "3.3V",
          current: "0.02A",
        },
        gas_sensor: {
          pin: 43,
          mode: "ADC",
          enabled: false,
          voltage: "5V",
          current: "0.15A",
        },
      },
    },
    feedback_encoders: {
      description: "Position Feedback Systems",
      icon: Gauge,
      pins: {
        wheel_encoder_a: {
          pin: 44,
          mode: "ENCODER",
          enabled: true,
          voltage: "5V",
          current: "0.02A",
        },
        wheel_encoder_b: {
          pin: 45,
          mode: "ENCODER",
          enabled: true,
          voltage: "5V",
          current: "0.02A",
        },
        arm_encoder_a: {
          pin: 46,
          mode: "ENCODER",
          enabled: true,
          voltage: "5V",
          current: "0.02A",
        },
        arm_encoder_b: {
          pin: 47,
          mode: "ENCODER",
          enabled: true,
          voltage: "5V",
          current: "0.02A",
        },
        linear_encoder: {
          pin: 48,
          mode: "ADC",
          enabled: false,
          voltage: "5V",
          current: "0.05A",
        },
      },
    },
    power_management: {
      description: "Power Control and Monitoring",
      icon: Zap,
      pins: {
        main_power_sense: {
          pin: 49,
          mode: "ADC",
          enabled: true,
          voltage: "3.3V",
          current: "0.01A",
        },
        battery_voltage: {
          pin: 50,
          mode: "ADC",
          enabled: true,
          voltage: "3.3V",
          current: "0.01A",
        },
        current_sensor: {
          pin: 51,
          mode: "ADC",
          enabled: true,
          voltage: "5V",
          current: "0.02A",
        },
        emergency_stop: {
          pin: 52,
          mode: "GPIO",
          enabled: true,
          voltage: "24V",
          current: "0.1A",
        },
        power_relay: {
          pin: 53,
          mode: "GPIO",
          enabled: true,
          voltage: "12V",
          current: "0.2A",
        },
      },
    },
  });

  // PID parameters
  const [pidParams, setPidParams] = useState({
    wheels: {
      front_left: { kp: 2.5, ki: 0.1, kd: 0.05, max_output: 255 },
      front_right: { kp: 2.5, ki: 0.1, kd: 0.05, max_output: 255 },
      rear_left: { kp: 2.5, ki: 0.1, kd: 0.05, max_output: 255 },
      rear_right: { kp: 2.5, ki: 0.1, kd: 0.05, max_output: 255 },
    },
    arms: {
      base: { kp: 1.0, ki: 0.05, kd: 0.02, max_output: 180 },
      shoulder: { kp: 1.2, ki: 0.08, kd: 0.03, max_output: 180 },
      elbow: { kp: 1.1, ki: 0.06, kd: 0.025, max_output: 180 },
      wrist: { kp: 0.8, ki: 0.04, kd: 0.015, max_output: 180 },
    },
  });

  // Kinematics parameters
  const [kinematicsParams, setKinematicsParams] = useState({
    wheels: {
      wheel_radius: 0.1,
      wheel_base: 0.5,
      track_width: 0.4,
      gear_ratio: 50.0,
      max_rpm: 300,
      encoder_ppr: 1024,
    },
    arms: {
      base_length: 0.0,
      shoulder_length: 0.3,
      elbow_length: 0.25,
      wrist_length: 0.15,
      max_reach: 0.7,
      joint_limits: {
        base: [-180, 180],
        shoulder: [-90, 90],
        elbow: [-150, 150],
      },
    },
  });

  // IMU configuration
  const [imuConfig, setImuConfig] = useState({
    sensor_type: "MPU6050",
    sample_rate: 100,
    accel_range: 2, // ±2g
    gyro_range: 250, // ±250°/s
    filter_type: "low_pass",
    filter_frequency: 10,
    calibration: {
      accel_offset: [0, 0, 0],
      gyro_offset: [0, 0, 0],
      mag_offset: [0, 0, 0],
    },
  });

  const updatePinConfig = (
    category: string,
    pinName: string,
    field: string,
    value: any,
  ) => {
    setPinConfigs((prev) => ({
      ...prev,
      [category]: {
        ...prev[category as keyof typeof prev],
        [pinName]: {
          ...prev[category as keyof typeof prev][
            pinName as keyof (typeof prev)[typeof category]
          ],
          [field]: value,
        },
      },
    }));
  };

  const updatePidParam = (
    category: string,
    controller: string,
    param: string,
    value: number,
  ) => {
    setPidParams((prev) => ({
      ...prev,
      [category]: {
        ...prev[category as keyof typeof prev],
        [controller]: {
          ...prev[category as keyof typeof prev][
            controller as keyof (typeof prev)[typeof category]
          ],
          [param]: value,
        },
      },
    }));
  };

  const saveConfiguration = () => {
    console.log("Saving configuration...");
    // Implementation for saving to ROS parameters
  };

  const loadConfiguration = () => {
    console.log("Loading configuration...");
    // Implementation for loading from ROS parameters
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      <div className="space-y-2">
        <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
          <div className="p-2 rounded-lg bg-primary/10 float-animation">
            <Settings className="h-6 w-6 md:h-8 w-8 text-primary" />
          </div>
          {t("io.title")}
        </h1>
        <p className="text-muted-foreground text-sm md:text-base">
          Configure hardware pins, PID parameters, and kinematics for all robot
          components with organized functional categories
        </p>
      </div>

      {/* Action Buttons */}
      <div className="flex flex-col sm:flex-row gap-2">
        <Button onClick={saveConfiguration} className="gap-2 hover-lift">
          <Save className="h-4 w-4" />
          {t("common.save")} Configuration
        </Button>
        <Button variant="outline" onClick={loadConfiguration} className="gap-2">
          <RefreshCw className="h-4 w-4" />
          Load from ROS
        </Button>
        <Button variant="outline" className="gap-2">
          <Download className="h-4 w-4" />
          Export
        </Button>
        <Button variant="outline" className="gap-2">
          <Upload className="h-4 w-4" />
          Import
        </Button>
      </div>

      <Tabs defaultValue="wheels" className="space-y-4">
        <TabsList className="grid w-full grid-cols-2 lg:grid-cols-5">
          <TabsTrigger
            value="wheels"
            className="gap-1 md:gap-2 text-xs md:text-sm"
          >
            <Circle className="h-3 w-3 md:h-4 md:w-4" />
            <span className="hidden sm:inline">Wheels</span>
            <span className="sm:hidden">Wheels</span>
          </TabsTrigger>
          <TabsTrigger
            value="pins"
            className="gap-1 md:gap-2 text-xs md:text-sm"
          >
            <Zap className="h-3 w-3 md:h-4 md:w-4" />
            <span className="hidden sm:inline">{t("io.pins")}</span>
            <span className="sm:hidden">Pins</span>
          </TabsTrigger>
          <TabsTrigger
            value="pid"
            className="gap-1 md:gap-2 text-xs md:text-sm"
          >
            <Settings className="h-3 w-3 md:h-4 md:w-4" />
            <span className="hidden sm:inline">{t("io.pid")}</span>
            <span className="sm:hidden">PID</span>
          </TabsTrigger>
          <TabsTrigger
            value="kinematics"
            className="gap-1 md:gap-2 text-xs md:text-sm"
          >
            <Cpu className="h-3 w-3 md:h-4 md:w-4" />
            <span className="hidden sm:inline">{t("io.kinematics")}</span>
            <span className="sm:hidden">Kinematics</span>
          </TabsTrigger>
          <TabsTrigger
            value="imu"
            className="gap-1 md:gap-2 text-xs md:text-sm"
          >
            <Compass className="h-3 w-3 md:h-4 md:w-4" />
            <span className="hidden sm:inline">{t("io.imu")}</span>
            <span className="sm:hidden">IMU</span>
          </TabsTrigger>
        </TabsList>

        {/* Wheel Configuration */}
        <TabsContent value="wheels" className="space-y-4">
          <WheelConfiguration />
        </TabsContent>

        {/* Pin Configuration */}
        <TabsContent value="pins" className="space-y-4">
          {/* USB Assignment Section */}
          <Card className="p-4 bg-gradient-to-br from-blue-50 to-blue-100 dark:from-blue-950/30 dark:to-blue-900/30">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Zap className="h-5 w-5 text-blue-500" />
              USB Port Assignments
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
              {Object.entries(pinConfigs).map(([category, categoryData]) => {
                const IconComponent = categoryData.icon;
                return (
                  <div
                    key={category}
                    className="p-3 bg-white/80 dark:bg-gray-800/80 rounded-lg border"
                  >
                    <div className="flex items-center gap-2 mb-2">
                      <IconComponent className="h-4 w-4 text-primary" />
                      <span className="font-medium text-sm capitalize">
                        {category.replace(/_/g, " ")}
                      </span>
                    </div>
                    <Select defaultValue="/dev/ttyUSB0">
                      <SelectTrigger className="h-8 text-xs">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="/dev/ttyUSB0">USB Port 0</SelectItem>
                        <SelectItem value="/dev/ttyUSB1">USB Port 1</SelectItem>
                        <SelectItem value="/dev/ttyUSB2">USB Port 2</SelectItem>
                        <SelectItem value="/dev/ttyACM0">ACM Port 0</SelectItem>
                        <SelectItem value="/dev/ttyACM1">ACM Port 1</SelectItem>
                        <SelectItem value="wifi:192.168.1.100">
                          WiFi Device
                        </SelectItem>
                        <SelectItem value="ethernet:192.168.1.101">
                          Ethernet
                        </SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                );
              })}
            </div>
          </Card>

          <ScrollArea className="h-[700px]">
            <div className="space-y-4">
              {Object.entries(pinConfigs).map(
                ([category, categoryData], categoryIndex) => {
                  const IconComponent = categoryData.icon;
                  const enabledPins = Object.values(categoryData.pins).filter(
                    (pin) => pin.enabled,
                  ).length;
                  const totalPins = Object.keys(categoryData.pins).length;

                  return (
                    <Card
                      key={category}
                      className={`p-4 md:p-6 hover-lift stagger-item`}
                      style={{ animationDelay: `${categoryIndex * 0.1}s` }}
                    >
                      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between mb-4 gap-2">
                        <div className="flex items-center gap-3">
                          <div className="p-2 rounded-lg bg-primary/10">
                            <IconComponent className="h-5 w-5 text-primary" />
                          </div>
                          <div>
                            <h3 className="text-lg font-light capitalize">
                              {category.replace(/_/g, " ")}
                            </h3>
                            <p className="text-xs text-muted-foreground">
                              {categoryData.description}
                            </p>
                          </div>
                        </div>
                        <div className="flex items-center gap-2">
                          <Badge variant="outline" className="text-xs">
                            {enabledPins}/{totalPins} Active
                          </Badge>
                          {enabledPins === totalPins ? (
                            <CheckCircle className="h-4 w-4 text-green-500" />
                          ) : enabledPins > 0 ? (
                            <AlertTriangle className="h-4 w-4 text-yellow-500" />
                          ) : (
                            <Circle className="h-4 w-4 text-muted-foreground" />
                          )}
                        </div>
                      </div>

                      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-4">
                        {Object.entries(categoryData.pins).map(
                          ([pinName, config], pinIndex) => (
                            <div
                              key={pinName}
                              className={`p-3 border rounded-lg space-y-3 transition-all hover:shadow-md ${
                                config.enabled
                                  ? "bg-gradient-to-br from-background to-primary/5"
                                  : "bg-muted/20"
                              }`}
                            >
                              <div className="flex items-center justify-between">
                                <Label className="font-medium text-sm capitalize">
                                  {pinName.replace(/_/g, " ")}
                                </Label>
                                <Switch
                                  checked={config.enabled}
                                  onCheckedChange={(enabled) =>
                                    updatePinConfig(
                                      category,
                                      pinName,
                                      "enabled",
                                      enabled,
                                    )
                                  }
                                />
                              </div>

                              <div className="space-y-2">
                                <div>
                                  <Label className="text-xs text-muted-foreground">
                                    Pin Number
                                  </Label>
                                  <Input
                                    type="number"
                                    value={config.pin}
                                    onChange={(e) =>
                                      updatePinConfig(
                                        category,
                                        pinName,
                                        "pin",
                                        parseInt(e.target.value),
                                      )
                                    }
                                    disabled={!config.enabled}
                                    className="h-8 text-xs font-mono"
                                  />
                                </div>

                                <div>
                                  <Label className="text-xs text-muted-foreground">
                                    Mode
                                  </Label>
                                  <Select
                                    value={config.mode}
                                    onValueChange={(value) =>
                                      updatePinConfig(
                                        category,
                                        pinName,
                                        "mode",
                                        value,
                                      )
                                    }
                                    disabled={!config.enabled}
                                  >
                                    <SelectTrigger className="h-8 text-xs">
                                      <SelectValue />
                                    </SelectTrigger>
                                    <SelectContent>
                                      <SelectItem value="GPIO">GPIO</SelectItem>
                                      <SelectItem value="PWM">PWM</SelectItem>
                                      <SelectItem value="ADC">ADC</SelectItem>
                                      <SelectItem value="I2C">I2C</SelectItem>
                                      <SelectItem value="UART">UART</SelectItem>
                                      <SelectItem value="ENCODER">
                                        ENCODER
                                      </SelectItem>
                                      <SelectItem value="USB">USB</SelectItem>
                                    </SelectContent>
                                  </Select>
                                </div>

                                <div className="grid grid-cols-2 gap-2">
                                  <div>
                                    <Label className="text-xs text-muted-foreground">
                                      Voltage
                                    </Label>
                                    <div className="text-xs font-mono bg-muted px-2 py-1 rounded">
                                      {config.voltage}
                                    </div>
                                  </div>
                                  <div>
                                    <Label className="text-xs text-muted-foreground">
                                      Current
                                    </Label>
                                    <div className="text-xs font-mono bg-muted px-2 py-1 rounded">
                                      {config.current}
                                    </div>
                                  </div>
                                </div>
                              </div>

                              <div className="flex items-center justify-between">
                                <Badge
                                  variant={
                                    config.enabled ? "default" : "secondary"
                                  }
                                  className="text-xs"
                                >
                                  {config.enabled ? "Active" : "Disabled"}
                                </Badge>
                                <Badge variant="outline" className="text-xs">
                                  Pin {config.pin}
                                </Badge>
                              </div>
                            </div>
                          ),
                        )}
                      </div>
                    </Card>
                  );
                },
              )}
            </div>
          </ScrollArea>
        </TabsContent>

        {/* PID Parameters */}
        <TabsContent value="pid" className="space-y-4">
          {Object.entries(pidParams).map(([category, controllers]) => (
            <Card key={category} className="p-6">
              <h3 className="text-lg font-semibold mb-4 capitalize flex items-center gap-2">
                {category === "wheels" && <Circle className="h-5 w-5" />}
                {category === "arms" && <Activity className="h-5 w-5" />}
                {t(`io.${category}`)} PID Parameters
              </h3>

              <div className="space-y-4">
                {Object.entries(controllers).map(([controller, params]) => (
                  <div key={controller} className="p-4 border rounded-lg">
                    <h4 className="font-medium mb-3 capitalize">
                      {controller.replace(/_/g, " ")} Controller
                    </h4>

                    <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                      <div>
                        <Label className="text-xs">Kp (Proportional)</Label>
                        <Input
                          type="number"
                          step="0.01"
                          value={params.kp}
                          onChange={(e) =>
                            updatePidParam(
                              category,
                              controller,
                              "kp",
                              parseFloat(e.target.value),
                            )
                          }
                          className="h-8"
                        />
                      </div>

                      <div>
                        <Label className="text-xs">Ki (Integral)</Label>
                        <Input
                          type="number"
                          step="0.001"
                          value={params.ki}
                          onChange={(e) =>
                            updatePidParam(
                              category,
                              controller,
                              "ki",
                              parseFloat(e.target.value),
                            )
                          }
                          className="h-8"
                        />
                      </div>

                      <div>
                        <Label className="text-xs">Kd (Derivative)</Label>
                        <Input
                          type="number"
                          step="0.001"
                          value={params.kd}
                          onChange={(e) =>
                            updatePidParam(
                              category,
                              controller,
                              "kd",
                              parseFloat(e.target.value),
                            )
                          }
                          className="h-8"
                        />
                      </div>

                      <div>
                        <Label className="text-xs">Max Output</Label>
                        <Input
                          type="number"
                          value={params.max_output}
                          onChange={(e) =>
                            updatePidParam(
                              category,
                              controller,
                              "max_output",
                              parseInt(e.target.value),
                            )
                          }
                          className="h-8"
                        />
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </Card>
          ))}
        </TabsContent>

        {/* Kinematics Parameters */}
        <TabsContent value="kinematics" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {/* Wheel Kinematics */}
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                {t("io.wheels")} Kinematics
              </h3>

              <div className="space-y-4">
                <div>
                  <Label>Wheel Radius (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.wheels.wheel_radius}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        wheels: {
                          ...prev.wheels,
                          wheel_radius: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Wheel Base (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.wheels.wheel_base}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        wheels: {
                          ...prev.wheels,
                          wheel_base: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Track Width (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.wheels.track_width}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        wheels: {
                          ...prev.wheels,
                          track_width: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Gear Ratio</Label>
                  <Input
                    type="number"
                    step="0.1"
                    value={kinematicsParams.wheels.gear_ratio}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        wheels: {
                          ...prev.wheels,
                          gear_ratio: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Max RPM</Label>
                  <Input
                    type="number"
                    value={kinematicsParams.wheels.max_rpm}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        wheels: {
                          ...prev.wheels,
                          max_rpm: parseInt(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Encoder PPR</Label>
                  <Input
                    type="number"
                    value={kinematicsParams.wheels.encoder_ppr}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        wheels: {
                          ...prev.wheels,
                          encoder_ppr: parseInt(e.target.value),
                        },
                      }))
                    }
                  />
                </div>
              </div>
            </Card>

            {/* Arm Kinematics */}
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">
                {t("io.arms")} Kinematics
              </h3>

              <div className="space-y-4">
                <div>
                  <Label>Base Length (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.arms.base_length}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        arms: {
                          ...prev.arms,
                          base_length: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Shoulder Length (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.arms.shoulder_length}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        arms: {
                          ...prev.arms,
                          shoulder_length: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Elbow Length (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.arms.elbow_length}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        arms: {
                          ...prev.arms,
                          elbow_length: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Wrist Length (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.arms.wrist_length}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        arms: {
                          ...prev.arms,
                          wrist_length: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Max Reach (m)</Label>
                  <Input
                    type="number"
                    step="0.001"
                    value={kinematicsParams.arms.max_reach}
                    onChange={(e) =>
                      setKinematicsParams((prev) => ({
                        ...prev,
                        arms: {
                          ...prev.arms,
                          max_reach: parseFloat(e.target.value),
                        },
                      }))
                    }
                  />
                </div>

                <Separator />

                <div>
                  <Label className="text-sm font-semibold">
                    Joint Limits (degrees)
                  </Label>
                  <div className="space-y-2 mt-2">
                    {Object.entries(kinematicsParams.arms.joint_limits).map(
                      ([joint, limits]) => (
                        <div key={joint} className="flex items-center gap-2">
                          <Label className="w-20 capitalize">{joint}:</Label>
                          <Input
                            type="number"
                            value={limits[0]}
                            onChange={(e) => {
                              const newLimits = [...limits];
                              newLimits[0] = parseInt(e.target.value);
                              setKinematicsParams((prev) => ({
                                ...prev,
                                arms: {
                                  ...prev.arms,
                                  joint_limits: {
                                    ...prev.arms.joint_limits,
                                    [joint]: newLimits,
                                  },
                                },
                              }));
                            }}
                            className="h-8"
                          />
                          <span>to</span>
                          <Input
                            type="number"
                            value={limits[1]}
                            onChange={(e) => {
                              const newLimits = [...limits];
                              newLimits[1] = parseInt(e.target.value);
                              setKinematicsParams((prev) => ({
                                ...prev,
                                arms: {
                                  ...prev.arms,
                                  joint_limits: {
                                    ...prev.arms.joint_limits,
                                    [joint]: newLimits,
                                  },
                                },
                              }));
                            }}
                            className="h-8"
                          />
                        </div>
                      ),
                    )}
                  </div>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* IMU Configuration */}
        <TabsContent value="imu" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">
              {t("io.imu")} Configuration
            </h3>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>Sensor Type</Label>
                  <Select
                    value={imuConfig.sensor_type}
                    onValueChange={(value) =>
                      setImuConfig((prev) => ({ ...prev, sensor_type: value }))
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="MPU6050">MPU6050</SelectItem>
                      <SelectItem value="MPU9250">MPU9250</SelectItem>
                      <SelectItem value="BNO055">BNO055</SelectItem>
                      <SelectItem value="LSM9DS1">LSM9DS1</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Sample Rate (Hz)</Label>
                  <Input
                    type="number"
                    value={imuConfig.sample_rate}
                    onChange={(e) =>
                      setImuConfig((prev) => ({
                        ...prev,
                        sample_rate: parseInt(e.target.value),
                      }))
                    }
                  />
                </div>

                <div>
                  <Label>Accelerometer Range (±g)</Label>
                  <Select
                    value={imuConfig.accel_range.toString()}
                    onValueChange={(value) =>
                      setImuConfig((prev) => ({
                        ...prev,
                        accel_range: parseInt(value),
                      }))
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="2">±2g</SelectItem>
                      <SelectItem value="4">±4g</SelectItem>
                      <SelectItem value="8">±8g</SelectItem>
                      <SelectItem value="16">±16g</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Gyroscope Range (±°/s)</Label>
                  <Select
                    value={imuConfig.gyro_range.toString()}
                    onValueChange={(value) =>
                      setImuConfig((prev) => ({
                        ...prev,
                        gyro_range: parseInt(value),
                      }))
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="250">±250°/s</SelectItem>
                      <SelectItem value="500">±500°/s</SelectItem>
                      <SelectItem value="1000">±1000°/s</SelectItem>
                      <SelectItem value="2000">±2000°/s</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>Filter Type</Label>
                  <Select
                    value={imuConfig.filter_type}
                    onValueChange={(value) =>
                      setImuConfig((prev) => ({ ...prev, filter_type: value }))
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="low_pass">Low Pass</SelectItem>
                      <SelectItem value="high_pass">High Pass</SelectItem>
                      <SelectItem value="complementary">
                        Complementary
                      </SelectItem>
                      <SelectItem value="kalman">Kalman</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Filter Frequency (Hz)</Label>
                  <Input
                    type="number"
                    value={imuConfig.filter_frequency}
                    onChange={(e) =>
                      setImuConfig((prev) => ({
                        ...prev,
                        filter_frequency: parseInt(e.target.value),
                      }))
                    }
                  />
                </div>

                <Separator />

                <div>
                  <Label className="text-sm font-semibold">
                    Calibration Offsets
                  </Label>
                  <div className="space-y-2 mt-2">
                    <div>
                      <Label className="text-xs">Accelerometer [X, Y, Z]</Label>
                      <div className="flex gap-2">
                        {imuConfig.calibration.accel_offset.map(
                          (offset, index) => (
                            <Input
                              key={index}
                              type="number"
                              step="0.01"
                              value={offset}
                              onChange={(e) => {
                                const newOffsets = [
                                  ...imuConfig.calibration.accel_offset,
                                ];
                                newOffsets[index] = parseFloat(e.target.value);
                                setImuConfig((prev) => ({
                                  ...prev,
                                  calibration: {
                                    ...prev.calibration,
                                    accel_offset: newOffsets,
                                  },
                                }));
                              }}
                              className="h-8"
                            />
                          ),
                        )}
                      </div>
                    </div>

                    <div>
                      <Label className="text-xs">Gyroscope [X, Y, Z]</Label>
                      <div className="flex gap-2">
                        {imuConfig.calibration.gyro_offset.map(
                          (offset, index) => (
                            <Input
                              key={index}
                              type="number"
                              step="0.01"
                              value={offset}
                              onChange={(e) => {
                                const newOffsets = [
                                  ...imuConfig.calibration.gyro_offset,
                                ];
                                newOffsets[index] = parseFloat(e.target.value);
                                setImuConfig((prev) => ({
                                  ...prev,
                                  calibration: {
                                    ...prev.calibration,
                                    gyro_offset: newOffsets,
                                  },
                                }));
                              }}
                              className="h-8"
                            />
                          ),
                        )}
                      </div>
                    </div>
                  </div>
                </div>

                <Button variant="outline" className="w-full gap-2">
                  <RefreshCw className="h-4 w-4" />
                  Auto Calibrate IMU
                </Button>
              </div>
            </div>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
};

export default IOConfiguration;
