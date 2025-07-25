import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { Switch } from "@/components/ui/switch";
import { Slider } from "@/components/ui/slider";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { Task } from "./TaskCard";
import { ConditionalLogicEditor } from "./ConditionalLogicEditor";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Move3D,
  Settings,
  Eye,
  Camera,
  Compass,
  Zap,
  Brain,
  Shield,
  Mic,
  Hand,
  Wifi,
  Cloud,
  BarChart3,
  Wrench,
  Battery,
  MapPin,
  Timer,
  AlertTriangle,
  Code,
  Navigation,
  Target,
  RotateCcw,
  Plus,
} from "lucide-react";

interface TaskParameterEditorProps {
  task: Task;
  isOpen: boolean;
  onClose: () => void;
  onSave: (task: Task) => void;
}

export function TaskParameterEditor({
  task,
  isOpen,
  onClose,
  onSave,
}: TaskParameterEditorProps) {
  const { t } = useLanguage();
  const [editedTask, setEditedTask] = useState<Task>(task);
  const [obstacleRegions, setObstacleRegions] = useState<
    Array<{ x1: number; y1: number; x2: number; y2: number; name: string }>
  >([]);
  const [newRegion, setNewRegion] = useState({
    x1: 0,
    y1: 0,
    x2: 0,
    y2: 0,
    name: "",
  });
  const [movementPattern, setMovementPattern] = useState<
    Array<{ x: number; y: number; yaw: number; name: string }>
  >([]);
  const [newWaypoint, setNewWaypoint] = useState({
    x: 0,
    y: 0,
    yaw: 0,
    name: "",
  });

  const taskTypes = {
    movement: {
      icon: Move3D,
      name: "🚗 การเคลื่อนที่ของหุ่นยนต์",
      description:
        "ควบคุมตำแหน่งและการหมุนของหุ่นยนต์ด้วยโหมดการเคลื่อนที่หลากหลาย",
      parameters: {
        movementMode: "auto_nav", // auto_nav, move_to_position, manual_relative
        position: { x: 0, y: 0, z: 0 },
        orientation: { yaw: 0 }, // For 2D navigation robots (angular_z in radians, displayed as degrees)
        relativeDistance: { x: 2.1, y: 0, z: 0 },
        targetFrame: "map", // map, base_link, odom
        speed: 1.0,
        acceleration: 0.5,
        precision: 0.01,
        moveType: "linear", // linear, joint, circular, spline
        coordinateFrame: "base_link",
        pathPlanning: true,
        obstacleAvoidance: true,
        useCostmap: true,
        costmapLayer: "global_costmap",
        plannerType: "navfn", // navfn, global_planner, astar
        recoveryBehavior: true,
        goalTolerance: { xy: 0.1, yaw: 0.1 },
        // Advanced Obstacle Detection & Avoidance
        obstacleDetectionMode: "dynamic", // dynamic, static, predictive, none
        obstacleTypes: ["static", "dynamic"], // static, dynamic, human, vehicle, unknown
        obstacleRegions: [], // Array of obstacle positions/regions in costmap
        avoidanceStrategy: "circumnavigate", // circumnavigate, wait, alternative_route, stop
        safetyDistance: 0.5, // meters
        obstacleDetectionRange: 5.0, // meters
        // Costmap Configuration
        costmapSettings: {
          inflationRadius: 0.55,
          costScalingFactor: 10.0,
          obstacleLayer: true,
          inflationLayer: true,
          staticLayer: true,
          voxelLayer: false,
          socialCostmapLayer: false,
        },
        // Dynamic Obstacle Handling
        dynamicObstacleSettings: {
          predictiveWindow: 3.0, // seconds
          velocityThreshold: 0.1, // m/s
          trackingEnabled: true,
          predictionModel: "linear", // linear, polynomial, kalman
        },
        // Emergency Behaviors
        emergencyBehaviors: {
          emergencyStop: true,
          backupDistance: 1.0,
          alertOperator: true,
          logIncident: true,
        },
        // Conditional logic
        enableConditions: false,
        conditions: [],
        onSuccess: "continue",
        onFailure: "retry",
      },
    },
    move_with_obstacle_check: {
      icon: Shield,
      name: "🛡️ เคลื่อนที่พร้อมตรวจสิ่งกีดขวาง",
      description:
        "การเคลื่อนที่อัจฉริยะพร้อมการตรวจจับสิ่งกีดขวางแบบเรียลไทม์",
      parameters: {
        movementMode: "move_to_position", // move_to_position, manual_relative, pattern_movement
        // Single position movement
        position: { x: 0, y: 0 },
        orientation: { yaw: 0 }, // Only yaw for 2D movement (angular_z)
        // Manual relative movement
        relativeDistance: { x: 0, y: 0, yaw: 0 },
        // Pattern/Multiple waypoints movement
        enablePatternMovement: false,
        movementPattern: [], // Array of waypoints
        patternType: "waypoints", // waypoints, circular, square, custom
        patternRepeat: 1, // Number of times to repeat pattern
        waitAtWaypoint: 2.0, // seconds to wait at each waypoint
        waypointTolerance: 0.1, // meters
        speed: 1.0,
        acceleration: 0.5,
        // Enhanced Obstacle Detection
        obstacleCheckMode: "continuous", // continuous, periodic, trigger_based
        checkInterval: 0.1, // seconds for periodic checking
        obstacleRegions: [], // Predefined obstacle regions from costmap
        costmapSource: "global_costmap", // global_costmap, local_costmap, custom
        loadCostmapRegions: true,
        // Movement Conditions
        enableConditionalMovement: true,
        movementConditions: [
          {
            condition: "obstacle_ahead",
            action: "circumnavigate",
            distance: 2.0,
          },
          {
            condition: "path_blocked",
            action: "alternative_route",
            timeout: 5.0,
          },
          {
            condition: "dynamic_obstacle",
            action: "wait_and_retry",
            maxWait: 10.0,
          },
        ],
        // Safety Parameters
        safetyDistance: 0.8,
        emergencyStopDistance: 0.3,
        maxDetourDistance: 5.0,
        // Behavioral Settings
        aggressiveness: "moderate", // conservative, moderate, aggressive
        patienceLevel: 5.0, // seconds to wait for obstacles
        learningEnabled: true, // Learn from previous obstacle encounters
        adaptiveSpeed: true, // Adjust speed based on obstacle density
        // Success Criteria
        goalTolerance: { xy: 0.1, yaw: 5 }, // yaw in degrees
        onObstacleDetected: "smart_avoid",
        onPathBlocked: "find_alternative",
        onGoalReached: "continue",
      },
    },
    manipulation: {
      icon: Hand,
      name: "🦾 การควบคุมแขนกล",
      description: "ควบคุมการเคลื่อนไหวของแขนกลพร้อมตรรกะเงื่อนไข",
      parameters: {
        manipulationMode: "position_control", // position_control, joint_control, cartesian_path
        targetPosition: { x: 0.5, y: 0, z: 0.3 },
        targetOrientation: { roll: 0, pitch: 0, yaw: 0 },
        jointAngles: { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 },
        gripperAction: "none", // none, open, close, grasp, release
        force: 50, // percentage
        approachVector: { x: 0, y: 0, z: -0.1 },
        retractVector: { x: 0, y: 0, z: 0.1 },
        planningGroup: "arm",
        endEffectorFrame: "end_effector",
        velocityScaling: 0.5,
        accelerationScaling: 0.5,
        allowedPlanningTime: 5.0,
        // Conditional logic
        enableConditions: false,
        conditions: [],
        onSuccess: "continue",
        onFailure: "retry",
        forceThreshold: 10.0,
        checkCollision: true,
      },
    },
    vision: {
      icon: Eye,
      name: "👁️ การประมวลผลภาพ",
      description: "วิชั่นคอมพิวเตอร์และการประมวลผลภาพพร้อมตรรกะเงื่อนไข",
      parameters: {
        camera: "front_camera",
        processingType: "object_detection", // object_detection, qr_code, face_recognition, color_tracking, lane_detection
        targetObject: "",
        confidence: 0.8,
        saveImage: true,
        imageFormat: "jpg",
        resolution: "1920x1080",
        roi: { x: 0, y: 0, width: 100, height: 100 }, // region of interest
        preprocessImage: true,
        filterType: "blur", // blur, sharpen, edge_enhance, none
        lightingCompensation: true,
        // Conditional logic
        enableConditions: true,
        conditions: [
          { type: "if", condition: "object_detected", action: "save_image" },
          { type: "if", condition: "confidence > 0.9", action: "continue" },
          { type: "else", action: "retry" },
        ],
        onObjectDetected: "continue",
        onNoObject: "wait",
        maxDetectionAttempts: 3,
      },
    },
    sensor_reading: {
      icon: Compass,
      name: "📡 การอ่านเซ็นเซอร์",
      description:
        "อ่านและประมวลผลข้อมูลเซ็นเซอร์พร้อมการรองรับเซ็นเซอร์หลากหลาย",
      parameters: {
        sensorType: "lidar", // lidar, camera, imu, ultrasonic, temperature, pressure, humidity, gps, magnetometer, accelerometer, gyroscope, proximity, force, torque, encoders, current, voltage
        topic: "/scan",
        duration: 1.0,
        samplingRate: 10,
        filterType: "none", // none, low_pass, high_pass, kalman, median, gaussian
        threshold: 0.5,
        units: "meters", // meters, degrees, celsius, pascal, amperes, volts, rpm, newtons
        logData: true,
        dataFormat: "json", // json, csv, rosbag, binary
        processingMode: "real_time", // real_time, batch, triggered
        calibrationRequired: false,
        autoCalibrate: true,
        // Sensor-specific parameters
        rangeMin: 0.1,
        rangeMax: 10.0,
        fieldOfView: 270, // degrees for lidar/camera
        resolution: 0.01, // meters or degrees
        noiseFilter: true,
        outlierRemoval: true,
      },
    },
    ai_processing: {
      icon: Brain,
      name: "AI Processing",
      description: "Machine learning and AI tasks",
      parameters: {
        modelType: "classification", // classification, detection, segmentation, nlp
        modelPath: "/models/default.onnx",
        inputTopic: "/camera/image",
        outputTopic: "/ai/results",
        confidence: 0.7,
        batchSize: 1,
        useGPU: false,
        preprocessing: true,
      },
    },
    safety_check: {
      icon: Shield,
      name: "Safety Check",
      description: "Safety monitoring and validation",
      parameters: {
        checkType: "collision", // collision, boundary, temperature, battery
        threshold: 0.1,
        action: "stop", // stop, slow_down, warn, abort
        recoveryAction: "backup",
        alertLevel: "warning", // info, warning, error, critical
        autoRecover: true,
        timeout: 5.0,
      },
    },
    communication: {
      icon: Wifi,
      name: "Communication",
      description: "Network and communication tasks",
      parameters: {
        protocol: "tcp", // tcp, udp, websocket, mqtt
        address: "localhost",
        port: 8080,
        message: "",
        encoding: "utf-8",
        timeout: 10.0,
        retry: 3,
        encryption: false,
      },
    },
    voice_command: {
      icon: Mic,
      name: "Voice Command",
      description: "Speech recognition and voice control",
      parameters: {
        language: "en-US",
        confidence: 0.8,
        timeout: 5.0,
        keywords: [],
        responseType: "speech", // speech, text, action
        voiceModel: "default",
        noiseReduction: true,
      },
    },
    data_logging: {
      icon: BarChart3,
      name: "Data Logging",
      description: "Data collection and logging",
      parameters: {
        dataSource: "/joint_states",
        logFormat: "csv", // csv, json, rosbag
        filename: "data_log",
        duration: 10.0,
        samplingRate: 50,
        compression: true,
        autoUpload: false,
        cloudStorage: "local",
      },
    },
    maintenance: {
      icon: Wrench,
      name: "Maintenance",
      description: "System maintenance and diagnostics",
      parameters: {
        checkType: "diagnostics", // diagnostics, calibration, cleaning, lubrication
        component: "all", // all, motors, sensors, joints
        severity: "routine", // routine, preventive, corrective
        autoFix: false,
        reportGeneration: true,
        scheduleNext: 30, // days
      },
    },
    retry: {
      icon: RotateCcw,
      name: "Retry Control",
      description:
        "Restart execution from this point or retry specific operations",
      parameters: {
        retryType: "restart_from_here", // restart_from_here, retry_last_action, retry_sequence
        maxRetryAttempts: 3,
        retryDelay: 5.0, // seconds
        retryCondition: "on_failure", // on_failure, manual, conditional
        resetState: true,
        preserveProgress: false,
        notifyOperator: true,
        logRetryAttempt: true,
        // Conditional retry logic
        enableConditions: false,
        conditions: [],
        onMaxRetries: "abort", // abort, manual_intervention, skip
      },
    },
    checkpoint: {
      icon: MapPin,
      name: "Checkpoint",
      description:
        "Create a checkpoint for state saving and recovery operations",
      parameters: {
        checkpointType: "save_state", // save_state, recovery_point, progress_marker
        saveData: true,
        dataTypes: ["position", "task_state", "sensor_data"], // position, task_state, sensor_data, variables
        autoSave: true,
        saveInterval: 10.0, // seconds for auto-save
        compressionEnabled: true,
        backupLocation: "local", // local, cloud, both
        restoreOnFailure: true,
        notificationLevel: "info", // info, warning, none
        // Recovery options
        enableRecovery: true,
        recoveryTimeout: 30.0,
        maxRecoveryAttempts: 2,
      },
    },
  };

  const handleParameterChange = (paramKey: string, value: any) => {
    const updatedTask = {
      ...editedTask,
      parameters: {
        ...editedTask.parameters,
        [paramKey]: value,
      },
    };
    setEditedTask(updatedTask);
    console.log("Parameter updated:", paramKey, value, updatedTask.parameters);
  };

  const handleNestedParameterChange = (
    groupKey: string,
    paramKey: string,
    value: any,
  ) => {
    const updatedTask = {
      ...editedTask,
      parameters: {
        ...editedTask.parameters,
        [groupKey]: {
          ...(editedTask.parameters[groupKey] || {}),
          [paramKey]: value,
        },
      },
    };
    setEditedTask(updatedTask);
    console.log("Nested parameter updated:", groupKey, paramKey, value);
  };

  // Handle obstacle region management
  const addObstacleRegion = () => {
    if (newRegion.name.trim() === "") {
      newRegion.name = `Region ${obstacleRegions.length + 1}`;
    }

    const region = { ...newRegion };
    const updatedRegions = [...obstacleRegions, region];
    setObstacleRegions(updatedRegions);

    // Update task parameters
    handleParameterChange("obstacleRegions", updatedRegions);

    // Reset form
    setNewRegion({ x1: 0, y1: 0, x2: 0, y2: 0, name: "" });
    console.log("Added obstacle region:", region);
  };

  const removeObstacleRegion = (index: number) => {
    const updatedRegions = obstacleRegions.filter((_, i) => i !== index);
    setObstacleRegions(updatedRegions);
    handleParameterChange("obstacleRegions", updatedRegions);
  };

  const loadCostmapRegions = () => {
    // Simulate loading predefined regions from costmap
    const predefinedRegions = [
      { x1: 2.0, y1: 3.0, x2: 4.0, y2: 5.0, name: "Permanent Obstacle A" },
      { x1: -1.0, y1: -2.0, x2: 1.0, y2: 0.0, name: "Static Structure B" },
      { x1: 5.0, y1: 1.0, x2: 7.0, y2: 2.5, name: "Equipment Zone C" },
    ];

    setObstacleRegions(predefinedRegions);
    handleParameterChange("obstacleRegions", predefinedRegions);
    console.log("Loaded costmap regions:", predefinedRegions);
  };

  // Handle waypoint management for pattern movement
  const addWaypoint = () => {
    if (newWaypoint.name.trim() === "") {
      newWaypoint.name = `Waypoint ${movementPattern.length + 1}`;
    }

    const waypoint = { ...newWaypoint };
    const updatedPattern = [...movementPattern, waypoint];
    setMovementPattern(updatedPattern);

    // Update task parameters
    handleParameterChange("movementPattern", updatedPattern);

    // Reset form
    setNewWaypoint({ x: 0, y: 0, yaw: 0, name: "" });
    console.log("Added waypoint:", waypoint);
  };

  const removeWaypoint = (index: number) => {
    const updatedPattern = movementPattern.filter((_, i) => i !== index);
    setMovementPattern(updatedPattern);
    handleParameterChange("movementPattern", updatedPattern);
  };

  const loadPatternPreset = (patternType: string) => {
    let preset: Array<{ x: number; y: number; yaw: number; name: string }> = [];

    switch (patternType) {
      case "square":
        preset = [
          { x: 0, y: 0, yaw: 0, name: "Start Point" },
          { x: 2, y: 0, yaw: 90, name: "Right Side" },
          { x: 2, y: 2, yaw: 180, name: "Top Right" },
          { x: 0, y: 2, yaw: -90, name: "Top Left" },
          { x: 0, y: 0, yaw: 0, name: "Back to Start" },
        ];
        break;
      case "circular":
        const radius = 2;
        for (let i = 0; i < 8; i++) {
          const angle = (i * 45 * Math.PI) / 180; // Convert to radians
          preset.push({
            x: radius * Math.cos(angle),
            y: radius * Math.sin(angle),
            yaw: (i * 45 + 90) % 360, // Face tangent to circle
            name: `Circle Point ${i + 1}`,
          });
        }
        break;
      case "patrol":
        preset = [
          { x: 0, y: 0, yaw: 0, name: "Base" },
          { x: 5, y: 0, yaw: 90, name: "Checkpoint A" },
          { x: 5, y: 3, yaw: 180, name: "Checkpoint B" },
          { x: -2, y: 3, yaw: -90, name: "Checkpoint C" },
          { x: -2, y: -2, yaw: 0, name: "Checkpoint D" },
          { x: 0, y: 0, yaw: 0, name: "Return to Base" },
        ];
        break;
    }

    setMovementPattern(preset);
    handleParameterChange("movementPattern", preset);
    console.log(`Loaded ${patternType} pattern:`, preset);
  };

  const handleSave = () => {
    // Ensure all parameters are properly merged
    const finalTask = {
      ...editedTask,
      parameters: { ...mergedParams, obstacleRegions, movementPattern },
    };
    console.log("Saving task with parameters:", finalTask);
    onSave(finalTask);
    onClose();
  };

  const taskTypeInfo = taskTypes[editedTask.type as keyof typeof taskTypes];
  const defaultParams = taskTypeInfo?.parameters || {};

  // Merge default parameters with existing ones
  const mergedParams = { ...defaultParams, ...editedTask.parameters };

  const renderParameterField = (key: string, value: any, label?: string) => {
    const displayLabel = label || key.replace(/([A-Z])/g, " $1").trim();

    if (typeof value === "boolean") {
      return (
        <div key={key} className="flex items-center justify-between">
          <Label className="text-sm">{displayLabel}</Label>
          <Switch
            checked={value}
            onCheckedChange={(checked) => handleParameterChange(key, checked)}
          />
        </div>
      );
    }

    if (typeof value === "number") {
      // Determine if it's a percentage, angle, or regular number
      const isPercentage = key.includes("confidence") || key.includes("force");
      const isAngle =
        key.includes("roll") || key.includes("pitch") || key.includes("yaw");
      const isPosition =
        key.includes("x") || key.includes("y") || key.includes("z");

      return (
        <div key={key} className="space-y-2">
          <div className="flex items-center justify-between">
            <Label className="text-sm">{displayLabel}</Label>
            <div className="flex items-center gap-2">
              <Input
                type="number"
                value={value}
                onChange={(e) =>
                  handleParameterChange(key, parseFloat(e.target.value) || 0)
                }
                className="h-7 w-20 text-xs"
                step={isPercentage ? 1 : isAngle ? 1 : isPosition ? 0.01 : 0.1}
              />
              <span className="text-xs text-muted-foreground min-w-fit">
                {isPercentage ? "%" : isAngle ? "°" : isPosition ? "m" : ""}
              </span>
            </div>
          </div>
          <Slider
            value={[value]}
            onValueChange={(values) => handleParameterChange(key, values[0])}
            min={isPercentage ? 0 : isAngle ? -180 : isPosition ? -10 : 0}
            max={isPercentage ? 100 : isAngle ? 180 : isPosition ? 10 : 100}
            step={isPercentage ? 1 : isAngle ? 1 : isPosition ? 0.01 : 0.1}
            className="flex-1"
          />
          <div className="flex justify-between text-xs text-muted-foreground">
            <span>
              {isPercentage ? 0 : isAngle ? -180 : isPosition ? -10 : 0}
            </span>
            <span>
              {isPercentage ? 100 : isAngle ? 180 : isPosition ? 10 : 100}
            </span>
          </div>
        </div>
      );
    }

    if (typeof value === "string") {
      // Check if it's a select field
      const selectOptions = {
        movementMode: ["auto_nav", "move_to_position", "manual_relative"],
        manipulationMode: [
          "position_control",
          "joint_control",
          "cartesian_path",
        ],
        moveType: ["linear", "joint", "circular", "spline"],
        coordinateFrame: ["base_link", "world", "map", "odom"],
        targetFrame: ["map", "base_link", "odom"],
        plannerType: ["navfn", "global_planner", "astar", "rrt"],
        costmapLayer: ["global_costmap", "local_costmap", "obstacle_layer"],
        gripperAction: ["none", "open", "close", "grasp", "release"],
        processingType: [
          "object_detection",
          "qr_code",
          "face_recognition",
          "color_tracking",
          "edge_detection",
          "lane_detection",
        ],
        sensorType: [
          "lidar",
          "camera",
          "imu",
          "ultrasonic",
          "temperature",
          "pressure",
          "humidity",
          "gps",
          "magnetometer",
          "accelerometer",
          "gyroscope",
          "proximity",
          "force",
          "torque",
          "encoders",
          "current",
          "voltage",
          "light",
          "sound",
          "vibration",
          "infrared",
          "thermal",
        ],
        modelType: [
          "classification",
          "detection",
          "segmentation",
          "nlp",
          "regression",
        ],
        checkType: [
          "collision",
          "boundary",
          "temperature",
          "battery",
          "pressure",
        ],
        action: ["stop", "slow_down", "warn", "abort", "continue"],
        onSuccess: ["continue", "wait", "repeat", "jump_to"],
        onFailure: ["retry", "skip", "abort", "manual", "alternative"],
        protocol: ["tcp", "udp", "websocket", "mqtt", "http"],
        language: ["en-US", "th-TH", "zh-CN", "ja-JP", "ko-KR"],
        logFormat: ["csv", "json", "rosbag", "txt"],
        filterType: ["blur", "sharpen", "edge_enhance", "none"],
        // Obstacle detection options
        obstacleDetectionMode: ["dynamic", "static", "predictive", "none"],
        avoidanceStrategy: [
          "circumnavigate",
          "wait",
          "alternative_route",
          "stop",
          "slow_down",
        ],
        obstacleTypes: [
          "static",
          "dynamic",
          "human",
          "vehicle",
          "robot",
          "unknown",
        ],
        predictionModel: ["linear", "polynomial", "kalman", "neural"],
        // Retry options
        retryType: [
          "restart_from_here",
          "retry_last_action",
          "retry_sequence",
          "retry_from_checkpoint",
        ],
        retryCondition: ["on_failure", "manual", "conditional", "timeout"],
        onMaxRetries: [
          "abort",
          "manual_intervention",
          "skip",
          "alternative_method",
        ],
        // Checkpoint options
        checkpointType: [
          "save_state",
          "recovery_point",
          "progress_marker",
          "milestone",
        ],
        dataTypes: [
          "position",
          "task_state",
          "sensor_data",
          "variables",
          "parameters",
        ],
        backupLocation: ["local", "cloud", "both", "network_drive"],
        // Sensor options
        units: [
          "meters",
          "degrees",
          "celsius",
          "pascal",
          "amperes",
          "volts",
          "rpm",
          "newtons",
          "lux",
          "decibels",
        ],
        processingMode: ["real_time", "batch", "triggered", "continuous"],
        dataFormat: ["json", "csv", "rosbag", "binary", "xml"],
        // Move with obstacle check options
        obstacleCheckMode: ["continuous", "periodic", "trigger_based"],
        costmapSource: [
          "global_costmap",
          "local_costmap",
          "custom",
          "sensor_fusion",
        ],
        aggressiveness: ["conservative", "moderate", "aggressive"],
        onObstacleDetected: [
          "smart_avoid",
          "stop",
          "slow_down",
          "alternative_route",
          "wait",
        ],
        onPathBlocked: ["find_alternative", "wait", "abort", "manual_control"],
        // Pattern movement options
        patternType: ["waypoints", "circular", "square", "patrol", "custom"],
      };

      if (selectOptions[key as keyof typeof selectOptions]) {
        return (
          <div key={key} className="space-y-2">
            <Label className="text-sm">{displayLabel}</Label>
            <Select
              value={value}
              onValueChange={(newValue) => handleParameterChange(key, newValue)}
            >
              <SelectTrigger>
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {selectOptions[key as keyof typeof selectOptions].map(
                  (option) => (
                    <SelectItem key={option} value={option}>
                      {option.replace(/_/g, " ").toUpperCase()}
                    </SelectItem>
                  ),
                )}
              </SelectContent>
            </Select>
          </div>
        );
      }

      // Text areas for longer content
      if (key.includes("message") || key.includes("description")) {
        return (
          <div key={key} className="space-y-2">
            <Label className="text-sm">{displayLabel}</Label>
            <Textarea
              value={value}
              onChange={(e) => handleParameterChange(key, e.target.value)}
              placeholder={`Enter ${displayLabel.toLowerCase()}...`}
              rows={3}
            />
          </div>
        );
      }

      // Regular text input
      return (
        <div key={key} className="space-y-2">
          <Label className="text-sm">{displayLabel}</Label>
          <Input
            value={value}
            onChange={(e) => handleParameterChange(key, e.target.value)}
            placeholder={`Enter ${displayLabel.toLowerCase()}...`}
          />
        </div>
      );
    }

    if (typeof value === "object" && value !== null) {
      return (
        <div key={key} className="space-y-3">
          <Label className="text-sm font-semibold">{displayLabel}</Label>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-3 p-3 border rounded-lg">
            {Object.entries(value).map(([subKey, subValue]) =>
              renderNestedParameter(key, subKey, subValue),
            )}
          </div>
        </div>
      );
    }

    return null;
  };

  const renderNestedParameter = (groupKey: string, key: string, value: any) => {
    const displayLabel = key.replace(/([A-Z])/g, " $1").trim();

    if (typeof value === "number") {
      return (
        <div key={`${groupKey}.${key}`} className="space-y-1">
          <Label className="text-xs">{displayLabel}</Label>
          <Input
            type="number"
            step="0.01"
            value={value}
            onChange={(e) =>
              handleNestedParameterChange(
                groupKey,
                key,
                parseFloat(e.target.value) || 0,
              )
            }
          />
        </div>
      );
    }

    if (typeof value === "string") {
      return (
        <div key={`${groupKey}.${key}`} className="space-y-1">
          <Label className="text-xs">{displayLabel}</Label>
          <Input
            value={value}
            onChange={(e) =>
              handleNestedParameterChange(groupKey, key, e.target.value)
            }
          />
        </div>
      );
    }

    return null;
  };

  return (
    <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto glass-effect border-0 scale-in">
        <DialogHeader className="fade-in-up">
          <DialogTitle className="flex items-center gap-3 font-extralight text-xl">
            {taskTypeInfo && (
              <taskTypeInfo.icon className="h-6 w-6 float-animation text-primary" />
            )}
            {t("task.configureTask")}:{" "}
            <span className="gradient-animate bg-clip-text text-transparent">
              {editedTask.name}
            </span>
          </DialogTitle>
          <DialogDescription className="font-extralight text-base">
            {taskTypeInfo?.description || t("task.configureTask")}
          </DialogDescription>
        </DialogHeader>

        <Tabs defaultValue="basic" className="space-y-4 slide-in-right">
          <TabsList className="grid w-full grid-cols-3 glass-effect border-0">
            <TabsTrigger value="basic" className="font-extralight hover-scale">
              {t("task.basicSettings")}
            </TabsTrigger>
            <TabsTrigger
              value="parameters"
              className="font-extralight hover-scale"
            >
              {t("task.parameters")}
            </TabsTrigger>
            <TabsTrigger
              value="advanced"
              className="font-extralight hover-scale"
            >
              {t("task.advanced")}
            </TabsTrigger>
          </TabsList>

          {/* Basic Settings */}
          <TabsContent value="basic" className="space-y-4">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div className="space-y-2 stagger-item">
                <Label className="font-extralight text-sm">
                  {t("task.taskName")}
                </Label>
                <Input
                  value={editedTask.name}
                  onChange={(e) =>
                    setEditedTask({ ...editedTask, name: e.target.value })
                  }
                  className="glass-effect border-0 font-extralight hover-lift transition-all duration-300"
                />
              </div>

              <div className="space-y-2 stagger-item">
                <Label className="font-extralight text-sm">
                  {t("task.taskType")}
                </Label>
                <Select
                  value={editedTask.type}
                  onValueChange={(value) =>
                    setEditedTask({ ...editedTask, type: value })
                  }
                >
                  <SelectTrigger className="glass-effect border-0 font-extralight hover-lift">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent className="glass-effect border-0">
                    {Object.entries(taskTypes).map(([key, info]) => (
                      <SelectItem
                        key={key}
                        value={key}
                        className="font-extralight hover-scale"
                      >
                        <div className="flex items-center gap-2">
                          <info.icon className="h-4 w-4 hover-rotate" />
                          {info.name}
                        </div>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
            </div>

            <div className="space-y-2 stagger-item">
              <Label className="font-extralight text-sm">
                {t("task.description")}
              </Label>
              <Textarea
                value={editedTask.description}
                onChange={(e) =>
                  setEditedTask({ ...editedTask, description: e.target.value })
                }
                rows={3}
                className="glass-effect border-0 font-extralight hover-lift transition-all duration-300"
              />
            </div>

            <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
              <div className="space-y-2 stagger-item">
                <Label className="font-extralight text-sm">
                  {t("task.timeout")} (วินาที)
                </Label>
                <Input
                  type="number"
                  value={editedTask.timeout}
                  onChange={(e) =>
                    setEditedTask({
                      ...editedTask,
                      timeout: parseInt(e.target.value) || 60,
                    })
                  }
                  className="glass-effect border-0 font-extralight hover-lift"
                />
              </div>

              <div className="space-y-2 stagger-item">
                <Label className="font-extralight text-sm">
                  {t("task.retries")}
                </Label>
                <Input
                  type="number"
                  value={editedTask.retries}
                  onChange={(e) =>
                    setEditedTask({
                      ...editedTask,
                      retries: parseInt(e.target.value) || 1,
                    })
                  }
                  className="glass-effect border-0 font-extralight hover-lift"
                />
              </div>

              <div className="flex items-center justify-between stagger-item">
                <Label className="font-extralight text-sm">
                  {t("task.waitForFeedback")}
                </Label>
                <Switch
                  checked={editedTask.waitForFeedback}
                  onCheckedChange={(checked) =>
                    setEditedTask({ ...editedTask, waitForFeedback: checked })
                  }
                />
              </div>

              {editedTask.waitForFeedback && (
                <div className="space-y-2">
                  <Label>Feedback Timeout</Label>
                  <Input
                    type="number"
                    value={editedTask.feedbackTimeout}
                    onChange={(e) =>
                      setEditedTask({
                        ...editedTask,
                        feedbackTimeout: parseInt(e.target.value) || 30,
                      })
                    }
                  />
                </div>
              )}
            </div>
          </TabsContent>

          {/* Parameters */}
          <TabsContent value="parameters" className="space-y-4">
            {taskTypeInfo && (
              <div className="p-4 bg-muted rounded-lg">
                <div className="flex items-center gap-2 mb-2">
                  <taskTypeInfo.icon className="h-5 w-5 text-primary" />
                  <h3 className="font-semibold">{taskTypeInfo.name}</h3>
                  <Badge variant="outline">{editedTask.type}</Badge>
                </div>
                <p className="text-sm text-muted-foreground">
                  {taskTypeInfo.description}
                </p>
              </div>
            )}

            {/* Movement Mode Selection for Movement Tasks */}
            {editedTask.type === "movement" && (
              <Card className="p-4">
                <h4 className="font-semibold mb-3 flex items-center gap-2">
                  <Navigation className="h-4 w-4" />
                  Movement Mode
                </h4>
                <div className="grid grid-cols-1 md:grid-cols-3 gap-3 mb-4">
                  {[
                    {
                      mode: "auto_nav",
                      title: "Auto Navigation",
                      description:
                        "Use ROS navigation stack with path planning",
                      icon: Navigation,
                    },
                    {
                      mode: "move_to_position",
                      title: "Move to Position",
                      description: "Direct movement using TF coordinates",
                      icon: Target,
                    },
                    {
                      mode: "manual_relative",
                      title: "Manual Relative",
                      description:
                        "Move relative distance from current position",
                      icon: Move3D,
                    },
                  ].map((option) => (
                    <Card
                      key={option.mode}
                      className={`p-3 cursor-pointer transition-all ${
                        mergedParams.movementMode === option.mode
                          ? "ring-2 ring-primary bg-primary/5"
                          : "hover:bg-accent"
                      }`}
                      onClick={() =>
                        handleParameterChange("movementMode", option.mode)
                      }
                    >
                      <div className="flex items-center gap-2 mb-2">
                        <option.icon className="h-4 w-4 text-primary" />
                        <span className="font-medium text-sm">
                          {option.title}
                        </span>
                      </div>
                      <p className="text-xs text-muted-foreground">
                        {option.description}
                      </p>
                    </Card>
                  ))}
                </div>

                {/* Mode-specific parameters */}
                {mergedParams.movementMode === "auto_nav" && (
                  <div className="space-y-3 p-3 bg-blue-50 dark:bg-blue-950/20 rounded-lg">
                    <Label className="text-sm font-semibold">
                      Navigation Settings
                    </Label>
                    <div className="grid grid-cols-2 gap-3">
                      <div>
                        <Label className="text-xs">Planner Type</Label>
                        <Select
                          value={mergedParams.plannerType || "navfn"}
                          onValueChange={(value) =>
                            handleParameterChange("plannerType", value)
                          }
                        >
                          <SelectTrigger className="h-8">
                            <SelectValue />
                          </SelectTrigger>
                          <SelectContent>
                            <SelectItem value="navfn">NavFn</SelectItem>
                            <SelectItem value="global_planner">
                              Global Planner
                            </SelectItem>
                            <SelectItem value="astar">A* Planner</SelectItem>
                            <SelectItem value="rrt">RRT Planner</SelectItem>
                          </SelectContent>
                        </Select>
                      </div>
                      <div className="flex items-center justify-between">
                        <Label className="text-xs">Use Costmap</Label>
                        <Switch
                          checked={mergedParams.useCostmap}
                          onCheckedChange={(checked) =>
                            handleParameterChange("useCostmap", checked)
                          }
                        />
                      </div>
                    </div>
                  </div>
                )}

                {mergedParams.movementMode === "manual_relative" && (
                  <div className="space-y-3 p-3 bg-green-50 dark:bg-green-950/20 rounded-lg">
                    <Label className="text-sm font-semibold">
                      Relative Movement (from current position)
                    </Label>
                    <div className="grid grid-cols-3 gap-3">
                      <div>
                        <Label className="text-xs">X Distance (m)</Label>
                        <Input
                          type="number"
                          step="0.1"
                          value={mergedParams.relativeDistance?.x || 0}
                          onChange={(e) =>
                            handleNestedParameterChange(
                              "relativeDistance",
                              "x",
                              parseFloat(e.target.value) || 0,
                            )
                          }
                          className="h-8"
                        />
                      </div>
                      <div>
                        <Label className="text-xs">Y Distance (m)</Label>
                        <Input
                          type="number"
                          step="0.1"
                          value={mergedParams.relativeDistance?.y || 0}
                          onChange={(e) =>
                            handleNestedParameterChange(
                              "relativeDistance",
                              "y",
                              parseFloat(e.target.value) || 0,
                            )
                          }
                          className="h-8"
                        />
                      </div>
                      <div>
                        <Label className="text-xs">Z Distance (m)</Label>
                        <Input
                          type="number"
                          step="0.1"
                          value={mergedParams.relativeDistance?.z || 0}
                          onChange={(e) =>
                            handleNestedParameterChange(
                              "relativeDistance",
                              "z",
                              parseFloat(e.target.value) || 0,
                            )
                          }
                          className="h-8"
                        />
                      </div>
                    </div>
                  </div>
                )}
              </Card>
            )}

            {/* Obstacle Detection Configuration for Movement Tasks */}
            {editedTask.type === "movement" && (
              <Card className="p-4">
                <h4 className="font-semibold mb-3 flex items-center gap-2">
                  <Shield className="h-4 w-4" />
                  Obstacle Detection & Avoidance
                </h4>

                <div className="space-y-4">
                  <div className="grid grid-cols-2 gap-3">
                    <div>
                      <Label className="text-sm">Detection Mode</Label>
                      <Select
                        value={mergedParams.obstacleDetectionMode || "dynamic"}
                        onValueChange={(value) =>
                          handleParameterChange("obstacleDetectionMode", value)
                        }
                      >
                        <SelectTrigger className="h-8">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          <SelectItem value="dynamic">
                            Dynamic Detection
                          </SelectItem>
                          <SelectItem value="static">Static Only</SelectItem>
                          <SelectItem value="predictive">Predictive</SelectItem>
                          <SelectItem value="none">Disabled</SelectItem>
                        </SelectContent>
                      </Select>
                    </div>

                    <div>
                      <Label className="text-sm">Avoidance Strategy</Label>
                      <Select
                        value={
                          mergedParams.avoidanceStrategy || "circumnavigate"
                        }
                        onValueChange={(value) =>
                          handleParameterChange("avoidanceStrategy", value)
                        }
                      >
                        <SelectTrigger className="h-8">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          <SelectItem value="circumnavigate">
                            Go Around
                          </SelectItem>
                          <SelectItem value="wait">Wait</SelectItem>
                          <SelectItem value="alternative_route">
                            Alternative Route
                          </SelectItem>
                          <SelectItem value="stop">Emergency Stop</SelectItem>
                          <SelectItem value="slow_down">Slow Down</SelectItem>
                        </SelectContent>
                      </Select>
                    </div>
                  </div>

                  <div className="grid grid-cols-2 gap-3">
                    <div>
                      <Label className="text-sm">Safety Distance (m)</Label>
                      <Input
                        type="number"
                        step="0.1"
                        value={mergedParams.safetyDistance || 0.5}
                        onChange={(e) =>
                          handleParameterChange(
                            "safetyDistance",
                            parseFloat(e.target.value) || 0.5,
                          )
                        }
                        className="h-8"
                      />
                    </div>

                    <div>
                      <Label className="text-sm">Detection Range (m)</Label>
                      <Input
                        type="number"
                        step="0.5"
                        value={mergedParams.obstacleDetectionRange || 5.0}
                        onChange={(e) =>
                          handleParameterChange(
                            "obstacleDetectionRange",
                            parseFloat(e.target.value) || 5.0,
                          )
                        }
                        className="h-8"
                      />
                    </div>
                  </div>

                  {/* Obstacle Regions in Costmap */}
                  <div>
                    <div className="flex items-center justify-between mb-2">
                      <Label className="text-sm">
                        Known Obstacle Regions (Costmap Coordinates)
                      </Label>
                      <Button
                        size="sm"
                        variant="secondary"
                        onClick={loadCostmapRegions}
                        className="h-7"
                      >
                        📊 Load from Costmap
                      </Button>
                    </div>

                    <div className="p-3 bg-muted rounded-lg space-y-3">
                      <p className="text-xs text-muted-foreground">
                        Define specific areas in the costmap where obstacles are
                        known to exist
                      </p>

                      {/* Add New Region Form */}
                      <div className="grid grid-cols-5 gap-2 text-xs">
                        <Input
                          placeholder="X1"
                          type="number"
                          step="0.1"
                          value={newRegion.x1}
                          onChange={(e) =>
                            setNewRegion({
                              ...newRegion,
                              x1: parseFloat(e.target.value) || 0,
                            })
                          }
                          className="h-7"
                        />
                        <Input
                          placeholder="Y1"
                          type="number"
                          step="0.1"
                          value={newRegion.y1}
                          onChange={(e) =>
                            setNewRegion({
                              ...newRegion,
                              y1: parseFloat(e.target.value) || 0,
                            })
                          }
                          className="h-7"
                        />
                        <Input
                          placeholder="X2"
                          type="number"
                          step="0.1"
                          value={newRegion.x2}
                          onChange={(e) =>
                            setNewRegion({
                              ...newRegion,
                              x2: parseFloat(e.target.value) || 0,
                            })
                          }
                          className="h-7"
                        />
                        <Input
                          placeholder="Y2"
                          type="number"
                          step="0.1"
                          value={newRegion.y2}
                          onChange={(e) =>
                            setNewRegion({
                              ...newRegion,
                              y2: parseFloat(e.target.value) || 0,
                            })
                          }
                          className="h-7"
                        />
                        <Input
                          placeholder="Region Name"
                          value={newRegion.name}
                          onChange={(e) =>
                            setNewRegion({ ...newRegion, name: e.target.value })
                          }
                          className="h-7"
                        />
                      </div>

                      <Button
                        size="sm"
                        variant="outline"
                        onClick={addObstacleRegion}
                        className="h-7"
                      >
                        <Plus className="h-3 w-3 mr-1" />
                        Add Region
                      </Button>

                      {/* Display Current Regions */}
                      {obstacleRegions.length > 0 && (
                        <div className="space-y-2">
                          <Label className="text-xs font-semibold">
                            Current Obstacle Regions:
                          </Label>
                          {obstacleRegions.map((region, index) => (
                            <div
                              key={index}
                              className="flex items-center justify-between p-2 bg-background rounded border"
                            >
                              <div className="text-xs">
                                <span className="font-medium">
                                  {region.name || `Region ${index + 1}`}
                                </span>
                                <span className="text-muted-foreground ml-2">
                                  ({region.x1}, {region.y1}) → ({region.x2},{" "}
                                  {region.y2})
                                </span>
                              </div>
                              <Button
                                size="sm"
                                variant="ghost"
                                onClick={() => removeObstacleRegion(index)}
                                className="h-6 w-6 p-0 hover:bg-destructive hover:text-destructive-foreground"
                              >
                                ×
                              </Button>
                            </div>
                          ))}
                        </div>
                      )}
                    </div>
                  </div>

                  {/* Obstacle Types */}
                  <div>
                    <Label className="text-sm">Obstacle Types to Detect</Label>
                    <div className="flex flex-wrap gap-2 mt-2">
                      {["static", "dynamic", "human", "vehicle", "robot"].map(
                        (type) => (
                          <div
                            key={type}
                            className="flex items-center space-x-2"
                          >
                            <input
                              type="checkbox"
                              id={`obstacle-${type}`}
                              defaultChecked={
                                type === "static" || type === "dynamic"
                              }
                              className="rounded"
                            />
                            <Label
                              htmlFor={`obstacle-${type}`}
                              className="text-xs"
                            >
                              {type.charAt(0).toUpperCase() + type.slice(1)}
                            </Label>
                          </div>
                        ),
                      )}
                    </div>
                  </div>
                </div>
              </Card>
            )}

            {/* Move with Obstacle Check Configuration */}
            {editedTask.type === "move_with_obstacle_check" && (
              <Card className="p-4">
                <h4 className="font-semibold mb-3 flex items-center gap-2">
                  <Shield className="h-4 w-4" />
                  Intelligent Movement with Obstacle Detection
                </h4>

                <div className="space-y-4">
                  {/* Movement Mode Selection */}
                  <div className="grid grid-cols-1 md:grid-cols-3 gap-3 mb-4">
                    {[
                      {
                        mode: "move_to_position",
                        title: "Move to Position",
                        desc: "Direct movement to specific coordinates",
                        icon: Target,
                      },
                      {
                        mode: "manual_relative",
                        title: "Manual Relative",
                        desc: "Relative movement from current position",
                        icon: Move3D,
                      },
                      {
                        mode: "pattern_movement",
                        title: "Pattern Movement",
                        desc: "Follow predefined waypoint patterns",
                        icon: Navigation,
                      },
                    ].map((option) => (
                      <Card
                        key={option.mode}
                        className={`p-3 cursor-pointer transition-all ${
                          mergedParams.movementMode === option.mode
                            ? "ring-2 ring-primary bg-primary/5"
                            : "hover:bg-accent"
                        }`}
                        onClick={() =>
                          handleParameterChange("movementMode", option.mode)
                        }
                      >
                        <div className="flex items-center gap-2 mb-2">
                          <option.icon className="h-4 w-4 text-primary" />
                          <span className="font-medium text-sm">
                            {option.title}
                          </span>
                        </div>
                        <p className="text-xs text-muted-foreground">
                          {option.desc}
                        </p>
                      </Card>
                    ))}
                  </div>

                  {/* Move to Position Mode */}
                  {mergedParams.movementMode === "move_to_position" && (
                    <div className="p-3 bg-blue-50 dark:bg-blue-950/20 rounded-lg">
                      <Label className="text-sm font-semibold mb-3 block">
                        Target Position & Orientation
                      </Label>
                      <div className="grid grid-cols-3 gap-3">
                        <div>
                          <Label className="text-xs">X Position (m)</Label>
                          <Input
                            type="number"
                            step="0.1"
                            value={mergedParams.position?.x || 0}
                            onChange={(e) =>
                              handleNestedParameterChange(
                                "position",
                                "x",
                                parseFloat(e.target.value) || 0,
                              )
                            }
                            className="h-8"
                          />
                        </div>
                        <div>
                          <Label className="text-xs">Y Position (m)</Label>
                          <Input
                            type="number"
                            step="0.1"
                            value={mergedParams.position?.y || 0}
                            onChange={(e) =>
                              handleNestedParameterChange(
                                "position",
                                "y",
                                parseFloat(e.target.value) || 0,
                              )
                            }
                            className="h-8"
                          />
                        </div>
                        <div>
                          <Label className="text-xs">Yaw Angle (degrees)</Label>
                          <Input
                            type="number"
                            step="1"
                            min="-180"
                            max="180"
                            value={mergedParams.orientation?.yaw || 0}
                            onChange={(e) =>
                              handleNestedParameterChange(
                                "orientation",
                                "yaw",
                                parseFloat(e.target.value) || 0,
                              )
                            }
                            className="h-8"
                          />
                        </div>
                      </div>
                      <p className="text-xs text-muted-foreground mt-2">
                        💡 Yaw: 0° = forward, 90° = left, -90° = right, 180° =
                        backward
                      </p>
                    </div>
                  )}

                  {/* Manual Relative Mode */}
                  {mergedParams.movementMode === "manual_relative" && (
                    <div className="p-3 bg-green-50 dark:bg-green-950/20 rounded-lg">
                      <Label className="text-sm font-semibold mb-3 block">
                        Relative Movement (from current position)
                      </Label>
                      <div className="grid grid-cols-3 gap-3">
                        <div>
                          <Label className="text-xs">X Distance (m)</Label>
                          <Input
                            type="number"
                            step="0.1"
                            value={mergedParams.relativeDistance?.x || 0}
                            onChange={(e) =>
                              handleNestedParameterChange(
                                "relativeDistance",
                                "x",
                                parseFloat(e.target.value) || 0,
                              )
                            }
                            className="h-8"
                          />
                        </div>
                        <div>
                          <Label className="text-xs">Y Distance (m)</Label>
                          <Input
                            type="number"
                            step="0.1"
                            value={mergedParams.relativeDistance?.y || 0}
                            onChange={(e) =>
                              handleNestedParameterChange(
                                "relativeDistance",
                                "y",
                                parseFloat(e.target.value) || 0,
                              )
                            }
                            className="h-8"
                          />
                        </div>
                        <div>
                          <Label className="text-xs">
                            Yaw Change (degrees)
                          </Label>
                          <Input
                            type="number"
                            step="1"
                            value={mergedParams.relativeDistance?.yaw || 0}
                            onChange={(e) =>
                              handleNestedParameterChange(
                                "relativeDistance",
                                "yaw",
                                parseFloat(e.target.value) || 0,
                              )
                            }
                            className="h-8"
                          />
                        </div>
                      </div>
                      <p className="text-xs text-muted-foreground mt-2">
                        💡 Positive X = forward, Positive Y = left, Positive Yaw
                        = counter-clockwise
                      </p>
                    </div>
                  )}

                  {/* Pattern Movement Mode */}
                  {mergedParams.movementMode === "pattern_movement" && (
                    <div className="p-3 bg-purple-50 dark:bg-purple-950/20 rounded-lg space-y-4">
                      <div className="flex items-center justify-between">
                        <Label className="text-sm font-semibold">
                          Movement Pattern Configuration
                        </Label>
                        <div className="flex items-center gap-2">
                          <Switch
                            checked={
                              mergedParams.enablePatternMovement || false
                            }
                            onCheckedChange={(checked) =>
                              handleParameterChange(
                                "enablePatternMovement",
                                checked,
                              )
                            }
                          />
                          <Label className="text-xs">
                            Enable Pattern Movement
                          </Label>
                        </div>
                      </div>

                      {mergedParams.enablePatternMovement && (
                        <>
                          {/* Pattern Presets */}
                          <div>
                            <Label className="text-sm mb-2 block">
                              Quick Pattern Presets
                            </Label>
                            <div className="grid grid-cols-2 md:grid-cols-4 gap-2">
                              <Button
                                size="sm"
                                variant="outline"
                                onClick={() => loadPatternPreset("square")}
                                className="h-8"
                              >
                                ⬜ Square
                              </Button>
                              <Button
                                size="sm"
                                variant="outline"
                                onClick={() => loadPatternPreset("circular")}
                                className="h-8"
                              >
                                ⭕ Circle
                              </Button>
                              <Button
                                size="sm"
                                variant="outline"
                                onClick={() => loadPatternPreset("patrol")}
                                className="h-8"
                              >
                                🔍 Patrol
                              </Button>
                              <Button
                                size="sm"
                                variant="outline"
                                onClick={() => setMovementPattern([])}
                                className="h-8"
                              >
                                🗑️ Clear
                              </Button>
                            </div>
                          </div>

                          {/* Add New Waypoint */}
                          <div>
                            <Label className="text-sm mb-2 block">
                              Add Waypoint
                            </Label>
                            <div className="grid grid-cols-4 gap-2">
                              <Input
                                placeholder="X"
                                type="number"
                                step="0.1"
                                value={newWaypoint.x}
                                onChange={(e) =>
                                  setNewWaypoint({
                                    ...newWaypoint,
                                    x: parseFloat(e.target.value) || 0,
                                  })
                                }
                                className="h-8"
                              />
                              <Input
                                placeholder="Y"
                                type="number"
                                step="0.1"
                                value={newWaypoint.y}
                                onChange={(e) =>
                                  setNewWaypoint({
                                    ...newWaypoint,
                                    y: parseFloat(e.target.value) || 0,
                                  })
                                }
                                className="h-8"
                              />
                              <Input
                                placeholder="Yaw (°)"
                                type="number"
                                step="1"
                                value={newWaypoint.yaw}
                                onChange={(e) =>
                                  setNewWaypoint({
                                    ...newWaypoint,
                                    yaw: parseFloat(e.target.value) || 0,
                                  })
                                }
                                className="h-8"
                              />
                              <Input
                                placeholder="Name"
                                value={newWaypoint.name}
                                onChange={(e) =>
                                  setNewWaypoint({
                                    ...newWaypoint,
                                    name: e.target.value,
                                  })
                                }
                                className="h-8"
                              />
                            </div>
                            <Button
                              size="sm"
                              variant="outline"
                              onClick={addWaypoint}
                              className="mt-2 h-8"
                            >
                              <Plus className="h-3 w-3 mr-1" />
                              Add Waypoint
                            </Button>
                          </div>

                          {/* Current Waypoints */}
                          {movementPattern.length > 0 && (
                            <div>
                              <Label className="text-sm font-semibold mb-2 block">
                                Current Pattern ({movementPattern.length}{" "}
                                waypoints)
                              </Label>
                              <div className="space-y-2 max-h-40 overflow-y-auto">
                                {movementPattern.map((waypoint, index) => (
                                  <div
                                    key={index}
                                    className="flex items-center justify-between p-2 bg-background rounded border"
                                  >
                                    <div className="text-xs">
                                      <span className="font-medium">
                                        {index + 1}.{" "}
                                        {waypoint.name ||
                                          `Waypoint ${index + 1}`}
                                      </span>
                                      <span className="text-muted-foreground ml-2">
                                        ({waypoint.x.toFixed(1)},{" "}
                                        {waypoint.y.toFixed(1)}, {waypoint.yaw}
                                        °)
                                      </span>
                                    </div>
                                    <Button
                                      size="sm"
                                      variant="ghost"
                                      onClick={() => removeWaypoint(index)}
                                      className="h-6 w-6 p-0 hover:bg-destructive hover:text-destructive-foreground"
                                    >
                                      ×
                                    </Button>
                                  </div>
                                ))}
                              </div>
                            </div>
                          )}

                          {/* Pattern Settings */}
                          <div className="grid grid-cols-3 gap-3">
                            <div>
                              <Label className="text-xs">Pattern Repeat</Label>
                              <Input
                                type="number"
                                min="1"
                                value={mergedParams.patternRepeat || 1}
                                onChange={(e) =>
                                  handleParameterChange(
                                    "patternRepeat",
                                    parseInt(e.target.value) || 1,
                                  )
                                }
                                className="h-8"
                              />
                            </div>
                            <div>
                              <Label className="text-xs">
                                Wait at Waypoint (s)
                              </Label>
                              <Input
                                type="number"
                                step="0.5"
                                value={mergedParams.waitAtWaypoint || 2.0}
                                onChange={(e) =>
                                  handleParameterChange(
                                    "waitAtWaypoint",
                                    parseFloat(e.target.value) || 2.0,
                                  )
                                }
                                className="h-8"
                              />
                            </div>
                            <div>
                              <Label className="text-xs">
                                Waypoint Tolerance (m)
                              </Label>
                              <Input
                                type="number"
                                step="0.01"
                                value={mergedParams.waypointTolerance || 0.1}
                                onChange={(e) =>
                                  handleParameterChange(
                                    "waypointTolerance",
                                    parseFloat(e.target.value) || 0.1,
                                  )
                                }
                                className="h-8"
                              />
                            </div>
                          </div>
                        </>
                      )}
                    </div>
                  )}

                  {/* Common Obstacle Detection Settings */}
                  <div className="p-3 bg-amber-50 dark:bg-amber-950/20 rounded-lg">
                    <Label className="text-sm font-semibold mb-3 block">
                      Obstacle Detection & Safety
                    </Label>
                    <div className="grid grid-cols-2 gap-3 mb-3">
                      <div>
                        <Label className="text-xs">Detection Mode</Label>
                        <Select
                          value={mergedParams.obstacleCheckMode || "continuous"}
                          onValueChange={(value) =>
                            handleParameterChange("obstacleCheckMode", value)
                          }
                        >
                          <SelectTrigger className="h-8">
                            <SelectValue />
                          </SelectTrigger>
                          <SelectContent>
                            <SelectItem value="continuous">
                              Continuous
                            </SelectItem>
                            <SelectItem value="periodic">Periodic</SelectItem>
                            <SelectItem value="trigger_based">
                              Trigger Based
                            </SelectItem>
                          </SelectContent>
                        </Select>
                      </div>
                      <div>
                        <Label className="text-xs">Safety Distance (m)</Label>
                        <Input
                          type="number"
                          step="0.1"
                          value={mergedParams.safetyDistance || 0.8}
                          onChange={(e) =>
                            handleParameterChange(
                              "safetyDistance",
                              parseFloat(e.target.value) || 0.8,
                            )
                          }
                          className="h-8"
                        />
                      </div>
                    </div>

                    <div className="space-y-2 text-sm">
                      <div className="flex items-center justify-between p-2 bg-background rounded border">
                        <span>
                          🚧 <strong>Obstacle Ahead</strong>
                        </span>
                        <Badge variant="outline">Smart Avoid</Badge>
                      </div>
                      <div className="flex items-center justify-between p-2 bg-background rounded border">
                        <span>
                          🛑 <strong>Path Blocked</strong>
                        </span>
                        <Badge variant="outline">Find Alternative</Badge>
                      </div>
                      <div className="flex items-center justify-between p-2 bg-background rounded border">
                        <span>
                          🚶 <strong>Dynamic Obstacle</strong>
                        </span>
                        <Badge variant="outline">Wait & Retry</Badge>
                      </div>
                    </div>
                  </div>
                </div>
              </Card>
            )}

            <div className="space-y-4">
              {Object.entries(mergedParams)
                .filter(
                  ([key]) =>
                    // Filter out mode-specific and obstacle params that are handled above
                    ![
                      "movementMode",
                      "relativeDistance",
                      "plannerType",
                      "useCostmap",
                      "obstacleDetectionMode",
                      "avoidanceStrategy",
                      "safetyDistance",
                      "obstacleDetectionRange",
                      "obstacleRegions",
                      "obstacleTypes",
                      "costmapSettings",
                      "dynamicObstacleSettings",
                      "emergencyBehaviors",
                      // Move with obstacle check params
                      "obstacleCheckMode",
                      "checkInterval",
                      "costmapSource",
                      "loadCostmapRegions",
                      "enableConditionalMovement",
                      "movementConditions",
                      "aggressiveness",
                      "patienceLevel",
                      "learningEnabled",
                      "adaptiveSpeed",
                      "maxDetourDistance",
                      "emergencyStopDistance",
                      "onObstacleDetected",
                      "onPathBlocked",
                      "onGoalReached",
                      "position",
                      "orientation",
                      "relativeDistance", // These are handled in special sections
                      // Pattern movement params
                      "enablePatternMovement",
                      "movementPattern",
                      "patternType",
                      "patternRepeat",
                      "waitAtWaypoint",
                      "waypointTolerance",
                    ].includes(key),
                )
                .map(([key, value]) => renderParameterField(key, value))}
            </div>

            {/* Conditional Logic Section */}
            {mergedParams.enableConditions && (
              <ConditionalLogicEditor
                conditions={mergedParams.conditions || []}
                onChange={(conditions) =>
                  handleParameterChange("conditions", conditions)
                }
                availableVariables={[
                  "task_status",
                  "execution_time",
                  "error_count",
                  "sensor_data",
                  "battery_level",
                  "obstacle_detected",
                  "goal_reached",
                  "confidence_score",
                  "distance_to_goal",
                ]}
                availableActions={[
                  "continue",
                  "retry",
                  "abort",
                  "wait",
                  "skip",
                  "emergency_stop",
                  "notify_operator",
                  "save_data",
                  "change_speed",
                  "alternative_path",
                ]}
              />
            )}
          </TabsContent>

          {/* Advanced */}
          <TabsContent value="advanced" className="space-y-4">
            <div className="space-y-4">
              <Card className="p-4">
                <h3 className="font-semibold mb-3">Subtask Configuration</h3>
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable Subtasks</Label>
                    <p className="text-sm text-muted-foreground">
                      Break this task into smaller subtasks
                    </p>
                  </div>
                  <Switch
                    checked={editedTask.hasSubtasks}
                    onCheckedChange={(checked) =>
                      setEditedTask({ ...editedTask, hasSubtasks: checked })
                    }
                  />
                </div>

                {editedTask.hasSubtasks && (
                  <div className="mt-4 space-y-3">
                    <div className="grid grid-cols-2 gap-3">
                      <div>
                        <Label>Execution Mode</Label>
                        <Select
                          value={editedTask.subtaskExecutionMode}
                          onValueChange={(value: "sequential" | "parallel") =>
                            setEditedTask({
                              ...editedTask,
                              subtaskExecutionMode: value,
                            })
                          }
                        >
                          <SelectTrigger>
                            <SelectValue />
                          </SelectTrigger>
                          <SelectContent>
                            <SelectItem value="sequential">
                              Sequential
                            </SelectItem>
                            <SelectItem value="parallel">Parallel</SelectItem>
                          </SelectContent>
                        </Select>
                      </div>

                      <div>
                        <Label>Max Concurrent Subtasks</Label>
                        <Input
                          type="number"
                          value={editedTask.maxSubtaskConcurrency}
                          onChange={(e) =>
                            setEditedTask({
                              ...editedTask,
                              maxSubtaskConcurrency:
                                parseInt(e.target.value) || 2,
                            })
                          }
                        />
                      </div>
                    </div>
                  </div>
                )}
              </Card>

              <Card className="p-4">
                <h3 className="font-semibold mb-3 flex items-center gap-2">
                  <Code className="h-4 w-4" />
                  Conditional Logic & Error Handling
                </h3>

                <div className="space-y-4">
                  <div className="flex items-center justify-between">
                    <div>
                      <Label>Enable Conditional Logic</Label>
                      <p className="text-sm text-muted-foreground">
                        Add IF/ELSE conditions and smart decision making
                      </p>
                    </div>
                    <Switch
                      checked={mergedParams.enableConditions || false}
                      onCheckedChange={(checked) =>
                        handleParameterChange("enableConditions", checked)
                      }
                    />
                  </div>

                  <Separator />

                  <div>
                    <Label>On Success Action</Label>
                    <Select
                      value={mergedParams.onSuccess || "continue"}
                      onValueChange={(value) =>
                        handleParameterChange("onSuccess", value)
                      }
                    >
                      <SelectTrigger>
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="continue">
                          Continue to Next Task
                        </SelectItem>
                        <SelectItem value="wait">
                          Wait for Confirmation
                        </SelectItem>
                        <SelectItem value="repeat">Repeat Task</SelectItem>
                        <SelectItem value="jump_to">
                          Jump to Specific Task
                        </SelectItem>
                      </SelectContent>
                    </Select>
                  </div>

                  <div>
                    <Label>On Failure Action</Label>
                    <Select
                      value={mergedParams.onFailure || "retry"}
                      onValueChange={(value) =>
                        handleParameterChange("onFailure", value)
                      }
                    >
                      <SelectTrigger>
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="retry">Retry Task</SelectItem>
                        <SelectItem value="skip">Skip Task</SelectItem>
                        <SelectItem value="abort">Abort Sequence</SelectItem>
                        <SelectItem value="manual">
                          Request Manual Intervention
                        </SelectItem>
                        <SelectItem value="alternative">
                          Try Alternative Method
                        </SelectItem>
                      </SelectContent>
                    </Select>
                  </div>

                  <div className="flex items-center justify-between">
                    <div>
                      <Label>Auto Recovery</Label>
                      <p className="text-sm text-muted-foreground">
                        Attempt automatic recovery on failure
                      </p>
                    </div>
                    <Switch defaultChecked />
                  </div>

                  {mergedParams.enableConditions && (
                    <div className="p-3 bg-primary/5 rounded-lg">
                      <p className="text-sm text-primary font-medium mb-2">
                        Conditional Logic Enabled
                      </p>
                      <p className="text-xs text-muted-foreground">
                        Configure your conditions in the Parameters tab. You can
                        use IF/ELSE statements, switch cases, and smart decision
                        trees.
                      </p>
                    </div>
                  )}
                </div>
              </Card>

              <Card className="p-4">
                <h3 className="font-semibold mb-3">Logging & Monitoring</h3>
                <div className="space-y-3">
                  <div className="flex items-center justify-between">
                    <Label>Enable Data Logging</Label>
                    <Switch defaultChecked />
                  </div>

                  <div className="flex items-center justify-between">
                    <Label>Performance Monitoring</Label>
                    <Switch defaultChecked />
                  </div>

                  <div className="flex items-center justify-between">
                    <Label>Real-time Telemetry</Label>
                    <Switch />
                  </div>
                </div>
              </Card>
            </div>
          </TabsContent>
        </Tabs>

        <DialogFooter className="fade-in-up flex gap-3">
          <Button
            variant="outline"
            onClick={onClose}
            className="glass-effect border-0 font-extralight hover-lift hover-scale"
          >
            ยกเลิก
          </Button>
          <Button
            onClick={handleSave}
            className="gradient-animate font-extralight hover-lift shadow-lg hover:shadow-xl transition-all duration-300"
          >
            บันทึกการตั้งค่า
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}
