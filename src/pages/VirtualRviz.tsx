import React, { useState, useEffect, useRef, useCallback } from "react";
import { useLanguage } from "@/contexts/LanguageContext";
import { useROSIntegration } from "@/services/ROSIntegrationService";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Switch } from "@/components/ui/switch";
import { Slider } from "@/components/ui/slider";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
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
  DialogTrigger,
} from "@/components/ui/dialog";
import {
  Accordion,
  AccordionContent,
  AccordionItem,
  AccordionTrigger,
} from "@/components/ui/accordion";
import { Tree, TreeViewElement } from "@/components/ui/tree-view";
import { useToast } from "@/hooks/use-toast";
import {
  Eye,
  EyeOff,
  RotateCcw,
  ZoomIn,
  ZoomOut,
  Move3d,
  Palette,
  Settings,
  Plus,
  Trash2,
  Box,
  Zap,
  MapPin,
  Camera,
  Grid3x3,
  Crosshair,
  RefreshCw,
  Play,
  Pause,
  Save,
  Download,
  Upload,
  Maximize,
  Minimize,
  Target,
  Navigation,
  ChevronRight,
  ChevronDown,
  Folder,
  FolderOpen,
  Info,
  AlertCircle,
  CheckCircle,
} from "lucide-react";

interface Display {
  id: string;
  name: string;
  type:
    | "pointcloud"
    | "markers"
    | "tf"
    | "robot_model"
    | "grid"
    | "axes"
    | "laser_scan"
    | "occupancy_grid"
    | "path"
    | "pose_array"
    | "pose"
    | "twist"
    | "image"
    | "camera_info"
    | "joint_states"
    | "imu"
    | "odometry";
  topic: string;
  enabled: boolean;
  color: string;
  size: number;
  alpha: number;
  frame: string;
  queueSize: number;
  settings: Record<string, any>;
  status: "ok" | "warn" | "error";
  messageType: string;
  frequency: number;
  lastMessage?: any;
}

interface TopicInfo {
  name: string;
  type: string;
  frequency: number;
  bandwidth: number;
  connections: number;
}

interface DisplayCategory {
  name: string;
  icon: any;
  displays: Display[];
  expanded: boolean;
}

interface ViewSettings {
  backgroundColor: string;
  gridEnabled: boolean;
  gridSize: number;
  gridColor: string;
  axesEnabled: boolean;
  axesSize: number;
  cameraType: "perspective" | "orthographic";
  fov: number;
  near: number;
  far: number;
  targetFrame: string;
}

interface CameraPosition {
  x: number;
  y: number;
  z: number;
  rotX: number;
  rotY: number;
  rotZ: number;
}

const VirtualRviz: React.FC = () => {
  const { t } = useLanguage();
  const { toast } = useToast();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [displays, setDisplays] = useState<Display[]>([]);
  const [displayCategories, setDisplayCategories] = useState<DisplayCategory[]>(
    [],
  );
  const [selectedDisplay, setSelectedDisplay] = useState<Display | null>(null);
  const [isPlaying, setIsPlaying] = useState(true);
  const [fps, setFps] = useState(30);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [isDragging, setIsDragging] = useState(false);
  const [lastMouse, setLastMouse] = useState({ x: 0, y: 0 });
  const [isAddDisplayOpen, setIsAddDisplayOpen] = useState(false);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [newDisplayType, setNewDisplayType] =
    useState<Display["type"]>("pointcloud");
  const [newDisplayTopic, setNewDisplayTopic] = useState("");
  const [newDisplayName, setNewDisplayName] = useState("");
  const [availableTopics, setAvailableTopics] = useState<TopicInfo[]>([]);
  const [filteredTopics, setFilteredTopics] = useState<TopicInfo[]>([]);
  const [topicFilter, setTopicFilter] = useState("");
  const [animationFrame, setAnimationFrame] = useState<number | null>(null);
  const [globalStatus, setGlobalStatus] = useState<"ok" | "warn" | "error">(
    "ok",
  );
  const [fixedFrame, setFixedFrame] = useState("map");
  const [targetFrame, setTargetFrame] = useState("base_link");

  const [viewSettings, setViewSettings] = useState<ViewSettings>({
    backgroundColor: "#2a2a2a",
    gridEnabled: true,
    gridSize: 10,
    gridColor: "#444444",
    axesEnabled: true,
    axesSize: 1,
    cameraType: "perspective",
    fov: 75,
    near: 0.1,
    far: 1000,
    targetFrame: "map",
  });

  const [cameraPosition, setCameraPosition] = useState<CameraPosition>({
    x: 5,
    y: 5,
    z: 5,
    rotX: -30,
    rotY: 45,
    rotZ: 0,
  });

  const { isConnected, subscribe, getTopics, callService } =
    useROSIntegration();

  // Initialize RViz-like default displays with categories
  useEffect(() => {
    const defaultDisplays: Display[] = [
      {
        id: "grid",
        name: "Grid",
        type: "grid",
        topic: "",
        enabled: true,
        color: "#444444",
        size: 10,
        alpha: 0.8,
        frame: "map",
        queueSize: 1,
        status: "ok",
        messageType: "",
        frequency: 0,
        settings: {
          cellSize: 1,
          cellCount: 10,
          lineWidth: 1,
          planeCellCount: 10,
          normalCellCount: 0,
          planeSize: 10,
        },
      },
      {
        id: "tf",
        name: "TF",
        type: "tf",
        topic: "/tf",
        enabled: true,
        color: "#00ff00",
        size: 0.5,
        alpha: 0.8,
        frame: "map",
        queueSize: 100,
        status: "ok",
        messageType: "tf2_msgs/TFMessage",
        frequency: 10,
        settings: {
          showNames: true,
          showAxes: true,
          showArrows: true,
          frameTimeout: 15,
          markerAlpha: 1,
          markerScale: 1,
        },
      },
    ];

    const categories: DisplayCategory[] = [
      {
        name: "Built-in",
        icon: Box,
        displays: defaultDisplays.filter((d) => d.type === "grid"),
        expanded: true,
      },
      {
        name: "TF",
        icon: Target,
        displays: defaultDisplays.filter((d) => d.type === "tf"),
        expanded: true,
      },
      {
        name: "Navigation",
        icon: Navigation,
        displays: [],
        expanded: false,
      },
      {
        name: "Sensors",
        icon: Zap,
        displays: [],
        expanded: false,
      },
      {
        name: "Robot",
        icon: Settings,
        displays: [],
        expanded: false,
      },
      {
        name: "Visualization",
        icon: Eye,
        displays: [],
        expanded: false,
      },
    ];

    setDisplays(defaultDisplays);
    setDisplayCategories(categories);
  }, []);

  // Load available topics with detailed information
  useEffect(() => {
    if (!isConnected) return;

    const loadTopics = async () => {
      try {
        // Mock topic data - in real implementation this would come from ROS
        const topicsData: TopicInfo[] = [
          {
            name: "/scan",
            type: "sensor_msgs/LaserScan",
            frequency: 10,
            bandwidth: 125,
            connections: 1,
          },
          {
            name: "/map",
            type: "nav_msgs/OccupancyGrid",
            frequency: 0,
            bandwidth: 0,
            connections: 1,
          },
          {
            name: "/tf",
            type: "tf2_msgs/TFMessage",
            frequency: 100,
            bandwidth: 50,
            connections: 3,
          },
          {
            name: "/tf_static",
            type: "tf2_msgs/TFMessage",
            frequency: 0,
            bandwidth: 0,
            connections: 2,
          },
          {
            name: "/cmd_vel",
            type: "geometry_msgs/Twist",
            frequency: 10,
            bandwidth: 25,
            connections: 1,
          },
          {
            name: "/odom",
            type: "nav_msgs/Odometry",
            frequency: 20,
            bandwidth: 75,
            connections: 2,
          },
          {
            name: "/amcl_pose",
            type: "geometry_msgs/PoseWithCovarianceStamped",
            frequency: 2,
            bandwidth: 15,
            connections: 1,
          },
          {
            name: "/move_base/goal",
            type: "move_base_msgs/MoveBaseActionGoal",
            frequency: 0,
            bandwidth: 0,
            connections: 1,
          },
          {
            name: "/move_base/status",
            type: "actionlib_msgs/GoalStatusArray",
            frequency: 5,
            bandwidth: 10,
            connections: 1,
          },
          {
            name: "/camera/image_raw",
            type: "sensor_msgs/Image",
            frequency: 30,
            bandwidth: 2000,
            connections: 1,
          },
          {
            name: "/camera/camera_info",
            type: "sensor_msgs/CameraInfo",
            frequency: 30,
            bandwidth: 50,
            connections: 1,
          },
          {
            name: "/joint_states",
            type: "sensor_msgs/JointState",
            frequency: 50,
            bandwidth: 100,
            connections: 1,
          },
          {
            name: "/imu/data",
            type: "sensor_msgs/Imu",
            frequency: 100,
            bandwidth: 200,
            connections: 1,
          },
          {
            name: "/pointcloud",
            type: "sensor_msgs/PointCloud2",
            frequency: 10,
            bandwidth: 1500,
            connections: 1,
          },
          {
            name: "/visualization_marker",
            type: "visualization_msgs/Marker",
            frequency: 1,
            bandwidth: 25,
            connections: 1,
          },
          {
            name: "/path",
            type: "nav_msgs/Path",
            frequency: 1,
            bandwidth: 50,
            connections: 1,
          },
          {
            name: "/goal_markers",
            type: "visualization_msgs/MarkerArray",
            frequency: 1,
            bandwidth: 30,
            connections: 1,
          },
        ];

        setAvailableTopics(topicsData);
        setFilteredTopics(topicsData);
      } catch (error) {
        console.error("Failed to load topics:", error);
        setGlobalStatus("error");
      }
    };

    loadTopics();
    const interval = setInterval(loadTopics, 5000); // Refresh every 5 seconds

    return () => clearInterval(interval);
  }, [isConnected, getTopics]);

  // Filter topics based on search
  useEffect(() => {
    if (!topicFilter) {
      setFilteredTopics(availableTopics);
    } else {
      setFilteredTopics(
        availableTopics.filter(
          (topic) =>
            topic.name.toLowerCase().includes(topicFilter.toLowerCase()) ||
            topic.type.toLowerCase().includes(topicFilter.toLowerCase()),
        ),
      );
    }
  }, [topicFilter, availableTopics]);

  // 3D Rendering
  const render3D = useCallback(() => {
    if (!canvasRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = viewSettings.backgroundColor;
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw grid
    if (viewSettings.gridEnabled) {
      drawGrid(ctx, canvas.width, canvas.height);
    }

    // Draw axes
    if (viewSettings.axesEnabled) {
      drawAxes(ctx, canvas.width, canvas.height);
    }

    // Draw all enabled displays
    displays.forEach((display) => {
      if (display.enabled) {
        drawDisplay(ctx, display, canvas.width, canvas.height);
      }
    });

    // Draw camera position indicator
    drawCameraInfo(ctx, canvas.width, canvas.height);
  }, [displays, viewSettings, cameraPosition]);

  const drawGrid = (
    ctx: CanvasRenderingContext2D,
    width: number,
    height: number,
  ) => {
    const centerX = width / 2;
    const centerY = height / 2;
    const gridSize = viewSettings.gridSize;
    const cellSize = 20; // pixels per meter

    ctx.strokeStyle = viewSettings.gridColor;
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.5;

    // Draw grid lines
    for (let i = -gridSize; i <= gridSize; i++) {
      const x = centerX + i * cellSize;
      const y = centerY + i * cellSize;

      // Vertical lines
      ctx.beginPath();
      ctx.moveTo(x, centerY - gridSize * cellSize);
      ctx.lineTo(x, centerY + gridSize * cellSize);
      ctx.stroke();

      // Horizontal lines
      ctx.beginPath();
      ctx.moveTo(centerX - gridSize * cellSize, y);
      ctx.lineTo(centerX + gridSize * cellSize, y);
      ctx.stroke();
    }

    ctx.globalAlpha = 1;
  };

  const drawAxes = (
    ctx: CanvasRenderingContext2D,
    width: number,
    height: number,
  ) => {
    const centerX = width / 2;
    const centerY = height / 2;
    const axisLength = viewSettings.axesSize * 50;

    ctx.lineWidth = 3;

    // X-axis (Red)
    ctx.strokeStyle = "#ff0000";
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX + axisLength, centerY);
    ctx.stroke();

    // Y-axis (Green)
    ctx.strokeStyle = "#00ff00";
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX, centerY - axisLength);
    ctx.stroke();

    // Z-axis (Blue) - simulated perspective
    ctx.strokeStyle = "#0000ff";
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX - axisLength * 0.7, centerY + axisLength * 0.7);
    ctx.stroke();

    // Draw axis labels
    ctx.fillStyle = "#ffffff";
    ctx.font = "14px sans-serif";
    ctx.fillText("X", centerX + axisLength + 5, centerY);
    ctx.fillText("Y", centerX, centerY - axisLength - 5);
    ctx.fillText(
      "Z",
      centerX - axisLength * 0.7 - 15,
      centerY + axisLength * 0.7 + 15,
    );
  };

  const drawDisplay = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    width: number,
    height: number,
  ) => {
    const centerX = width / 2;
    const centerY = height / 2;

    ctx.globalAlpha = display.alpha;

    switch (display.type) {
      case "pointcloud":
        drawPointCloud(ctx, display, centerX, centerY);
        break;
      case "markers":
        drawMarkers(ctx, display, centerX, centerY);
        break;
      case "tf":
        drawTF(ctx, display, centerX, centerY);
        break;
      case "robot_model":
        drawRobotModel(ctx, display, centerX, centerY);
        break;
      case "laser_scan":
        drawLaserScan(ctx, display, centerX, centerY);
        break;
      case "path":
        drawPath(ctx, display, centerX, centerY);
        break;
      case "pose_array":
        drawPoseArray(ctx, display, centerX, centerY);
        break;
    }

    ctx.globalAlpha = 1;
  };

  const drawPointCloud = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    centerX: number,
    centerY: number,
  ) => {
    ctx.fillStyle = display.color;

    // Simulate point cloud data
    for (let i = 0; i < 1000; i++) {
      const x = centerX + (Math.random() - 0.5) * 400;
      const y = centerY + (Math.random() - 0.5) * 400;
      ctx.fillRect(x, y, display.size, display.size);
    }
  };

  const drawMarkers = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    centerX: number,
    centerY: number,
  ) => {
    ctx.fillStyle = display.color;
    ctx.strokeStyle = display.color;
    ctx.lineWidth = 2;

    // Draw sample markers
    const markers = [
      { x: centerX + 50, y: centerY - 50, type: "sphere" },
      { x: centerX - 50, y: centerY + 50, type: "cube" },
      { x: centerX + 100, y: centerY + 100, type: "arrow" },
    ];

    markers.forEach((marker) => {
      switch (marker.type) {
        case "sphere":
          ctx.beginPath();
          ctx.arc(marker.x, marker.y, display.size * 10, 0, 2 * Math.PI);
          ctx.fill();
          break;
        case "cube":
          ctx.fillRect(
            marker.x - display.size * 5,
            marker.y - display.size * 5,
            display.size * 10,
            display.size * 10,
          );
          break;
        case "arrow":
          drawArrow(
            ctx,
            marker.x,
            marker.y,
            marker.x + 30,
            marker.y - 30,
            display.size * 5,
          );
          break;
      }
    });
  };

  const drawTF = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    centerX: number,
    centerY: number,
  ) => {
    // Draw TF frames as coordinate systems
    const frames = [
      { name: "map", x: centerX, y: centerY },
      { name: "odom", x: centerX + 20, y: centerY + 10 },
      { name: "base_link", x: centerX + 40, y: centerY + 20 },
      { name: "laser", x: centerX + 60, y: centerY + 10 },
    ];

    frames.forEach((frame) => {
      // Draw small coordinate system
      const size = display.size * 20;

      // X-axis (red)
      ctx.strokeStyle = "#ff0000";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(frame.x, frame.y);
      ctx.lineTo(frame.x + size, frame.y);
      ctx.stroke();

      // Y-axis (green)
      ctx.strokeStyle = "#00ff00";
      ctx.beginPath();
      ctx.moveTo(frame.x, frame.y);
      ctx.lineTo(frame.x, frame.y - size);
      ctx.stroke();

      // Frame name
      if (display.settings.showNames) {
        ctx.fillStyle = "#ffffff";
        ctx.font = "10px sans-serif";
        ctx.fillText(frame.name, frame.x + size + 5, frame.y);
      }
    });
  };

  const drawRobotModel = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    centerX: number,
    centerY: number,
  ) => {
    ctx.fillStyle = display.color;
    ctx.strokeStyle = display.color;
    ctx.lineWidth = 2;

    // Simple robot representation
    const robotX = centerX + 40;
    const robotY = centerY + 20;
    const size = display.size * 30;

    // Main body (rectangle)
    ctx.fillRect(robotX - size / 2, robotY - size / 3, size, size / 1.5);

    // Direction indicator (triangle)
    ctx.beginPath();
    ctx.moveTo(robotX + size / 2, robotY);
    ctx.lineTo(robotX + size, robotY - size / 4);
    ctx.lineTo(robotX + size, robotY + size / 4);
    ctx.closePath();
    ctx.fill();

    // Wheels
    ctx.fillStyle = "#333333";
    ctx.fillRect(robotX - size / 2 - 5, robotY - size / 2, 5, 10);
    ctx.fillRect(robotX - size / 2 - 5, robotY + size / 2 - 10, 5, 10);
    ctx.fillRect(robotX + size / 2, robotY - size / 2, 5, 10);
    ctx.fillRect(robotX + size / 2, robotY + size / 2 - 10, 5, 10);
  };

  const drawLaserScan = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    centerX: number,
    centerY: number,
  ) => {
    ctx.strokeStyle = display.color;
    ctx.lineWidth = 1;

    const robotX = centerX + 40;
    const robotY = centerY + 20;

    // Simulate laser scan data
    for (let angle = 0; angle < 2 * Math.PI; angle += 0.1) {
      const range = 100 + Math.random() * 100; // Random range
      const endX = robotX + Math.cos(angle) * range;
      const endY = robotY + Math.sin(angle) * range;

      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(endX, endY);
      ctx.stroke();
    }
  };

  const drawPath = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    centerX: number,
    centerY: number,
  ) => {
    ctx.strokeStyle = display.color;
    ctx.lineWidth = display.size;

    // Sample path
    const path = [
      { x: centerX, y: centerY },
      { x: centerX + 50, y: centerY - 20 },
      { x: centerX + 100, y: centerY + 30 },
      { x: centerX + 150, y: centerY - 10 },
      { x: centerX + 200, y: centerY + 40 },
    ];

    ctx.beginPath();
    ctx.moveTo(path[0].x, path[0].y);
    for (let i = 1; i < path.length; i++) {
      ctx.lineTo(path[i].x, path[i].y);
    }
    ctx.stroke();
  };

  const drawPoseArray = (
    ctx: CanvasRenderingContext2D,
    display: Display,
    centerX: number,
    centerY: number,
  ) => {
    // Draw multiple pose arrows
    const poses = [
      { x: centerX + 50, y: centerY + 50, angle: 0 },
      { x: centerX - 50, y: centerY - 50, angle: Math.PI / 4 },
      { x: centerX + 100, y: centerY - 100, angle: Math.PI / 2 },
    ];

    poses.forEach((pose) => {
      const arrowLength = display.size * 20;
      const endX = pose.x + Math.cos(pose.angle) * arrowLength;
      const endY = pose.y + Math.sin(pose.angle) * arrowLength;

      drawArrow(ctx, pose.x, pose.y, endX, endY, display.size * 5);
    });
  };

  const drawArrow = (
    ctx: CanvasRenderingContext2D,
    fromX: number,
    fromY: number,
    toX: number,
    toY: number,
    headLength: number,
  ) => {
    ctx.strokeStyle = ctx.fillStyle;
    ctx.lineWidth = 2;

    // Draw line
    ctx.beginPath();
    ctx.moveTo(fromX, fromY);
    ctx.lineTo(toX, toY);
    ctx.stroke();

    // Draw arrowhead
    const angle = Math.atan2(toY - fromY, toX - fromX);
    ctx.beginPath();
    ctx.moveTo(toX, toY);
    ctx.lineTo(
      toX - headLength * Math.cos(angle - Math.PI / 6),
      toY - headLength * Math.sin(angle - Math.PI / 6),
    );
    ctx.moveTo(toX, toY);
    ctx.lineTo(
      toX - headLength * Math.cos(angle + Math.PI / 6),
      toY - headLength * Math.sin(angle + Math.PI / 6),
    );
    ctx.stroke();
  };

  const drawCameraInfo = (
    ctx: CanvasRenderingContext2D,
    width: number,
    height: number,
  ) => {
    ctx.fillStyle = "rgba(0, 0, 0, 0.7)";
    ctx.fillRect(10, height - 80, 200, 70);

    ctx.fillStyle = "#ffffff";
    ctx.font = "12px sans-serif";
    ctx.fillText(
      `Camera: ${cameraPosition.x.toFixed(1)}, ${cameraPosition.y.toFixed(1)}, ${cameraPosition.z.toFixed(1)}`,
      20,
      height - 60,
    );
    ctx.fillText(
      `Rotation: ${cameraPosition.rotX.toFixed(1)}°, ${cameraPosition.rotY.toFixed(1)}°`,
      20,
      height - 45,
    );
    ctx.fillText(`Target Frame: ${viewSettings.targetFrame}`, 20, height - 30);
    ctx.fillText(`FPS: ${fps}`, 20, height - 15);
  };

  // Animation loop
  useEffect(() => {
    if (isPlaying) {
      const animate = () => {
        render3D();
        setAnimationFrame(requestAnimationFrame(animate));
      };
      setAnimationFrame(requestAnimationFrame(animate));
    } else {
      if (animationFrame) {
        cancelAnimationFrame(animationFrame);
        setAnimationFrame(null);
      }
    }

    return () => {
      if (animationFrame) {
        cancelAnimationFrame(animationFrame);
      }
    };
  }, [isPlaying, render3D]);

  // Mouse controls
  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    setLastMouse({ x: e.clientX, y: e.clientY });
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (!isDragging) return;

    const deltaX = e.clientX - lastMouse.x;
    const deltaY = e.clientY - lastMouse.y;

    setCameraPosition((prev) => ({
      ...prev,
      rotY: prev.rotY + deltaX * 0.5,
      rotX: prev.rotX - deltaY * 0.5,
    }));

    setLastMouse({ x: e.clientX, y: e.clientY });
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const zoomFactor = e.deltaY > 0 ? 1.1 : 0.9;
    setCameraPosition((prev) => ({
      ...prev,
      z: Math.max(0.5, Math.min(50, prev.z * zoomFactor)),
    }));
  };

  const addDisplay = () => {
    if (!newDisplayName || !newDisplayTopic) return;

    const newDisplay: Display = {
      id: Date.now().toString(),
      name: newDisplayName,
      type: newDisplayType,
      topic: newDisplayTopic,
      enabled: true,
      color: "#ffffff",
      size: 1,
      alpha: 1,
      frame: "map",
      queueSize: 10,
      settings: {},
    };

    setDisplays((prev) => [...prev, newDisplay]);
    setIsAddDisplayOpen(false);
    setNewDisplayName("");
    setNewDisplayTopic("");

    toast({
      title: t("rviz.displayAdded"),
      description: t("rviz.displayAddedSuccessfully"),
    });
  };

  const removeDisplay = (displayId: string) => {
    setDisplays((prev) => prev.filter((d) => d.id !== displayId));
    if (selectedDisplay?.id === displayId) {
      setSelectedDisplay(null);
    }
  };

  const toggleDisplay = (displayId: string) => {
    setDisplays((prev) =>
      prev.map((d) => (d.id === displayId ? { ...d, enabled: !d.enabled } : d)),
    );
  };

  const resetCamera = () => {
    setCameraPosition({
      x: 5,
      y: 5,
      z: 5,
      rotX: -30,
      rotY: 45,
      rotZ: 0,
    });
  };

  const displayTypeOptions = [
    {
      value: "pointcloud",
      label: "PointCloud2",
      description: "Display 3D point cloud data (sensor_msgs/PointCloud2)",
    },
    {
      value: "markers",
      label: "Marker",
      description: "Display 3D markers and shapes (visualization_msgs/Marker)",
    },
    {
      value: "tf",
      label: "TF",
      description: "Display coordinate frame transforms (tf2_msgs/TFMessage)",
    },
    {
      value: "robot_model",
      label: "RobotModel",
      description: "Display robot URDF model",
    },
    {
      value: "grid",
      label: "Grid",
      description: "Display reference grid in fixed frame",
    },
    {
      value: "axes",
      label: "Axes",
      description: "Display coordinate axes",
    },
    {
      value: "laser_scan",
      label: "LaserScan",
      description: "Display 2D laser scan data (sensor_msgs/LaserScan)",
    },
    {
      value: "occupancy_grid",
      label: "Map",
      description: "Display occupancy grid map (nav_msgs/OccupancyGrid)",
    },
    {
      value: "path",
      label: "Path",
      description: "Display navigation path (nav_msgs/Path)",
    },
    {
      value: "pose_array",
      label: "PoseArray",
      description: "Display array of poses (geometry_msgs/PoseArray)",
    },
    {
      value: "pose",
      label: "Pose",
      description: "Display single pose (geometry_msgs/PoseStamped)",
    },
    {
      value: "odometry",
      label: "Odometry",
      description: "Display odometry data (nav_msgs/Odometry)",
    },
    {
      value: "image",
      label: "Image",
      description: "Display camera image (sensor_msgs/Image)",
    },
    {
      value: "imu",
      label: "IMU",
      description: "Display IMU orientation (sensor_msgs/Imu)",
    },
  ];

  return (
    <div className="h-screen flex flex-col bg-background">
      {/* RViz-style Header */}
      <div className="border-b border-border/50 bg-background/95 backdrop-blur-sm">
        <div className="flex items-center justify-between px-4 py-3">
          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2">
              <Box className="h-6 w-6 text-primary" />
              <h1 className="text-xl font-semibold">{t("rviz.title")}</h1>
            </div>

            {/* Global Status */}
            <div className="flex items-center gap-2">
              <div
                className={`w-2 h-2 rounded-full ${
                  globalStatus === "ok"
                    ? "bg-green-400"
                    : globalStatus === "warn"
                      ? "bg-yellow-400"
                      : "bg-red-400"
                }`}
              />
              <Badge
                variant={isConnected ? "default" : "secondary"}
                className="text-xs"
              >
                {isConnected ? "Connected" : "Disconnected"}
              </Badge>
            </div>
          </div>

          {/* Top Controls */}
          <div className="flex items-center gap-2">
            <Button
              variant="outline"
              size="sm"
              onClick={() => setIsPlaying(!isPlaying)}
              className="gap-2"
            >
              {isPlaying ? (
                <Pause className="h-4 w-4" />
              ) : (
                <Play className="h-4 w-4" />
              )}
              {isPlaying ? "Pause" : "Play"}
            </Button>

            <Dialog open={isSettingsOpen} onOpenChange={setIsSettingsOpen}>
              <DialogTrigger asChild>
                <Button variant="outline" size="sm">
                  <Settings className="h-4 w-4" />
                </Button>
              </DialogTrigger>
              <DialogContent>
                <DialogHeader>
                  <DialogTitle>Global Options</DialogTitle>
                </DialogHeader>
                <div className="space-y-4">
                  <div>
                    <Label>Fixed Frame</Label>
                    <Input
                      value={fixedFrame}
                      onChange={(e) => setFixedFrame(e.target.value)}
                      placeholder="map"
                    />
                  </div>
                  <div>
                    <Label>Target Frame</Label>
                    <Input
                      value={targetFrame}
                      onChange={(e) => setTargetFrame(e.target.value)}
                      placeholder="base_link"
                    />
                  </div>
                  <div>
                    <Label>Frame Rate: {fps} Hz</Label>
                    <Slider
                      value={[fps]}
                      onValueChange={([value]) => setFps(value)}
                      min={1}
                      max={60}
                      step={1}
                      className="mt-2"
                    />
                  </div>
                </div>
              </DialogContent>
            </Dialog>
          </div>
        </div>
      </div>

      <div className="flex-1 flex overflow-hidden">
        {/* Left Panel - Displays */}
        <div className="w-80 border-r border-border/50 bg-background/50 backdrop-blur-sm flex flex-col">
          {/* Displays Header */}
          <div className="border-b border-border/50 p-3">
            <div className="flex items-center justify-between mb-3">
              <h3 className="font-semibold text-sm">Displays</h3>
              <Dialog
                open={isAddDisplayOpen}
                onOpenChange={setIsAddDisplayOpen}
              >
                <DialogTrigger asChild>
                  <Button size="sm" className="h-7">
                    <Plus className="h-3 w-3 mr-1" />
                    Add
                  </Button>
                </DialogTrigger>
                <DialogContent className="max-w-2xl">
                  <DialogHeader>
                    <DialogTitle>Add Display</DialogTitle>
                    <DialogDescription>
                      Choose a display type and topic to visualize ROS data
                    </DialogDescription>
                  </DialogHeader>

                  <div className="grid grid-cols-2 gap-4">
                    {/* Display Types */}
                    <div className="space-y-3">
                      <Label>Display Type</Label>
                      <ScrollArea className="h-60 border rounded-md">
                        <div className="p-2">
                          {displayTypeOptions.map((option) => (
                            <div
                              key={option.value}
                              className={`p-2 rounded cursor-pointer hover:bg-accent ${
                                newDisplayType === option.value
                                  ? "bg-primary/10 border border-primary"
                                  : ""
                              }`}
                              onClick={() =>
                                setNewDisplayType(
                                  option.value as Display["type"],
                                )
                              }
                            >
                              <div className="font-medium text-sm">
                                {option.label}
                              </div>
                              <div className="text-xs text-muted-foreground">
                                {option.description}
                              </div>
                            </div>
                          ))}
                        </div>
                      </ScrollArea>
                    </div>

                    {/* Topics */}
                    <div className="space-y-3">
                      <Label>Available Topics</Label>
                      <Input
                        placeholder="Filter topics..."
                        value={topicFilter}
                        onChange={(e) => setTopicFilter(e.target.value)}
                        className="h-8"
                      />
                      <ScrollArea className="h-52 border rounded-md">
                        <div className="p-2">
                          {filteredTopics.map((topic) => (
                            <div
                              key={topic.name}
                              className={`p-2 rounded cursor-pointer hover:bg-accent ${
                                newDisplayTopic === topic.name
                                  ? "bg-primary/10 border border-primary"
                                  : ""
                              }`}
                              onClick={() => {
                                setNewDisplayTopic(topic.name);
                                if (!newDisplayName) {
                                  setNewDisplayName(
                                    topic.name.replace("/", ""),
                                  );
                                }
                              }}
                            >
                              <div className="font-medium text-sm">
                                {topic.name}
                              </div>
                              <div className="text-xs text-muted-foreground">
                                {topic.type}
                              </div>
                              <div className="text-xs text-muted-foreground">
                                {topic.frequency > 0
                                  ? `${topic.frequency} Hz`
                                  : "Latched"}
                              </div>
                            </div>
                          ))}
                        </div>
                      </ScrollArea>
                    </div>
                  </div>

                  <div className="space-y-3">
                    <div>
                      <Label>Display Name</Label>
                      <Input
                        value={newDisplayName}
                        onChange={(e) => setNewDisplayName(e.target.value)}
                        placeholder="Enter display name"
                      />
                    </div>
                  </div>

                  <DialogFooter>
                    <Button
                      variant="outline"
                      onClick={() => setIsAddDisplayOpen(false)}
                    >
                      Cancel
                    </Button>
                    <Button
                      onClick={addDisplay}
                      disabled={!newDisplayName || !newDisplayTopic}
                    >
                      OK
                    </Button>
                  </DialogFooter>
                </DialogContent>
              </Dialog>
            </div>
          </div>

          {/* Displays Tree */}
          <ScrollArea className="flex-1">
            <div className="p-2">
              {displayCategories.map((category) => (
                <div key={category.name} className="mb-2">
                  <div
                    className="flex items-center gap-2 p-2 hover:bg-accent/50 rounded cursor-pointer"
                    onClick={() => {
                      setDisplayCategories((prev) =>
                        prev.map((cat) =>
                          cat.name === category.name
                            ? { ...cat, expanded: !cat.expanded }
                            : cat,
                        ),
                      );
                    }}
                  >
                    {category.expanded ? (
                      <ChevronDown className="h-4 w-4" />
                    ) : (
                      <ChevronRight className="h-4 w-4" />
                    )}
                    {category.expanded ? (
                      <FolderOpen className="h-4 w-4 text-blue-500" />
                    ) : (
                      <Folder className="h-4 w-4 text-blue-500" />
                    )}
                    <span className="text-sm font-medium">{category.name}</span>
                    <span className="text-xs text-muted-foreground">
                      ({category.displays.length})
                    </span>
                  </div>

                  {category.expanded && (
                    <div className="ml-6 space-y-1">
                      {category.displays.map((display) => (
                        <div
                          key={display.id}
                          className={`flex items-center gap-2 p-2 rounded cursor-pointer transition-colors ${
                            selectedDisplay?.id === display.id
                              ? "bg-primary/10 border border-primary/20"
                              : "hover:bg-accent/50"
                          }`}
                          onClick={() => setSelectedDisplay(display)}
                        >
                          <Switch
                            checked={display.enabled}
                            onCheckedChange={() => toggleDisplay(display.id)}
                            size="sm"
                          />
                          <div className="flex-1 min-w-0">
                            <div className="flex items-center gap-2">
                              <span className="text-sm font-medium truncate">
                                {display.name}
                              </span>
                              {display.status === "ok" && (
                                <CheckCircle className="h-3 w-3 text-green-500" />
                              )}
                              {display.status === "warn" && (
                                <AlertCircle className="h-3 w-3 text-yellow-500" />
                              )}
                              {display.status === "error" && (
                                <AlertCircle className="h-3 w-3 text-red-500" />
                              )}
                            </div>
                            <div className="text-xs text-muted-foreground truncate">
                              {display.topic || display.type}
                            </div>
                          </div>
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={(e) => {
                              e.stopPropagation();
                              removeDisplay(display.id);
                            }}
                            className="h-6 w-6 p-0 opacity-50 hover:opacity-100"
                          >
                            <Trash2 className="h-3 w-3" />
                          </Button>
                        </div>
                      ))}

                      {/* Show unassigned displays */}
                      {displays
                        .filter(
                          (d) =>
                            !category.displays.some((cd) => cd.id === d.id),
                        )
                        .filter((d) => {
                          if (category.name === "Built-in")
                            return ["grid", "axes"].includes(d.type);
                          if (category.name === "TF") return d.type === "tf";
                          if (category.name === "Navigation")
                            return [
                              "path",
                              "pose",
                              "pose_array",
                              "occupancy_grid",
                            ].includes(d.type);
                          if (category.name === "Sensors")
                            return [
                              "laser_scan",
                              "pointcloud",
                              "imu",
                              "image",
                            ].includes(d.type);
                          if (category.name === "Robot")
                            return ["robot_model", "joint_states"].includes(
                              d.type,
                            );
                          if (category.name === "Visualization")
                            return ["markers"].includes(d.type);
                          return false;
                        })
                        .map((display) => (
                          <div
                            key={display.id}
                            className={`flex items-center gap-2 p-2 rounded cursor-pointer transition-colors ${
                              selectedDisplay?.id === display.id
                                ? "bg-primary/10 border border-primary/20"
                                : "hover:bg-accent/50"
                            }`}
                            onClick={() => setSelectedDisplay(display)}
                          >
                            <Switch
                              checked={display.enabled}
                              onCheckedChange={() => toggleDisplay(display.id)}
                              size="sm"
                            />
                            <div className="flex-1 min-w-0">
                              <div className="flex items-center gap-2">
                                <span className="text-sm font-medium truncate">
                                  {display.name}
                                </span>
                                {display.status === "ok" && (
                                  <CheckCircle className="h-3 w-3 text-green-500" />
                                )}
                                {display.status === "warn" && (
                                  <AlertCircle className="h-3 w-3 text-yellow-500" />
                                )}
                                {display.status === "error" && (
                                  <AlertCircle className="h-3 w-3 text-red-500" />
                                )}
                              </div>
                              <div className="text-xs text-muted-foreground truncate">
                                {display.topic || display.type}
                              </div>
                            </div>
                            <Button
                              variant="ghost"
                              size="sm"
                              onClick={(e) => {
                                e.stopPropagation();
                                removeDisplay(display.id);
                              }}
                              className="h-6 w-6 p-0 opacity-50 hover:opacity-100"
                            >
                              <Trash2 className="h-3 w-3" />
                            </Button>
                          </div>
                        ))}
                    </div>
                  )}
                </div>
              ))}
            </div>
          </ScrollArea>
        </div>

        {/* Main 3D Viewer */}
        <div className="flex-1 flex flex-col">
          {/* Viewer Controls */}
          <div className="border-b border-border/50 p-2 bg-background/50">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Button variant="outline" size="sm" onClick={resetCamera}>
                  <RotateCcw className="h-4 w-4" />
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() =>
                    setCameraPosition((prev) => ({ ...prev, z: prev.z * 0.8 }))
                  }
                >
                  <ZoomIn className="h-4 w-4" />
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() =>
                    setCameraPosition((prev) => ({ ...prev, z: prev.z * 1.2 }))
                  }
                >
                  <ZoomOut className="h-4 w-4" />
                </Button>
                <Separator orientation="vertical" className="h-6" />
                <span className="text-sm text-muted-foreground">
                  Fixed Frame: {fixedFrame}
                </span>
              </div>

              <div className="flex items-center gap-2 text-sm text-muted-foreground">
                <span>FPS: {fps}</span>
                <div
                  className={`w-2 h-2 rounded-full ${isPlaying ? "bg-green-400" : "bg-red-400"}`}
                />
              </div>
            </div>
          </div>

          {/* 3D Canvas */}
          <div className="flex-1 relative bg-gray-900">
            <canvas
              ref={canvasRef}
              width={800}
              height={600}
              className="w-full h-full cursor-move"
              onMouseDown={handleMouseDown}
              onMouseMove={handleMouseMove}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              onWheel={handleWheel}
            />

            {/* View Info Overlay */}
            <div className="absolute bottom-4 left-4 bg-black/80 text-white p-2 rounded text-xs">
              <div>
                Camera: ({cameraPosition.x.toFixed(1)},{" "}
                {cameraPosition.y.toFixed(1)}, {cameraPosition.z.toFixed(1)})
              </div>
              <div>
                Rotation: ({cameraPosition.rotX.toFixed(0)}°,{" "}
                {cameraPosition.rotY.toFixed(0)}°)
              </div>
              <div>Zoom: {((1 / cameraPosition.z) * 10).toFixed(1)}x</div>
            </div>
          </div>
        </div>

        {/* Right Panel - Properties */}
        {selectedDisplay && (
          <div className="w-80 border-l border-border/50 bg-background/50 backdrop-blur-sm flex flex-col">
            <div className="border-b border-border/50 p-3">
              <h3 className="font-semibold text-sm">Properties</h3>
              <p className="text-xs text-muted-foreground">
                {selectedDisplay.name}
              </p>
            </div>

            <ScrollArea className="flex-1 p-3">
              <div className="space-y-4">
                {/* Status */}
                <div className="space-y-2">
                  <Label className="text-xs font-semibold">Status</Label>
                  <div className="flex items-center gap-2">
                    {selectedDisplay.status === "ok" && (
                      <>
                        <CheckCircle className="h-4 w-4 text-green-500" />
                        <span className="text-sm">OK</span>
                      </>
                    )}
                    {selectedDisplay.status === "warn" && (
                      <>
                        <AlertCircle className="h-4 w-4 text-yellow-500" />
                        <span className="text-sm">Warning</span>
                      </>
                    )}
                    {selectedDisplay.status === "error" && (
                      <>
                        <AlertCircle className="h-4 w-4 text-red-500" />
                        <span className="text-sm">Error</span>
                      </>
                    )}
                  </div>
                </div>

                <Separator />

                {/* Basic Properties */}
                <div className="space-y-3">
                  <div>
                    <Label className="text-xs">Display Name</Label>
                    <Input
                      value={selectedDisplay.name}
                      onChange={(e) =>
                        setDisplays((prev) =>
                          prev.map((d) =>
                            d.id === selectedDisplay.id
                              ? { ...d, name: e.target.value }
                              : d,
                          ),
                        )
                      }
                      className="h-8 text-sm"
                    />
                  </div>

                  <div>
                    <Label className="text-xs">Topic</Label>
                    <Select
                      value={selectedDisplay.topic}
                      onValueChange={(value) =>
                        setDisplays((prev) =>
                          prev.map((d) =>
                            d.id === selectedDisplay.id
                              ? { ...d, topic: value }
                              : d,
                          ),
                        )
                      }
                    >
                      <SelectTrigger className="h-8 text-sm">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        {availableTopics.map((topic) => (
                          <SelectItem key={topic.name} value={topic.name}>
                            <div>
                              <div className="font-medium">{topic.name}</div>
                              <div className="text-xs text-muted-foreground">
                                {topic.type}
                              </div>
                            </div>
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                  </div>

                  {selectedDisplay.topic && (
                    <div className="text-xs text-muted-foreground space-y-1">
                      <div>Type: {selectedDisplay.messageType}</div>
                      <div>
                        Rate:{" "}
                        {selectedDisplay.frequency > 0
                          ? `${selectedDisplay.frequency} Hz`
                          : "Latched"}
                      </div>
                    </div>
                  )}
                </div>

                <Separator />

                {/* Visual Properties */}
                <div className="space-y-3">
                  <Label className="text-xs font-semibold">Visual</Label>

                  <div>
                    <Label className="text-xs">Color</Label>
                    <div className="flex gap-2">
                      <Input
                        type="color"
                        value={selectedDisplay.color}
                        onChange={(e) =>
                          setDisplays((prev) =>
                            prev.map((d) =>
                              d.id === selectedDisplay.id
                                ? { ...d, color: e.target.value }
                                : d,
                            ),
                          )
                        }
                        className="w-12 h-8 p-1"
                      />
                      <Input
                        value={selectedDisplay.color}
                        onChange={(e) =>
                          setDisplays((prev) =>
                            prev.map((d) =>
                              d.id === selectedDisplay.id
                                ? { ...d, color: e.target.value }
                                : d,
                            ),
                          )
                        }
                        className="flex-1 h-8 text-sm"
                      />
                    </div>
                  </div>

                  <div>
                    <Label className="text-xs">
                      Alpha: {(selectedDisplay.alpha * 100).toFixed(0)}%
                    </Label>
                    <Slider
                      value={[selectedDisplay.alpha]}
                      onValueChange={([value]) =>
                        setDisplays((prev) =>
                          prev.map((d) =>
                            d.id === selectedDisplay.id
                              ? { ...d, alpha: value }
                              : d,
                          ),
                        )
                      }
                      min={0}
                      max={1}
                      step={0.01}
                      className="mt-1"
                    />
                  </div>

                  <div>
                    <Label className="text-xs">
                      Size: {selectedDisplay.size.toFixed(2)}
                    </Label>
                    <Slider
                      value={[selectedDisplay.size]}
                      onValueChange={([value]) =>
                        setDisplays((prev) =>
                          prev.map((d) =>
                            d.id === selectedDisplay.id
                              ? { ...d, size: value }
                              : d,
                          ),
                        )
                      }
                      min={0.1}
                      max={5}
                      step={0.1}
                      className="mt-1"
                    />
                  </div>
                </div>

                <Separator />

                {/* Frame */}
                <div className="space-y-3">
                  <Label className="text-xs font-semibold">Transform</Label>

                  <div>
                    <Label className="text-xs">Reference Frame</Label>
                    <Input
                      value={selectedDisplay.frame}
                      onChange={(e) =>
                        setDisplays((prev) =>
                          prev.map((d) =>
                            d.id === selectedDisplay.id
                              ? { ...d, frame: e.target.value }
                              : d,
                          ),
                        )
                      }
                      className="h-8 text-sm"
                      placeholder="map"
                    />
                  </div>

                  <div>
                    <Label className="text-xs">Queue Size</Label>
                    <Input
                      type="number"
                      value={selectedDisplay.queueSize}
                      onChange={(e) =>
                        setDisplays((prev) =>
                          prev.map((d) =>
                            d.id === selectedDisplay.id
                              ? {
                                  ...d,
                                  queueSize: parseInt(e.target.value) || 1,
                                }
                              : d,
                          ),
                        )
                      }
                      className="h-8 text-sm"
                      min="1"
                      max="1000"
                    />
                  </div>
                </div>

                {/* Type-specific settings */}
                {selectedDisplay.type === "grid" && (
                  <>
                    <Separator />
                    <div className="space-y-3">
                      <Label className="text-xs font-semibold">
                        Grid Settings
                      </Label>

                      <div>
                        <Label className="text-xs">Cell Count</Label>
                        <Input
                          type="number"
                          value={selectedDisplay.settings.cellCount || 10}
                          onChange={(e) =>
                            setDisplays((prev) =>
                              prev.map((d) =>
                                d.id === selectedDisplay.id
                                  ? {
                                      ...d,
                                      settings: {
                                        ...d.settings,
                                        cellCount:
                                          parseInt(e.target.value) || 10,
                                      },
                                    }
                                  : d,
                              ),
                            )
                          }
                          className="h-8 text-sm"
                          min="1"
                          max="100"
                        />
                      </div>

                      <div>
                        <Label className="text-xs">Cell Size</Label>
                        <Input
                          type="number"
                          step="0.1"
                          value={selectedDisplay.settings.cellSize || 1}
                          onChange={(e) =>
                            setDisplays((prev) =>
                              prev.map((d) =>
                                d.id === selectedDisplay.id
                                  ? {
                                      ...d,
                                      settings: {
                                        ...d.settings,
                                        cellSize:
                                          parseFloat(e.target.value) || 1,
                                      },
                                    }
                                  : d,
                              ),
                            )
                          }
                          className="h-8 text-sm"
                          min="0.1"
                          max="10"
                        />
                      </div>
                    </div>
                  </>
                )}

                {selectedDisplay.type === "tf" && (
                  <>
                    <Separator />
                    <div className="space-y-3">
                      <Label className="text-xs font-semibold">
                        TF Settings
                      </Label>

                      <div className="flex items-center justify-between">
                        <Label className="text-xs">Show Frame Names</Label>
                        <Switch
                          checked={selectedDisplay.settings.showNames || true}
                          onCheckedChange={(checked) =>
                            setDisplays((prev) =>
                              prev.map((d) =>
                                d.id === selectedDisplay.id
                                  ? {
                                      ...d,
                                      settings: {
                                        ...d.settings,
                                        showNames: checked,
                                      },
                                    }
                                  : d,
                              ),
                            )
                          }
                          size="sm"
                        />
                      </div>

                      <div className="flex items-center justify-between">
                        <Label className="text-xs">Show Axes</Label>
                        <Switch
                          checked={selectedDisplay.settings.showAxes || true}
                          onCheckedChange={(checked) =>
                            setDisplays((prev) =>
                              prev.map((d) =>
                                d.id === selectedDisplay.id
                                  ? {
                                      ...d,
                                      settings: {
                                        ...d.settings,
                                        showAxes: checked,
                                      },
                                    }
                                  : d,
                              ),
                            )
                          }
                          size="sm"
                        />
                      </div>

                      <div>
                        <Label className="text-xs">Frame Timeout (s)</Label>
                        <Input
                          type="number"
                          value={selectedDisplay.settings.frameTimeout || 15}
                          onChange={(e) =>
                            setDisplays((prev) =>
                              prev.map((d) =>
                                d.id === selectedDisplay.id
                                  ? {
                                      ...d,
                                      settings: {
                                        ...d.settings,
                                        frameTimeout:
                                          parseFloat(e.target.value) || 15,
                                      },
                                    }
                                  : d,
                              ),
                            )
                          }
                          className="h-8 text-sm"
                          min="0.1"
                          max="60"
                          step="0.1"
                        />
                      </div>
                    </div>
                  </>
                )}
              </div>
            </ScrollArea>
          </div>
        )}
      </div>
    </div>
  );
};

export default VirtualRviz;
