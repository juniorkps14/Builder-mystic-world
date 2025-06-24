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
    | "pose_array";
  topic: string;
  enabled: boolean;
  color: string;
  size: number;
  alpha: number;
  frame: string;
  queueSize: number;
  settings: Record<string, any>;
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
  const [availableTopics, setAvailableTopics] = useState<string[]>([]);
  const [animationFrame, setAnimationFrame] = useState<number | null>(null);

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

  // Initialize default displays
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
        settings: {
          cellSize: 1,
          cellCount: 10,
          lineWidth: 1,
        },
      },
      {
        id: "axes",
        name: "Global Axes",
        type: "axes",
        topic: "",
        enabled: true,
        color: "#ffffff",
        size: 1,
        alpha: 1,
        frame: "map",
        queueSize: 1,
        settings: {
          length: 2,
          radius: 0.1,
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
        settings: {
          showNames: true,
          showAxes: true,
          showArrows: true,
          frameTimeout: 15,
        },
      },
      {
        id: "robot_model",
        name: "Robot Model",
        type: "robot_model",
        topic: "/robot_description",
        enabled: true,
        color: "#0088ff",
        size: 1,
        alpha: 1,
        frame: "base_link",
        queueSize: 1,
        settings: {
          visual: true,
          collision: false,
          showJoints: true,
        },
      },
    ];

    setDisplays(defaultDisplays);
  }, []);

  // Load available topics
  useEffect(() => {
    if (!isConnected) return;

    const loadTopics = async () => {
      try {
        const topics = await getTopics();
        setAvailableTopics(topics || []);
      } catch (error) {
        console.error("Failed to load topics:", error);
      }
    };

    loadTopics();
    const interval = setInterval(loadTopics, 5000); // Refresh every 5 seconds

    return () => clearInterval(interval);
  }, [isConnected, getTopics]);

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
    { value: "pointcloud", label: "PointCloud2" },
    { value: "markers", label: "Markers" },
    { value: "tf", label: "TF" },
    { value: "robot_model", label: "Robot Model" },
    { value: "grid", label: "Grid" },
    { value: "axes", label: "Axes" },
    { value: "laser_scan", label: "Laser Scan" },
    { value: "occupancy_grid", label: "Occupancy Grid" },
    { value: "path", label: "Path" },
    { value: "pose_array", label: "Pose Array" },
  ];

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            {t("rviz.title")}
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            {t("rviz.subtitle")}
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge
            variant={isConnected ? "default" : "secondary"}
            className="gap-2"
          >
            <div
              className={`w-2 h-2 rounded-full ${isConnected ? "bg-green-400" : "bg-gray-400"}`}
            />
            {isConnected ? t("common.connected") : t("common.disconnected")}
          </Badge>

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
            {isPlaying ? t("common.pause") : t("common.start")}
          </Button>

          <Dialog open={isAddDisplayOpen} onOpenChange={setIsAddDisplayOpen}>
            <DialogTrigger asChild>
              <Button className="gap-2">
                <Plus className="h-4 w-4" />
                {t("rviz.addDisplay")}
              </Button>
            </DialogTrigger>
            <DialogContent>
              <DialogHeader>
                <DialogTitle>{t("rviz.addNewDisplay")}</DialogTitle>
                <DialogDescription>
                  {t("rviz.addDisplayDescription")}
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div>
                  <Label htmlFor="displayName">{t("rviz.displayName")}</Label>
                  <Input
                    id="displayName"
                    value={newDisplayName}
                    onChange={(e) => setNewDisplayName(e.target.value)}
                    placeholder={t("rviz.enterDisplayName")}
                  />
                </div>
                <div>
                  <Label htmlFor="displayType">{t("rviz.displayType")}</Label>
                  <Select
                    value={newDisplayType}
                    onValueChange={(value) =>
                      setNewDisplayType(value as Display["type"])
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {displayTypeOptions.map((option) => (
                        <SelectItem key={option.value} value={option.value}>
                          {option.label}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>
                <div>
                  <Label htmlFor="displayTopic">{t("rviz.topic")}</Label>
                  <Select
                    value={newDisplayTopic}
                    onValueChange={setNewDisplayTopic}
                  >
                    <SelectTrigger>
                      <SelectValue placeholder={t("rviz.selectTopic")} />
                    </SelectTrigger>
                    <SelectContent>
                      {availableTopics.map((topic) => (
                        <SelectItem key={topic} value={topic}>
                          {topic}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>
              </div>
              <DialogFooter>
                <Button
                  variant="outline"
                  onClick={() => setIsAddDisplayOpen(false)}
                >
                  {t("common.cancel")}
                </Button>
                <Button
                  onClick={addDisplay}
                  disabled={!newDisplayName || !newDisplayTopic}
                >
                  {t("common.add")}
                </Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Displays Panel */}
        <Card className="lg:col-span-1 glass-effect">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Eye className="h-5 w-5" />
              {t("rviz.displays")}
            </CardTitle>
            <CardDescription>{t("rviz.manageDisplays")}</CardDescription>
          </CardHeader>
          <CardContent>
            <ScrollArea className="h-[500px]">
              <div className="space-y-2">
                {displays.map((display) => (
                  <div
                    key={display.id}
                    className={`p-3 rounded-lg border cursor-pointer transition-all duration-200 hover:bg-accent/50 ${
                      selectedDisplay?.id === display.id
                        ? "border-primary bg-primary/10"
                        : "border-border"
                    }`}
                    onClick={() => setSelectedDisplay(display)}
                  >
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-2">
                        <Switch
                          checked={display.enabled}
                          onCheckedChange={() => toggleDisplay(display.id)}
                          size="sm"
                        />
                        <span className="text-sm font-medium">
                          {display.name}
                        </span>
                      </div>
                      <Button
                        variant="ghost"
                        size="sm"
                        onClick={(e) => {
                          e.stopPropagation();
                          removeDisplay(display.id);
                        }}
                        className="h-6 w-6 p-0"
                      >
                        <Trash2 className="h-3 w-3" />
                      </Button>
                    </div>
                    <div className="text-xs text-muted-foreground space-y-1">
                      <div>Type: {display.type}</div>
                      <div>Topic: {display.topic || "N/A"}</div>
                      <div className="flex items-center gap-2">
                        <div
                          className="w-3 h-3 rounded border"
                          style={{ backgroundColor: display.color }}
                        />
                        Alpha: {(display.alpha * 100).toFixed(0)}%
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </ScrollArea>
          </CardContent>
        </Card>

        {/* 3D Viewer */}
        <Card className="lg:col-span-3 glass-effect">
          <CardHeader>
            <div className="flex items-center justify-between">
              <div>
                <CardTitle className="flex items-center gap-2">
                  <Box className="h-5 w-5" />
                  {t("rviz.viewer3D")}
                </CardTitle>
                <CardDescription>
                  {t("rviz.interactiveVisualization")}
                </CardDescription>
              </div>

              <div className="flex items-center gap-2">
                {/* View Controls */}
                <div className="flex items-center gap-1 bg-background/50 rounded-lg p-1">
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() =>
                      setCameraPosition((prev) => ({
                        ...prev,
                        z: prev.z * 0.8,
                      }))
                    }
                  >
                    <ZoomIn className="h-4 w-4" />
                  </Button>
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() =>
                      setCameraPosition((prev) => ({
                        ...prev,
                        z: prev.z * 1.2,
                      }))
                    }
                  >
                    <ZoomOut className="h-4 w-4" />
                  </Button>
                  <Button variant="ghost" size="sm" onClick={resetCamera}>
                    <RotateCcw className="h-4 w-4" />
                  </Button>
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => setIsFullscreen(!isFullscreen)}
                  >
                    {isFullscreen ? (
                      <Minimize className="h-4 w-4" />
                    ) : (
                      <Maximize className="h-4 w-4" />
                    )}
                  </Button>
                </div>

                {/* Settings */}
                <Dialog open={isSettingsOpen} onOpenChange={setIsSettingsOpen}>
                  <DialogTrigger asChild>
                    <Button variant="ghost" size="sm">
                      <Settings className="h-4 w-4" />
                    </Button>
                  </DialogTrigger>
                  <DialogContent className="max-w-md">
                    <DialogHeader>
                      <DialogTitle>{t("rviz.viewSettings")}</DialogTitle>
                    </DialogHeader>
                    <div className="space-y-4">
                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <Label>{t("rviz.showGrid")}</Label>
                          <Switch
                            checked={viewSettings.gridEnabled}
                            onCheckedChange={(checked) =>
                              setViewSettings((prev) => ({
                                ...prev,
                                gridEnabled: checked,
                              }))
                            }
                          />
                        </div>
                        <div className="flex items-center justify-between">
                          <Label>{t("rviz.showAxes")}</Label>
                          <Switch
                            checked={viewSettings.axesEnabled}
                            onCheckedChange={(checked) =>
                              setViewSettings((prev) => ({
                                ...prev,
                                axesEnabled: checked,
                              }))
                            }
                          />
                        </div>
                      </div>

                      <Separator />

                      <div className="space-y-3">
                        <div>
                          <Label>{t("rviz.backgroundColor")}</Label>
                          <Input
                            type="color"
                            value={viewSettings.backgroundColor}
                            onChange={(e) =>
                              setViewSettings((prev) => ({
                                ...prev,
                                backgroundColor: e.target.value,
                              }))
                            }
                            className="mt-1"
                          />
                        </div>
                        <div>
                          <Label>
                            {t("rviz.gridSize")}: {viewSettings.gridSize}
                          </Label>
                          <Slider
                            value={[viewSettings.gridSize]}
                            onValueChange={([value]) =>
                              setViewSettings((prev) => ({
                                ...prev,
                                gridSize: value,
                              }))
                            }
                            min={5}
                            max={50}
                            step={5}
                            className="mt-2"
                          />
                        </div>
                        <div>
                          <Label>
                            {t("rviz.fps")}: {fps}
                          </Label>
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
                    </div>
                  </DialogContent>
                </Dialog>
              </div>
            </div>
          </CardHeader>
          <CardContent>
            <div
              className={`relative bg-black rounded-lg overflow-hidden ${isFullscreen ? "fixed inset-4 z-50" : ""}`}
            >
              <canvas
                ref={canvasRef}
                width={isFullscreen ? window.innerWidth - 32 : 800}
                height={isFullscreen ? window.innerHeight - 200 : 600}
                className="cursor-move border border-border/20"
                onMouseDown={handleMouseDown}
                onMouseMove={handleMouseMove}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                onWheel={handleWheel}
              />

              {/* Status Overlay */}
              <div className="absolute top-4 right-4 bg-background/90 backdrop-blur-sm rounded-lg p-3 border border-border/20">
                <div className="text-sm space-y-1">
                  <div className="flex items-center gap-2">
                    <div
                      className={`w-2 h-2 rounded-full ${isPlaying ? "bg-green-400" : "bg-red-400"}`}
                    />
                    {isPlaying ? t("rviz.rendering") : t("rviz.paused")}
                  </div>
                  <div>
                    {t("rviz.fps")}: {fps}
                  </div>
                  <div>
                    {t("rviz.displays")}:{" "}
                    {displays.filter((d) => d.enabled).length}/{displays.length}
                  </div>
                </div>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Display Properties Panel */}
      {selectedDisplay && (
        <Card className="glass-effect">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Settings className="h-5 w-5" />
              {t("rviz.displayProperties")}: {selectedDisplay.name}
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>{t("rviz.displayName")}</Label>
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
                  />
                </div>
                <div>
                  <Label>{t("rviz.topic")}</Label>
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
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {availableTopics.map((topic) => (
                        <SelectItem key={topic} value={topic}>
                          {topic}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>{t("rviz.color")}</Label>
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
                  />
                </div>
                <div>
                  <Label>
                    {t("rviz.size")}: {selectedDisplay.size}
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
                    className="mt-2"
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>
                    {t("rviz.alpha")}:{" "}
                    {(selectedDisplay.alpha * 100).toFixed(0)}%
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
                    step={0.1}
                    className="mt-2"
                  />
                </div>
                <div>
                  <Label>{t("rviz.frame")}</Label>
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
                    placeholder="map"
                  />
                </div>
              </div>
            </div>
          </CardContent>
        </Card>
      )}
    </div>
  );
};

export default VirtualRviz;
