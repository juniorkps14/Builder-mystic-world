import { useState, useRef, useEffect, useCallback } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Slider } from "@/components/ui/slider";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Input } from "@/components/ui/input";
import { Switch } from "@/components/ui/switch";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Collapsible,
  CollapsibleContent,
  CollapsibleTrigger,
} from "@/components/ui/collapsible";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Map,
  Navigation,
  RotateCcw,
  ZoomIn,
  ZoomOut,
  Home,
  Target,
  MapPin,
  Activity,
  Eye,
  EyeOff,
  Settings,
  Download,
  RefreshCw,
  Layers,
  Grid3X3,
  Compass,
  Camera,
  Cube,
  Route,
  Zap,
  Wifi,
  ChevronDown,
  ChevronRight,
  Plus,
  Minus,
  RotateCw,
  Move3D,
  Palette,
  Monitor,
  Save,
  FolderOpen,
  Play,
  Pause,
  SkipForward,
  Volume2,
  VolumeX,
} from "lucide-react";

interface DisplayItem {
  id: string;
  name: string;
  type: string;
  enabled: boolean;
  topic?: string;
  color?: string;
  size?: number;
  alpha?: number;
  icon: any;
  config?: Record<string, any>;
}

interface ViewConfig {
  zoom: number;
  pan: { x: number; y: number };
  rotation: number;
  projection: "2D" | "3D" | "Orbit";
  backgroundColor: string;
  gridEnabled: boolean;
  axisEnabled: boolean;
}

const RVizMapViewer = () => {
  const { t } = useLanguage();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [selectedDisplay, setSelectedDisplay] = useState<string | null>(null);
  const [dragState, setDragState] = useState<{
    isDragging: boolean;
    lastX: number;
    lastY: number;
  }>({ isDragging: false, lastX: 0, lastY: 0 });

  // RViz-style Display Items
  const [displays, setDisplays] = useState<DisplayItem[]>([
    {
      id: "map",
      name: "Map",
      type: "Map",
      enabled: true,
      topic: "/map",
      alpha: 0.7,
      icon: Map,
      config: { colorScheme: "occupancy" },
    },
    {
      id: "robot_model",
      name: "Robot Model",
      type: "RobotModel",
      enabled: true,
      alpha: 1.0,
      icon: Cube,
      config: { urdfTopic: "/robot_description" },
    },
    {
      id: "laser_scan",
      name: "LaserScan",
      type: "LaserScan",
      enabled: true,
      topic: "/scan",
      color: "#ff0000",
      size: 0.05,
      icon: Target,
      config: { style: "Points", decay: 0 },
    },
    {
      id: "robot_path",
      name: "Path",
      type: "Path",
      enabled: true,
      topic: "/move_base/global_plan",
      color: "#00ff00",
      size: 0.02,
      alpha: 0.8,
      icon: Route,
      config: { buffer: 1 },
    },
    {
      id: "tf_frames",
      name: "TF",
      type: "TF",
      enabled: false,
      alpha: 1.0,
      icon: Compass,
      config: { showNames: true, showAxes: true, showArrows: true },
    },
    {
      id: "camera",
      name: "Camera",
      type: "Camera",
      enabled: false,
      topic: "/camera/image_raw",
      alpha: 1.0,
      icon: Camera,
      config: { imageTransport: "raw" },
    },
    {
      id: "pointcloud",
      name: "PointCloud2",
      type: "PointCloud2",
      enabled: false,
      topic: "/velodyne_points",
      color: "#ffffff",
      size: 0.01,
      alpha: 1.0,
      icon: Grid3X3,
      config: { style: "Points", colorTransformer: "Intensity" },
    },
    {
      id: "markers",
      name: "Interactive Markers",
      type: "InteractiveMarkers",
      enabled: false,
      topic: "/interactive_markers",
      alpha: 1.0,
      icon: Zap,
      config: {},
    },
    {
      id: "costmap",
      name: "Costmap",
      type: "Costmap",
      enabled: false,
      topic: "/move_base/global_costmap/costmap",
      alpha: 0.5,
      icon: Layers,
      config: { colorScheme: "costmap" },
    },
  ]);

  // View Configuration
  const [viewConfig, setViewConfig] = useState<ViewConfig>({
    zoom: 100,
    pan: { x: 0, y: 0 },
    rotation: 0,
    projection: "2D",
    backgroundColor: "#1a1a1a",
    gridEnabled: true,
    axisEnabled: true,
  });

  // Robot State for simulation
  const [robotState, setRobotState] = useState({
    position: { x: 2.34, y: 1.67, z: 0, roll: 0, pitch: 0, yaw: 0.45 },
    velocity: {
      linear: { x: 0.25, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0.1 },
    },
    joints: {
      base_link: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
      laser_link: { x: 0.2, y: 0, z: 0.15, roll: 0, pitch: 0, yaw: 0 },
    },
    laserScan: [] as Array<{ angle: number; range: number; intensity: number }>,
  });

  // Generate laser scan data
  const generateLaserScan = useCallback(() => {
    const scan = [];
    for (let i = 0; i < 360; i += 2) {
      const angle = (i * Math.PI) / 180;
      const baseRange = 3 + Math.random() * 4;
      const noise = (Math.random() - 0.5) * 0.2;
      const range = Math.max(0.1, baseRange + noise);
      const intensity = Math.random() * 100;

      scan.push({ angle, range, intensity });
    }
    return scan;
  }, []);

  // Initialize laser scan
  useEffect(() => {
    setRobotState((prev) => ({
      ...prev,
      laserScan: generateLaserScan(),
    }));
  }, [generateLaserScan]);

  // World to Canvas coordinate transformation
  const worldToCanvas = useCallback(
    (x: number, y: number) => {
      const canvas = canvasRef.current;
      if (!canvas) return { x: 0, y: 0 };

      const scale = viewConfig.zoom / 100;
      const centerX = canvas.width / 2;
      const centerY = canvas.height / 2;

      return {
        x: centerX + x * 20 * scale + viewConfig.pan.x,
        y: centerY - y * 20 * scale + viewConfig.pan.y,
      };
    },
    [viewConfig.zoom, viewConfig.pan],
  );

  // Draw functions
  const drawGrid = useCallback(
    (ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement) => {
      if (!viewConfig.gridEnabled) return;

      ctx.save();
      ctx.strokeStyle = "#333333";
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 2]);

      const scale = viewConfig.zoom / 100;
      const gridSize = 20 * scale;
      const offsetX = viewConfig.pan.x % gridSize;
      const offsetY = viewConfig.pan.y % gridSize;

      // Vertical lines
      for (let x = offsetX; x < canvas.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
      }

      // Horizontal lines
      for (let y = offsetY; y < canvas.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }

      ctx.restore();
    },
    [viewConfig],
  );

  const drawAxis = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      if (!viewConfig.axisEnabled) return;

      ctx.save();
      const origin = worldToCanvas(0, 0);
      const scale = viewConfig.zoom / 100;
      const axisLength = 50 * scale;

      // X-axis (Red)
      ctx.strokeStyle = "#ff0000";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(origin.x, origin.y);
      ctx.lineTo(origin.x + axisLength, origin.y);
      ctx.stroke();

      // Arrow head for X
      ctx.fillStyle = "#ff0000";
      ctx.beginPath();
      ctx.moveTo(origin.x + axisLength, origin.y);
      ctx.lineTo(origin.x + axisLength - 10, origin.y - 5);
      ctx.lineTo(origin.x + axisLength - 10, origin.y + 5);
      ctx.closePath();
      ctx.fill();

      // Y-axis (Green)
      ctx.strokeStyle = "#00ff00";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(origin.x, origin.y);
      ctx.lineTo(origin.x, origin.y - axisLength);
      ctx.stroke();

      // Arrow head for Y
      ctx.fillStyle = "#00ff00";
      ctx.beginPath();
      ctx.moveTo(origin.x, origin.y - axisLength);
      ctx.lineTo(origin.x - 5, origin.y - axisLength + 10);
      ctx.lineTo(origin.x + 5, origin.y - axisLength + 10);
      ctx.closePath();
      ctx.fill();

      ctx.restore();
    },
    [viewConfig, worldToCanvas],
  );

  const drawRobotModel = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      if (!displays.find((d) => d.id === "robot_model")?.enabled) return;

      ctx.save();
      const robotPos = worldToCanvas(
        robotState.position.x,
        robotState.position.y,
      );
      const scale = viewConfig.zoom / 100;

      // Robot body (rectangle)
      ctx.translate(robotPos.x, robotPos.y);
      ctx.rotate(-robotState.position.yaw);

      ctx.fillStyle = "#4a9eff";
      ctx.strokeStyle = "#2563eb";
      ctx.lineWidth = 2;
      const robotWidth = 30 * scale;
      const robotHeight = 20 * scale;

      ctx.fillRect(-robotWidth / 2, -robotHeight / 2, robotWidth, robotHeight);
      ctx.strokeRect(
        -robotWidth / 2,
        -robotHeight / 2,
        robotWidth,
        robotHeight,
      );

      // Direction arrow
      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.moveTo(robotWidth / 2 - 5, 0);
      ctx.lineTo(robotWidth / 2 - 15, -8);
      ctx.lineTo(robotWidth / 2 - 15, 8);
      ctx.closePath();
      ctx.fill();

      // Laser scanner
      ctx.fillStyle = "#ff6b6b";
      ctx.beginPath();
      ctx.arc(robotWidth / 4, 0, 4 * scale, 0, 2 * Math.PI);
      ctx.fill();

      ctx.restore();
    },
    [displays, robotState, viewConfig, worldToCanvas],
  );

  const drawLaserScan = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      const laserDisplay = displays.find((d) => d.id === "laser_scan");
      if (!laserDisplay?.enabled) return;

      ctx.save();
      const robotPos = worldToCanvas(
        robotState.position.x,
        robotState.position.y,
      );
      const scale = viewConfig.zoom / 100;

      ctx.globalAlpha = laserDisplay.alpha || 1;
      ctx.fillStyle = laserDisplay.color || "#ff0000";

      robotState.laserScan.forEach((point) => {
        const x =
          robotState.position.x +
          point.range * Math.cos(point.angle + robotState.position.yaw);
        const y =
          robotState.position.y +
          point.range * Math.sin(point.angle + robotState.position.yaw);
        const screenPos = worldToCanvas(x, y);

        const pointSize = (laserDisplay.size || 0.05) * 100 * scale;
        ctx.beginPath();
        ctx.arc(screenPos.x, screenPos.y, pointSize, 0, 2 * Math.PI);
        ctx.fill();
      });

      ctx.restore();
    },
    [displays, robotState, viewConfig, worldToCanvas],
  );

  const drawPath = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      const pathDisplay = displays.find((d) => d.id === "robot_path");
      if (!pathDisplay?.enabled) return;

      ctx.save();
      ctx.strokeStyle = pathDisplay.color || "#00ff00";
      ctx.lineWidth =
        (pathDisplay.size || 0.02) * 100 * (viewConfig.zoom / 100);
      ctx.globalAlpha = pathDisplay.alpha || 0.8;

      // Example path
      const path = [
        { x: 0, y: 0 },
        { x: 1, y: 1 },
        { x: robotState.position.x, y: robotState.position.y },
        { x: 5, y: 3 },
        { x: 8, y: 2 },
      ];

      ctx.beginPath();
      path.forEach((point, index) => {
        const screenPos = worldToCanvas(point.x, point.y);
        if (index === 0) {
          ctx.moveTo(screenPos.x, screenPos.y);
        } else {
          ctx.lineTo(screenPos.x, screenPos.y);
        }
      });
      ctx.stroke();

      ctx.restore();
    },
    [displays, robotState, viewConfig, worldToCanvas],
  );

  const drawMap = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = viewConfig.backgroundColor;
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw grid
    drawGrid(ctx, canvas);

    // Draw map (if enabled)
    if (displays.find((d) => d.id === "map")?.enabled) {
      ctx.save();
      ctx.fillStyle = "rgba(100, 100, 100, 0.3)";
      // Simulate occupancy grid
      for (let i = 0; i < 20; i++) {
        const x = Math.random() * 10 - 5;
        const y = Math.random() * 10 - 5;
        const pos = worldToCanvas(x, y);
        ctx.fillRect(pos.x - 5, pos.y - 5, 10, 10);
      }
      ctx.restore();
    }

    // Draw displays in order
    drawAxis(ctx);
    drawPath(ctx);
    drawLaserScan(ctx);
    drawRobotModel(ctx);
  }, [
    viewConfig,
    displays,
    drawGrid,
    drawAxis,
    drawPath,
    drawLaserScan,
    drawRobotModel,
  ]);

  // Mouse interaction handlers
  const handleMouseDown = (event: React.MouseEvent<HTMLCanvasElement>) => {
    setDragState({
      isDragging: true,
      lastX: event.clientX,
      lastY: event.clientY,
    });
  };

  const handleMouseMove = (event: React.MouseEvent<HTMLCanvasElement>) => {
    if (!dragState.isDragging) return;

    const deltaX = event.clientX - dragState.lastX;
    const deltaY = event.clientY - dragState.lastY;

    setViewConfig((prev) => ({
      ...prev,
      pan: {
        x: prev.pan.x + deltaX,
        y: prev.pan.y + deltaY,
      },
    }));

    setDragState({
      isDragging: true,
      lastX: event.clientX,
      lastY: event.clientY,
    });
  };

  const handleMouseUp = () => {
    setDragState((prev) => ({ ...prev, isDragging: false }));
  };

  const handleWheel = (event: React.WheelEvent<HTMLCanvasElement>) => {
    event.preventDefault();
    const delta = event.deltaY > 0 ? 0.9 : 1.1;
    setViewConfig((prev) => ({
      ...prev,
      zoom: Math.max(10, Math.min(500, prev.zoom * delta)),
    }));
  };

  // Toggle display visibility
  const toggleDisplay = (displayId: string) => {
    setDisplays((prev) =>
      prev.map((display) =>
        display.id === displayId
          ? { ...display, enabled: !display.enabled }
          : display,
      ),
    );
  };

  // Update display config
  const updateDisplayConfig = (displayId: string, key: string, value: any) => {
    setDisplays((prev) =>
      prev.map((display) =>
        display.id === displayId ? { ...display, [key]: value } : display,
      ),
    );
  };

  // Canvas resize effect
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    let timeoutId: NodeJS.Timeout;
    let isResizing = false;

    const updateCanvasSize = () => {
      if (isResizing) return;

      clearTimeout(timeoutId);
      timeoutId = setTimeout(() => {
        if (!canvas.parentElement) return;

        const rect = canvas.parentElement.getBoundingClientRect();
        if (rect && rect.width > 0 && rect.height > 0) {
          const newWidth = Math.floor(rect.width);
          const newHeight = Math.floor(rect.height);

          if (canvas.width !== newWidth || canvas.height !== newHeight) {
            canvas.width = newWidth;
            canvas.height = newHeight;
            requestAnimationFrame(() => drawMap());
          }
        }
        isResizing = false;
      }, 150);
    };

    updateCanvasSize();

    let resizeObserver: ResizeObserver | null = null;

    try {
      resizeObserver = new ResizeObserver(() => {
        if (isResizing) return;
        isResizing = true;
        requestAnimationFrame(updateCanvasSize);
      });

      if (canvas.parentElement) {
        resizeObserver.observe(canvas.parentElement, { box: "border-box" });
      }
    } catch (error) {
      console.warn("ResizeObserver not supported");
    }

    return () => {
      clearTimeout(timeoutId);
      if (resizeObserver) {
        resizeObserver.disconnect();
      }
    };
  }, []);

  // Animation loop
  useEffect(() => {
    const interval = setInterval(() => {
      drawMap();

      // Simulate robot movement
      setRobotState((prev) => ({
        ...prev,
        position: {
          ...prev.position,
          x: prev.position.x + Math.sin(Date.now() / 3000) * 0.01,
          y: prev.position.y + Math.cos(Date.now() / 3000) * 0.01,
          yaw: prev.position.yaw + 0.005,
        },
        laserScan: generateLaserScan(),
      }));
    }, 100);

    return () => clearInterval(interval);
  }, [drawMap, generateLaserScan]);

  return (
    <div className="h-full flex bg-background">
      {/* Left Panel - RViz Style */}
      <div className="w-80 border-r border-border flex flex-col bg-card">
        {/* Header */}
        <div className="p-4 border-b border-border">
          <div className="flex items-center gap-2 mb-3">
            <Monitor className="h-5 w-5 text-primary" />
            <h2 className="font-extralight text-lg">
              RViz - ROS Visualization
            </h2>
          </div>

          <div className="flex gap-2">
            <Button
              size="sm"
              variant="outline"
              className="flex-1 font-extralight"
            >
              <Save className="h-4 w-4 mr-1" />
              Save Config
            </Button>
            <Button
              size="sm"
              variant="outline"
              className="flex-1 font-extralight"
            >
              <FolderOpen className="h-4 w-4 mr-1" />
              Load Config
            </Button>
          </div>
        </div>

        <Tabs defaultValue="displays" className="flex-1 flex flex-col">
          <TabsList className="grid w-full grid-cols-2 mx-4 my-2">
            <TabsTrigger value="displays" className="font-extralight">
              Displays
            </TabsTrigger>
            <TabsTrigger value="views" className="font-extralight">
              Views
            </TabsTrigger>
          </TabsList>

          {/* Displays Panel */}
          <TabsContent value="displays" className="flex-1 p-0">
            <div className="p-4">
              <Button
                size="sm"
                className="w-full mb-3 font-extralight"
                onClick={() => {
                  // Add new display logic
                }}
              >
                <Plus className="h-4 w-4 mr-2" />
                Add Display
              </Button>
            </div>

            <ScrollArea className="flex-1">
              <div className="p-4 pt-0 space-y-2">
                {displays.map((display) => (
                  <Collapsible key={display.id}>
                    <CollapsibleTrigger asChild>
                      <div
                        className={`flex items-center p-3 rounded-lg hover:bg-accent cursor-pointer transition-colors ${
                          selectedDisplay === display.id ? "bg-accent" : ""
                        }`}
                        onClick={() => setSelectedDisplay(display.id)}
                      >
                        <div className="flex items-center gap-2 flex-1">
                          <Switch
                            checked={display.enabled}
                            onCheckedChange={() => toggleDisplay(display.id)}
                            onClick={(e) => e.stopPropagation()}
                          />
                          <display.icon className="h-4 w-4" />
                          <span className="font-extralight text-sm">
                            {display.name}
                          </span>
                        </div>
                        <ChevronRight className="h-4 w-4 transition-transform ui-state-open:rotate-90" />
                      </div>
                    </CollapsibleTrigger>

                    <CollapsibleContent className="p-3 pt-0">
                      <div className="ml-6 space-y-3 border-l border-border pl-3">
                        {display.topic && (
                          <div>
                            <Label className="text-xs font-extralight">
                              Topic
                            </Label>
                            <Input
                              value={display.topic}
                              className="h-7 text-xs font-extralight"
                              onChange={(e) =>
                                updateDisplayConfig(
                                  display.id,
                                  "topic",
                                  e.target.value,
                                )
                              }
                            />
                          </div>
                        )}

                        {display.color && (
                          <div>
                            <Label className="text-xs font-extralight">
                              Color
                            </Label>
                            <div className="flex items-center gap-2">
                              <div
                                className="w-6 h-6 rounded border border-border cursor-pointer"
                                style={{ backgroundColor: display.color }}
                                onClick={() => {
                                  // Color picker logic
                                }}
                              />
                              <Input
                                value={display.color}
                                className="h-7 text-xs font-extralight"
                                onChange={(e) =>
                                  updateDisplayConfig(
                                    display.id,
                                    "color",
                                    e.target.value,
                                  )
                                }
                              />
                            </div>
                          </div>
                        )}

                        {display.size !== undefined && (
                          <div>
                            <Label className="text-xs font-extralight">
                              Size: {display.size?.toFixed(3)}
                            </Label>
                            <Slider
                              value={[display.size]}
                              onValueChange={(value) =>
                                updateDisplayConfig(
                                  display.id,
                                  "size",
                                  value[0],
                                )
                              }
                              min={0.001}
                              max={0.1}
                              step={0.001}
                              className="mt-1"
                            />
                          </div>
                        )}

                        {display.alpha !== undefined && (
                          <div>
                            <Label className="text-xs font-extralight">
                              Alpha: {display.alpha?.toFixed(2)}
                            </Label>
                            <Slider
                              value={[display.alpha]}
                              onValueChange={(value) =>
                                updateDisplayConfig(
                                  display.id,
                                  "alpha",
                                  value[0],
                                )
                              }
                              min={0}
                              max={1}
                              step={0.01}
                              className="mt-1"
                            />
                          </div>
                        )}
                      </div>
                    </CollapsibleContent>
                  </Collapsible>
                ))}
              </div>
            </ScrollArea>
          </TabsContent>

          {/* Views Panel */}
          <TabsContent value="views" className="flex-1 p-4">
            <div className="space-y-4">
              {/* View Controls */}
              <Card className="p-4">
                <Label className="text-sm font-extralight mb-2 block">
                  View Type
                </Label>
                <Select
                  value={viewConfig.projection}
                  onValueChange={(value: "2D" | "3D" | "Orbit") =>
                    setViewConfig((prev) => ({ ...prev, projection: value }))
                  }
                >
                  <SelectTrigger className="font-extralight">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="2D">Top Down (2D)</SelectItem>
                    <SelectItem value="3D">Third Person (3D)</SelectItem>
                    <SelectItem value="Orbit">Orbit</SelectItem>
                  </SelectContent>
                </Select>
              </Card>

              {/* Camera Controls */}
              <Card className="p-4">
                <h3 className="text-sm font-extralight mb-3">
                  Camera Controls
                </h3>

                <div className="space-y-3">
                  <div>
                    <Label className="text-xs font-extralight">
                      Zoom: {viewConfig.zoom}%
                    </Label>
                    <Slider
                      value={[viewConfig.zoom]}
                      onValueChange={(value) =>
                        setViewConfig((prev) => ({ ...prev, zoom: value[0] }))
                      }
                      min={10}
                      max={500}
                      step={5}
                      className="mt-1"
                    />
                  </div>

                  <div className="grid grid-cols-2 gap-2">
                    <Button
                      size="sm"
                      variant="outline"
                      className="font-extralight"
                      onClick={() =>
                        setViewConfig((prev) => ({
                          ...prev,
                          pan: { x: 0, y: 0 },
                          zoom: 100,
                        }))
                      }
                    >
                      <Home className="h-4 w-4 mr-1" />
                      Reset
                    </Button>
                    <Button
                      size="sm"
                      variant="outline"
                      className="font-extralight"
                      onClick={() => {
                        const robotPos = worldToCanvas(
                          robotState.position.x,
                          robotState.position.y,
                        );
                        const canvas = canvasRef.current;
                        if (canvas) {
                          setViewConfig((prev) => ({
                            ...prev,
                            pan: {
                              x: canvas.width / 2 - robotPos.x,
                              y: canvas.height / 2 - robotPos.y,
                            },
                          }));
                        }
                      }}
                    >
                      <Target className="h-4 w-4 mr-1" />
                      Focus Robot
                    </Button>
                  </div>
                </div>
              </Card>

              {/* Display Options */}
              <Card className="p-4">
                <h3 className="text-sm font-extralight mb-3">
                  Display Options
                </h3>

                <div className="space-y-3">
                  <div className="flex items-center justify-between">
                    <Label className="text-xs font-extralight">Grid</Label>
                    <Switch
                      checked={viewConfig.gridEnabled}
                      onCheckedChange={(checked) =>
                        setViewConfig((prev) => ({
                          ...prev,
                          gridEnabled: checked,
                        }))
                      }
                    />
                  </div>

                  <div className="flex items-center justify-between">
                    <Label className="text-xs font-extralight">Axis</Label>
                    <Switch
                      checked={viewConfig.axisEnabled}
                      onCheckedChange={(checked) =>
                        setViewConfig((prev) => ({
                          ...prev,
                          axisEnabled: checked,
                        }))
                      }
                    />
                  </div>

                  <div>
                    <Label className="text-xs font-extralight">
                      Background Color
                    </Label>
                    <div className="flex items-center gap-2 mt-1">
                      <div
                        className="w-6 h-6 rounded border border-border cursor-pointer"
                        style={{ backgroundColor: viewConfig.backgroundColor }}
                        onClick={() => {
                          // Color picker for background
                        }}
                      />
                      <Input
                        value={viewConfig.backgroundColor}
                        className="h-7 text-xs font-extralight"
                        onChange={(e) =>
                          setViewConfig((prev) => ({
                            ...prev,
                            backgroundColor: e.target.value,
                          }))
                        }
                      />
                    </div>
                  </div>
                </div>
              </Card>
            </div>
          </TabsContent>
        </Tabs>
      </div>

      {/* Main Visualization Area */}
      <div className="flex-1 flex flex-col">
        {/* Top Toolbar */}
        <div className="h-12 border-b border-border flex items-center px-4 gap-2 bg-card/50">
          <Badge variant="outline" className="font-extralight">
            FPS: 10
          </Badge>
          <Separator orientation="vertical" className="h-6" />

          <Button size="sm" variant="ghost" className="font-extralight">
            <Play className="h-4 w-4" />
          </Button>
          <Button size="sm" variant="ghost" className="font-extralight">
            <Pause className="h-4 w-4" />
          </Button>
          <Button size="sm" variant="ghost" className="font-extralight">
            <SkipForward className="h-4 w-4" />
          </Button>

          <Separator orientation="vertical" className="h-6" />

          <div className="flex items-center gap-2">
            <Label className="text-xs font-extralight">Time:</Label>
            <Badge variant="secondary" className="font-extralight">
              {new Date().toLocaleTimeString()}
            </Badge>
          </div>

          <div className="ml-auto flex items-center gap-2">
            <Badge
              variant={robotState ? "default" : "destructive"}
              className="font-extralight"
            >
              {robotState ? "Connected" : "Disconnected"}
            </Badge>
            <Button size="sm" variant="ghost" className="font-extralight">
              <RefreshCw className="h-4 w-4" />
            </Button>
          </div>
        </div>

        {/* Canvas Container */}
        <div className="flex-1 relative bg-background">
          <canvas
            ref={canvasRef}
            className="w-full h-full cursor-move"
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
            onWheel={handleWheel}
          />

          {/* Overlay Info */}
          <div className="absolute top-4 right-4 space-y-2">
            <Card className="p-3 bg-card/90 backdrop-blur-sm">
              <div className="text-xs font-extralight space-y-1">
                <div>
                  Position: ({robotState.position.x.toFixed(2)},{" "}
                  {robotState.position.y.toFixed(2)})
                </div>
                <div>
                  Orientation:{" "}
                  {((robotState.position.yaw * 180) / Math.PI).toFixed(1)}°
                </div>
                <div>Zoom: {viewConfig.zoom}%</div>
                <div>
                  Displays: {displays.filter((d) => d.enabled).length}/
                  {displays.length}
                </div>
              </div>
            </Card>
          </div>

          {/* Bottom Status Bar */}
          <div className="absolute bottom-0 left-0 right-0 h-8 bg-card/90 backdrop-blur-sm border-t border-border flex items-center px-4 text-xs font-extralight">
            <div className="flex items-center gap-4">
              <span>Mode: {viewConfig.projection}</span>
              <span>Robot: turtle1</span>
              <span>Map: /map</span>
              <span>Frame: map</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RVizMapViewer;
