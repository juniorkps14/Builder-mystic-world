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
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "@/components/ui/alert-dialog";
import { useToast } from "@/hooks/use-toast";
import {
  Map,
  Plus,
  Download,
  Upload,
  Trash2,
  Play,
  Pause,
  RotateCcw,
  ZoomIn,
  ZoomOut,
  Grid3x3,
  Crosshair,
  Navigation,
  Save,
  RefreshCw,
  MapPin,
  Target,
  Settings,
  Eye,
  EyeOff,
  Maximize,
  Minimize,
} from "lucide-react";

interface MapData {
  id: string;
  name: string;
  description: string;
  created: Date;
  lastModified: Date;
  resolution: number; // meters per pixel
  width: number; // pixels
  height: number; // pixels
  origin: { x: number; y: number; theta: number };
  data: Uint8Array; // occupancy grid data (0-100, 255 = unknown)
  size: number; // file size in bytes
}

interface RobotPosition {
  x: number;
  y: number;
  theta: number;
  timestamp: Date;
}

interface MapSettings {
  showGrid: boolean;
  showRobot: boolean;
  showGoal: boolean;
  showTrajectory: boolean;
  fps: number;
  occupancyThreshold: number;
  robotColor: string;
  goalColor: string;
  trajectoryColor: string;
}

const MapViewer: React.FC = () => {
  const { t } = useLanguage();
  const { toast } = useToast();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [maps, setMaps] = useState<MapData[]>([]);
  const [selectedMap, setSelectedMap] = useState<MapData | null>(null);
  const [isPlaying, setIsPlaying] = useState(false);
  const [robotPosition, setRobotPosition] = useState<RobotPosition | null>(
    null,
  );
  const [goalPosition, setGoalPosition] = useState<{
    x: number;
    y: number;
  } | null>(null);
  const [trajectory, setTrajectory] = useState<Array<{ x: number; y: number }>>(
    [],
  );
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [lastMouse, setLastMouse] = useState({ x: 0, y: 0 });
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [newMapName, setNewMapName] = useState("");
  const [newMapDescription, setNewMapDescription] = useState("");
  const [isCreateDialogOpen, setIsCreateDialogOpen] = useState(false);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [refreshInterval, setRefreshInterval] = useState<NodeJS.Timeout | null>(
    null,
  );

  const [settings, setSettings] = useState<MapSettings>({
    showGrid: true,
    showRobot: true,
    showGoal: true,
    showTrajectory: true,
    fps: 10,
    occupancyThreshold: 50,
    robotColor: "#00ff00",
    goalColor: "#ff0000",
    trajectoryColor: "#0088ff",
  });

  const { isConnected, subscribe, publish, callService, connectionStatus } =
    useROSIntegration();

  // Initialize sample maps
  useEffect(() => {
    const sampleMaps: MapData[] = [
      {
        id: "map_1",
        name: "Ground Floor Layout",
        description: "Main building ground floor map",
        created: new Date("2024-01-15"),
        lastModified: new Date("2024-01-20"),
        resolution: 0.05,
        width: 800,
        height: 600,
        origin: { x: -20, y: -15, theta: 0 },
        data: new Uint8Array(800 * 600).fill(255),
        size: 480000,
      },
      {
        id: "map_2",
        name: "Laboratory Map",
        description: "Research laboratory detailed map",
        created: new Date("2024-01-10"),
        lastModified: new Date("2024-01-22"),
        resolution: 0.02,
        width: 1000,
        height: 800,
        origin: { x: -10, y: -8, theta: 0 },
        data: new Uint8Array(1000 * 800).fill(255),
        size: 800000,
      },
      {
        id: "map_3",
        name: "Warehouse Navigation",
        description: "Warehouse floor navigation map",
        created: new Date("2024-01-05"),
        lastModified: new Date("2024-01-25"),
        resolution: 0.1,
        width: 1200,
        height: 900,
        origin: { x: -60, y: -45, theta: 0 },
        data: new Uint8Array(1200 * 900).fill(255),
        size: 1080000,
      },
    ];

    // Generate sample occupancy data
    sampleMaps.forEach((map) => {
      for (let i = 0; i < map.data.length; i++) {
        const x = i % map.width;
        const y = Math.floor(i / map.width);

        // Create walls around perimeter
        if (x < 5 || x > map.width - 5 || y < 5 || y > map.height - 5) {
          map.data[i] = 100; // occupied
        }
        // Create some internal obstacles
        else if (
          (x > 100 && x < 120 && y > 100 && y < 200) ||
          (x > 200 && x < 220 && y > 50 && y < 150) ||
          (x > 300 && x < 400 && y > 200 && y < 220)
        ) {
          map.data[i] = 100; // occupied
        }
        // Free space
        else if (Math.random() > 0.95) {
          map.data[i] = 100; // random obstacles
        } else {
          map.data[i] = 0; // free
        }
      }
    });

    setMaps(sampleMaps);
    if (sampleMaps.length > 0) {
      setSelectedMap(sampleMaps[0]);
    }
  }, []);

  // Subscribe to ROS topics for real-time data
  useEffect(() => {
    if (!isConnected) return;

    const unsubscribePosition = subscribe("/amcl_pose", (message: any) => {
      const pose = message.pose.pose;
      setRobotPosition({
        x: pose.position.x,
        y: pose.position.y,
        theta: Math.atan2(
          2 *
            (pose.orientation.w * pose.orientation.z +
              pose.orientation.x * pose.orientation.y),
          1 -
            2 *
              (pose.orientation.y * pose.orientation.y +
                pose.orientation.z * pose.orientation.z),
        ),
        timestamp: new Date(),
      });
    });

    const unsubscribeGoal = subscribe(
      "/move_base_simple/goal",
      (message: any) => {
        const goal = message.pose;
        setGoalPosition({
          x: goal.position.x,
          y: goal.position.y,
        });
      },
    );

    const unsubscribePath = subscribe(
      "/move_base/DWAPlannerROS/global_plan",
      (message: any) => {
        const path = message.poses || [];
        setTrajectory(
          path.map((pose: any) => ({
            x: pose.pose.position.x,
            y: pose.pose.position.y,
          })),
        );
      },
    );

    return () => {
      unsubscribePosition();
      unsubscribeGoal();
      unsubscribePath();
    };
  }, [isConnected, subscribe]);

  // FPS control
  useEffect(() => {
    if (refreshInterval) {
      clearInterval(refreshInterval);
    }

    if (isPlaying && selectedMap) {
      const interval = setInterval(() => {
        drawMap();
      }, 1000 / settings.fps);
      setRefreshInterval(interval);
    }

    return () => {
      if (refreshInterval) {
        clearInterval(refreshInterval);
      }
    };
  }, [isPlaying, settings.fps, selectedMap, zoom, pan, settings]);

  // Draw map on canvas
  const drawMap = useCallback(() => {
    if (!selectedMap || !canvasRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Apply transformations
    ctx.save();
    ctx.translate(pan.x, pan.y);
    ctx.scale(zoom, zoom);

    // Draw occupancy grid
    const imageData = ctx.createImageData(
      selectedMap.width,
      selectedMap.height,
    );
    for (let i = 0; i < selectedMap.data.length; i++) {
      const value = selectedMap.data[i];
      const pixelIndex = i * 4;

      if (value === 255) {
        // Unknown space - gray
        imageData.data[pixelIndex] = 128;
        imageData.data[pixelIndex + 1] = 128;
        imageData.data[pixelIndex + 2] = 128;
      } else if (value >= settings.occupancyThreshold) {
        // Occupied space - black
        imageData.data[pixelIndex] = 0;
        imageData.data[pixelIndex + 1] = 0;
        imageData.data[pixelIndex + 2] = 0;
      } else {
        // Free space - white
        imageData.data[pixelIndex] = 255;
        imageData.data[pixelIndex + 1] = 255;
        imageData.data[pixelIndex + 2] = 255;
      }
      imageData.data[pixelIndex + 3] = 255; // alpha
    }

    ctx.putImageData(imageData, 0, 0);

    // Draw grid
    if (settings.showGrid) {
      ctx.strokeStyle = "rgba(0, 0, 0, 0.1)";
      ctx.lineWidth = 1 / zoom;
      const gridSize = 20; // pixels

      for (let x = 0; x < selectedMap.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, selectedMap.height);
        ctx.stroke();
      }

      for (let y = 0; y < selectedMap.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(selectedMap.width, y);
        ctx.stroke();
      }
    }

    // Draw trajectory
    if (settings.showTrajectory && trajectory.length > 1) {
      ctx.strokeStyle = settings.trajectoryColor;
      ctx.lineWidth = 2 / zoom;
      ctx.beginPath();

      trajectory.forEach((point, index) => {
        const pixelX =
          (point.x - selectedMap.origin.x) / selectedMap.resolution;
        const pixelY =
          selectedMap.height -
          (point.y - selectedMap.origin.y) / selectedMap.resolution;

        if (index === 0) {
          ctx.moveTo(pixelX, pixelY);
        } else {
          ctx.lineTo(pixelX, pixelY);
        }
      });
      ctx.stroke();
    }

    // Draw goal position
    if (settings.showGoal && goalPosition) {
      const goalPixelX =
        (goalPosition.x - selectedMap.origin.x) / selectedMap.resolution;
      const goalPixelY =
        selectedMap.height -
        (goalPosition.y - selectedMap.origin.y) / selectedMap.resolution;

      ctx.fillStyle = settings.goalColor;
      ctx.beginPath();
      ctx.arc(goalPixelX, goalPixelY, 10 / zoom, 0, 2 * Math.PI);
      ctx.fill();

      // Goal marker
      ctx.strokeStyle = settings.goalColor;
      ctx.lineWidth = 2 / zoom;
      ctx.beginPath();
      ctx.moveTo(goalPixelX - 15 / zoom, goalPixelY);
      ctx.lineTo(goalPixelX + 15 / zoom, goalPixelY);
      ctx.moveTo(goalPixelX, goalPixelY - 15 / zoom);
      ctx.lineTo(goalPixelX, goalPixelY + 15 / zoom);
      ctx.stroke();
    }

    // Draw robot position
    if (settings.showRobot && robotPosition) {
      const robotPixelX =
        (robotPosition.x - selectedMap.origin.x) / selectedMap.resolution;
      const robotPixelY =
        selectedMap.height -
        (robotPosition.y - selectedMap.origin.y) / selectedMap.resolution;

      ctx.fillStyle = settings.robotColor;
      ctx.beginPath();
      ctx.arc(robotPixelX, robotPixelY, 8 / zoom, 0, 2 * Math.PI);
      ctx.fill();

      // Robot orientation arrow
      ctx.strokeStyle = settings.robotColor;
      ctx.lineWidth = 3 / zoom;
      ctx.beginPath();
      const arrowLength = 20 / zoom;
      const arrowEndX =
        robotPixelX + Math.cos(robotPosition.theta) * arrowLength;
      const arrowEndY =
        robotPixelY - Math.sin(robotPosition.theta) * arrowLength;
      ctx.moveTo(robotPixelX, robotPixelY);
      ctx.lineTo(arrowEndX, arrowEndY);
      ctx.stroke();

      // Arrow head
      const headLength = 8 / zoom;
      const headAngle = Math.PI / 6;
      ctx.beginPath();
      ctx.moveTo(arrowEndX, arrowEndY);
      ctx.lineTo(
        arrowEndX - headLength * Math.cos(robotPosition.theta - headAngle),
        arrowEndY + headLength * Math.sin(robotPosition.theta - headAngle),
      );
      ctx.moveTo(arrowEndX, arrowEndY);
      ctx.lineTo(
        arrowEndX - headLength * Math.cos(robotPosition.theta + headAngle),
        arrowEndY + headLength * Math.sin(robotPosition.theta + headAngle),
      );
      ctx.stroke();
    }

    ctx.restore();
  }, [
    selectedMap,
    zoom,
    pan,
    settings,
    robotPosition,
    goalPosition,
    trajectory,
  ]);

  // Canvas mouse event handlers
  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    setLastMouse({ x: e.clientX, y: e.clientY });
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (!isDragging) return;

    const deltaX = e.clientX - lastMouse.x;
    const deltaY = e.clientY - lastMouse.y;

    setPan((prev) => ({
      x: prev.x + deltaX,
      y: prev.y + deltaY,
    }));

    setLastMouse({ x: e.clientX, y: e.clientY });
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
    setZoom((prev) => Math.max(0.1, Math.min(5, prev * zoomFactor)));
  };

  const handleCanvasClick = (e: React.MouseEvent) => {
    if (!selectedMap || isDragging) return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const canvasX = e.clientX - rect.left;
    const canvasY = e.clientY - rect.top;

    // Convert canvas coordinates to map coordinates
    const mapX = (canvasX - pan.x) / zoom;
    const mapY = (canvasY - pan.y) / zoom;

    const worldX = selectedMap.origin.x + mapX * selectedMap.resolution;
    const worldY =
      selectedMap.origin.y +
      (selectedMap.height - mapY) * selectedMap.resolution;

    // Set navigation goal
    if (isConnected) {
      const goal = {
        header: {
          stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 },
          frame_id: "map",
        },
        pose: {
          position: { x: worldX, y: worldY, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      };

      publish("/move_base_simple/goal", goal);

      toast({
        title: t("map.goalSet"),
        description: `${t("map.coordinates")}: (${worldX.toFixed(2)}, ${worldY.toFixed(2)})`,
      });
    }
  };

  const handleCreateMap = async () => {
    if (!newMapName.trim()) return;

    try {
      if (isConnected) {
        // Call ROS service to create new map
        const result = await callService("/make_map", {
          name: newMapName,
          description: newMapDescription,
        });

        if (result) {
          toast({
            title: t("map.mapCreated"),
            description: t("map.mapCreatedSuccessfully"),
          });

          // Refresh map list
          loadMapsFromROS();
        }
      } else {
        // Create mock map for demonstration
        const newMap: MapData = {
          id: `map_${Date.now()}`,
          name: newMapName,
          description: newMapDescription,
          created: new Date(),
          lastModified: new Date(),
          resolution: 0.05,
          width: 800,
          height: 600,
          origin: { x: -20, y: -15, theta: 0 },
          data: new Uint8Array(800 * 600).fill(255),
          size: 480000,
        };

        setMaps((prev) => [...prev, newMap]);
        toast({
          title: t("map.mapCreated"),
          description: t("map.mapCreatedSuccessfully"),
        });
      }

      setIsCreateDialogOpen(false);
      setNewMapName("");
      setNewMapDescription("");
    } catch (error) {
      toast({
        title: t("common.error"),
        description: t("map.createMapError"),
        variant: "destructive",
      });
    }
  };

  const handleDeleteMap = async (mapId: string) => {
    try {
      if (isConnected) {
        await callService("/delete_map", { map_id: mapId });
      }

      setMaps((prev) => prev.filter((map) => map.id !== mapId));

      if (selectedMap?.id === mapId) {
        setSelectedMap(null);
      }

      toast({
        title: t("map.mapDeleted"),
        description: t("map.mapDeletedSuccessfully"),
      });
    } catch (error) {
      toast({
        title: t("common.error"),
        description: t("map.deleteMapError"),
        variant: "destructive",
      });
    }
  };

  const loadMapsFromROS = async () => {
    if (!isConnected) return;

    try {
      const result = await callService("/list_maps", {});
      if (result && result.maps) {
        setMaps(result.maps);
      }
    } catch (error) {
      console.error("Failed to load maps from ROS:", error);
    }
  };

  const resetView = () => {
    setZoom(1);
    setPan({ x: 0, y: 0 });
  };

  const formatFileSize = (bytes: number): string => {
    if (bytes === 0) return "0 B";
    const k = 1024;
    const sizes = ["B", "KB", "MB", "GB"];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + " " + sizes[i];
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            {t("map.title")}
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            {t("map.subtitle")}
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

          <Dialog
            open={isCreateDialogOpen}
            onOpenChange={setIsCreateDialogOpen}
          >
            <DialogTrigger asChild>
              <Button className="gap-2">
                <Plus className="h-4 w-4" />
                {t("map.createMap")}
              </Button>
            </DialogTrigger>
            <DialogContent>
              <DialogHeader>
                <DialogTitle>{t("map.createNewMap")}</DialogTitle>
                <DialogDescription>
                  {t("map.createNewMapDescription")}
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div>
                  <Label htmlFor="mapName">{t("map.mapName")}</Label>
                  <Input
                    id="mapName"
                    value={newMapName}
                    onChange={(e) => setNewMapName(e.target.value)}
                    placeholder={t("map.enterMapName")}
                  />
                </div>
                <div>
                  <Label htmlFor="mapDescription">{t("map.description")}</Label>
                  <Input
                    id="mapDescription"
                    value={newMapDescription}
                    onChange={(e) => setNewMapDescription(e.target.value)}
                    placeholder={t("map.enterDescription")}
                  />
                </div>
              </div>
              <DialogFooter>
                <Button
                  variant="outline"
                  onClick={() => setIsCreateDialogOpen(false)}
                >
                  {t("common.cancel")}
                </Button>
                <Button onClick={handleCreateMap} disabled={!newMapName.trim()}>
                  {t("common.create")}
                </Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Map List Panel */}
        <Card className="lg:col-span-1 glass-effect">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Map className="h-5 w-5" />
              {t("map.availableMaps")}
            </CardTitle>
            <CardDescription>{t("map.manageMaps")}</CardDescription>
          </CardHeader>
          <CardContent>
            <ScrollArea className="h-[400px]">
              <div className="space-y-3">
                {maps.map((map) => (
                  <div
                    key={map.id}
                    className={`p-3 rounded-lg border cursor-pointer transition-all duration-200 hover:bg-accent/50 ${
                      selectedMap?.id === map.id
                        ? "border-primary bg-primary/10"
                        : "border-border"
                    }`}
                    onClick={() => setSelectedMap(map)}
                  >
                    <div className="flex items-center justify-between mb-2">
                      <h4 className="font-medium text-sm">{map.name}</h4>
                      <AlertDialog>
                        <AlertDialogTrigger asChild>
                          <Button
                            variant="ghost"
                            size="sm"
                            className="h-8 w-8 p-0"
                          >
                            <Trash2 className="h-4 w-4" />
                          </Button>
                        </AlertDialogTrigger>
                        <AlertDialogContent>
                          <AlertDialogHeader>
                            <AlertDialogTitle>
                              {t("map.deleteMap")}
                            </AlertDialogTitle>
                            <AlertDialogDescription>
                              {t("map.deleteMapConfirmation")}
                            </AlertDialogDescription>
                          </AlertDialogHeader>
                          <AlertDialogFooter>
                            <AlertDialogCancel>
                              {t("common.cancel")}
                            </AlertDialogCancel>
                            <AlertDialogAction
                              onClick={() => handleDeleteMap(map.id)}
                            >
                              {t("common.delete")}
                            </AlertDialogAction>
                          </AlertDialogFooter>
                        </AlertDialogContent>
                      </AlertDialog>
                    </div>
                    <p className="text-xs text-muted-foreground mb-2">
                      {map.description}
                    </p>
                    <div className="text-xs text-muted-foreground space-y-1">
                      <div>
                        {t("map.resolution")}: {map.resolution}m/px
                      </div>
                      <div>
                        {t("map.size")}: {map.width}×{map.height}
                      </div>
                      <div>
                        {t("map.fileSize")}: {formatFileSize(map.size)}
                      </div>
                      <div>
                        {t("map.lastModified")}:{" "}
                        {map.lastModified.toLocaleDateString()}
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </ScrollArea>
          </CardContent>
        </Card>

        {/* Map Viewer */}
        <Card className="lg:col-span-3 glass-effect">
          <CardHeader>
            <div className="flex items-center justify-between">
              <div>
                <CardTitle className="flex items-center gap-2">
                  <Navigation className="h-5 w-5" />
                  {selectedMap ? selectedMap.name : t("map.noMapSelected")}
                </CardTitle>
                <CardDescription>
                  {selectedMap
                    ? selectedMap.description
                    : t("map.selectMapToView")}
                </CardDescription>
              </div>

              <div className="flex items-center gap-2">
                {/* View Controls */}
                <div className="flex items-center gap-1 bg-background/50 rounded-lg p-1">
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => setZoom((prev) => Math.min(5, prev * 1.2))}
                  >
                    <ZoomIn className="h-4 w-4" />
                  </Button>
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => setZoom((prev) => Math.max(0.1, prev * 0.8))}
                  >
                    <ZoomOut className="h-4 w-4" />
                  </Button>
                  <Button variant="ghost" size="sm" onClick={resetView}>
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

                {/* Playback Controls */}
                <div className="flex items-center gap-1 bg-background/50 rounded-lg p-1">
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => setIsPlaying(!isPlaying)}
                  >
                    {isPlaying ? (
                      <Pause className="h-4 w-4" />
                    ) : (
                      <Play className="h-4 w-4" />
                    )}
                  </Button>
                  <Button variant="ghost" size="sm" onClick={drawMap}>
                    <RefreshCw className="h-4 w-4" />
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
                      <DialogTitle>{t("map.displaySettings")}</DialogTitle>
                    </DialogHeader>
                    <div className="space-y-4">
                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <Label>{t("map.showGrid")}</Label>
                          <Switch
                            checked={settings.showGrid}
                            onCheckedChange={(checked) =>
                              setSettings((prev) => ({
                                ...prev,
                                showGrid: checked,
                              }))
                            }
                          />
                        </div>
                        <div className="flex items-center justify-between">
                          <Label>{t("map.showRobot")}</Label>
                          <Switch
                            checked={settings.showRobot}
                            onCheckedChange={(checked) =>
                              setSettings((prev) => ({
                                ...prev,
                                showRobot: checked,
                              }))
                            }
                          />
                        </div>
                        <div className="flex items-center justify-between">
                          <Label>{t("map.showGoal")}</Label>
                          <Switch
                            checked={settings.showGoal}
                            onCheckedChange={(checked) =>
                              setSettings((prev) => ({
                                ...prev,
                                showGoal: checked,
                              }))
                            }
                          />
                        </div>
                        <div className="flex items-center justify-between">
                          <Label>{t("map.showTrajectory")}</Label>
                          <Switch
                            checked={settings.showTrajectory}
                            onCheckedChange={(checked) =>
                              setSettings((prev) => ({
                                ...prev,
                                showTrajectory: checked,
                              }))
                            }
                          />
                        </div>
                      </div>

                      <Separator />

                      <div className="space-y-3">
                        <div>
                          <Label>
                            {t("map.refreshRate")}: {settings.fps} FPS
                          </Label>
                          <Slider
                            value={[settings.fps]}
                            onValueChange={([value]) =>
                              setSettings((prev) => ({ ...prev, fps: value }))
                            }
                            min={1}
                            max={60}
                            step={1}
                            className="mt-2"
                          />
                        </div>

                        <div>
                          <Label>
                            {t("map.occupancyThreshold")}:{" "}
                            {settings.occupancyThreshold}%
                          </Label>
                          <Slider
                            value={[settings.occupancyThreshold]}
                            onValueChange={([value]) =>
                              setSettings((prev) => ({
                                ...prev,
                                occupancyThreshold: value,
                              }))
                            }
                            min={0}
                            max={100}
                            step={5}
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
              className={`relative bg-white rounded-lg overflow-hidden ${isFullscreen ? "fixed inset-4 z-50" : ""}`}
            >
              {selectedMap ? (
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
                  onClick={handleCanvasClick}
                />
              ) : (
                <div className="h-[600px] flex items-center justify-center text-muted-foreground">
                  <div className="text-center">
                    <Map className="h-16 w-16 mx-auto mb-4 opacity-50" />
                    <p className="text-lg font-light">
                      {t("map.noMapSelected")}
                    </p>
                    <p className="text-sm">{t("map.selectMapFromList")}</p>
                  </div>
                </div>
              )}

              {/* Map Info Overlay */}
              {selectedMap && (
                <div className="absolute top-4 left-4 bg-background/90 backdrop-blur-sm rounded-lg p-3 border border-border/20">
                  <div className="text-sm space-y-1">
                    <div className="font-medium">{selectedMap.name}</div>
                    <div className="text-muted-foreground">
                      {t("map.zoom")}: {(zoom * 100).toFixed(0)}%
                    </div>
                    {robotPosition && (
                      <div className="text-muted-foreground">
                        {t("map.robotPos")}: ({robotPosition.x.toFixed(2)},{" "}
                        {robotPosition.y.toFixed(2)})
                      </div>
                    )}
                    {goalPosition && (
                      <div className="text-muted-foreground">
                        {t("map.goalPos")}: ({goalPosition.x.toFixed(2)},{" "}
                        {goalPosition.y.toFixed(2)})
                      </div>
                    )}
                  </div>
                </div>
              )}

              {/* Status Overlay */}
              <div className="absolute top-4 right-4 bg-background/90 backdrop-blur-sm rounded-lg p-3 border border-border/20">
                <div className="text-sm space-y-1">
                  <div className="flex items-center gap-2">
                    <div
                      className={`w-2 h-2 rounded-full ${isPlaying ? "bg-green-400" : "bg-red-400"}`}
                    />
                    {isPlaying ? t("map.playing") : t("map.paused")}
                  </div>
                  <div>
                    {t("map.fps")}: {settings.fps}
                  </div>
                  {selectedMap && (
                    <div>
                      {t("map.mapSize")}: {selectedMap.width}×
                      {selectedMap.height}
                    </div>
                  )}
                </div>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Map Statistics */}
      {selectedMap && (
        <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
          <Card className="glass-effect">
            <CardContent className="p-4">
              <div className="flex items-center gap-3">
                <div className="p-2 bg-blue-500/10 rounded-lg">
                  <MapPin className="h-5 w-5 text-blue-500" />
                </div>
                <div>
                  <div className="text-sm text-muted-foreground">
                    {t("map.resolution")}
                  </div>
                  <div className="text-lg font-semibold">
                    {selectedMap.resolution}m/px
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card className="glass-effect">
            <CardContent className="p-4">
              <div className="flex items-center gap-3">
                <div className="p-2 bg-green-500/10 rounded-lg">
                  <Grid3x3 className="h-5 w-5 text-green-500" />
                </div>
                <div>
                  <div className="text-sm text-muted-foreground">
                    {t("map.dimensions")}
                  </div>
                  <div className="text-lg font-semibold">
                    {selectedMap.width}×{selectedMap.height}
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card className="glass-effect">
            <CardContent className="p-4">
              <div className="flex items-center gap-3">
                <div className="p-2 bg-purple-500/10 rounded-lg">
                  <Target className="h-5 w-5 text-purple-500" />
                </div>
                <div>
                  <div className="text-sm text-muted-foreground">
                    {t("map.origin")}
                  </div>
                  <div className="text-lg font-semibold">
                    ({selectedMap.origin.x}, {selectedMap.origin.y})
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card className="glass-effect">
            <CardContent className="p-4">
              <div className="flex items-center gap-3">
                <div className="p-2 bg-orange-500/10 rounded-lg">
                  <Save className="h-5 w-5 text-orange-500" />
                </div>
                <div>
                  <div className="text-sm text-muted-foreground">
                    {t("map.fileSize")}
                  </div>
                  <div className="text-lg font-semibold">
                    {formatFileSize(selectedMap.size)}
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      )}
    </div>
  );
};

export default MapViewer;
