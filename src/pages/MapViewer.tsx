import { useState, useRef, useEffect, useCallback } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Slider } from "@/components/ui/slider";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
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
} from "lucide-react";

const MapViewer = () => {
  const { t } = useLanguage();
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Map state
  const [mapData, setMapData] = useState({
    resolution: 0.05, // meters per pixel
    width: 800,
    height: 600,
    origin: [-20, -15, 0], // [x, y, theta] in meters
  });

  // View state
  const [viewState, setViewState] = useState({
    zoom: [100], // percentage
    panX: [0],
    panY: [0],
    rotation: [0],
  });

  // Layer visibility
  const [layerVisibility, setLayerVisibility] = useState({
    occupancyGrid: true,
    robotPath: true,
    plannedPath: true,
    obstacles: true,
    waypoints: true,
    robotPosition: true,
    laserScan: true,
    costmap: false,
  });

  // Helper function to generate laser scan (moved before state initialization)
  const generateLaserScan = useCallback(
    (
      position: { x: number; y: number; theta: number } = {
        x: 2.34,
        y: 1.67,
        theta: 0.45,
      },
    ) => {
      const scan = [];
      for (let i = 0; i < 360; i++) {
        const angle = (i * Math.PI) / 180;
        const range = 3 + Math.random() * 2; // Random range between 3-5 meters
        scan.push({
          angle,
          range,
          x: position.x + range * Math.cos(angle + position.theta),
          y: position.y + range * Math.sin(angle + position.theta),
        });
      }
      return scan;
    },
    [],
  );

  // Robot state
  const [robotState, setRobotState] = useState(() => {
    const initialPosition = { x: 2.34, y: 1.67, theta: 0.45 };
    return {
      position: initialPosition,
      velocity: { linear: 0.25, angular: 0.1 },
      path: [
        { x: 0, y: 0 },
        { x: 1.2, y: 0.8 },
        { x: 2.34, y: 1.67 },
      ],
      plannedPath: [
        { x: 2.34, y: 1.67 },
        { x: 3.5, y: 2.1 },
        { x: 5.0, y: 2.5 },
        { x: 6.2, y: 1.8 },
      ],
      laserScan: generateLaserScan(initialPosition),
    };
  });

  // Waypoints
  const [waypoints, setWaypoints] = useState([
    { id: 1, x: 5.0, y: 2.0, name: "Checkpoint A", type: "patrol" },
    { id: 2, x: -3.0, y: 4.0, name: "Checkpoint B", type: "patrol" },
    { id: 3, x: 0, y: 0, name: "Home", type: "home" },
    { id: 4, x: 8.0, y: -2.0, name: "Goal", type: "goal" },
  ]);

  // Map navigation mode
  const [navigationMode, setNavigationMode] = useState<
    "view" | "setGoal" | "addWaypoint"
  >("view");

  // Canvas drawing functions
  const drawMap = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = "#1a1a1a";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Save context for transformations
    ctx.save();

    // Apply zoom and pan
    const scale = viewState.zoom[0] / 100;
    ctx.scale(scale, scale);
    ctx.translate(
      canvas.width / (2 * scale) + viewState.panX[0],
      canvas.height / (2 * scale) + viewState.panY[0],
    );

    // Convert world coordinates to canvas coordinates
    const worldToCanvas = (worldX: number, worldY: number) => {
      const pixelX = (worldX - mapData.origin[0]) / mapData.resolution;
      const pixelY = (mapData.origin[1] - worldY) / mapData.resolution; // Flip Y axis
      return { x: pixelX, y: pixelY };
    };

    // Draw occupancy grid (simplified)
    if (layerVisibility.occupancyGrid) {
      ctx.fillStyle = "#444444";
      for (let x = -20; x < 20; x += 0.5) {
        for (let y = -15; y < 15; y += 0.5) {
          if (Math.random() > 0.95) {
            // Random obstacles
            const pos = worldToCanvas(x, y);
            ctx.fillRect(pos.x, pos.y, 10, 10);
          }
        }
      }
    }

    // Draw costmap
    if (layerVisibility.costmap) {
      ctx.fillStyle = "rgba(255, 0, 0, 0.3)";
      // Simplified costmap around obstacles
      const pos = worldToCanvas(robotState.position.x, robotState.position.y);
      ctx.fillRect(pos.x - 50, pos.y - 50, 100, 100);
    }

    // Draw laser scan
    if (layerVisibility.laserScan) {
      ctx.strokeStyle = "#00ff00";
      ctx.lineWidth = 1;
      ctx.beginPath();
      robotState.laserScan.forEach((point, index) => {
        const pos = worldToCanvas(point.x, point.y);
        if (index === 0) {
          ctx.moveTo(pos.x, pos.y);
        } else {
          ctx.lineTo(pos.x, pos.y);
        }
      });
      ctx.stroke();
    }

    // Draw robot path
    if (layerVisibility.robotPath) {
      ctx.strokeStyle = "#0099ff";
      ctx.lineWidth = 3;
      ctx.beginPath();
      robotState.path.forEach((point, index) => {
        const pos = worldToCanvas(point.x, point.y);
        if (index === 0) {
          ctx.moveTo(pos.x, pos.y);
        } else {
          ctx.lineTo(pos.x, pos.y);
        }
      });
      ctx.stroke();
    }

    // Draw planned path
    if (layerVisibility.plannedPath) {
      ctx.strokeStyle = "#ffaa00";
      ctx.lineWidth = 3;
      ctx.setLineDash([10, 5]);
      ctx.beginPath();
      robotState.plannedPath.forEach((point, index) => {
        const pos = worldToCanvas(point.x, point.y);
        if (index === 0) {
          ctx.moveTo(pos.x, pos.y);
        } else {
          ctx.lineTo(pos.x, pos.y);
        }
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Draw waypoints
    if (layerVisibility.waypoints) {
      waypoints.forEach((waypoint) => {
        const pos = worldToCanvas(waypoint.x, waypoint.y);

        // Draw waypoint circle
        ctx.fillStyle =
          waypoint.type === "home"
            ? "#00ff00"
            : waypoint.type === "goal"
              ? "#ff0000"
              : "#ffaa00";
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 8, 0, 2 * Math.PI);
        ctx.fill();

        // Draw waypoint label
        ctx.fillStyle = "#ffffff";
        ctx.font = "12px sans-serif";
        ctx.fillText(waypoint.name, pos.x + 12, pos.y + 4);
      });
    }

    // Draw robot position
    if (layerVisibility.robotPosition) {
      const robotPos = worldToCanvas(
        robotState.position.x,
        robotState.position.y,
      );

      // Robot body (circle)
      ctx.fillStyle = "#ff6600";
      ctx.beginPath();
      ctx.arc(robotPos.x, robotPos.y, 15, 0, 2 * Math.PI);
      ctx.fill();

      // Robot orientation (line)
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(robotPos.x, robotPos.y);
      const endX = robotPos.x + 20 * Math.cos(robotState.position.theta);
      const endY = robotPos.y - 20 * Math.sin(robotState.position.theta); // Flip Y
      ctx.lineTo(endX, endY);
      ctx.stroke();
    }

    ctx.restore();
  };

  // Update map periodically
  useEffect(() => {
    const interval = setInterval(() => {
      drawMap();

      // Simulate robot movement
      setRobotState((prev) => {
        const newPosition = {
          ...prev.position,
          x: prev.position.x + (Math.random() - 0.5) * 0.02,
          y: prev.position.y + (Math.random() - 0.5) * 0.02,
        };
        return {
          ...prev,
          position: newPosition,
          laserScan: generateLaserScan(newPosition),
        };
      });
    }, 500); // Reduced frequency to prevent performance issues

    return () => clearInterval(interval);
  }, [layerVisibility, viewState, generateLaserScan]);

  const handleCanvasClick = (event: React.MouseEvent<HTMLCanvasElement>) => {
    if (navigationMode === "view") return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    // Convert canvas coordinates to world coordinates
    const scale = viewState.zoom[0] / 100;
    const worldX =
      (x / scale - canvas.width / (2 * scale) - viewState.panX[0]) *
        mapData.resolution +
      mapData.origin[0];
    const worldY =
      mapData.origin[1] -
      (y / scale - canvas.height / (2 * scale) - viewState.panY[0]) *
        mapData.resolution;

    if (navigationMode === "setGoal") {
      console.log(
        `Setting goal at: (${worldX.toFixed(2)}, ${worldY.toFixed(2)})`,
      );
      setNavigationMode("view");
    } else if (navigationMode === "addWaypoint") {
      const newWaypoint = {
        id: waypoints.length + 1,
        x: worldX,
        y: worldY,
        name: `Waypoint ${waypoints.length + 1}`,
        type: "patrol" as const,
      };
      setWaypoints((prev) => [...prev, newWaypoint]);
      setNavigationMode("view");
    }
  };

  const resetView = () => {
    setViewState({
      zoom: [100],
      panX: [0],
      panY: [0],
      rotation: [0],
    });
  };

  const centerOnRobot = () => {
    setViewState((prev) => ({
      ...prev,
      panX: [-robotState.position.x * 20],
      panY: [robotState.position.y * 20],
    }));
  };

  return (
    <div className="space-y-6">
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Map Viewer</h1>
        <p className="text-muted-foreground">
          Real-time map visualization with robot position and navigation
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Map Canvas */}
        <Card className="lg:col-span-3 p-4">
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-semibold flex items-center gap-2">
              <Map className="h-5 w-5" />
              Occupancy Grid Map
            </h3>

            <div className="flex items-center gap-2">
              <Badge variant="outline" className="gap-1">
                <Activity className="h-3 w-3" />
                Live
              </Badge>
              <Badge variant="secondary">{viewState.zoom[0]}% zoom</Badge>
            </div>
          </div>

          {/* Map Controls */}
          <div className="flex items-center gap-2 mb-4">
            <Button
              size="sm"
              variant={navigationMode === "view" ? "default" : "outline"}
              onClick={() => setNavigationMode("view")}
              className="gap-1"
            >
              <Eye className="h-3 w-3" />
              View
            </Button>
            <Button
              size="sm"
              variant={navigationMode === "setGoal" ? "default" : "outline"}
              onClick={() => setNavigationMode("setGoal")}
              className="gap-1"
            >
              <Target className="h-3 w-3" />
              Set Goal
            </Button>
            <Button
              size="sm"
              variant={navigationMode === "addWaypoint" ? "default" : "outline"}
              onClick={() => setNavigationMode("addWaypoint")}
              className="gap-1"
            >
              <MapPin className="h-3 w-3" />
              Add Waypoint
            </Button>

            <Separator orientation="vertical" className="h-6" />

            <Button
              size="sm"
              variant="outline"
              onClick={centerOnRobot}
              className="gap-1"
            >
              <Navigation className="h-3 w-3" />
              Center on Robot
            </Button>
            <Button
              size="sm"
              variant="outline"
              onClick={resetView}
              className="gap-1"
            >
              <RotateCcw className="h-3 w-3" />
              Reset View
            </Button>
          </div>

          {/* Canvas */}
          <div className="border rounded-lg overflow-hidden bg-gray-900">
            <canvas
              ref={canvasRef}
              width={800}
              height={600}
              onClick={handleCanvasClick}
              className="cursor-crosshair"
              style={{ width: "100%", height: "auto" }}
            />
          </div>
        </Card>

        {/* Controls Panel */}
        <div className="space-y-4">
          {/* View Controls */}
          <Card className="p-4">
            <h3 className="text-lg font-semibold mb-4">View Controls</h3>

            <div className="space-y-4">
              <div>
                <Label className="text-sm mb-2 block">
                  Zoom: {viewState.zoom[0]}%
                </Label>
                <Slider
                  value={viewState.zoom}
                  onValueChange={(value) =>
                    setViewState((prev) => ({ ...prev, zoom: value }))
                  }
                  min={25}
                  max={400}
                  step={5}
                />
                <div className="flex gap-1 mt-2">
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={() =>
                      setViewState((prev) => ({
                        ...prev,
                        zoom: [Math.max(25, prev.zoom[0] - 25)],
                      }))
                    }
                  >
                    <ZoomOut className="h-3 w-3" />
                  </Button>
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={() =>
                      setViewState((prev) => ({
                        ...prev,
                        zoom: [Math.min(400, prev.zoom[0] + 25)],
                      }))
                    }
                  >
                    <ZoomIn className="h-3 w-3" />
                  </Button>
                </div>
              </div>

              <div>
                <Label className="text-sm mb-2 block">
                  Pan X: {viewState.panX[0]}
                </Label>
                <Slider
                  value={viewState.panX}
                  onValueChange={(value) =>
                    setViewState((prev) => ({ ...prev, panX: value }))
                  }
                  min={-200}
                  max={200}
                  step={1}
                />
              </div>

              <div>
                <Label className="text-sm mb-2 block">
                  Pan Y: {viewState.panY[0]}
                </Label>
                <Slider
                  value={viewState.panY}
                  onValueChange={(value) =>
                    setViewState((prev) => ({ ...prev, panY: value }))
                  }
                  min={-200}
                  max={200}
                  step={1}
                />
              </div>
            </div>
          </Card>

          {/* Layer Controls */}
          <Card className="p-4">
            <h3 className="text-lg font-semibold mb-4">Map Layers</h3>

            <div className="space-y-3">
              {Object.entries(layerVisibility).map(([layer, visible]) => (
                <div key={layer} className="flex items-center justify-between">
                  <Label className="text-sm capitalize">
                    {layer.replace(/([A-Z])/g, " $1").trim()}
                  </Label>
                  <Button
                    size="sm"
                    variant="ghost"
                    onClick={() =>
                      setLayerVisibility((prev) => ({
                        ...prev,
                        [layer]: !visible,
                      }))
                    }
                  >
                    {visible ? (
                      <Eye className="h-3 w-3" />
                    ) : (
                      <EyeOff className="h-3 w-3" />
                    )}
                  </Button>
                </div>
              ))}
            </div>
          </Card>

          {/* Robot Info */}
          <Card className="p-4">
            <h3 className="text-lg font-semibold mb-4">Robot Status</h3>

            <div className="space-y-3 text-sm">
              <div className="flex justify-between">
                <span>Position X:</span>
                <span className="font-mono">
                  {robotState.position.x.toFixed(2)} m
                </span>
              </div>
              <div className="flex justify-between">
                <span>Position Y:</span>
                <span className="font-mono">
                  {robotState.position.y.toFixed(2)} m
                </span>
              </div>
              <div className="flex justify-between">
                <span>Orientation:</span>
                <span className="font-mono">
                  {((robotState.position.theta * 180) / Math.PI).toFixed(1)}°
                </span>
              </div>
              <div className="flex justify-between">
                <span>Linear Vel:</span>
                <span className="font-mono">
                  {robotState.velocity.linear.toFixed(2)} m/s
                </span>
              </div>
              <div className="flex justify-between">
                <span>Angular Vel:</span>
                <span className="font-mono">
                  {robotState.velocity.angular.toFixed(2)} rad/s
                </span>
              </div>
            </div>
          </Card>

          {/* Map Controls */}
          <Card className="p-4">
            <h3 className="text-lg font-semibold mb-4">Map Controls</h3>

            <div className="space-y-2">
              <Button variant="outline" className="w-full gap-2">
                <Download className="h-4 w-4" />
                Save Map
              </Button>
              <Button variant="outline" className="w-full gap-2">
                <RefreshCw className="h-4 w-4" />
                Reload Map
              </Button>
              <Button variant="outline" className="w-full gap-2">
                <Settings className="h-4 w-4" />
                Map Settings
              </Button>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
};

export default MapViewer;
