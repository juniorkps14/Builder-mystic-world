import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  GitBranch,
  RefreshCw,
  Download,
  Settings,
  Activity,
  AlertTriangle,
  CheckCircle,
  Clock,
} from "lucide-react";

interface TFFrame {
  name: string;
  parent: string | null;
  children: string[];
  translation: [number, number, number];
  rotation: [number, number, number, number]; // quaternion
  lastUpdate: Date;
  status: "active" | "stale" | "error";
}

const TFTree = () => {
  const { t } = useLanguage();

  // Mock TF tree data
  const [tfFrames, setTfFrames] = useState<TFFrame[]>([
    {
      name: "map",
      parent: null,
      children: ["odom"],
      translation: [0, 0, 0],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(),
      status: "active",
    },
    {
      name: "odom",
      parent: "map",
      children: ["base_link"],
      translation: [2.34, 1.67, 0],
      rotation: [0, 0, 0.225, 0.974],
      lastUpdate: new Date(Date.now() - 100),
      status: "active",
    },
    {
      name: "base_link",
      parent: "odom",
      children: [
        "base_footprint",
        "laser_link",
        "camera_link",
        "arm_base_link",
      ],
      translation: [0, 0, 0.1],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(Date.now() - 50),
      status: "active",
    },
    {
      name: "base_footprint",
      parent: "base_link",
      children: [],
      translation: [0, 0, -0.1],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(Date.now() - 200),
      status: "active",
    },
    {
      name: "laser_link",
      parent: "base_link",
      children: [],
      translation: [0.2, 0, 0.3],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(Date.now() - 150),
      status: "active",
    },
    {
      name: "camera_link",
      parent: "base_link",
      children: ["camera_optical_frame"],
      translation: [0.25, 0, 0.4],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(Date.now() - 80),
      status: "active",
    },
    {
      name: "camera_optical_frame",
      parent: "camera_link",
      children: [],
      translation: [0, 0, 0],
      rotation: [-0.5, 0.5, -0.5, 0.5],
      lastUpdate: new Date(Date.now() - 80),
      status: "active",
    },
    {
      name: "arm_base_link",
      parent: "base_link",
      children: ["arm_shoulder_link"],
      translation: [0, 0, 0.2],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(Date.now() - 120),
      status: "active",
    },
    {
      name: "arm_shoulder_link",
      parent: "arm_base_link",
      children: ["arm_elbow_link"],
      translation: [0, 0, 0.1],
      rotation: [0, 0.1736, 0, 0.9848],
      lastUpdate: new Date(Date.now() - 120),
      status: "active",
    },
    {
      name: "arm_elbow_link",
      parent: "arm_shoulder_link",
      children: ["arm_wrist_link"],
      translation: [0.3, 0, 0],
      rotation: [0, -0.2588, 0, 0.9659],
      lastUpdate: new Date(Date.now() - 120),
      status: "active",
    },
    {
      name: "arm_wrist_link",
      parent: "arm_elbow_link",
      children: ["arm_end_effector"],
      translation: [0.25, 0, 0],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(Date.now() - 120),
      status: "active",
    },
    {
      name: "arm_end_effector",
      parent: "arm_wrist_link",
      children: [],
      translation: [0.15, 0, 0],
      rotation: [0, 0, 0, 1],
      lastUpdate: new Date(Date.now() - 120),
      status: "active",
    },
  ]);

  const [selectedFrame, setSelectedFrame] = useState<string | null>(
    "base_link",
  );
  const [isAutoRefresh, setIsAutoRefresh] = useState(true);

  // Simulate real-time updates
  useEffect(() => {
    if (!isAutoRefresh) return;

    const interval = setInterval(() => {
      setTfFrames((prev) =>
        prev.map((frame) => ({
          ...frame,
          lastUpdate: new Date(),
          // Simulate small position changes for some frames
          translation:
            frame.name === "odom"
              ? [
                  frame.translation[0] + (Math.random() - 0.5) * 0.01,
                  frame.translation[1] + (Math.random() - 0.5) * 0.01,
                  frame.translation[2],
                ]
              : frame.translation,
        })),
      );
    }, 1000);

    return () => clearInterval(interval);
  }, [isAutoRefresh]);

  const getFramesByParent = (parent: string | null) => {
    return tfFrames.filter((frame) => frame.parent === parent);
  };

  const renderFrame = (frame: TFFrame, depth: number = 0) => {
    const children = getFramesByParent(frame.name);
    const isSelected = selectedFrame === frame.name;
    const timeSinceUpdate = Date.now() - frame.lastUpdate.getTime();

    return (
      <div key={frame.name} className="space-y-1">
        <div
          className={`flex items-center gap-2 p-2 rounded cursor-pointer transition-colors ${
            isSelected
              ? "bg-primary/20 border border-primary/30"
              : "hover:bg-accent"
          }`}
          style={{ marginLeft: `${depth * 20}px` }}
          onClick={() => setSelectedFrame(frame.name)}
        >
          <div className="flex items-center gap-2 flex-1">
            {children.length > 0 && (
              <GitBranch className="h-3 w-3 text-muted-foreground" />
            )}
            <span className="font-mono text-sm">{frame.name}</span>

            <div className="flex items-center gap-1">
              {frame.status === "active" && (
                <CheckCircle className="h-3 w-3 text-ros-success" />
              )}
              {frame.status === "stale" && (
                <Clock className="h-3 w-3 text-ros-warning" />
              )}
              {frame.status === "error" && (
                <AlertTriangle className="h-3 w-3 text-destructive" />
              )}

              <Badge variant="outline" className="text-xs">
                {timeSinceUpdate}ms
              </Badge>
            </div>
          </div>
        </div>

        {children.map((child) => renderFrame(child, depth + 1))}
      </div>
    );
  };

  const selectedFrameData = tfFrames.find((f) => f.name === selectedFrame);

  return (
    <div className="space-y-6">
      <div className="mb-6">
        <h1 className="text-3xl font-bold">TF Tree Viewer</h1>
        <p className="text-muted-foreground">
          Real-time Transform Frame visualization and monitoring
        </p>
      </div>

      {/* Controls */}
      <div className="flex items-center justify-between">
        <div className="flex gap-2">
          <Button
            variant={isAutoRefresh ? "default" : "outline"}
            onClick={() => setIsAutoRefresh(!isAutoRefresh)}
            className="gap-2"
          >
            <RefreshCw
              className={`h-4 w-4 ${isAutoRefresh ? "animate-spin" : ""}`}
            />
            Auto Refresh
          </Button>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export TF Tree
          </Button>
          <Button variant="outline" className="gap-2">
            <Settings className="h-4 w-4" />
            Settings
          </Button>
        </div>

        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2 text-sm">
            <Activity className="h-4 w-4 text-primary" />
            <span>{tfFrames.length} frames</span>
          </div>
          <div className="flex items-center gap-2 text-sm">
            <CheckCircle className="h-4 w-4 text-ros-success" />
            <span>
              {tfFrames.filter((f) => f.status === "active").length} active
            </span>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* TF Tree Structure */}
        <Card className="lg:col-span-2 p-4">
          <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
            <GitBranch className="h-5 w-5" />
            Transform Tree
          </h3>

          <ScrollArea className="h-[600px]">
            <div className="space-y-1">
              {getFramesByParent(null).map((rootFrame) =>
                renderFrame(rootFrame),
              )}
            </div>
          </ScrollArea>
        </Card>

        {/* Frame Details */}
        <div className="space-y-4">
          <Card className="p-4">
            <h3 className="text-lg font-semibold mb-4">Frame Details</h3>

            {selectedFrameData ? (
              <div className="space-y-4">
                <div>
                  <Label className="text-sm font-medium">Frame Name</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedFrameData.name}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Parent Frame</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedFrameData.parent || "None (root)"}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Children</Label>
                  <div className="space-y-1 mt-1">
                    {selectedFrameData.children.length > 0 ? (
                      selectedFrameData.children.map((child) => (
                        <div
                          key={child}
                          className="font-mono text-sm bg-muted p-2 rounded"
                        >
                          {child}
                        </div>
                      ))
                    ) : (
                      <div className="text-sm text-muted-foreground">
                        No children
                      </div>
                    )}
                  </div>
                </div>

                <Separator />

                <div>
                  <Label className="text-sm font-medium">Translation (m)</Label>
                  <div className="grid grid-cols-3 gap-2 mt-1">
                    <div className="text-center">
                      <div className="text-xs text-muted-foreground">X</div>
                      <div className="font-mono text-sm bg-muted p-2 rounded">
                        {selectedFrameData.translation[0].toFixed(3)}
                      </div>
                    </div>
                    <div className="text-center">
                      <div className="text-xs text-muted-foreground">Y</div>
                      <div className="font-mono text-sm bg-muted p-2 rounded">
                        {selectedFrameData.translation[1].toFixed(3)}
                      </div>
                    </div>
                    <div className="text-center">
                      <div className="text-xs text-muted-foreground">Z</div>
                      <div className="font-mono text-sm bg-muted p-2 rounded">
                        {selectedFrameData.translation[2].toFixed(3)}
                      </div>
                    </div>
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">
                    Rotation (quaternion)
                  </Label>
                  <div className="grid grid-cols-2 gap-2 mt-1">
                    <div className="text-center">
                      <div className="text-xs text-muted-foreground">X</div>
                      <div className="font-mono text-sm bg-muted p-2 rounded">
                        {selectedFrameData.rotation[0].toFixed(3)}
                      </div>
                    </div>
                    <div className="text-center">
                      <div className="text-xs text-muted-foreground">Y</div>
                      <div className="font-mono text-sm bg-muted p-2 rounded">
                        {selectedFrameData.rotation[1].toFixed(3)}
                      </div>
                    </div>
                    <div className="text-center">
                      <div className="text-xs text-muted-foreground">Z</div>
                      <div className="font-mono text-sm bg-muted p-2 rounded">
                        {selectedFrameData.rotation[2].toFixed(3)}
                      </div>
                    </div>
                    <div className="text-center">
                      <div className="text-xs text-muted-foreground">W</div>
                      <div className="font-mono text-sm bg-muted p-2 rounded">
                        {selectedFrameData.rotation[3].toFixed(3)}
                      </div>
                    </div>
                  </div>
                </div>

                <Separator />

                <div>
                  <Label className="text-sm font-medium">Status</Label>
                  <div className="flex items-center gap-2 mt-1">
                    <Badge
                      variant={
                        selectedFrameData.status === "active"
                          ? "default"
                          : "destructive"
                      }
                      className="gap-1"
                    >
                      {selectedFrameData.status === "active" && (
                        <CheckCircle className="h-3 w-3" />
                      )}
                      {selectedFrameData.status === "stale" && (
                        <Clock className="h-3 w-3" />
                      )}
                      {selectedFrameData.status === "error" && (
                        <AlertTriangle className="h-3 w-3" />
                      )}
                      {selectedFrameData.status.toUpperCase()}
                    </Badge>
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Last Update</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedFrameData.lastUpdate.toLocaleTimeString()}
                  </div>
                </div>
              </div>
            ) : (
              <div className="text-center text-muted-foreground py-8">
                Select a frame to view details
              </div>
            )}
          </Card>

          {/* TF Statistics */}
          <Card className="p-4">
            <h3 className="text-lg font-semibold mb-4">TF Statistics</h3>

            <div className="space-y-3">
              <div className="flex justify-between">
                <span className="text-sm">Total Frames:</span>
                <Badge variant="outline">{tfFrames.length}</Badge>
              </div>

              <div className="flex justify-between">
                <span className="text-sm">Active Frames:</span>
                <Badge variant="default">
                  {tfFrames.filter((f) => f.status === "active").length}
                </Badge>
              </div>

              <div className="flex justify-between">
                <span className="text-sm">Stale Frames:</span>
                <Badge variant="secondary">
                  {tfFrames.filter((f) => f.status === "stale").length}
                </Badge>
              </div>

              <div className="flex justify-between">
                <span className="text-sm">Error Frames:</span>
                <Badge variant="destructive">
                  {tfFrames.filter((f) => f.status === "error").length}
                </Badge>
              </div>

              <Separator />

              <div className="flex justify-between">
                <span className="text-sm">Root Frames:</span>
                <Badge variant="outline">
                  {tfFrames.filter((f) => f.parent === null).length}
                </Badge>
              </div>

              <div className="flex justify-between">
                <span className="text-sm">Leaf Frames:</span>
                <Badge variant="outline">
                  {tfFrames.filter((f) => f.children.length === 0).length}
                </Badge>
              </div>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
};

function Label({
  children,
  className,
}: {
  children: React.ReactNode;
  className?: string;
}) {
  return <div className={className}>{children}</div>;
}

export default TFTree;
