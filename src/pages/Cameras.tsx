import React, { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Camera,
  Video,
  Play,
  Pause,
  Square,
  Download,
  Settings,
  Maximize2,
  Minimize2,
  RotateCcw,
  ZoomIn,
  ZoomOut,
  Grid3X3,
  Eye,
  EyeOff,
  Wifi,
  WifiOff,
  Activity,
  Circle,
} from "lucide-react";

interface CameraStream {
  id: string;
  name: string;
  topic: string;
  resolution: string;
  fps: number;
  status: "active" | "inactive" | "error";
  isRecording: boolean;
  quality: number;
}

export default function Cameras() {
  const [cameras, setCameras] = useState<CameraStream[]>([
    {
      id: "cam_1",
      name: "Front Camera",
      topic: "/camera/front/image_raw",
      resolution: "1920x1080",
      fps: 30,
      status: "active",
      isRecording: false,
      quality: 95,
    },
    {
      id: "cam_2",
      name: "Rear Camera",
      topic: "/camera/rear/image_raw",
      resolution: "1280x720",
      fps: 25,
      status: "active",
      isRecording: true,
      quality: 87,
    },
    {
      id: "cam_3",
      name: "Left Side Camera",
      topic: "/camera/left/image_raw",
      resolution: "640x480",
      fps: 15,
      status: "inactive",
      isRecording: false,
      quality: 0,
    },
    {
      id: "cam_4",
      name: "Right Side Camera",
      topic: "/camera/right/image_raw",
      resolution: "640x480",
      fps: 15,
      status: "error",
      isRecording: false,
      quality: 0,
    },
  ]);

  const [selectedCamera, setSelectedCamera] = useState(cameras[0]);
  const [viewMode, setViewMode] = useState<"single" | "grid">("single");

  const getStatusColor = (status: string) => {
    switch (status) {
      case "active":
        return "from-emerald-500 to-teal-500";
      case "error":
        return "from-red-500 to-pink-500";
      case "inactive":
        return "from-gray-500 to-slate-500";
      default:
        return "from-blue-500 to-cyan-500";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "active":
        return <Circle className="h-3 w-3 text-emerald-400 fill-emerald-400" />;
      case "error":
        return <Circle className="h-3 w-3 text-red-400 fill-red-400" />;
      case "inactive":
        return <Circle className="h-3 w-3 text-gray-400 fill-gray-400" />;
      default:
        return <Circle className="h-3 w-3 text-blue-400 fill-blue-400" />;
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Tesla-inspired Header */}
      <div className="mb-8">
        <div className="bg-white/10 backdrop-blur-xl border border-white/20 rounded-3xl p-6 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="space-y-2">
              <h1 className="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
                Camera Feeds
              </h1>
              <p className="text-slate-300 font-light">
                Real-time camera monitoring and recording system
              </p>
            </div>

            <div className="flex items-center gap-3">
              <Button
                onClick={() =>
                  setViewMode(viewMode === "single" ? "grid" : "single")
                }
                className="bg-white/10 hover:bg-white/20 border border-white/20 backdrop-blur-sm transition-all duration-300"
              >
                <Grid3X3 className="h-4 w-4 mr-2" />
                {viewMode === "single" ? "Grid View" : "Single View"}
              </Button>
              <Button className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600 text-white shadow-lg transition-all duration-300">
                <Download className="h-4 w-4 mr-2" />
                Export All
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* Camera Status Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        {cameras.map((camera) => (
          <Card
            key={camera.id}
            onClick={() => setSelectedCamera(camera)}
            className={`bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300 cursor-pointer ${
              selectedCamera.id === camera.id ? "ring-2 ring-blue-400" : ""
            }`}
          >
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-2">
                <Camera className="h-5 w-5 text-blue-400" />
                <span className="font-medium text-white text-sm">
                  {camera.name}
                </span>
              </div>
              {getStatusIcon(camera.status)}
            </div>

            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span className="text-slate-400">Resolution</span>
                <span className="text-white font-mono">
                  {camera.resolution}
                </span>
              </div>
              <div className="flex justify-between text-sm">
                <span className="text-slate-400">FPS</span>
                <span className="text-white font-mono">{camera.fps}</span>
              </div>
              <div className="flex justify-between text-sm">
                <span className="text-slate-400">Quality</span>
                <span className="text-white font-mono">{camera.quality}%</span>
              </div>
            </div>

            {camera.status === "active" && (
              <div className="mt-4">
                <Progress value={camera.quality} className="h-2 bg-slate-700" />
              </div>
            )}

            {camera.isRecording && (
              <Badge className="mt-3 bg-red-500/20 text-red-300 border border-red-500/30">
                <Circle className="h-2 w-2 mr-1 fill-red-400 animate-pulse" />
                Recording
              </Badge>
            )}
          </Card>
        ))}
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-8">
        {/* Main Camera View */}
        <div className="lg:col-span-3">
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-2xl">
            <div className="p-6 border-b border-white/10">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-3">
                  <h2 className="text-xl font-light text-white">
                    {selectedCamera.name}
                  </h2>
                  <Badge
                    className={`bg-gradient-to-r ${getStatusColor(selectedCamera.status)} text-white border-0`}
                  >
                    {selectedCamera.status}
                  </Badge>
                </div>
                <div className="flex items-center gap-2">
                  <Button
                    size="sm"
                    className="bg-white/10 hover:bg-white/20 border border-white/20"
                  >
                    <ZoomIn className="h-4 w-4" />
                  </Button>
                  <Button
                    size="sm"
                    className="bg-white/10 hover:bg-white/20 border border-white/20"
                  >
                    <ZoomOut className="h-4 w-4" />
                  </Button>
                  <Button
                    size="sm"
                    className="bg-white/10 hover:bg-white/20 border border-white/20"
                  >
                    <Maximize2 className="h-4 w-4" />
                  </Button>
                  <Button
                    size="sm"
                    className="bg-white/10 hover:bg-white/20 border border-white/20"
                  >
                    <Settings className="h-4 w-4" />
                  </Button>
                </div>
              </div>
            </div>

            <div className="p-6">
              {/* Camera Feed Placeholder */}
              <div className="aspect-video bg-gradient-to-br from-slate-800 to-slate-900 rounded-xl border border-white/10 flex items-center justify-center">
                {selectedCamera.status === "active" ? (
                  <div className="text-center">
                    <Video className="h-16 w-16 text-blue-400 mx-auto mb-4" />
                    <p className="text-slate-300">
                      Camera Feed: {selectedCamera.topic}
                    </p>
                    <p className="text-slate-400 text-sm mt-2">
                      {selectedCamera.resolution} @ {selectedCamera.fps}fps
                    </p>
                  </div>
                ) : (
                  <div className="text-center">
                    <EyeOff className="h-16 w-16 text-slate-500 mx-auto mb-4" />
                    <p className="text-slate-400">Camera Offline</p>
                    <p className="text-slate-500 text-sm mt-2">
                      Check connection and settings
                    </p>
                  </div>
                )}
              </div>

              {/* Controls */}
              <div className="flex items-center justify-center gap-4 mt-6">
                <Button className="bg-gradient-to-r from-emerald-500 to-teal-500 hover:from-emerald-600 hover:to-teal-600">
                  <Play className="h-4 w-4 mr-2" />
                  Start Stream
                </Button>
                <Button className="bg-white/10 hover:bg-white/20 border border-white/20">
                  <Pause className="h-4 w-4 mr-2" />
                  Pause
                </Button>
                <Button className="bg-gradient-to-r from-red-500 to-pink-500 hover:from-red-600 hover:to-pink-600">
                  <Square className="h-4 w-4 mr-2" />
                  Stop
                </Button>
                <Button className="bg-white/10 hover:bg-white/20 border border-white/20">
                  <Download className="h-4 w-4 mr-2" />
                  Save Frame
                </Button>
              </div>
            </div>
          </Card>
        </div>

        {/* Camera Settings & Info */}
        <div className="space-y-6">
          {/* Camera Info */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                Camera Details
              </h3>
              <div className="space-y-3">
                <div>
                  <p className="text-sm text-slate-400">Topic</p>
                  <p className="text-white font-mono text-sm">
                    {selectedCamera.topic}
                  </p>
                </div>
                <div>
                  <p className="text-sm text-slate-400">Resolution</p>
                  <p className="text-white">{selectedCamera.resolution}</p>
                </div>
                <div>
                  <p className="text-sm text-slate-400">Frame Rate</p>
                  <p className="text-white">{selectedCamera.fps} FPS</p>
                </div>
                <div>
                  <p className="text-sm text-slate-400">Quality</p>
                  <div className="mt-2">
                    <Progress
                      value={selectedCamera.quality}
                      className="h-2 bg-slate-700"
                    />
                    <p className="text-xs text-slate-400 mt-1">
                      {selectedCamera.quality}%
                    </p>
                  </div>
                </div>
              </div>
            </div>
          </Card>

          {/* Recording Controls */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">Recording</h3>
              <div className="space-y-4">
                {selectedCamera.isRecording ? (
                  <div className="text-center">
                    <div className="w-16 h-16 bg-red-500/20 rounded-full flex items-center justify-center mx-auto mb-4">
                      <Circle className="h-8 w-8 text-red-400 fill-red-400 animate-pulse" />
                    </div>
                    <p className="text-red-300 font-medium">Recording Active</p>
                    <Button className="mt-4 bg-red-500/20 hover:bg-red-500/30 border border-red-500/30 text-red-300 w-full">
                      <Square className="h-4 w-4 mr-2" />
                      Stop Recording
                    </Button>
                  </div>
                ) : (
                  <div className="text-center">
                    <Button className="bg-gradient-to-r from-red-500 to-pink-500 hover:from-red-600 hover:to-pink-600 w-full">
                      <Circle className="h-4 w-4 mr-2" />
                      Start Recording
                    </Button>
                  </div>
                )}
              </div>
            </div>
          </Card>

          {/* Quick Settings */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                Quick Settings
              </h3>
              <div className="space-y-3">
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <Settings className="h-4 w-4 mr-2" />
                  Camera Settings
                </Button>
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <RotateCcw className="h-4 w-4 mr-2" />
                  Reset View
                </Button>
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <Download className="h-4 w-4 mr-2" />
                  Export Config
                </Button>
              </div>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
}
