import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Camera, Play, Pause, Maximize, Settings } from "lucide-react";

const Cameras = () => {
  const cameras = [
    {
      name: "Front Camera",
      topic: "/camera/front/image_raw",
      status: "streaming",
      resolution: "1920x1080",
      fps: "30",
      encoding: "bgr8",
    },
    {
      name: "Rear Camera",
      topic: "/camera/rear/image_raw",
      status: "streaming",
      resolution: "1280x720",
      fps: "30",
      encoding: "bgr8",
    },
    {
      name: "Left Side Camera",
      topic: "/camera/left/image_raw",
      status: "offline",
      resolution: "640x480",
      fps: "15",
      encoding: "mono8",
    },
    {
      name: "Right Side Camera",
      topic: "/camera/right/image_raw",
      status: "streaming",
      resolution: "640x480",
      fps: "15",
      encoding: "mono8",
    },
  ];

  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Camera Feeds</h1>
        <p className="text-muted-foreground">
          Live camera streams and image processing
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {cameras.map((camera, index) => (
          <Card key={index} className="overflow-hidden">
            <div className="p-4 border-b border-border">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-3">
                  <Camera className="h-5 w-5 text-primary" />
                  <div>
                    <h3 className="font-semibold">{camera.name}</h3>
                    <p className="text-sm text-muted-foreground font-mono">
                      {camera.topic}
                    </p>
                  </div>
                </div>
                <Badge
                  variant={
                    camera.status === "streaming" ? "default" : "destructive"
                  }
                >
                  {camera.status}
                </Badge>
              </div>
            </div>

            <div className="aspect-video bg-muted relative flex items-center justify-center">
              {camera.status === "streaming" ? (
                <div className="text-center">
                  <div className="w-16 h-16 rounded-full bg-primary/20 flex items-center justify-center mb-4">
                    <Camera className="h-8 w-8 text-primary" />
                  </div>
                  <p className="text-sm text-muted-foreground">Live Feed</p>
                  <p className="text-xs text-muted-foreground">
                    {camera.resolution} @ {camera.fps}fps
                  </p>
                </div>
              ) : (
                <div className="text-center">
                  <div className="w-16 h-16 rounded-full bg-muted-foreground/20 flex items-center justify-center mb-4">
                    <Camera className="h-8 w-8 text-muted-foreground" />
                  </div>
                  <p className="text-sm text-muted-foreground">
                    Camera Offline
                  </p>
                </div>
              )}
            </div>

            <div className="p-4">
              <div className="flex items-center justify-between mb-4">
                <div className="text-sm space-y-1">
                  <div className="flex justify-between">
                    <span className="text-muted-foreground">Resolution:</span>
                    <span className="font-mono">{camera.resolution}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-muted-foreground">FPS:</span>
                    <span className="font-mono">{camera.fps}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-muted-foreground">Encoding:</span>
                    <span className="font-mono">{camera.encoding}</span>
                  </div>
                </div>
              </div>

              <div className="flex gap-2">
                <Button size="sm" variant="outline" className="gap-2">
                  {camera.status === "streaming" ? (
                    <Pause className="h-4 w-4" />
                  ) : (
                    <Play className="h-4 w-4" />
                  )}
                  {camera.status === "streaming" ? "Pause" : "Start"}
                </Button>
                <Button size="sm" variant="outline" className="gap-2">
                  <Maximize className="h-4 w-4" />
                  Fullscreen
                </Button>
                <Button size="sm" variant="outline" className="gap-2">
                  <Settings className="h-4 w-4" />
                  Settings
                </Button>
              </div>
            </div>
          </Card>
        ))}
      </div>
    </div>
  );
};

export default Cameras;
