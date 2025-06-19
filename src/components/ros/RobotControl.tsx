import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  RotateCcw,
  RotateCw,
  Square,
  Home,
  MapPin,
  Navigation,
  Zap,
  Shield,
  AlertTriangle,
} from "lucide-react";

export function RobotControl() {
  const [linearSpeed, setLinearSpeed] = useState([0.5]);
  const [angularSpeed, setAngularSpeed] = useState([0.3]);
  const [isEmergencyStop, setIsEmergencyStop] = useState(false);
  const [controlMode, setControlMode] = useState<"manual" | "autonomous">(
    "manual",
  );

  const controlButtons = [
    { icon: ArrowUp, label: "Forward", action: "forward" },
    { icon: ArrowLeft, label: "Left", action: "left" },
    { icon: Square, label: "Stop", action: "stop" },
    { icon: ArrowRight, label: "Right", action: "right" },
    { icon: ArrowDown, label: "Backward", action: "backward" },
    { icon: RotateCcw, label: "Rotate Left", action: "rotate_left" },
    { icon: RotateCw, label: "Rotate Right", action: "rotate_right" },
  ];

  const presetPositions = [
    { name: "Home", x: 0, y: 0, theta: 0 },
    { name: "Charging Station", x: -2.5, y: 3.1, theta: 1.57 },
    { name: "Patrol Point A", x: 5.2, y: 2.8, theta: 0 },
    { name: "Patrol Point B", x: 3.7, y: -1.4, theta: -1.57 },
  ];

  const robotState = {
    position: { x: 2.34, y: 1.67, theta: 0.45 },
    velocity: { linear: 0.25, angular: 0.1 },
    battery: 87,
    obstacles: false,
    localized: true,
  };

  return (
    <div className="space-y-6">
      {/* Emergency Stop and Mode Selection */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <Button
            size="lg"
            variant={isEmergencyStop ? "default" : "destructive"}
            className="gap-2"
            onClick={() => setIsEmergencyStop(!isEmergencyStop)}
          >
            <Shield className="h-5 w-5" />
            {isEmergencyStop ? "Resume" : "EMERGENCY STOP"}
          </Button>

          <Badge variant={controlMode === "manual" ? "default" : "secondary"}>
            {controlMode.toUpperCase()} MODE
          </Badge>
        </div>

        <div className="flex items-center gap-2">
          <Button
            variant={controlMode === "manual" ? "default" : "outline"}
            onClick={() => setControlMode("manual")}
          >
            Manual
          </Button>
          <Button
            variant={controlMode === "autonomous" ? "default" : "outline"}
            onClick={() => setControlMode("autonomous")}
          >
            Autonomous
          </Button>
        </div>
      </div>

      {/* Robot Status */}
      <Card className="p-6">
        <h3 className="text-lg font-semibold mb-4">Robot Status</h3>
        <div className="grid grid-cols-2 md:grid-cols-5 gap-4">
          <div className="text-center">
            <p className="text-sm text-muted-foreground mb-1">Position X</p>
            <p className="text-xl font-mono">
              {robotState.position.x.toFixed(2)} m
            </p>
          </div>
          <div className="text-center">
            <p className="text-sm text-muted-foreground mb-1">Position Y</p>
            <p className="text-xl font-mono">
              {robotState.position.y.toFixed(2)} m
            </p>
          </div>
          <div className="text-center">
            <p className="text-sm text-muted-foreground mb-1">Orientation</p>
            <p className="text-xl font-mono">
              {robotState.position.theta.toFixed(2)} rad
            </p>
          </div>
          <div className="text-center">
            <p className="text-sm text-muted-foreground mb-1">Linear Vel</p>
            <p className="text-xl font-mono">
              {robotState.velocity.linear.toFixed(2)} m/s
            </p>
          </div>
          <div className="text-center">
            <p className="text-sm text-muted-foreground mb-1">Angular Vel</p>
            <p className="text-xl font-mono">
              {robotState.velocity.angular.toFixed(2)} rad/s
            </p>
          </div>
        </div>

        <Separator className="my-4" />

        <div className="flex items-center justify-between">
          <div className="flex items-center gap-6">
            <div className="flex items-center gap-2">
              <Zap className="h-4 w-4 text-ros-success" />
              <span className="text-sm">Battery: {robotState.battery}%</span>
              <Progress value={robotState.battery} className="w-20" />
            </div>

            <div className="flex items-center gap-2">
              <div
                className={`ros-status-indicator ${robotState.localized ? "ros-status-active" : "ros-status-error"}`}
              />
              <span className="text-sm">Localized</span>
            </div>

            <div className="flex items-center gap-2">
              {robotState.obstacles ? (
                <>
                  <AlertTriangle className="h-4 w-4 text-ros-warning" />
                  <span className="text-sm">Obstacles Detected</span>
                </>
              ) : (
                <>
                  <div className="ros-status-indicator ros-status-active" />
                  <span className="text-sm">Path Clear</span>
                </>
              )}
            </div>
          </div>
        </div>
      </Card>

      {/* Manual Control */}
      {controlMode === "manual" && (
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Movement Controls */}
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">Manual Movement</h3>

            {/* Speed Controls */}
            <div className="mb-6 space-y-4">
              <div>
                <label className="text-sm font-medium mb-2 block">
                  Linear Speed: {linearSpeed[0]} m/s
                </label>
                <Slider
                  value={linearSpeed}
                  onValueChange={setLinearSpeed}
                  max={2.0}
                  min={0.1}
                  step={0.1}
                  className="w-full"
                />
              </div>

              <div>
                <label className="text-sm font-medium mb-2 block">
                  Angular Speed: {angularSpeed[0]} rad/s
                </label>
                <Slider
                  value={angularSpeed}
                  onValueChange={setAngularSpeed}
                  max={1.0}
                  min={0.1}
                  step={0.1}
                  className="w-full"
                />
              </div>
            </div>

            {/* Direction Controls */}
            <div className="grid grid-cols-3 gap-3">
              <div></div>
              <Button
                size="lg"
                variant="outline"
                className="aspect-square"
                disabled={isEmergencyStop}
              >
                <ArrowUp className="h-6 w-6" />
              </Button>
              <div></div>

              <Button
                size="lg"
                variant="outline"
                className="aspect-square"
                disabled={isEmergencyStop}
              >
                <ArrowLeft className="h-6 w-6" />
              </Button>
              <Button
                size="lg"
                variant="destructive"
                className="aspect-square"
                disabled={isEmergencyStop}
              >
                <Square className="h-6 w-6" />
              </Button>
              <Button
                size="lg"
                variant="outline"
                className="aspect-square"
                disabled={isEmergencyStop}
              >
                <ArrowRight className="h-6 w-6" />
              </Button>

              <div></div>
              <Button
                size="lg"
                variant="outline"
                className="aspect-square"
                disabled={isEmergencyStop}
              >
                <ArrowDown className="h-6 w-6" />
              </Button>
              <div></div>
            </div>

            {/* Rotation Controls */}
            <div className="flex justify-center gap-3 mt-4">
              <Button
                size="lg"
                variant="outline"
                className="gap-2"
                disabled={isEmergencyStop}
              >
                <RotateCcw className="h-5 w-5" />
                Rotate Left
              </Button>
              <Button
                size="lg"
                variant="outline"
                className="gap-2"
                disabled={isEmergencyStop}
              >
                <RotateCw className="h-5 w-5" />
                Rotate Right
              </Button>
            </div>
          </Card>

          {/* Preset Positions */}
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">Quick Navigation</h3>

            <div className="space-y-3 mb-6">
              {presetPositions.map((position, index) => (
                <div
                  key={index}
                  className="flex items-center justify-between p-3 rounded-lg border border-border"
                >
                  <div className="flex items-center gap-3">
                    <MapPin className="h-4 w-4 text-primary" />
                    <div>
                      <p className="font-medium">{position.name}</p>
                      <p className="text-sm text-muted-foreground">
                        ({position.x}, {position.y}, {position.theta})
                      </p>
                    </div>
                  </div>
                  <Button size="sm" disabled={isEmergencyStop}>
                    Go
                  </Button>
                </div>
              ))}
            </div>

            <Separator className="my-4" />

            <div className="space-y-3">
              <Button className="w-full gap-2" disabled={isEmergencyStop}>
                <Home className="h-4 w-4" />
                Return to Home
              </Button>
              <Button
                variant="outline"
                className="w-full gap-2"
                disabled={isEmergencyStop}
              >
                <Navigation className="h-4 w-4" />
                Start Navigation
              </Button>
            </div>
          </Card>
        </div>
      )}

      {/* Autonomous Mode */}
      {controlMode === "autonomous" && (
        <Card className="p-6">
          <h3 className="text-lg font-semibold mb-6">Autonomous Navigation</h3>
          <div className="text-center py-12">
            <Navigation className="h-16 w-16 text-primary mx-auto mb-4" />
            <h4 className="text-xl font-semibold mb-2">
              Autonomous Mode Active
            </h4>
            <p className="text-muted-foreground mb-6">
              Robot is navigating autonomously. Monitor status and intervene if
              necessary.
            </p>
            <div className="flex justify-center gap-4">
              <Button variant="outline">View Path</Button>
              <Button variant="outline">Set Goal</Button>
              <Button variant="destructive">Override</Button>
            </div>
          </div>
        </Card>
      )}
    </div>
  );
}
