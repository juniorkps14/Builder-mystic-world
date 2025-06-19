import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import {
  Activity,
  Camera,
  Radar,
  Thermometer,
  Gauge,
  Compass,
} from "lucide-react";

const Sensors = () => {
  const sensorData = [
    {
      name: "LiDAR",
      type: "sensor_msgs/LaserScan",
      status: "active",
      rate: "10 Hz",
      icon: Radar,
      value: "360° scan",
      health: 95,
    },
    {
      name: "IMU",
      type: "sensor_msgs/Imu",
      status: "active",
      rate: "100 Hz",
      icon: Compass,
      value: "0.02° precision",
      health: 98,
    },
    {
      name: "RGB Camera",
      type: "sensor_msgs/Image",
      status: "active",
      rate: "30 Hz",
      icon: Camera,
      value: "1920x1080",
      health: 87,
    },
    {
      name: "Temperature",
      type: "sensor_msgs/Temperature",
      status: "active",
      rate: "1 Hz",
      icon: Thermometer,
      value: "42°C",
      health: 92,
    },
    {
      name: "Ultrasonic",
      type: "sensor_msgs/Range",
      status: "warning",
      rate: "20 Hz",
      icon: Activity,
      value: "0.5m range",
      health: 75,
    },
    {
      name: "Pressure",
      type: "sensor_msgs/FluidPressure",
      status: "active",
      rate: "5 Hz",
      icon: Gauge,
      value: "1013 hPa",
      health: 89,
    },
  ];

  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Sensor Monitoring</h1>
        <p className="text-muted-foreground">
          Real-time sensor data and health monitoring
        </p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {sensorData.map((sensor, index) => (
          <Card key={index} className="p-6">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-3">
                <div className="p-2 rounded-lg bg-primary/10">
                  <sensor.icon className="h-5 w-5 text-primary" />
                </div>
                <div>
                  <h3 className="font-semibold">{sensor.name}</h3>
                  <p className="text-sm text-muted-foreground">{sensor.type}</p>
                </div>
              </div>
              <Badge
                variant={sensor.status === "active" ? "default" : "destructive"}
                className="gap-1"
              >
                <div
                  className={`ros-status-indicator ${sensor.status === "active" ? "ros-status-active" : "ros-status-error"}`}
                />
                {sensor.status}
              </Badge>
            </div>

            <div className="space-y-3">
              <div className="flex justify-between text-sm">
                <span className="text-muted-foreground">Rate:</span>
                <span className="font-mono">{sensor.rate}</span>
              </div>
              <div className="flex justify-between text-sm">
                <span className="text-muted-foreground">Value:</span>
                <span className="font-mono">{sensor.value}</span>
              </div>
              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-muted-foreground">Health:</span>
                  <span className="font-mono">{sensor.health}%</span>
                </div>
                <Progress value={sensor.health} className="h-2" />
              </div>
            </div>
          </Card>
        ))}
      </div>
    </div>
  );
};

export default Sensors;
