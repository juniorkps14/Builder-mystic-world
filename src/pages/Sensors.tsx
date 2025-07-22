import React, { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Eye,
  Radar,
  Compass,
  Thermometer,
  Gauge,
  Radio,
  Zap,
  Activity,
  Wifi,
  WifiOff,
  CheckCircle,
  AlertTriangle,
  Clock,
  Settings,
  BarChart3,
  TrendingUp,
  TrendingDown,
  Minus,
  RefreshCw,
} from "lucide-react";

interface SensorData {
  id: string;
  name: string;
  type:
    | "lidar"
    | "camera"
    | "imu"
    | "gps"
    | "ultrasonic"
    | "temperature"
    | "pressure";
  status: "active" | "inactive" | "error" | "calibrating";
  value: string;
  unit: string;
  quality: number;
  lastUpdate: Date;
  topic: string;
  frequency: number;
}

interface SensorReading {
  timestamp: Date;
  value: number;
  sensor: string;
}

export default function Sensors() {
  const [sensors, setSensors] = useState<SensorData[]>([
    {
      id: "lidar_1",
      name: "LIDAR Scanner",
      type: "lidar",
      status: "active",
      value: "360°",
      unit: "points",
      quality: 98,
      lastUpdate: new Date(),
      topic: "/scan",
      frequency: 10,
    },
    {
      id: "camera_front",
      name: "Front Camera",
      type: "camera",
      status: "active",
      value: "1920x1080",
      unit: "pixels",
      quality: 94,
      lastUpdate: new Date(),
      topic: "/camera/front/image_raw",
      frequency: 30,
    },
    {
      id: "imu_main",
      name: "IMU Sensor",
      type: "imu",
      status: "active",
      value: "9-axis",
      unit: "deg/s",
      quality: 99,
      lastUpdate: new Date(),
      topic: "/imu/data",
      frequency: 100,
    },
    {
      id: "gps_main",
      name: "GPS Module",
      type: "gps",
      status: "active",
      value: "RTK Fix",
      unit: "satellites",
      quality: 89,
      lastUpdate: new Date(),
      topic: "/gps/fix",
      frequency: 5,
    },
    {
      id: "ultrasonic_1",
      name: "Ultrasonic Array",
      type: "ultrasonic",
      status: "active",
      value: "8 sensors",
      unit: "cm",
      quality: 92,
      lastUpdate: new Date(),
      topic: "/ultrasonic",
      frequency: 20,
    },
    {
      id: "temp_1",
      name: "Temperature Sensor",
      type: "temperature",
      status: "calibrating",
      value: "45.2°C",
      unit: "°C",
      quality: 76,
      lastUpdate: new Date(),
      topic: "/temperature",
      frequency: 1,
    },
  ]);

  const [selectedSensor, setSelectedSensor] = useState(sensors[0]);
  const [sensorReadings, setSensorReadings] = useState<SensorReading[]>([]);

  // Simulate real-time sensor data
  useEffect(() => {
    const interval = setInterval(() => {
      setSensors((prev) =>
        prev.map((sensor) => ({
          ...sensor,
          quality: Math.max(
            70,
            Math.min(100, sensor.quality + (Math.random() - 0.5) * 5),
          ),
          lastUpdate: new Date(),
        })),
      );

      // Add new reading for chart
      setSensorReadings((prev) => {
        const newReading: SensorReading = {
          timestamp: new Date(),
          value: Math.random() * 100,
          sensor: selectedSensor.id,
        };
        return [...prev.slice(-19), newReading]; // Keep last 20 readings
      });
    }, 2000);

    return () => clearInterval(interval);
  }, [selectedSensor.id]);

  const getSensorIcon = (type: string) => {
    switch (type) {
      case "lidar":
        return Radar;
      case "camera":
        return Eye;
      case "imu":
        return Compass;
      case "gps":
        return Radio;
      case "ultrasonic":
        return Gauge;
      case "temperature":
        return Thermometer;
      default:
        return Activity;
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case "active":
        return "from-emerald-500 to-teal-500";
      case "error":
        return "from-red-500 to-pink-500";
      case "calibrating":
        return "from-yellow-500 to-orange-500";
      case "inactive":
        return "from-gray-500 to-slate-500";
      default:
        return "from-blue-500 to-cyan-500";
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "active":
        return <CheckCircle className="h-4 w-4 text-emerald-400" />;
      case "error":
        return <AlertTriangle className="h-4 w-4 text-red-400" />;
      case "calibrating":
        return <RefreshCw className="h-4 w-4 text-yellow-400 animate-spin" />;
      case "inactive":
        return <Minus className="h-4 w-4 text-gray-400" />;
      default:
        return <Activity className="h-4 w-4 text-blue-400" />;
    }
  };

  const getQualityTrend = (quality: number) => {
    if (quality > 90)
      return <TrendingUp className="h-4 w-4 text-emerald-400" />;
    if (quality < 80) return <TrendingDown className="h-4 w-4 text-red-400" />;
    return <Minus className="h-4 w-4 text-gray-400" />;
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Tesla-inspired Header */}
      <div className="mb-8">
        <div className="bg-white/10 backdrop-blur-xl border border-white/20 rounded-3xl p-6 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="space-y-2">
              <h1 className="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
                Sensor Monitoring
              </h1>
              <p className="text-slate-300 font-light">
                Real-time sensor data monitoring and diagnostics
              </p>
            </div>

            <div className="flex items-center gap-3">
              <Badge className="bg-emerald-500/20 text-emerald-300 border border-emerald-500/30 px-4 py-2">
                <Activity className="h-4 w-4 mr-2" />
                {sensors.filter((s) => s.status === "active").length} Active
              </Badge>
              <Button className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600 text-white shadow-lg transition-all duration-300">
                <Settings className="h-4 w-4 mr-2" />
                Calibrate All
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* Sensor Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 mb-8">
        {sensors.map((sensor) => {
          const IconComponent = getSensorIcon(sensor.type);
          return (
            <Card
              key={sensor.id}
              onClick={() => setSelectedSensor(sensor)}
              className={`bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300 cursor-pointer ${
                selectedSensor.id === sensor.id ? "ring-2 ring-blue-400" : ""
              }`}
            >
              <div className="flex items-center justify-between mb-4">
                <div className="flex items-center gap-3">
                  <div
                    className={`p-2 rounded-lg bg-gradient-to-br ${getStatusColor(sensor.status)}/20 border border-white/20`}
                  >
                    <IconComponent className="h-5 w-5 text-white" />
                  </div>
                  <div>
                    <h3 className="font-medium text-white text-sm">
                      {sensor.name}
                    </h3>
                    <p className="text-xs text-slate-400">{sensor.topic}</p>
                  </div>
                </div>
                {getStatusIcon(sensor.status)}
              </div>

              <div className="space-y-3">
                <div className="flex justify-between text-sm">
                  <span className="text-slate-400">Value</span>
                  <span className="text-white font-mono">{sensor.value}</span>
                </div>
                <div className="flex justify-between text-sm">
                  <span className="text-slate-400">Frequency</span>
                  <span className="text-white font-mono">
                    {sensor.frequency}Hz
                  </span>
                </div>
                <div className="flex justify-between text-sm items-center">
                  <span className="text-slate-400">Quality</span>
                  <div className="flex items-center gap-2">
                    <span className="text-white font-mono text-sm">
                      {sensor.quality}%
                    </span>
                    {getQualityTrend(sensor.quality)}
                  </div>
                </div>
                <div className="mt-3">
                  <Progress
                    value={sensor.quality}
                    className="h-2 bg-slate-700"
                  />
                </div>
              </div>

              <Badge
                className={`mt-4 bg-gradient-to-r ${getStatusColor(sensor.status)} text-white border-0`}
              >
                {sensor.status.charAt(0).toUpperCase() + sensor.status.slice(1)}
              </Badge>
            </Card>
          );
        })}
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Sensor Details */}
        <div className="lg:col-span-2">
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-2xl">
            <div className="p-6">
              <div className="flex items-center justify-between mb-6">
                <div className="flex items-center gap-3">
                  <div
                    className={`p-3 rounded-xl bg-gradient-to-br ${getStatusColor(selectedSensor.status)}/20 border border-white/20`}
                  >
                    {React.createElement(getSensorIcon(selectedSensor.type), {
                      className: "h-6 w-6 text-white",
                    })}
                  </div>
                  <div>
                    <h2 className="text-xl font-light text-white">
                      {selectedSensor.name}
                    </h2>
                    <p className="text-slate-400 text-sm">
                      {selectedSensor.topic}
                    </p>
                  </div>
                </div>
                <Badge
                  className={`bg-gradient-to-r ${getStatusColor(selectedSensor.status)} text-white border-0`}
                >
                  {selectedSensor.status}
                </Badge>
              </div>

              <Tabs defaultValue="realtime" className="w-full">
                <TabsList className="grid w-full grid-cols-3 bg-white/10 border border-white/20">
                  <TabsTrigger
                    value="realtime"
                    className="data-[state=active]:bg-white/20"
                  >
                    Real-time Data
                  </TabsTrigger>
                  <TabsTrigger
                    value="history"
                    className="data-[state=active]:bg-white/20"
                  >
                    Historical Data
                  </TabsTrigger>
                  <TabsTrigger
                    value="config"
                    className="data-[state=active]:bg-white/20"
                  >
                    Configuration
                  </TabsTrigger>
                </TabsList>

                <TabsContent value="realtime" className="mt-6">
                  {/* Real-time Chart Placeholder */}
                  <div className="aspect-video bg-gradient-to-br from-slate-800 to-slate-900 rounded-xl border border-white/10 flex items-center justify-center mb-6">
                    <div className="text-center">
                      <BarChart3 className="h-16 w-16 text-blue-400 mx-auto mb-4" />
                      <p className="text-slate-300">Real-time Sensor Data</p>
                      <p className="text-slate-400 text-sm mt-2">
                        Live data visualization for {selectedSensor.name}
                      </p>
                    </div>
                  </div>

                  {/* Current Readings */}
                  <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                    <div className="bg-white/5 rounded-lg p-4 border border-white/10">
                      <p className="text-slate-400 text-sm">Current Value</p>
                      <p className="text-white text-lg font-mono">
                        {selectedSensor.value}
                      </p>
                    </div>
                    <div className="bg-white/5 rounded-lg p-4 border border-white/10">
                      <p className="text-slate-400 text-sm">Quality</p>
                      <p className="text-white text-lg font-mono">
                        {selectedSensor.quality}%
                      </p>
                    </div>
                    <div className="bg-white/5 rounded-lg p-4 border border-white/10">
                      <p className="text-slate-400 text-sm">Frequency</p>
                      <p className="text-white text-lg font-mono">
                        {selectedSensor.frequency}Hz
                      </p>
                    </div>
                    <div className="bg-white/5 rounded-lg p-4 border border-white/10">
                      <p className="text-slate-400 text-sm">Last Update</p>
                      <p className="text-white text-sm font-mono">
                        {selectedSensor.lastUpdate.toLocaleTimeString()}
                      </p>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="history" className="mt-6">
                  <div className="aspect-video bg-gradient-to-br from-slate-800 to-slate-900 rounded-xl border border-white/10 flex items-center justify-center">
                    <div className="text-center">
                      <TrendingUp className="h-16 w-16 text-green-400 mx-auto mb-4" />
                      <p className="text-slate-300">Historical Data Analysis</p>
                      <p className="text-slate-400 text-sm mt-2">
                        Sensor performance over time
                      </p>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="config" className="mt-6">
                  <div className="space-y-4">
                    <div className="bg-white/5 rounded-lg p-4 border border-white/10">
                      <h4 className="text-white font-medium mb-3">
                        Sensor Configuration
                      </h4>
                      <div className="grid grid-cols-2 gap-4">
                        <div>
                          <p className="text-slate-400 text-sm">ROS Topic</p>
                          <p className="text-white font-mono">
                            {selectedSensor.topic}
                          </p>
                        </div>
                        <div>
                          <p className="text-slate-400 text-sm">Update Rate</p>
                          <p className="text-white font-mono">
                            {selectedSensor.frequency}Hz
                          </p>
                        </div>
                        <div>
                          <p className="text-slate-400 text-sm">Sensor Type</p>
                          <p className="text-white">
                            {selectedSensor.type.toUpperCase()}
                          </p>
                        </div>
                        <div>
                          <p className="text-slate-400 text-sm">Unit</p>
                          <p className="text-white">{selectedSensor.unit}</p>
                        </div>
                      </div>
                    </div>

                    <div className="flex gap-4">
                      <Button className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600">
                        <RefreshCw className="h-4 w-4 mr-2" />
                        Calibrate
                      </Button>
                      <Button className="bg-white/10 hover:bg-white/20 border border-white/20">
                        <Settings className="h-4 w-4 mr-2" />
                        Configure
                      </Button>
                    </div>
                  </div>
                </TabsContent>
              </Tabs>
            </div>
          </Card>
        </div>

        {/* Sensor Summary */}
        <div className="space-y-6">
          {/* System Health */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                System Health
              </h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <span className="text-slate-400 text-sm">Active Sensors</span>
                  <span className="text-emerald-400 font-mono">
                    {sensors.filter((s) => s.status === "active").length}/
                    {sensors.length}
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-slate-400 text-sm">
                    Average Quality
                  </span>
                  <span className="text-white font-mono">
                    {Math.round(
                      sensors.reduce((acc, s) => acc + s.quality, 0) /
                        sensors.length,
                    )}
                    %
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-slate-400 text-sm">Errors</span>
                  <span className="text-red-400 font-mono">
                    {sensors.filter((s) => s.status === "error").length}
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-slate-400 text-sm">Calibrating</span>
                  <span className="text-yellow-400 font-mono">
                    {sensors.filter((s) => s.status === "calibrating").length}
                  </span>
                </div>
              </div>
            </div>
          </Card>

          {/* Quick Actions */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                Quick Actions
              </h3>
              <div className="space-y-3">
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <RefreshCw className="h-4 w-4 mr-2" />
                  Refresh All Sensors
                </Button>
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <Settings className="h-4 w-4 mr-2" />
                  Sensor Settings
                </Button>
                <Button className="w-full bg-white/10 hover:bg-white/20 border border-white/20">
                  <BarChart3 className="h-4 w-4 mr-2" />
                  Export Data
                </Button>
              </div>
            </div>
          </Card>

          {/* Recent Alerts */}
          <Card className="bg-white/10 backdrop-blur-xl border border-white/20 shadow-xl">
            <div className="p-6">
              <h3 className="text-lg font-light text-white mb-4">
                Recent Alerts
              </h3>
              <div className="space-y-3">
                <div className="bg-yellow-500/20 border border-yellow-500/30 rounded-lg p-3">
                  <div className="flex items-center gap-2 mb-1">
                    <Clock className="h-4 w-4 text-yellow-400" />
                    <span className="text-yellow-300 text-sm font-medium">
                      Calibration Required
                    </span>
                  </div>
                  <p className="text-yellow-200 text-xs">
                    Temperature sensor needs calibration
                  </p>
                </div>

                <div className="bg-blue-500/20 border border-blue-500/30 rounded-lg p-3">
                  <div className="flex items-center gap-2 mb-1">
                    <CheckCircle className="h-4 w-4 text-blue-400" />
                    <span className="text-blue-300 text-sm font-medium">
                      System Update
                    </span>
                  </div>
                  <p className="text-blue-200 text-xs">
                    All sensors updated successfully
                  </p>
                </div>
              </div>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
}
