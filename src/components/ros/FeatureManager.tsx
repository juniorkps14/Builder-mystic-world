import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Switch } from "@/components/ui/switch";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { ScrollArea } from "@/components/ui/scroll-area";
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
  Settings,
  Zap,
  Eye,
  Brain,
  Shield,
  Mic,
  Wifi,
  Cloud,
  BarChart3,
  Wrench,
  Battery,
  MapPin,
  Timer,
  AlertTriangle,
  CheckCircle,
  Download,
  Upload,
  RefreshCw,
  Cpu,
  Globe,
  Smartphone,
  Gamepad2,
  Camera,
  Activity,
} from "lucide-react";

interface Feature {
  id: string;
  name: string;
  description: string;
  category: string;
  icon: any;
  enabled: boolean;
  requiresLicense: boolean;
  dependencies: string[];
  version: string;
  status: "stable" | "beta" | "experimental" | "deprecated";
  resourceUsage: {
    cpu: number;
    memory: number;
    storage: number;
  };
  platforms: string[];
  lastUpdated: Date;
}

export function FeatureManager() {
  const [features, setFeatures] = useState<Feature[]>([
    {
      id: "vision_processing",
      name: "Advanced Vision Processing",
      description:
        "Real-time computer vision with object detection, tracking, and recognition",
      category: "AI & Vision",
      icon: Eye,
      enabled: true,
      requiresLicense: false,
      dependencies: ["opencv", "tensorflow"],
      version: "2.1.0",
      status: "stable",
      resourceUsage: { cpu: 25, memory: 512, storage: 200 },
      platforms: ["linux", "windows", "macos"],
      lastUpdated: new Date(),
    },
    {
      id: "ai_navigation",
      name: "AI-Powered Navigation",
      description:
        "Intelligent path planning with machine learning optimization",
      category: "AI & Vision",
      icon: Brain,
      enabled: false,
      requiresLicense: true,
      dependencies: ["tensorflow", "pytorch"],
      version: "1.5.2",
      status: "beta",
      resourceUsage: { cpu: 40, memory: 1024, storage: 500 },
      platforms: ["linux", "windows"],
      lastUpdated: new Date(Date.now() - 86400000),
    },
    {
      id: "safety_monitoring",
      name: "Advanced Safety Systems",
      description:
        "Comprehensive safety monitoring with predictive collision avoidance",
      category: "Safety & Security",
      icon: Shield,
      enabled: true,
      requiresLicense: false,
      dependencies: ["safety_controller"],
      version: "3.0.1",
      status: "stable",
      resourceUsage: { cpu: 15, memory: 256, storage: 100 },
      platforms: ["linux", "windows", "macos", "embedded"],
      lastUpdated: new Date(Date.now() - 3600000),
    },
    {
      id: "voice_control",
      name: "Voice Command Interface",
      description: "Natural language processing for voice-controlled robotics",
      category: "Human Interface",
      icon: Mic,
      enabled: false,
      requiresLicense: false,
      dependencies: ["speech_recognition", "nlp"],
      version: "1.2.3",
      status: "experimental",
      resourceUsage: { cpu: 20, memory: 384, storage: 150 },
      platforms: ["linux", "windows", "macos"],
      lastUpdated: new Date(Date.now() - 172800000),
    },
    {
      id: "cloud_integration",
      name: "Cloud Integration",
      description:
        "Seamless cloud connectivity for data sync and remote control",
      category: "Connectivity",
      icon: Cloud,
      enabled: true,
      requiresLicense: true,
      dependencies: ["aws_sdk", "azure_sdk"],
      version: "2.0.0",
      status: "stable",
      resourceUsage: { cpu: 10, memory: 128, storage: 50 },
      platforms: ["linux", "windows", "macos"],
      lastUpdated: new Date(Date.now() - 7200000),
    },
    {
      id: "fleet_management",
      name: "Multi-Robot Fleet Management",
      description: "Coordinate and manage multiple robots simultaneously",
      category: "Management",
      icon: Activity,
      enabled: false,
      requiresLicense: true,
      dependencies: ["fleet_coordinator", "swarm_intelligence"],
      version: "1.8.0",
      status: "stable",
      resourceUsage: { cpu: 30, memory: 768, storage: 300 },
      platforms: ["linux", "windows"],
      lastUpdated: new Date(Date.now() - 259200000),
    },
    {
      id: "mobile_app",
      name: "Mobile App Integration",
      description:
        "Control robots from mobile devices with intuitive interface",
      category: "Human Interface",
      icon: Smartphone,
      enabled: true,
      requiresLicense: false,
      dependencies: ["react_native", "websocket"],
      version: "1.4.1",
      status: "stable",
      resourceUsage: { cpu: 5, memory: 64, storage: 25 },
      platforms: ["android", "ios"],
      lastUpdated: new Date(Date.now() - 43200000),
    },
    {
      id: "ar_visualization",
      name: "Augmented Reality Visualization",
      description: "AR overlay for robot status and environment visualization",
      category: "Visualization",
      icon: Camera,
      enabled: false,
      requiresLicense: true,
      dependencies: ["arcore", "arkit"],
      version: "0.9.5",
      status: "experimental",
      resourceUsage: { cpu: 35, memory: 512, storage: 200 },
      platforms: ["android", "ios"],
      lastUpdated: new Date(Date.now() - 518400000),
    },
    {
      id: "predictive_maintenance",
      name: "Predictive Maintenance",
      description: "AI-powered maintenance scheduling and fault prediction",
      category: "Maintenance",
      icon: Wrench,
      enabled: true,
      requiresLicense: true,
      dependencies: ["machine_learning", "analytics"],
      version: "2.2.0",
      status: "stable",
      resourceUsage: { cpu: 20, memory: 384, storage: 100 },
      platforms: ["linux", "windows", "macos"],
      lastUpdated: new Date(Date.now() - 86400000),
    },
    {
      id: "energy_optimization",
      name: "Energy Optimization",
      description: "Intelligent power management and battery optimization",
      category: "Power Management",
      icon: Battery,
      enabled: true,
      requiresLicense: false,
      dependencies: ["power_monitor"],
      version: "1.6.3",
      status: "stable",
      resourceUsage: { cpu: 8, memory: 128, storage: 30 },
      platforms: ["linux", "embedded"],
      lastUpdated: new Date(Date.now() - 172800000),
    },
  ]);

  const [selectedCategory, setSelectedCategory] = useState("all");
  const [showLicenseDialog, setShowLicenseDialog] = useState(false);
  const [pendingFeature, setPendingFeature] = useState<string | null>(null);

  const categories = [
    { id: "all", name: "All Features", icon: Settings },
    { id: "AI & Vision", name: "AI & Vision", icon: Eye },
    { id: "Safety & Security", name: "Safety & Security", icon: Shield },
    { id: "Human Interface", name: "Human Interface", icon: Gamepad2 },
    { id: "Connectivity", name: "Connectivity", icon: Wifi },
    { id: "Management", name: "Management", icon: BarChart3 },
    { id: "Visualization", name: "Visualization", icon: Camera },
    { id: "Maintenance", name: "Maintenance", icon: Wrench },
    { id: "Power Management", name: "Power Management", icon: Battery },
  ];

  const toggleFeature = (featureId: string) => {
    const feature = features.find((f) => f.id === featureId);
    if (!feature) return;

    if (!feature.enabled && feature.requiresLicense) {
      setPendingFeature(featureId);
      setShowLicenseDialog(true);
      return;
    }

    setFeatures((prev) =>
      prev.map((f) => (f.id === featureId ? { ...f, enabled: !f.enabled } : f)),
    );
  };

  const confirmLicenseAndEnable = () => {
    if (pendingFeature) {
      setFeatures((prev) =>
        prev.map((f) =>
          f.id === pendingFeature ? { ...f, enabled: true } : f,
        ),
      );
      setPendingFeature(null);
    }
    setShowLicenseDialog(false);
  };

  const filteredFeatures = features.filter(
    (feature) =>
      selectedCategory === "all" || feature.category === selectedCategory,
  );

  const getStatusColor = (status: string) => {
    switch (status) {
      case "stable":
        return "default";
      case "beta":
        return "secondary";
      case "experimental":
        return "destructive";
      case "deprecated":
        return "outline";
      default:
        return "outline";
    }
  };

  const getTotalResourceUsage = () => {
    const enabledFeatures = features.filter((f) => f.enabled);
    return enabledFeatures.reduce(
      (total, feature) => ({
        cpu: total.cpu + feature.resourceUsage.cpu,
        memory: total.memory + feature.resourceUsage.memory,
        storage: total.storage + feature.resourceUsage.storage,
      }),
      { cpu: 0, memory: 0, storage: 0 },
    );
  };

  const resourceUsage = getTotalResourceUsage();

  return (
    <div className="space-y-6">
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Feature Management</h1>
        <p className="text-muted-foreground">
          Enable, configure, and manage robot platform features
        </p>
      </div>

      {/* System Resource Overview */}
      <Card className="p-6">
        <h3 className="text-lg font-semibold mb-4">System Resource Usage</h3>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          <div className="space-y-2">
            <div className="flex justify-between text-sm">
              <span>CPU Usage</span>
              <span>{resourceUsage.cpu}%</span>
            </div>
            <div className="w-full bg-muted rounded-full h-2">
              <div
                className="bg-primary h-2 rounded-full transition-all"
                style={{ width: `${Math.min(resourceUsage.cpu, 100)}%` }}
              />
            </div>
          </div>

          <div className="space-y-2">
            <div className="flex justify-between text-sm">
              <span>Memory Usage</span>
              <span>{resourceUsage.memory} MB</span>
            </div>
            <div className="w-full bg-muted rounded-full h-2">
              <div
                className="bg-accent h-2 rounded-full transition-all"
                style={{
                  width: `${Math.min((resourceUsage.memory / 4096) * 100, 100)}%`,
                }}
              />
            </div>
          </div>

          <div className="space-y-2">
            <div className="flex justify-between text-sm">
              <span>Storage Usage</span>
              <span>{resourceUsage.storage} MB</span>
            </div>
            <div className="w-full bg-muted rounded-full h-2">
              <div
                className="bg-ros-success h-2 rounded-full transition-all"
                style={{
                  width: `${Math.min((resourceUsage.storage / 2048) * 100, 100)}%`,
                }}
              />
            </div>
          </div>
        </div>
      </Card>

      <Tabs defaultValue="features" className="space-y-4">
        <TabsList className="grid w-full grid-cols-3">
          <TabsTrigger value="features">Feature Library</TabsTrigger>
          <TabsTrigger value="categories">Categories</TabsTrigger>
          <TabsTrigger value="settings">Global Settings</TabsTrigger>
        </TabsList>

        {/* Feature Library */}
        <TabsContent value="features" className="space-y-4">
          {/* Category Filter */}
          <div className="flex flex-wrap gap-2">
            {categories.map((category) => (
              <Button
                key={category.id}
                variant={
                  selectedCategory === category.id ? "default" : "outline"
                }
                size="sm"
                onClick={() => setSelectedCategory(category.id)}
                className="gap-2"
              >
                <category.icon className="h-4 w-4" />
                {category.name}
              </Button>
            ))}
          </div>

          {/* Feature Cards */}
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
            {filteredFeatures.map((feature) => (
              <Card key={feature.id} className="p-4">
                <div className="flex items-start justify-between mb-3">
                  <div className="flex items-center gap-3">
                    <div className="p-2 rounded-lg bg-primary/10">
                      <feature.icon className="h-5 w-5 text-primary" />
                    </div>
                    <div>
                      <h4 className="font-semibold flex items-center gap-2">
                        {feature.name}
                        {feature.requiresLicense && (
                          <Badge variant="outline" className="text-xs">
                            License
                          </Badge>
                        )}
                      </h4>
                      <p className="text-sm text-muted-foreground">
                        v{feature.version}
                      </p>
                    </div>
                  </div>

                  <div className="flex items-center gap-2">
                    <Badge variant={getStatusColor(feature.status)}>
                      {feature.status}
                    </Badge>
                    <Switch
                      checked={feature.enabled}
                      onCheckedChange={() => toggleFeature(feature.id)}
                    />
                  </div>
                </div>

                <p className="text-sm mb-4">{feature.description}</p>

                <div className="space-y-2">
                  <div className="flex justify-between text-xs">
                    <span className="text-muted-foreground">
                      Resource Usage:
                    </span>
                    <span>
                      CPU: {feature.resourceUsage.cpu}% | RAM:{" "}
                      {feature.resourceUsage.memory}MB
                    </span>
                  </div>

                  <div className="flex justify-between text-xs">
                    <span className="text-muted-foreground">Platforms:</span>
                    <div className="flex gap-1">
                      {feature.platforms.map((platform) => (
                        <Badge
                          key={platform}
                          variant="outline"
                          className="text-xs"
                        >
                          {platform}
                        </Badge>
                      ))}
                    </div>
                  </div>

                  {feature.dependencies.length > 0 && (
                    <div className="flex justify-between text-xs">
                      <span className="text-muted-foreground">
                        Dependencies:
                      </span>
                      <span className="text-right">
                        {feature.dependencies.slice(0, 2).join(", ")}
                        {feature.dependencies.length > 2 &&
                          ` +${feature.dependencies.length - 2}`}
                      </span>
                    </div>
                  )}
                </div>

                {feature.enabled && (
                  <div className="mt-3 pt-3 border-t">
                    <div className="flex justify-between items-center">
                      <div className="flex items-center gap-2">
                        <CheckCircle className="h-4 w-4 text-ros-success" />
                        <span className="text-sm font-medium">Active</span>
                      </div>
                      <Button size="sm" variant="outline">
                        Configure
                      </Button>
                    </div>
                  </div>
                )}
              </Card>
            ))}
          </div>
        </TabsContent>

        {/* Categories */}
        <TabsContent value="categories" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
            {categories.slice(1).map((category) => {
              const categoryFeatures = features.filter(
                (f) => f.category === category.id,
              );
              const enabledCount = categoryFeatures.filter(
                (f) => f.enabled,
              ).length;

              return (
                <Card key={category.id} className="p-4">
                  <div className="flex items-center gap-3 mb-3">
                    <div className="p-2 rounded-lg bg-primary/10">
                      <category.icon className="h-5 w-5 text-primary" />
                    </div>
                    <div>
                      <h4 className="font-semibold">{category.name}</h4>
                      <p className="text-sm text-muted-foreground">
                        {enabledCount}/{categoryFeatures.length} enabled
                      </p>
                    </div>
                  </div>

                  <div className="space-y-2">
                    {categoryFeatures.map((feature) => (
                      <div
                        key={feature.id}
                        className="flex items-center justify-between text-sm"
                      >
                        <span
                          className={
                            feature.enabled
                              ? "text-foreground"
                              : "text-muted-foreground"
                          }
                        >
                          {feature.name}
                        </span>
                        <Switch
                          checked={feature.enabled}
                          onCheckedChange={() => toggleFeature(feature.id)}
                          size="sm"
                        />
                      </div>
                    ))}
                  </div>
                </Card>
              );
            })}
          </div>
        </TabsContent>

        {/* Global Settings */}
        <TabsContent value="settings" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Auto-Updates</h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Automatic Feature Updates</Label>
                    <p className="text-sm text-muted-foreground">
                      Automatically update stable features
                    </p>
                  </div>
                  <Switch defaultChecked />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Beta Feature Updates</Label>
                    <p className="text-sm text-muted-foreground">
                      Include beta features in updates
                    </p>
                  </div>
                  <Switch />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Experimental Features</Label>
                    <p className="text-sm text-muted-foreground">
                      Allow experimental feature installation
                    </p>
                  </div>
                  <Switch />
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Resource Limits</h3>
              <div className="space-y-4">
                <div>
                  <Label className="text-sm">Maximum CPU Usage: 80%</Label>
                  <Slider defaultValue={[80]} max={100} step={5} />
                </div>

                <div>
                  <Label className="text-sm">
                    Maximum Memory Usage: 2048 MB
                  </Label>
                  <Slider defaultValue={[2048]} max={4096} step={128} />
                </div>

                <div className="flex items-center justify-between">
                  <Label>Enable Resource Monitoring</Label>
                  <Switch defaultChecked />
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">Backup & Restore</h3>
              <div className="space-y-3">
                <Button className="w-full gap-2">
                  <Download className="h-4 w-4" />
                  Export Feature Configuration
                </Button>
                <Button variant="outline" className="w-full gap-2">
                  <Upload className="h-4 w-4" />
                  Import Feature Configuration
                </Button>
                <Button variant="outline" className="w-full gap-2">
                  <RefreshCw className="h-4 w-4" />
                  Reset to Defaults
                </Button>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-4">License Management</h3>
              <div className="space-y-3">
                <Button className="w-full">Manage Licenses</Button>
                <Button variant="outline" className="w-full">
                  Check for Updates
                </Button>
                <div className="text-sm text-muted-foreground">
                  {
                    features.filter((f) => f.requiresLicense && f.enabled)
                      .length
                  }{" "}
                  licensed features active
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>
      </Tabs>

      {/* License Dialog */}
      <Dialog open={showLicenseDialog} onOpenChange={setShowLicenseDialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Feature License Required</DialogTitle>
            <DialogDescription>
              This feature requires a valid license to enable. Please ensure you
              have the appropriate license before proceeding.
            </DialogDescription>
          </DialogHeader>

          <Alert>
            <AlertTriangle className="h-4 w-4" />
            <AlertDescription>
              Enabling licensed features may incur additional costs. Please
              review your license agreement.
            </AlertDescription>
          </Alert>

          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => setShowLicenseDialog(false)}
            >
              Cancel
            </Button>
            <Button onClick={confirmLicenseAndEnable}>
              I Have a License - Enable Feature
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}

function Label({ children }: { children: React.ReactNode }) {
  return <div className="text-sm font-medium">{children}</div>;
}

function Slider({
  defaultValue,
  max,
  step,
}: {
  defaultValue: number[];
  max: number;
  step: number;
}) {
  return (
    <div className="w-full h-2 bg-muted rounded-full mt-2">
      <div
        className="h-2 bg-primary rounded-full"
        style={{ width: `${(defaultValue[0] / max) * 100}%` }}
      />
    </div>
  );
}
