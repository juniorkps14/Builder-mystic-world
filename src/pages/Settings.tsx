import React, { useState } from "react";
import { usePersistentStore } from "@/hooks/use-persistence";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Slider } from "@/components/ui/slider";
import { Badge } from "@/components/ui/badge";
import {
  Save,
  Monitor,
  Globe,
  User,
  Bell,
  Palette,
  Languages,
  Accessibility,
  Shield,
  Zap,
  Database,
  RefreshCw,
  Download,
  Upload,
  AlertTriangle,
  CheckCircle,
} from "lucide-react";

export default function Settings() {
  const [isApplying, setIsApplying] = useState(false);
  const [isDirty, setIsDirty] = useState(false);

  // Settings State
  const [settings, setSettings] = useState({
    // Interface & Theme
    theme: "system",
    language: "en",
    fontSize: 14,
    accentColor: "blue",
    reducedMotion: false,
    highContrast: false,

    // Display
    showGrid: true,
    showTooltips: true,
    showBadges: true,
    compactMode: false,
    sidebarCollapsed: false,

    // Notifications
    enableNotifications: true,
    soundEnabled: true,
    systemAlerts: true,
    taskNotifications: true,
    errorAlerts: true,

    // Robot Control
    confirmDangerousActions: true,
    autoSaveSequences: true,
    defaultMovementSpeed: 50,
    emergencyStopEnabled: true,
    joystickSensitivity: 75,

    // Data & Privacy
    collectAnalytics: false,
    shareErrorReports: true,
    autoBackup: true,
    backupFrequency: "daily",
    dataRetention: 30,

    // Performance
    maxConcurrentTasks: 5,
    cacheSize: 100,
    enableGpuAcceleration: true,
    autoOptimize: true,
    backgroundProcessing: true,

    // Advanced
    developerMode: false,
    debugMode: false,
    experimentalFeatures: false,
    betaUpdates: false,
    verboseLogging: false,
  });

  const updateSetting = (key: string, value: any) => {
    setSettings((prev) => ({ ...prev, [key]: value }));
    setIsDirty(true);
  };

  const handleApply = async () => {
    setIsApplying(true);
    console.log("Applying settings...", settings);

    // Simulate applying settings
    await new Promise((resolve) => setTimeout(resolve, 1500));

    // Save to localStorage
    localStorage.setItem("userSettings", JSON.stringify(settings));

    setIsApplying(false);
    setIsDirty(false);
  };

  const handleReset = () => {
    setSettings({
      theme: "system",
      language: "en",
      fontSize: 14,
      accentColor: "blue",
      reducedMotion: false,
      highContrast: false,
      showGrid: true,
      showTooltips: true,
      showBadges: true,
      compactMode: false,
      sidebarCollapsed: false,
      enableNotifications: true,
      soundEnabled: true,
      systemAlerts: true,
      taskNotifications: true,
      errorAlerts: true,
      confirmDangerousActions: true,
      autoSaveSequences: true,
      defaultMovementSpeed: 50,
      emergencyStopEnabled: true,
      joystickSensitivity: 75,
      collectAnalytics: false,
      shareErrorReports: true,
      autoBackup: true,
      backupFrequency: "daily",
      dataRetention: 30,
      maxConcurrentTasks: 5,
      cacheSize: 100,
      enableGpuAcceleration: true,
      autoOptimize: true,
      backgroundProcessing: true,
      developerMode: false,
      debugMode: false,
      experimentalFeatures: false,
      betaUpdates: false,
      verboseLogging: false,
    });
    setIsDirty(true);
  };

  const handleExport = () => {
    const exportData = {
      timestamp: new Date().toISOString(),
      settings,
      version: "2.0.0",
    };

    const blob = new Blob([JSON.stringify(exportData, null, 2)], {
      type: "application/json",
    });

    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `settings-${new Date().toISOString().split("T")[0]}.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const handleImport = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const importData = JSON.parse(e.target?.result as string);
        setSettings(importData.settings);
        setIsDirty(true);
        console.log("Settings imported successfully");
      } catch (error) {
        console.error("Failed to import settings:", error);
      }
    };
    reader.readAsText(file);
  };

  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            Settings
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Customize your experience and configure application preferences
          </p>
        </div>
        <div className="flex items-center gap-3">
          <input
            type="file"
            accept=".json"
            onChange={handleImport}
            className="hidden"
            id="import-settings"
          />
          <Button
            variant="outline"
            onClick={() => document.getElementById("import-settings")?.click()}
            className="gap-2"
          >
            <Upload className="h-4 w-4" />
            Import
          </Button>
          <Button variant="outline" onClick={handleExport} className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
          <Button variant="outline" onClick={handleReset} className="gap-2">
            <RefreshCw className="h-4 w-4" />
            Reset
          </Button>
          <Button
            onClick={handleApply}
            disabled={isApplying || !isDirty}
            className="gap-2"
          >
            {isApplying ? (
              <RefreshCw className="h-4 w-4 animate-spin" />
            ) : (
              <Save className="h-4 w-4" />
            )}
            Apply Settings
            {isDirty && <Badge className="ml-2 bg-yellow-500">Unsaved</Badge>}
          </Button>
        </div>
      </div>

      {/* Settings Tabs */}
      <Tabs defaultValue="interface" className="space-y-6">
        <TabsList className="grid w-full grid-cols-6">
          <TabsTrigger value="interface" className="gap-2">
            <Palette className="h-4 w-4" />
            Interface
          </TabsTrigger>
          <TabsTrigger value="notifications" className="gap-2">
            <Bell className="h-4 w-4" />
            Notifications
          </TabsTrigger>
          <TabsTrigger value="robot" className="gap-2">
            <Zap className="h-4 w-4" />
            Robot
          </TabsTrigger>
          <TabsTrigger value="privacy" className="gap-2">
            <Shield className="h-4 w-4" />
            Privacy
          </TabsTrigger>
          <TabsTrigger value="performance" className="gap-2">
            <Monitor className="h-4 w-4" />
            Performance
          </TabsTrigger>
          <TabsTrigger value="advanced" className="gap-2">
            <User className="h-4 w-4" />
            Advanced
          </TabsTrigger>
        </TabsList>

        {/* Interface Settings */}
        <TabsContent value="interface">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-6">Theme & Appearance</h3>
              <div className="space-y-4">
                <div>
                  <Label>Theme</Label>
                  <Select
                    value={settings.theme}
                    onValueChange={(value) => updateSetting("theme", value)}
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="light">Light</SelectItem>
                      <SelectItem value="dark">Dark</SelectItem>
                      <SelectItem value="system">System</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Language</Label>
                  <Select
                    value={settings.language}
                    onValueChange={(value) => updateSetting("language", value)}
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="en">English</SelectItem>
                      <SelectItem value="th">ไทย (Thai)</SelectItem>
                      <SelectItem value="ja">日本語 (Japanese)</SelectItem>
                      <SelectItem value="ko">한국어 (Korean)</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Font Size</Label>
                  <Slider
                    value={[settings.fontSize]}
                    onValueChange={([value]) =>
                      updateSetting("fontSize", value)
                    }
                    min={12}
                    max={20}
                    step={1}
                    className="mt-2"
                  />
                  <p className="text-sm text-gray-500 mt-1">
                    {settings.fontSize}px
                  </p>
                </div>

                <div>
                  <Label>Accent Color</Label>
                  <Select
                    value={settings.accentColor}
                    onValueChange={(value) =>
                      updateSetting("accentColor", value)
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="blue">Blue</SelectItem>
                      <SelectItem value="green">Green</SelectItem>
                      <SelectItem value="purple">Purple</SelectItem>
                      <SelectItem value="orange">Orange</SelectItem>
                      <SelectItem value="red">Red</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              </div>
            </Card>

            <Card className="p-6">
              <h3 className="text-lg font-semibold mb-6">Display Options</h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Show Grid Lines</Label>
                    <p className="text-sm text-gray-500">
                      Display grid lines in interfaces
                    </p>
                  </div>
                  <Switch
                    checked={settings.showGrid}
                    onCheckedChange={(checked) =>
                      updateSetting("showGrid", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Show Tooltips</Label>
                    <p className="text-sm text-gray-500">
                      Display helpful tooltips on hover
                    </p>
                  </div>
                  <Switch
                    checked={settings.showTooltips}
                    onCheckedChange={(checked) =>
                      updateSetting("showTooltips", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Show Badges</Label>
                    <p className="text-sm text-gray-500">
                      Display status badges and counters
                    </p>
                  </div>
                  <Switch
                    checked={settings.showBadges}
                    onCheckedChange={(checked) =>
                      updateSetting("showBadges", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Compact Mode</Label>
                    <p className="text-sm text-gray-500">
                      Use smaller interface elements
                    </p>
                  </div>
                  <Switch
                    checked={settings.compactMode}
                    onCheckedChange={(checked) =>
                      updateSetting("compactMode", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>High Contrast</Label>
                    <p className="text-sm text-gray-500">
                      Improve visibility with high contrast
                    </p>
                  </div>
                  <Switch
                    checked={settings.highContrast}
                    onCheckedChange={(checked) =>
                      updateSetting("highContrast", checked)
                    }
                  />
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        {/* Notifications Settings */}
        <TabsContent value="notifications">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">
              Notification Preferences
            </h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Enable Notifications</Label>
                    <p className="text-sm text-gray-500">
                      Allow system notifications
                    </p>
                  </div>
                  <Switch
                    checked={settings.enableNotifications}
                    onCheckedChange={(checked) =>
                      updateSetting("enableNotifications", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Sound Enabled</Label>
                    <p className="text-sm text-gray-500">
                      Play notification sounds
                    </p>
                  </div>
                  <Switch
                    checked={settings.soundEnabled}
                    onCheckedChange={(checked) =>
                      updateSetting("soundEnabled", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>System Alerts</Label>
                    <p className="text-sm text-gray-500">
                      Show system status alerts
                    </p>
                  </div>
                  <Switch
                    checked={settings.systemAlerts}
                    onCheckedChange={(checked) =>
                      updateSetting("systemAlerts", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Task Notifications</Label>
                    <p className="text-sm text-gray-500">
                      Notify when tasks complete
                    </p>
                  </div>
                  <Switch
                    checked={settings.taskNotifications}
                    onCheckedChange={(checked) =>
                      updateSetting("taskNotifications", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Error Alerts</Label>
                    <p className="text-sm text-gray-500">
                      Show error notifications
                    </p>
                  </div>
                  <Switch
                    checked={settings.errorAlerts}
                    onCheckedChange={(checked) =>
                      updateSetting("errorAlerts", checked)
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Robot Control Settings */}
        <TabsContent value="robot">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">
              Robot Control Preferences
            </h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Confirm Dangerous Actions</Label>
                    <p className="text-sm text-gray-500">
                      Require confirmation for potentially harmful actions
                    </p>
                  </div>
                  <Switch
                    checked={settings.confirmDangerousActions}
                    onCheckedChange={(checked) =>
                      updateSetting("confirmDangerousActions", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Auto-save Sequences</Label>
                    <p className="text-sm text-gray-500">
                      Automatically save sequence changes
                    </p>
                  </div>
                  <Switch
                    checked={settings.autoSaveSequences}
                    onCheckedChange={(checked) =>
                      updateSetting("autoSaveSequences", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Emergency Stop</Label>
                    <p className="text-sm text-gray-500">
                      Enable emergency stop functionality
                    </p>
                  </div>
                  <Switch
                    checked={settings.emergencyStopEnabled}
                    onCheckedChange={(checked) =>
                      updateSetting("emergencyStopEnabled", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>Default Movement Speed (%)</Label>
                  <Slider
                    value={[settings.defaultMovementSpeed]}
                    onValueChange={([value]) =>
                      updateSetting("defaultMovementSpeed", value)
                    }
                    min={1}
                    max={100}
                    step={1}
                    className="mt-2"
                  />
                  <p className="text-sm text-gray-500 mt-1">
                    {settings.defaultMovementSpeed}%
                  </p>
                </div>

                <div>
                  <Label>Joystick Sensitivity (%)</Label>
                  <Slider
                    value={[settings.joystickSensitivity]}
                    onValueChange={([value]) =>
                      updateSetting("joystickSensitivity", value)
                    }
                    min={10}
                    max={100}
                    step={5}
                    className="mt-2"
                  />
                  <p className="text-sm text-gray-500 mt-1">
                    {settings.joystickSensitivity}%
                  </p>
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Privacy & Data Settings */}
        <TabsContent value="privacy">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">
              Privacy & Data Settings
            </h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Collect Analytics</Label>
                    <p className="text-sm text-gray-500">
                      Help improve the system by sharing usage data
                    </p>
                  </div>
                  <Switch
                    checked={settings.collectAnalytics}
                    onCheckedChange={(checked) =>
                      updateSetting("collectAnalytics", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Share Error Reports</Label>
                    <p className="text-sm text-gray-500">
                      Automatically send crash reports
                    </p>
                  </div>
                  <Switch
                    checked={settings.shareErrorReports}
                    onCheckedChange={(checked) =>
                      updateSetting("shareErrorReports", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Auto Backup</Label>
                    <p className="text-sm text-gray-500">
                      Automatically backup settings and data
                    </p>
                  </div>
                  <Switch
                    checked={settings.autoBackup}
                    onCheckedChange={(checked) =>
                      updateSetting("autoBackup", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div>
                  <Label>Backup Frequency</Label>
                  <Select
                    value={settings.backupFrequency}
                    onValueChange={(value) =>
                      updateSetting("backupFrequency", value)
                    }
                  >
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="hourly">Hourly</SelectItem>
                      <SelectItem value="daily">Daily</SelectItem>
                      <SelectItem value="weekly">Weekly</SelectItem>
                      <SelectItem value="monthly">Monthly</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div>
                  <Label>Data Retention (days)</Label>
                  <Input
                    type="number"
                    value={settings.dataRetention}
                    onChange={(e) =>
                      updateSetting("dataRetention", parseInt(e.target.value))
                    }
                    min={1}
                    max={365}
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Performance Settings */}
        <TabsContent value="performance">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6">Performance Settings</h3>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div>
                  <Label>Max Concurrent Tasks</Label>
                  <Input
                    type="number"
                    value={settings.maxConcurrentTasks}
                    onChange={(e) =>
                      updateSetting(
                        "maxConcurrentTasks",
                        parseInt(e.target.value),
                      )
                    }
                    min={1}
                    max={20}
                  />
                </div>

                <div>
                  <Label>Cache Size (MB)</Label>
                  <Input
                    type="number"
                    value={settings.cacheSize}
                    onChange={(e) =>
                      updateSetting("cacheSize", parseInt(e.target.value))
                    }
                    min={50}
                    max={1000}
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>GPU Acceleration</Label>
                    <p className="text-sm text-gray-500">
                      Use GPU for rendering and calculations
                    </p>
                  </div>
                  <Switch
                    checked={settings.enableGpuAcceleration}
                    onCheckedChange={(checked) =>
                      updateSetting("enableGpuAcceleration", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Auto Optimize</Label>
                    <p className="text-sm text-gray-500">
                      Automatically optimize performance
                    </p>
                  </div>
                  <Switch
                    checked={settings.autoOptimize}
                    onCheckedChange={(checked) =>
                      updateSetting("autoOptimize", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Background Processing</Label>
                    <p className="text-sm text-gray-500">
                      Continue tasks in background
                    </p>
                  </div>
                  <Switch
                    checked={settings.backgroundProcessing}
                    onCheckedChange={(checked) =>
                      updateSetting("backgroundProcessing", checked)
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        {/* Advanced Settings */}
        <TabsContent value="advanced">
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
              <AlertTriangle className="h-5 w-5 text-orange-500" />
              Advanced Settings
            </h3>
            <div className="bg-orange-50 border border-orange-200 rounded-lg p-4 mb-6">
              <p className="text-sm text-orange-700">
                ⚠️ These settings are for advanced users only. Changing these
                settings may affect system stability.
              </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Developer Mode</Label>
                    <p className="text-sm text-gray-500">
                      Enable development tools and features
                    </p>
                  </div>
                  <Switch
                    checked={settings.developerMode}
                    onCheckedChange={(checked) =>
                      updateSetting("developerMode", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Debug Mode</Label>
                    <p className="text-sm text-gray-500">
                      Show debug information
                    </p>
                  </div>
                  <Switch
                    checked={settings.debugMode}
                    onCheckedChange={(checked) =>
                      updateSetting("debugMode", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Experimental Features</Label>
                    <p className="text-sm text-gray-500">
                      Enable experimental functionality
                    </p>
                  </div>
                  <Switch
                    checked={settings.experimentalFeatures}
                    onCheckedChange={(checked) =>
                      updateSetting("experimentalFeatures", checked)
                    }
                  />
                </div>
              </div>

              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div>
                    <Label>Beta Updates</Label>
                    <p className="text-sm text-gray-500">
                      Receive beta version updates
                    </p>
                  </div>
                  <Switch
                    checked={settings.betaUpdates}
                    onCheckedChange={(checked) =>
                      updateSetting("betaUpdates", checked)
                    }
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div>
                    <Label>Verbose Logging</Label>
                    <p className="text-sm text-gray-500">
                      Enable detailed logging
                    </p>
                  </div>
                  <Switch
                    checked={settings.verboseLogging}
                    onCheckedChange={(checked) =>
                      updateSetting("verboseLogging", checked)
                    }
                  />
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
}
