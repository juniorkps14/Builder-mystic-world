import React, { useState, useEffect } from "react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import {
  Save,
  Database,
  Trash2,
  HardDrive,
  AlertTriangle,
  CheckCircle,
  Clock,
  Info,
} from "lucide-react";
import { usePersistence } from "@/contexts/PersistenceContext";

export function PersistenceStatus() {
  const { getStorageInfo, clearAll } = usePersistence();
  const [storageInfo, setStorageInfo] = useState({
    used: 0,
    available: 0,
    percentage: 0,
  });
  const [lastSaved, setLastSaved] = useState<Date | null>(null);

  useEffect(() => {
    const updateStorageInfo = () => {
      const info = getStorageInfo();
      setStorageInfo(info);
    };

    updateStorageInfo();
    const interval = setInterval(updateStorageInfo, 5000);
    return () => clearInterval(interval);
  }, [getStorageInfo]);

  useEffect(() => {
    // Track when data is saved
    setLastSaved(new Date());
  }, []);

  const formatBytes = (bytes: number) => {
    if (bytes === 0) return "0 B";
    const k = 1024;
    const sizes = ["B", "KB", "MB"];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + " " + sizes[i];
  };

  const handleClearData = () => {
    if (
      confirm(
        "Are you sure you want to delete all saved data? This action cannot be undone.",
      )
    ) {
      clearAll();
      setStorageInfo({ used: 0, available: 0, percentage: 0 });
    }
  };

  const getStatusColor = () => {
    if (storageInfo.percentage > 80) return "text-red-600";
    if (storageInfo.percentage > 60) return "text-yellow-600";
    return "text-green-600";
  };

  const getStatusIcon = () => {
    if (storageInfo.percentage > 80) return AlertTriangle;
    return CheckCircle;
  };

  const StatusIcon = getStatusIcon();

  return (
    <Popover>
      <PopoverTrigger asChild>
        <Button variant="ghost" size="sm" className="h-8 px-2 text-xs gap-1.5">
          <StatusIcon className={`h-3 w-3 ${getStatusColor()}`} />
          <span className="hidden sm:inline">
            {formatBytes(storageInfo.used)}
          </span>
        </Button>
      </PopoverTrigger>
      <PopoverContent className="w-80 p-4" align="end">
        <div className="space-y-4">
          <div className="flex items-center gap-2">
            <Database className="h-4 w-4" />
            <h4 className="font-semibold">Persistence Status</h4>
          </div>

          <div className="space-y-3">
            <div>
              <div className="flex justify-between text-sm mb-1">
                <span>Storage Used</span>
                <span className={getStatusColor()}>
                  {storageInfo.percentage.toFixed(1)}%
                </span>
              </div>
              <Progress value={storageInfo.percentage} className="h-2" />
              <div className="flex justify-between text-xs text-muted-foreground mt-1">
                <span>{formatBytes(storageInfo.used)} used</span>
                <span>{formatBytes(storageInfo.available)} available</span>
              </div>
            </div>

            {lastSaved && (
              <div className="flex items-center gap-2 text-sm text-muted-foreground">
                <Clock className="h-3 w-3" />
                <span>Last saved: {lastSaved.toLocaleTimeString()}</span>
              </div>
            )}

            <Separator />

            <div className="space-y-2">
              <div className="flex items-center gap-2 text-sm">
                <Save className="h-3 w-3" />
                <span>Auto-save enabled</span>
                <Badge variant="secondary" className="text-xs">
                  Active
                </Badge>
              </div>

              <div className="text-xs text-muted-foreground">
                <div className="flex items-start gap-1">
                  <Info className="h-3 w-3 mt-0.5 flex-shrink-0" />
                  <span>
                    Settings, command history, and work state are
                    automatically saved and restored when reopening the system
                  </span>
                </div>
              </div>
            </div>

            <Separator />

            <Button
              onClick={handleClearData}
              variant="destructive"
              size="sm"
              className="w-full gap-2"
            >
              <Trash2 className="h-3 w-3" />
              Clear All Saved Data
            </Button>
          </div>
        </div>
      </PopoverContent>
    </Popover>
  );
}
