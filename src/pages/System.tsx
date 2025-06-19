import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import {
  BarChart3,
  Cpu,
  HardDrive,
  MemoryStick,
  Wifi,
  Thermometer,
} from "lucide-react";

const System = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">System Monitor</h1>
        <p className="text-muted-foreground">
          System performance and health monitoring
        </p>
      </div>

      <div className="text-center py-12">
        <BarChart3 className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">System Monitoring</h3>
        <p className="text-muted-foreground">
          Real-time system performance metrics and diagnostics will be displayed
          here.
        </p>
      </div>
    </div>
  );
};

export default System;
