import { RobotControl as RobotControlComponent } from "@/components/ros/RobotControl";
import { Badge } from "@/components/ui/badge";
import { Activity } from "lucide-react";

const RobotControl = () => {
  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            Robot Control
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Manual and autonomous robot control interface
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge className="bg-green-100 text-green-700">
            <Activity className="h-3 w-3 mr-1" />
            Active
          </Badge>
        </div>
      </div>
      <RobotControlComponent />
    </div>
  );
};

export default RobotControl;
