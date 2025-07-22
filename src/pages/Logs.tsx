import { FileText } from "lucide-react";
import { Badge } from "@/components/ui/badge";

const Logs = () => {
  return (
    <div className="p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            System Logs
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            View and analyze system logs
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge className="bg-blue-100 text-blue-700">
            <FileText className="h-3 w-3 mr-1" />
            Live
          </Badge>
        </div>
      </div>

      <div className="text-center py-12">
        <FileText className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">Log Viewer</h3>
        <p className="text-muted-foreground">
          Real-time log streaming, filtering, and analysis tools will be
          displayed here.
        </p>
      </div>
    </div>
  );
};

export default Logs;
