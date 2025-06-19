import { FileText } from "lucide-react";

const Logs = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">System Logs</h1>
        <p className="text-muted-foreground">View and analyze system logs</p>
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
