import { Layers } from "lucide-react";

const Parameters = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">ROS Parameters</h1>
        <p className="text-muted-foreground">
          Configure and manage ROS parameters
        </p>
      </div>

      <div className="text-center py-12">
        <Layers className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">Parameter Server</h3>
        <p className="text-muted-foreground">
          Parameter management, configuration, and runtime adjustment tools will
          be displayed here.
        </p>
      </div>
    </div>
  );
};

export default Parameters;
