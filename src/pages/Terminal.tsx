import { Terminal as TerminalIcon } from "lucide-react";

const Terminal = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Terminal</h1>
        <p className="text-muted-foreground">Command line interface for ROS</p>
      </div>

      <div className="text-center py-12">
        <TerminalIcon className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">ROS Terminal</h3>
        <p className="text-muted-foreground">
          Interactive terminal for executing ROS commands and debugging will be
          displayed here.
        </p>
      </div>
    </div>
  );
};

export default Terminal;
