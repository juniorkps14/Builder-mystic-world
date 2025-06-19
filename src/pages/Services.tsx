import { Zap } from "lucide-react";

const Services = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">ROS Services</h1>
        <p className="text-muted-foreground">Manage and call ROS services</p>
      </div>

      <div className="text-center py-12">
        <Zap className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">Service Manager</h3>
        <p className="text-muted-foreground">
          Service discovery, testing, and invocation tools will be displayed
          here.
        </p>
      </div>
    </div>
  );
};

export default Services;
