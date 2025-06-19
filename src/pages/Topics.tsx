import { Radio } from "lucide-react";

const Topics = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">ROS Topics</h1>
        <p className="text-muted-foreground">Monitor and analyze ROS topics</p>
      </div>

      <div className="text-center py-12">
        <Radio className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">Topic Browser</h3>
        <p className="text-muted-foreground">
          Real-time topic monitoring, message inspection, and data visualization
          will be displayed here.
        </p>
      </div>
    </div>
  );
};

export default Topics;
