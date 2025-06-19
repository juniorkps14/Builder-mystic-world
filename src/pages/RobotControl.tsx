import { RobotControl as RobotControlComponent } from "@/components/ros/RobotControl";

const RobotControl = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Robot Control</h1>
        <p className="text-muted-foreground">
          Manual and autonomous robot control interface
        </p>
      </div>
      <RobotControlComponent />
    </div>
  );
};

export default RobotControl;
