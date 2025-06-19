import { Map } from "lucide-react";

const Navigation = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Navigation</h1>
        <p className="text-muted-foreground">
          Robot navigation and path planning
        </p>
      </div>

      <div className="text-center py-12">
        <Map className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">Navigation System</h3>
        <p className="text-muted-foreground">
          Interactive map, path planning, and navigation controls will be
          displayed here.
        </p>
      </div>
    </div>
  );
};

export default Navigation;
