import { Settings as SettingsIcon } from "lucide-react";

const Settings = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Settings</h1>
        <p className="text-muted-foreground">
          Configure system and interface settings
        </p>
      </div>

      <div className="text-center py-12">
        <SettingsIcon className="h-16 w-16 text-primary mx-auto mb-4" />
        <h3 className="text-xl font-semibold mb-2">System Configuration</h3>
        <p className="text-muted-foreground">
          System configuration, user preferences, and interface settings will be
          displayed here.
        </p>
      </div>
    </div>
  );
};

export default Settings;
