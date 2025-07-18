import React, { useState } from "react";
import { useLanguage } from "@/contexts/LanguageContext";
import { BasicSequenceManager } from "@/components/ros/BasicSequenceManager";
import { EnhancedSequenceManager } from "@/components/ros/EnhancedSequenceManager";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Settings, User } from "lucide-react";

const Sequences: React.FC = () => {
  const { t } = useLanguage();
  const [mode, setMode] = useState<"basic" | "advanced">("basic");

  return (
    <div className="space-y-6">
      {/* Header with Mode Switcher */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            Robot Sequences
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Create and manage robot task sequences
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Badge variant="outline">Mode:</Badge>
          <div className="flex rounded-lg border">
            <Button
              variant={mode === "basic" ? "default" : "ghost"}
              size="sm"
              onClick={() => setMode("basic")}
              className="rounded-r-none gap-2"
            >
              <User className="h-4 w-4" />
              Basic
            </Button>
            <Button
              variant={mode === "advanced" ? "default" : "ghost"}
              size="sm"
              onClick={() => setMode("advanced")}
              className="rounded-l-none border-l gap-2"
            >
              <Settings className="h-4 w-4" />
              Advanced
            </Button>
          </div>
        </div>
      </div>

      {/* Dynamic Content Based on Mode */}
      {mode === "basic" ? (
        <BasicSequenceManager />
      ) : (
        <EnhancedSequenceManager />
      )}
    </div>
  );
};

export default Sequences;
