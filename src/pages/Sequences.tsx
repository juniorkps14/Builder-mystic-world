import React from "react";
import { useLanguage } from "@/contexts/LanguageContext";
import { EnhancedSequenceManager } from "@/components/ros/EnhancedSequenceManager";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Save, Download, Upload } from "lucide-react";

const Sequences: React.FC = () => {
  const { t } = useLanguage();

  const handleApply = () => {
    console.log("Applying sequence configuration...");
    // TODO: Save to JSON and apply settings
  };

  const handleExport = () => {
    console.log("Exporting sequences...");
    // TODO: Export to JSON file
  };

  const handleImport = () => {
    console.log("Importing sequences...");
    // TODO: Import from JSON file
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            Robot Sequences
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            Advanced sequence management and execution
          </p>
        </div>

        <div className="flex items-center gap-3">
          <Button variant="outline" onClick={handleImport} className="gap-2">
            <Upload className="h-4 w-4" />
            Import
          </Button>
          <Button variant="outline" onClick={handleExport} className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
          <Button onClick={handleApply} className="gap-2">
            <Save className="h-4 w-4" />
            Apply Changes
          </Button>
        </div>
      </div>

      {/* Enhanced Sequence Manager */}
      <EnhancedSequenceManager />
    </div>
  );
};

export default Sequences;
