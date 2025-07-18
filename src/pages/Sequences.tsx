import React, { useState, useEffect } from "react";
import { useLanguage } from "@/contexts/LanguageContext";
import { EnhancedSequenceManager } from "@/components/ros/EnhancedSequenceManager";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";

const Sequences: React.FC = () => {
  const { t } = useLanguage();

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-4xl font-extralight bg-gradient-to-r from-foreground via-foreground/80 to-foreground/60 bg-clip-text text-transparent">
            {t("sequence.title")}
          </h1>
          <p className="text-muted-foreground font-light mt-2">
            {t("sequence.subtitle")}
          </p>
        </div>
      </div>

      {/* Enhanced Sequence Manager with all advanced functionality */}
      <EnhancedSequenceManager />
    </div>
  );
};

export default Sequences;
