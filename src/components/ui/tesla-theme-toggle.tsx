import React, { useState, useEffect } from "react";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import { Badge } from "@/components/ui/badge";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Label } from "@/components/ui/label";
import { usePersistentStore } from "@/hooks/use-persistence";
import {
  Palette,
  Monitor,
  Sun,
  Moon,
  Zap,
  Eye,
  EyeOff,
  Settings,
  Sparkles,
  Contrast,
  Type,
  Accessibility,
} from "lucide-react";

export function TeslaThemeToggle() {
  const { store: themePrefs, updateField } = usePersistentStore(
    "tesla-theme-preferences",
    {
      mode: "tesla" as "tesla" | "flat" | "light",
      glassEffect: true,
      animations: true,
      highContrast: false,
      largeText: false,
      reducedMotion: false,
      autoTransparency: true,
      glowEffects: true,
      opacity: 90,
      blurIntensity: 20,
    }
  );

  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    // Apply theme to document
    const root = document.documentElement;

    // Remove existing theme classes
    root.classList.remove("tesla-ui", "flat-ui", "light");

    // Apply selected theme
    switch (themePrefs.mode) {
      case "tesla":
        root.classList.add("tesla-ui");
        break;
      case "flat":
        root.classList.add("flat-ui");
        break;
      case "light":
        root.classList.add("light");
        break;
    }

    // Apply accessibility settings
    if (themePrefs.highContrast) {
      root.classList.add("tesla-high-contrast");
    } else {
      root.classList.remove("tesla-high-contrast");
    }

    if (themePrefs.largeText) {
      root.classList.add("tesla-large-text");
    } else {
      root.classList.remove("tesla-large-text");
    }

    if (themePrefs.reducedMotion) {
      root.style.setProperty("--tesla-transition", "none");
      root.style.setProperty("--tesla-transition-fast", "none");
    } else {
      root.style.removeProperty("--tesla-transition");
      root.style.removeProperty("--tesla-transition-fast");
    }

    // Apply visual effects
    root.style.setProperty("--glass-opacity", `${themePrefs.opacity}%`);
    root.style.setProperty("--glass-blur", `${themePrefs.blurIntensity}px`);

  }, [themePrefs]);

  const themeOptions = [
    {
      id: "tesla",
      name: "Tesla UI",
      description: "Dark glass morphism with modern animations",
      icon: Zap,
      preview: "bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900",
    },
    {
      id: "flat",
      name: "Flat Vector",
      description: "Clean minimal design with light colors",
      icon: Monitor,
      preview: "bg-gradient-to-br from-gray-100 via-white to-gray-50",
    },
    {
      id: "light",
      name: "Light Mode",
      description: "Traditional light interface",
      icon: Sun,
      preview: "bg-white",
    },
  ];

  const getCurrentThemeIcon = () => {
    switch (themePrefs.mode) {
      case "tesla":
        return Zap;
      case "flat":
        return Monitor;
      case "light":
        return Sun;
      default:
        return Palette;
    }
  };

  const CurrentIcon = getCurrentThemeIcon();

  return (
    <Popover open={isOpen} onOpenChange={setIsOpen}>
      <PopoverTrigger asChild>
        <Button
          variant="ghost"
          size="sm"
          className="relative h-8 w-8 p-0 tesla-btn hover:tesla-glass"
        >
          <CurrentIcon className="h-4 w-4" />
          {themePrefs.mode === "tesla" && (
            <div className="absolute -top-1 -right-1 w-2 h-2 bg-blue-500 rounded-full animate-pulse" />
          )}
        </Button>
      </PopoverTrigger>
      <PopoverContent className="w-80 p-0 tesla-glass border-white/20" align="end">
        <div className="p-4 space-y-4">
          {/* Header */}
          <div className="flex items-center gap-2">
            <Sparkles className="h-4 w-4 text-blue-400" />
            <h4 className="font-medium text-white">Theme & Experience</h4>
          </div>

          {/* Theme Selection */}
          <div className="space-y-2">
            <Label className="text-sm text-slate-400">Interface Theme</Label>
            <div className="grid gap-2">
              {themeOptions.map((theme) => {
                const IconComponent = theme.icon;
                return (
                  <div
                    key={theme.id}
                    onClick={() => updateField("mode", theme.id)}
                    className={`p-3 rounded-lg border cursor-pointer transition-all duration-200 ${
                      themePrefs.mode === theme.id
                        ? "border-blue-400 bg-blue-500/10"
                        : "border-white/10 hover:border-white/20 hover:bg-white/5"
                    }`}
                  >
                    <div className="flex items-center gap-3">
                      <div className={`w-8 h-8 rounded-lg ${theme.preview} flex items-center justify-center`}>
                        <IconComponent className="h-4 w-4 text-slate-600" />
                      </div>
                      <div className="flex-1">
                        <p className="font-medium text-white text-sm">{theme.name}</p>
                        <p className="text-xs text-slate-400">{theme.description}</p>
                      </div>
                      {themePrefs.mode === theme.id && (
                        <Badge className="bg-blue-500 text-white text-xs">Active</Badge>
                      )}
                    </div>
                  </div>
                );
              })}
            </div>
          </div>

          {/* Visual Effects (only for Tesla theme) */}
          {themePrefs.mode === "tesla" && (
            <div className="space-y-4 border-t border-white/10 pt-4">
              <Label className="text-sm text-slate-400">Visual Effects</Label>

              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <Eye className="h-4 w-4 text-slate-400" />
                    <span className="text-sm text-white">Glass Effect</span>
                  </div>
                  <Switch
                    checked={themePrefs.glassEffect}
                    onCheckedChange={(checked) => updateField("glassEffect", checked)}
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <Sparkles className="h-4 w-4 text-slate-400" />
                    <span className="text-sm text-white">Glow Effects</span>
                  </div>
                  <Switch
                    checked={themePrefs.glowEffects}
                    onCheckedChange={(checked) => updateField("glowEffects", checked)}
                  />
                </div>

                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <Zap className="h-4 w-4 text-slate-400" />
                    <span className="text-sm text-white">Animations</span>
                  </div>
                  <Switch
                    checked={themePrefs.animations}
                    onCheckedChange={(checked) => updateField("animations", checked)}
                  />
                </div>

                {/* Opacity Slider */}
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-white">Transparency</span>
                    <span className="text-xs text-slate-400">{themePrefs.opacity}%</span>
                  </div>
                  <Slider
                    value={[themePrefs.opacity]}
                    onValueChange={([value]) => updateField("opacity", value)}
                    min={60}
                    max={100}
                    step={5}
                    className="w-full"
                  />
                </div>

                {/* Blur Intensity */}
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-white">Blur Intensity</span>
                    <span className="text-xs text-slate-400">{themePrefs.blurIntensity}px</span>
                  </div>
                  <Slider
                    value={[themePrefs.blurIntensity]}
                    onValueChange={([value]) => updateField("blurIntensity", value)}
                    min={5}
                    max={40}
                    step={5}
                    className="w-full"
                  />
                </div>
              </div>
            </div>
          )}

          {/* Accessibility Options */}
          <div className="space-y-3 border-t border-white/10 pt-4">
            <Label className="text-sm text-slate-400 flex items-center gap-2">
              <Accessibility className="h-4 w-4" />
              Accessibility
            </Label>

            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Contrast className="h-4 w-4 text-slate-400" />
                  <span className="text-sm text-white">High Contrast</span>
                </div>
                <Switch
                  checked={themePrefs.highContrast}
                  onCheckedChange={(checked) => updateField("highContrast", checked)}
                />
              </div>

              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Type className="h-4 w-4 text-slate-400" />
                  <span className="text-sm text-white">Large Text</span>
                </div>
                <Switch
                  checked={themePrefs.largeText}
                  onCheckedChange={(checked) => updateField("largeText", checked)}
                />
              </div>

              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <EyeOff className="h-4 w-4 text-slate-400" />
                  <span className="text-sm text-white">Reduced Motion</span>
                </div>
                <Switch
                  checked={themePrefs.reducedMotion}
                  onCheckedChange={(checked) => updateField("reducedMotion", checked)}
                />
              </div>
            </div>
          </div>

          {/* Theme Info */}
          <div className="bg-blue-500/10 border border-blue-500/20 rounded-lg p-3">
            <p className="text-xs text-blue-300">
              🚀 Tesla UI provides a modern, accessible interface inspired by automotive displays.
              All settings are automatically saved and synchronized.
            </p>
          </div>
        </div>
      </PopoverContent>
    </Popover>
  );
}
