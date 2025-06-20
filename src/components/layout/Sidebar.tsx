import { cn } from "@/lib/utils";
import { Button } from "@/components/ui/button";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Badge } from "@/components/ui/badge";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import {
  Home,
  Gamepad2,
  Camera,
  Activity,
  Network,
  Terminal,
  Map,
  Settings,
  FileText,
  BarChart3,
  Zap,
  Radio,
  Layers,
  Sun,
  Moon,
  Monitor,
  Languages,
  GitBranch,
  MapPin,
  Cpu,
  Circle,
  Compass,
  ChevronLeft,
  ChevronRight,
  Bot,
  Eye,
  Wifi,
  Smartphone,
  Globe,
  Sparkles,
  Shield,
} from "lucide-react";
import { Link, useLocation } from "react-router-dom";
import { useLanguage } from "@/contexts/LanguageContext";
import { useTheme } from "@/contexts/ThemeContext";
import { useState } from "react";

const navigationItems = [
  {
    titleKey: "nav.dashboard",
    icon: Home,
    href: "/",
    badge: null,
    color: "text-blue-500",
    bgColor: "bg-blue-50 dark:bg-blue-950/30",
  },
  {
    titleKey: "nav.robotControl",
    icon: Bot,
    href: "/control",
    badge: null,
    color: "text-green-500",
    bgColor: "bg-green-50 dark:bg-green-950/30",
  },
  {
    titleKey: "nav.sequences",
    icon: Layers,
    href: "/sequences",
    badge: "ใหม่",
    color: "text-purple-500",
    bgColor: "bg-purple-50 dark:bg-purple-950/30",
  },
  {
    titleKey: "nav.roboticArm",
    icon: Activity,
    href: "/robotic-arm",
    badge: "ใหม่",
    color: "text-orange-500",
    bgColor: "bg-orange-50 dark:bg-orange-950/30",
  },
  {
    titleKey: "nav.holonomic",
    icon: Circle,
    href: "/holonomic-drive",
    badge: "ใหม่",
    color: "text-cyan-500",
    bgColor: "bg-cyan-50 dark:bg-cyan-950/30",
  },
  {
    titleKey: "nav.cameras",
    icon: Eye,
    href: "/cameras",
    badge: "สด",
    color: "text-red-500",
    bgColor: "bg-red-50 dark:bg-red-950/30",
  },
  {
    titleKey: "nav.sensors",
    icon: Compass,
    href: "/sensors",
    badge: "8",
    color: "text-indigo-500",
    bgColor: "bg-indigo-50 dark:bg-indigo-950/30",
  },
  {
    titleKey: "nav.navigation",
    icon: Map,
    href: "/navigation",
    badge: null,
    color: "text-emerald-500",
    bgColor: "bg-emerald-50 dark:bg-emerald-950/30",
  },
  {
    titleKey: "nav.system",
    icon: BarChart3,
    href: "/system",
    badge: null,
    color: "text-slate-500",
    bgColor: "bg-slate-50 dark:bg-slate-950/30",
  },
];

const rosItems = [
  {
    titleKey: "nav.ioConfig",
    icon: Cpu,
    href: "/io-config",
    badge: "New",
  },
  {
    titleKey: "nav.features",
    icon: Zap,
    href: "/features",
    badge: "Pro",
  },
  {
    titleKey: "nav.tfTree",
    icon: GitBranch,
    href: "/tf-tree",
    badge: "Live",
  },
  {
    titleKey: "nav.mapView",
    icon: MapPin,
    href: "/map-viewer",
    badge: "Live",
  },
  {
    titleKey: "nav.nodes",
    icon: Network,
    href: "/nodes",
    badge: "12",
  },
  {
    titleKey: "nav.topics",
    icon: Radio,
    href: "/topics",
    badge: "24",
  },
  {
    titleKey: "nav.services",
    icon: Zap,
    href: "/services",
    badge: "8",
  },
  {
    titleKey: "nav.parameters",
    icon: Layers,
    href: "/parameters",
    badge: null,
  },
  {
    titleKey: "nav.logs",
    icon: FileText,
    href: "/logs",
    badge: "New",
  },
  {
    titleKey: "nav.terminal",
    icon: Terminal,
    href: "/terminal",
    badge: null,
  },
];

export function Sidebar() {
  const location = useLocation();
  const { t, language, setLanguage } = useLanguage();
  const { theme, setTheme } = useTheme();

  return (
    <div className="flex h-full w-64 flex-col border-r border-border bg-sidebar">
      <div className="p-6">
        <div className="flex items-center gap-3 mb-6">
          <div className="w-8 h-8 rounded-lg bg-primary flex items-center justify-center">
            <Activity className="h-5 w-5 text-primary-foreground" />
          </div>
          <div>
            <h2 className="text-lg font-bold text-sidebar-foreground">
              {t("app.name")}
            </h2>
            <p className="text-xs text-sidebar-foreground/60">
              {t("app.subtitle")}
            </p>
          </div>
        </div>

        {/* Quick Status */}
        <div className="bg-sidebar-accent rounded-lg p-3 mb-6">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-sidebar-accent-foreground">
              {t("app.systemStatus")}
            </span>
            <div className="ros-status-indicator ros-status-active" />
          </div>
          <div className="text-xs text-sidebar-accent-foreground/70">
            {t("app.allSystemsOperational")}
          </div>
        </div>
      </div>

      <ScrollArea className="flex-1 px-6">
        {/* Main Navigation */}
        <div className="mb-6">
          <h3 className="mb-3 text-sm font-semibold text-sidebar-foreground/60 uppercase tracking-wider">
            Navigation
          </h3>
          <nav className="space-y-1">
            {navigationItems.map((item) => {
              const isActive = location.pathname === item.href;
              return (
                <Link
                  key={item.href}
                  to={item.href}
                  className={cn(
                    "flex items-center justify-between rounded-lg px-3 py-2 text-sm transition-all duration-200",
                    isActive
                      ? "bg-sidebar-primary text-sidebar-primary-foreground"
                      : "text-sidebar-foreground hover:bg-sidebar-accent hover:text-sidebar-accent-foreground",
                  )}
                >
                  <div className="flex items-center gap-3">
                    <item.icon className="h-4 w-4" />
                    <span>{t(item.titleKey)}</span>
                  </div>
                  {item.badge && (
                    <Badge
                      variant={isActive ? "secondary" : "outline"}
                      className="text-xs"
                    >
                      {item.badge}
                    </Badge>
                  )}
                </Link>
              );
            })}
          </nav>
        </div>

        <Separator className="my-4" />

        {/* ROS System */}
        <div className="mb-6">
          <h3 className="mb-3 text-sm font-semibold text-sidebar-foreground/60 uppercase tracking-wider">
            ROS System
          </h3>
          <nav className="space-y-1">
            {rosItems.map((item) => {
              const isActive = location.pathname === item.href;
              return (
                <Link
                  key={item.href}
                  to={item.href}
                  className={cn(
                    "flex items-center justify-between rounded-lg px-3 py-2 text-sm transition-all duration-200",
                    isActive
                      ? "bg-sidebar-primary text-sidebar-primary-foreground"
                      : "text-sidebar-foreground hover:bg-sidebar-accent hover:text-sidebar-accent-foreground",
                  )}
                >
                  <div className="flex items-center gap-3">
                    <item.icon className="h-4 w-4" />
                    <span>{t(item.titleKey)}</span>
                  </div>
                  {item.badge && (
                    <Badge
                      variant={isActive ? "secondary" : "outline"}
                      className="text-xs"
                    >
                      {item.badge}
                    </Badge>
                  )}
                </Link>
              );
            })}
          </nav>
        </div>
      </ScrollArea>

      {/* Bottom Settings */}
      <div className="p-6 border-t border-sidebar-border space-y-3">
        {/* Language Selector */}
        <div>
          <div className="flex items-center gap-2 mb-2">
            <Languages className="h-4 w-4 text-sidebar-foreground" />
            <span className="text-sm font-medium text-sidebar-foreground">
              Language
            </span>
          </div>
          <Select value={language} onValueChange={setLanguage}>
            <SelectTrigger className="w-full">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="en">{t("language.english")}</SelectItem>
              <SelectItem value="th">{t("language.thai")}</SelectItem>
            </SelectContent>
          </Select>
        </div>

        {/* Theme Selector */}
        <div>
          <div className="flex items-center gap-2 mb-2">
            {theme === "light" && (
              <Sun className="h-4 w-4 text-sidebar-foreground" />
            )}
            {theme === "dark" && (
              <Moon className="h-4 w-4 text-sidebar-foreground" />
            )}
            {theme === "system" && (
              <Monitor className="h-4 w-4 text-sidebar-foreground" />
            )}
            <span className="text-sm font-medium text-sidebar-foreground">
              Theme
            </span>
          </div>
          <Select value={theme} onValueChange={setTheme}>
            <SelectTrigger className="w-full">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="light">
                <div className="flex items-center gap-2">
                  <Sun className="h-4 w-4" />
                  {t("theme.light")}
                </div>
              </SelectItem>
              <SelectItem value="dark">
                <div className="flex items-center gap-2">
                  <Moon className="h-4 w-4" />
                  {t("theme.dark")}
                </div>
              </SelectItem>
              <SelectItem value="system">
                <div className="flex items-center gap-2">
                  <Monitor className="h-4 w-4" />
                  {t("theme.system")}
                </div>
              </SelectItem>
            </SelectContent>
          </Select>
        </div>

        <Separator />

        <Button
          variant="ghost"
          className="w-full justify-start gap-3 text-sidebar-foreground hover:bg-sidebar-accent"
          asChild
        >
          <Link to="/about">
            <FileText className="h-4 w-4" />
            {t("nav.about")}
          </Link>
        </Button>
        <Button
          variant="ghost"
          className="w-full justify-start gap-3 text-sidebar-foreground hover:bg-sidebar-accent"
          asChild
        >
          <Link to="/settings">
            <Settings className="h-4 w-4" />
            {t("nav.settings")}
          </Link>
        </Button>
      </div>
    </div>
  );
}
