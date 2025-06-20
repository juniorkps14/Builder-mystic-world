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
    badge: "ใหม่",
    color: "text-violet-500",
    bgColor: "bg-violet-50 dark:bg-violet-950/30",
  },
  {
    titleKey: "nav.features",
    icon: Sparkles,
    href: "/features",
    badge: "Pro",
    color: "text-yellow-500",
    bgColor: "bg-yellow-50 dark:bg-yellow-950/30",
  },
  {
    titleKey: "nav.tfTree",
    icon: GitBranch,
    href: "/tf-tree",
    badge: "สด",
    color: "text-teal-500",
    bgColor: "bg-teal-50 dark:bg-teal-950/30",
  },
  {
    titleKey: "nav.mapView",
    icon: Globe,
    href: "/map-viewer",
    badge: "สด",
    color: "text-blue-600",
    bgColor: "bg-blue-50 dark:bg-blue-950/30",
  },
  {
    titleKey: "nav.nodes",
    icon: Network,
    href: "/nodes",
    badge: "12",
    color: "text-green-600",
    bgColor: "bg-green-50 dark:bg-green-950/30",
  },
  {
    titleKey: "nav.topics",
    icon: Wifi,
    href: "/topics",
    badge: "24",
    color: "text-blue-500",
    bgColor: "bg-blue-50 dark:bg-blue-950/30",
  },
  {
    titleKey: "nav.services",
    icon: Shield,
    href: "/services",
    badge: "8",
    color: "text-red-500",
    bgColor: "bg-red-50 dark:bg-red-950/30",
  },
  {
    titleKey: "nav.parameters",
    icon: Settings,
    href: "/parameters",
    badge: null,
    color: "text-gray-500",
    bgColor: "bg-gray-50 dark:bg-gray-950/30",
  },
  {
    titleKey: "nav.logs",
    icon: FileText,
    href: "/logs",
    badge: "ใหม่",
    color: "text-orange-500",
    bgColor: "bg-orange-50 dark:bg-orange-950/30",
  },
  {
    titleKey: "nav.terminal",
    icon: Terminal,
    href: "/terminal",
    badge: null,
    color: "text-slate-600",
    bgColor: "bg-slate-50 dark:bg-slate-950/30",
  },
];

export function Sidebar() {
  const location = useLocation();
  const { t, language, setLanguage } = useLanguage();
  const { theme, setTheme } = useTheme();
  const [isCollapsed, setIsCollapsed] = useState(false);

  const renderNavItem = (item: any) => {
    const isActive = location.pathname === item.href;

    const NavButton = (
      <Link
        to={item.href}
        className={cn(
          "flex items-center justify-between rounded-2xl p-4 text-sm transition-all duration-500 group relative overflow-hidden",
          isActive
            ? "bg-gradient-to-r from-primary/15 via-primary/10 to-primary/5 text-primary border border-primary/20 shadow-lg backdrop-blur-sm"
            : "text-sidebar-foreground/70 hover:text-sidebar-foreground hover:bg-white/5 dark:hover:bg-white/5 border border-transparent hover:border-white/10",
          isCollapsed ? "justify-center" : "",
          "hover:shadow-xl hover:scale-[1.02] active:scale-[0.98]",
        )}
      >
        {/* Background Glow Effect - Only for Active */}
        {isActive && (
          <div
            className="absolute inset-0 bg-gradient-to-r from-primary/5 to-transparent rounded-2xl animate-pulse"
            style={{ animationDuration: "3s" }}
          />
        )}

        <div className="flex items-center gap-4 relative z-10">
          <div
            className={cn(
              "relative p-3 rounded-xl transition-all duration-500 group-hover:scale-110",
              isActive
                ? `${item.bgColor} shadow-lg ring-2 ring-primary/20`
                : `hover:${item.bgColor} group-hover:shadow-md`,
            )}
          >
            <item.icon
              className={cn(
                "h-5 w-5 transition-all duration-500",
                isActive
                  ? item.color
                  : `text-sidebar-foreground/60 group-hover:${item.color}`,
              )}
            />

            {/* Icon Glow Effect for Active */}
            {isActive && (
              <div
                className="absolute inset-0 rounded-xl opacity-20 animate-pulse"
                style={{
                  background: `radial-gradient(circle, ${item.color.replace("text-", "rgb(var(--")} 0%, transparent 70%)`,
                  animationDuration: "4s",
                }}
              />
            )}
          </div>

          {!isCollapsed && (
            <div className="flex flex-col">
              <span className="font-extralight text-sm leading-tight">
                {t(item.titleKey)}
              </span>
              {isActive && (
                <div className="h-0.5 w-8 bg-gradient-to-r from-primary to-transparent rounded-full mt-1 opacity-60" />
              )}
            </div>
          )}
        </div>

        {!isCollapsed && item.badge && (
          <div className="relative z-10">
            <Badge
              variant={isActive ? "default" : "secondary"}
              className={cn(
                "text-xs font-extralight border-0 shadow-md transition-all duration-300",
                isActive
                  ? "bg-gradient-to-r from-primary/80 to-primary/60 text-white shadow-primary/30"
                  : "bg-white/10 text-sidebar-foreground/80 hover:bg-white/20",
              )}
            >
              {item.badge}
            </Badge>
          </div>
        )}

        {/* Subtle Hover Effect */}
        <div className="absolute inset-0 rounded-2xl bg-gradient-to-r from-white/0 via-white/5 to-white/0 opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
      </Link>
    );

    if (isCollapsed) {
      return (
        <Tooltip key={item.href}>
          <TooltipTrigger asChild>{NavButton}</TooltipTrigger>
          <TooltipContent side="right" className="glass-effect">
            <p>{t(item.titleKey)}</p>
          </TooltipContent>
        </Tooltip>
      );
    }

    return <div key={item.href}>{NavButton}</div>;
  };

  return (
    <TooltipProvider>
      <div
        className={cn(
          "flex h-full flex-col border-r border-border/50 sidebar-glass transition-all duration-300 ease-in-out relative",
          isCollapsed ? "w-20" : "w-64",
        )}
      >
        {/* Toggle Button */}
        <Button
          variant="ghost"
          size="sm"
          onClick={() => setIsCollapsed(!isCollapsed)}
          className="absolute -right-4 top-8 h-8 w-8 rounded-full bg-gradient-to-r from-primary/20 to-primary/10 backdrop-blur-md border border-primary/20 hover:from-primary/30 hover:to-primary/20 z-50 shadow-xl hover:shadow-2xl transition-all duration-500 hover:scale-110 active:scale-95"
        >
          <div className="relative">
            {isCollapsed ? (
              <ChevronRight className="h-4 w-4 text-primary transition-transform duration-300" />
            ) : (
              <ChevronLeft className="h-4 w-4 text-primary transition-transform duration-300" />
            )}

            {/* Subtle glow effect */}
            <div
              className="absolute inset-0 rounded-full bg-primary/20 blur-sm animate-pulse opacity-50"
              style={{ animationDuration: "3s" }}
            />
          </div>
        </Button>

        {/* Header */}
        <div className="p-6">
          {!isCollapsed ? (
            <div className="flex items-center gap-3 mb-6 slide-in-left">
              <div className="w-12 h-12 rounded-2xl bg-gradient-to-br from-primary via-primary/80 to-primary/60 flex items-center justify-center shadow-xl shadow-primary/20 hover-lift glow-on-hover float-animation">
                <Bot className="h-7 w-7 text-primary-foreground" />
              </div>
              <div className="fade-in-up">
                <h2 className="text-xl font-extralight text-sidebar-foreground">
                  {t("app.name")}
                </h2>
                <p className="text-xs text-sidebar-foreground/70 font-extralight">
                  {t("app.subtitle")}
                </p>
              </div>
            </div>
          ) : (
            <div className="flex justify-center mb-6 scale-in">
              <div className="w-12 h-12 rounded-2xl bg-gradient-to-br from-primary via-primary/80 to-primary/60 flex items-center justify-center shadow-xl shadow-primary/20 hover-lift glow-on-hover float-animation">
                <Bot className="h-7 w-7 text-primary-foreground" />
              </div>
            </div>
          )}

          {/* Quick Status */}
          {!isCollapsed && (
            <div className="glass-effect rounded-2xl p-5 mb-6 hover-lift fade-in-up border border-white/10">
              <div className="flex items-center justify-between mb-3">
                <span className="text-sm font-extralight text-sidebar-accent-foreground">
                  {t("app.systemStatus")}
                </span>
                <div className="relative">
                  <div className="w-3 h-3 rounded-full bg-emerald-400 shadow-lg shadow-emerald-400/30" />
                  <div
                    className="absolute inset-0 w-3 h-3 rounded-full bg-emerald-400 animate-ping opacity-20"
                    style={{ animationDuration: "3s" }}
                  />
                  <div
                    className="absolute -inset-1 w-5 h-5 rounded-full bg-emerald-400/10 animate-pulse"
                    style={{ animationDuration: "4s" }}
                  />
                </div>
              </div>
              <div className="text-xs text-sidebar-accent-foreground/80 font-extralight leading-relaxed">
                {t("app.allSystemsOperational")}
              </div>
            </div>
          )}
        </div>

        <ScrollArea className="flex-1 px-4">
          {/* Main Navigation */}
          <div className="mb-6">
            {!isCollapsed && (
              <h3 className="mb-3 text-xs font-extralight text-sidebar-foreground/60 uppercase tracking-wider px-3 slide-in-right">
                หลัก
              </h3>
            )}
            <nav className="space-y-2">
              {navigationItems.map((item, index) => (
                <div key={item.href} className="stagger-item">
                  {renderNavItem(item)}
                </div>
              ))}
            </nav>
          </div>

          <Separator className="my-6 mx-3 bg-border/30" />

          {/* ROS System */}
          <div className="mb-6">
            {!isCollapsed && (
              <h3 className="mb-3 text-xs font-extralight text-sidebar-foreground/60 uppercase tracking-wider px-3 slide-in-right">
                ระบบ ROS
              </h3>
            )}
            <nav className="space-y-2">
              {rosItems.map((item, index) => (
                <div key={item.href} className="stagger-item">
                  {renderNavItem(item)}
                </div>
              ))}
            </nav>
          </div>
        </ScrollArea>

        {/* Bottom Settings */}
        {!isCollapsed && (
          <div className="p-6 border-t border-sidebar-border/30 space-y-4 fade-in-up">
            {/* Language Selector */}
            <div className="hover-lift">
              <div className="flex items-center gap-2 mb-2">
                <Languages className="h-4 w-4 text-sidebar-foreground/70 hover-rotate" />
                <span className="text-sm font-extralight text-sidebar-foreground">
                  ภาษา
                </span>
              </div>
              <Select value={language} onValueChange={setLanguage}>
                <SelectTrigger className="w-full glass-effect border-0 hover-scale font-extralight">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent className="glass-effect">
                  <SelectItem value="en" className="font-extralight">
                    {t("language.english")}
                  </SelectItem>
                  <SelectItem value="th" className="font-extralight">
                    {t("language.thai")}
                  </SelectItem>
                </SelectContent>
              </Select>
            </div>

            {/* Theme Selector */}
            <div className="hover-lift">
              <div className="flex items-center gap-2 mb-2">
                {theme === "light" && (
                  <Sun className="h-4 w-4 text-sidebar-foreground/70 spin-slow" />
                )}
                {theme === "dark" && (
                  <Moon className="h-4 w-4 text-sidebar-foreground/70 float-delay-2" />
                )}
                {theme === "system" && (
                  <Monitor className="h-4 w-4 text-sidebar-foreground/70 pulse-soft" />
                )}
                <span className="text-sm font-extralight text-sidebar-foreground">
                  ธีม
                </span>
              </div>
              <Select value={theme} onValueChange={setTheme}>
                <SelectTrigger className="w-full glass-effect border-0 hover-scale font-extralight">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent className="glass-effect">
                  <SelectItem value="light" className="font-extralight">
                    <div className="flex items-center gap-2">
                      <Sun className="h-4 w-4" />
                      {t("theme.light")}
                    </div>
                  </SelectItem>
                  <SelectItem value="dark" className="font-extralight">
                    <div className="flex items-center gap-2">
                      <Moon className="h-4 w-4" />
                      {t("theme.dark")}
                    </div>
                  </SelectItem>
                  <SelectItem value="system" className="font-extralight">
                    <div className="flex items-center gap-2">
                      <Monitor className="h-4 w-4" />
                      {t("theme.system")}
                    </div>
                  </SelectItem>
                </SelectContent>
              </Select>
            </div>

            <Separator className="bg-border/30" />

            <div className="space-y-3">
              <Button
                variant="ghost"
                className="w-full justify-start gap-3 p-3 rounded-xl text-sidebar-foreground/70 hover:text-sidebar-foreground hover:bg-white/5 font-extralight transition-all duration-500 hover:scale-[1.02] border border-transparent hover:border-white/10"
                asChild
              >
                <Link to="/about">
                  <div className="p-2 rounded-lg hover:bg-blue-50 dark:hover:bg-blue-950/30 transition-colors duration-300">
                    <FileText className="h-4 w-4 transition-transform duration-300 hover:scale-110" />
                  </div>
                  {t("nav.about")}
                </Link>
              </Button>
              <Button
                variant="ghost"
                className="w-full justify-start gap-3 p-3 rounded-xl text-sidebar-foreground/70 hover:text-sidebar-foreground hover:bg-white/5 font-extralight transition-all duration-500 hover:scale-[1.02] border border-transparent hover:border-white/10"
                asChild
              >
                <Link to="/settings">
                  <div className="p-2 rounded-lg hover:bg-purple-50 dark:hover:bg-purple-950/30 transition-colors duration-300">
                    <Settings className="h-4 w-4 transition-transform duration-700 hover:rotate-90" />
                  </div>
                  {t("nav.settings")}
                </Link>
              </Button>
            </div>
          </div>
        )}

        {/* Collapsed Settings */}
        {isCollapsed && (
          <div className="p-4 border-t border-sidebar-border/30 space-y-3">
            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant="ghost"
                  size="sm"
                  className="w-full h-12 p-0"
                  asChild
                >
                  <Link to="/settings">
                    <Settings className="h-5 w-5" />
                  </Link>
                </Button>
              </TooltipTrigger>
              <TooltipContent side="right" className="glass-effect">
                <p>{t("nav.settings")}</p>
              </TooltipContent>
            </Tooltip>
          </div>
        )}
      </div>
    </TooltipProvider>
  );
}
