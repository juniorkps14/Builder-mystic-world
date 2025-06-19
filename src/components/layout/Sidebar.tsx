import { cn } from "@/lib/utils";
import { Button } from "@/components/ui/button";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Badge } from "@/components/ui/badge";
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
} from "lucide-react";
import { Link, useLocation } from "react-router-dom";

const navigationItems = [
  {
    title: "Dashboard",
    icon: Home,
    href: "/",
    badge: null,
  },
  {
    title: "Robot Control",
    icon: Gamepad2,
    href: "/control",
    badge: null,
  },
  {
    title: "Sequences",
    icon: Layers,
    href: "/sequences",
    badge: "New",
  },
  {
    title: "Camera Feeds",
    icon: Camera,
    href: "/cameras",
    badge: "Live",
  },
  {
    title: "Sensors",
    icon: Activity,
    href: "/sensors",
    badge: "8",
  },
  {
    title: "Navigation",
    icon: Map,
    href: "/navigation",
    badge: null,
  },
  {
    title: "System Monitor",
    icon: BarChart3,
    href: "/system",
    badge: null,
  },
];

const rosItems = [
  {
    title: "Nodes",
    icon: Network,
    href: "/nodes",
    badge: "12",
  },
  {
    title: "Topics",
    icon: Radio,
    href: "/topics",
    badge: "24",
  },
  {
    title: "Services",
    icon: Zap,
    href: "/services",
    badge: "8",
  },
  {
    title: "Parameters",
    icon: Layers,
    href: "/parameters",
    badge: null,
  },
  {
    title: "Logs",
    icon: FileText,
    href: "/logs",
    badge: "New",
  },
  {
    title: "Terminal",
    icon: Terminal,
    href: "/terminal",
    badge: null,
  },
];

export function Sidebar() {
  const location = useLocation();

  return (
    <div className="flex h-full w-64 flex-col border-r border-border bg-sidebar">
      <div className="p-6">
        <div className="flex items-center gap-3 mb-6">
          <div className="w-8 h-8 rounded-lg bg-primary flex items-center justify-center">
            <Activity className="h-5 w-5 text-primary-foreground" />
          </div>
          <div>
            <h2 className="text-lg font-bold text-sidebar-foreground">
              ROS Web
            </h2>
            <p className="text-xs text-sidebar-foreground/60">
              Control Interface
            </p>
          </div>
        </div>

        {/* Quick Status */}
        <div className="bg-sidebar-accent rounded-lg p-3 mb-6">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-sidebar-accent-foreground">
              System Status
            </span>
            <div className="ros-status-indicator ros-status-active" />
          </div>
          <div className="text-xs text-sidebar-accent-foreground/70">
            All systems operational
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
                    <span>{item.title}</span>
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
                    <span>{item.title}</span>
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
      <div className="p-6 border-t border-sidebar-border">
        <Button
          variant="ghost"
          className="w-full justify-start gap-3 text-sidebar-foreground hover:bg-sidebar-accent"
          asChild
        >
          <Link to="/settings">
            <Settings className="h-4 w-4" />
            Settings
          </Link>
        </Button>
      </div>
    </div>
  );
}
