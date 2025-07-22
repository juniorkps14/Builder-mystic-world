import React from "react";
import { Link, useLocation } from "react-router-dom";
import { Badge } from "@/components/ui/badge";
import { ScrollArea } from "@/components/ui/scroll-area";
import {
  Home,
  Bot,
  Layers,
  Activity,
  Circle,
  Eye,
  Compass,
  Map,
  BarChart3,
  Terminal,
  Cpu,
  Sparkles,
  GitBranch,
  Globe,
  Network,
  Wifi,
  Shield,
  Settings,
  BookOpen,
  HelpCircle,
  Code,
  Database,
  X,
  Zap,
  Camera,
  Navigation,
  Gamepad2,
} from "lucide-react";

interface NavItem {
  title: string;
  icon: React.ComponentType<any>;
  href: string;
  badge?: string;
}

interface NavSection {
  title: string;
  items: NavItem[];
}

const navigationSections: NavSection[] = [
  {
    title: "Main Controls",
    items: [
      {
        title: "Dashboard",
        icon: Home,
        href: "/",
      },
      {
        title: "Robot Control",
        icon: Bot,
        href: "/control",
      },
      {
        title: "Sequences",
        icon: Layers,
        href: "/sequences",
        badge: "New",
      },
      {
        title: "Robotic Arm",
        icon: Activity,
        href: "/robotic-arm",
      },
      {
        title: "Holonomic Drive",
        icon: Circle,
        href: "/holonomic-drive",
      },
    ],
  },
  {
    title: "Monitoring",
    items: [
      {
        title: "System Monitor",
        icon: BarChart3,
        href: "/system-monitoring",
        badge: "Live",
      },
      {
        title: "Terminal",
        icon: Terminal,
        href: "/terminal",
      },
      {
        title: "Cameras",
        icon: Camera,
        href: "/cameras",
      },
      {
        title: "Sensors",
        icon: Eye,
        href: "/sensors",
      },
    ],
  },
  {
    title: "Navigation",
    items: [
      {
        title: "Navigation",
        icon: Navigation,
        href: "/navigation",
      },
      {
        title: "Map Viewer",
        icon: Map,
        href: "/map-viewer",
      },
      {
        title: "Virtual RViz",
        icon: Sparkles,
        href: "/virtual-rviz",
      },
    ],
  },
  {
    title: "ROS System",
    items: [
      {
        title: "Nodes",
        icon: Network,
        href: "/nodes",
      },
      {
        title: "Topics",
        icon: Globe,
        href: "/topics",
      },
      {
        title: "Services",
        icon: Wifi,
        href: "/services",
      },
      {
        title: "Parameters",
        icon: Settings,
        href: "/parameters",
      },
    ],
  },
  {
    title: "Development",
    items: [
      {
        title: "Code Development",
        icon: Code,
        href: "/code-development",
      },
      {
        title: "Python Development",
        icon: Cpu,
        href: "/python-development",
      },
      {
        title: "API Management",
        icon: Database,
        href: "/api-management",
      },
    ],
  },
  {
    title: "Configuration",
    items: [
      {
        title: "System Config",
        icon: Cpu,
        href: "/system-configuration",
      },
      {
        title: "Settings",
        icon: Settings,
        href: "/settings",
      },
      {
        title: "About",
        icon: HelpCircle,
        href: "/about",
      },
    ],
  },
];

interface FlatSidebarProps {
  isOpen: boolean;
  onClose: () => void;
}

export const FlatSidebar: React.FC<FlatSidebarProps> = ({
  isOpen,
  onClose,
}) => {
  const location = useLocation();

  return (
    <>
      {/* Tesla-themed Sidebar */}
      <aside
        className={`fixed top-0 left-0 z-50 h-full bg-white/5 backdrop-blur-xl border-r border-white/20 shadow-2xl transform transition-all duration-300 ${
          isOpen ? "translate-x-0" : "-translate-x-full"
        } lg:relative lg:translate-x-0 w-[280px]`}
      >
        {/* Header */}
        <div className="flex items-center justify-between p-6 border-b border-white/10">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-xl flex items-center justify-center shadow-lg">
              <Zap className="h-6 w-6 text-white" />
            </div>
            <div>
              <h2 className="text-lg font-light text-white tracking-tight">Dino Core</h2>
              <p className="text-xs text-slate-300 font-light">Robot Control</p>
            </div>
          </div>
          
          {/* Close button for mobile */}
          <button
            onClick={onClose}
            className="lg:hidden p-2 rounded-lg bg-white/10 hover:bg-white/20 transition-colors"
          >
            <X className="h-5 w-5 text-white" />
          </button>
        </div>

        {/* Navigation */}
        <ScrollArea className="flex-1 px-4 py-6">
          <nav className="space-y-8">
            {navigationSections.map((section, sectionIndex) => (
              <div key={section.title}>
                <h3 className="px-3 text-xs font-medium text-slate-400 uppercase tracking-wider mb-4">
                  {section.title}
                </h3>
                <ul className="space-y-2">
                  {section.items.map((item) => {
                    const isActive = location.pathname === item.href;
                    const Icon = item.icon;

                    return (
                      <li key={item.href}>
                        <Link
                          to={item.href}
                          onClick={() => window.innerWidth < 1024 && onClose()}
                          className={`group flex items-center gap-4 px-4 py-3 rounded-xl transition-all duration-300 ${
                            isActive
                              ? "bg-white/15 text-white shadow-lg border border-white/30 backdrop-blur-sm"
                              : "text-slate-300 hover:bg-white/10 hover:text-white hover:border hover:border-white/20"
                          }`}
                        >
                          {/* Icon Container */}
                          <div
                            className={`p-2 rounded-lg backdrop-blur-sm transition-all duration-300 ${
                              isActive
                                ? "bg-gradient-to-br from-blue-500/30 to-cyan-500/30 border border-white/20"
                                : "bg-white/10 border border-white/10 group-hover:bg-white/15"
                            }`}
                          >
                            <Icon
                              className={`h-5 w-5 transition-colors duration-300 ${
                                isActive
                                  ? "text-white"
                                  : "text-slate-400 group-hover:text-white"
                              }`}
                            />
                          </div>

                          {/* Text Content */}
                          <div className="flex-1 min-w-0">
                            <div className="flex items-center justify-between">
                              <span className="font-medium text-sm truncate">
                                {item.title}
                              </span>
                              {item.badge && (
                                <Badge className="ml-2 text-xs bg-gradient-to-r from-blue-500 to-cyan-500 text-white border-0">
                                  {item.badge}
                                </Badge>
                              )}
                            </div>
                          </div>

                          {/* Active Indicator */}
                          {isActive && (
                            <div className="w-1 h-6 bg-gradient-to-b from-blue-400 to-cyan-400 rounded-full" />
                          )}
                        </Link>
                      </li>
                    );
                  })}
                </ul>
              </div>
            ))}
          </nav>
        </ScrollArea>

        {/* Footer */}
        <div className="p-4 border-t border-white/10">
          <div className="bg-white/5 rounded-xl p-4 border border-white/10">
            <div className="flex items-center gap-3">
              <div className="w-3 h-3 bg-emerald-500 rounded-full animate-pulse"></div>
              <div>
                <p className="text-sm font-medium text-white">System Online</p>
                <p className="text-xs text-slate-400">All systems operational</p>
              </div>
            </div>
          </div>
        </div>
      </aside>
    </>
  );
};
