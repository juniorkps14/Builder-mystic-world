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
} from "lucide-react";
import "../../styles/flat-vector-theme.css";

interface NavItem {
  title: string;
  icon: React.ComponentType<any>;
  href: string;
  badge?: string;
  color: string;
  bgColor: string;
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
        color: "text-blue-600",
        bgColor: "bg-blue-100",
      },
      {
        title: "Robot Control",
        icon: Bot,
        href: "/control",
        color: "text-green-600",
        bgColor: "bg-green-100",
      },
      {
        title: "Sequences",
        icon: Layers,
        href: "/sequences",
        badge: "New",
        color: "text-purple-600",
        bgColor: "bg-purple-100",
      },
      {
        title: "Robotic Arm",
        icon: Activity,
        href: "/robotic-arm",
        color: "text-orange-600",
        bgColor: "bg-orange-100",
      },
      {
        title: "Holonomic Drive",
        icon: Circle,
        href: "/holonomic-drive",
        color: "text-cyan-600",
        bgColor: "bg-cyan-100",
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
        color: "text-slate-600",
        bgColor: "bg-slate-100",
      },
      {
        title: "Terminal",
        icon: Terminal,
        href: "/terminal",
        badge: "Multi",
        color: "text-green-600",
        bgColor: "bg-green-100",
      },
      {
        title: "Cameras",
        icon: Eye,
        href: "/cameras",
        badge: "Live",
        color: "text-red-600",
        bgColor: "bg-red-100",
      },
      {
        title: "Sensors",
        icon: Compass,
        href: "/sensors",
        badge: "8",
        color: "text-indigo-600",
        bgColor: "bg-indigo-100",
      },
      {
        title: "Navigation",
        icon: Map,
        href: "/navigation",
        color: "text-emerald-600",
        bgColor: "bg-emerald-100",
      },
    ],
  },
  {
    title: "ROS System",
    items: [
      {
        title: "ROS Setup",
        icon: Terminal,
        href: "/ros-setup",
        badge: "Guide",
        color: "text-orange-600",
        bgColor: "bg-orange-100",
      },
      {
        title: "I/O Config",
        icon: Cpu,
        href: "/io-config",
        color: "text-violet-600",
        bgColor: "bg-violet-100",
      },
      {
        title: "Features",
        icon: Sparkles,
        href: "/features",
        badge: "Pro",
        color: "text-yellow-600",
        bgColor: "bg-yellow-100",
      },
      {
        title: "TF Tree",
        icon: GitBranch,
        href: "/tf-tree",
        badge: "Live",
        color: "text-teal-600",
        bgColor: "bg-teal-100",
      },
      {
        title: "Map Viewer",
        icon: Globe,
        href: "/map-viewer",
        color: "text-blue-600",
        bgColor: "bg-blue-100",
      },
      {
        title: "Virtual RViz",
        icon: Eye,
        href: "/virtual-rviz",
        badge: "3D",
        color: "text-purple-600",
        bgColor: "bg-purple-100",
      },
      {
        title: "Nodes",
        icon: Network,
        href: "/nodes",
        badge: "12",
        color: "text-green-600",
        bgColor: "bg-green-100",
      },
      {
        title: "Topics",
        icon: Wifi,
        href: "/topics",
        badge: "24",
        color: "text-blue-600",
        bgColor: "bg-blue-100",
      },
      {
        title: "Services",
        icon: Shield,
        href: "/services",
        badge: "8",
        color: "text-red-600",
        bgColor: "bg-red-100",
      },
      {
        title: "Python Dev Guide",
        icon: Code,
        href: "/python-development",
        badge: "Guide",
        color: "text-green-600",
        bgColor: "bg-green-100",
      },
    ],
  },
  {
    title: "Settings",
    items: [
      {
        title: "Settings",
        icon: Settings,
        href: "/settings",
        color: "text-gray-600",
        bgColor: "bg-gray-100",
      },
      {
        title: "System Config",
        icon: Cpu,
        href: "/system-configuration",
        badge: "Pro",
        color: "text-violet-600",
        bgColor: "bg-violet-100",
      },
      {
        title: "About",
        icon: HelpCircle,
        href: "/about",
        color: "text-gray-600",
        bgColor: "bg-gray-100",
      },
    ],
  },
];

interface FlatSidebarProps {
  isOpen?: boolean;
  onClose?: () => void;
}

export const FlatSidebar: React.FC<FlatSidebarProps> = ({
  isOpen = true,
  onClose,
}) => {
  const location = useLocation();

  return (
    <aside
      className={`flat-sidebar ${
        isOpen ? "translate-x-0" : "-translate-x-full"
      } transition-transform duration-300 ease-out lg:translate-x-0`}
    >
      <div className="p-6">
        {/* Logo Section */}
        <div className="flex items-center gap-3 mb-8">
          <div className="w-10 h-10 bg-gradient-to-br from-blue-500 to-purple-600 rounded-xl flex items-center justify-center">
            <Activity className="w-5 h-5 text-white" />
          </div>
          <div>
            <h2 className="font-bold text-lg text-gray-900">Dino Core</h2>
            <p className="text-xs text-gray-500">Robot Control</p>
          </div>
        </div>

        {/* Navigation Sections */}
        <ScrollArea className="h-[calc(100vh-180px)]">
          <div className="space-y-6">
            {navigationSections.map((section, sectionIndex) => (
              <div key={sectionIndex} className="space-y-2">
                <h3 className="text-xs font-semibold text-gray-400 uppercase tracking-wider px-3">
                  {section.title}
                </h3>
                <div className="space-y-1">
                  {section.items.map((item, itemIndex) => {
                    const IconComponent = item.icon;
                    const isActive = location.pathname === item.href;

                    return (
                      <Link
                        key={itemIndex}
                        to={item.href}
                        className={`flat-nav-item group ${
                          isActive ? "active" : ""
                        }`}
                        onClick={onClose}
                      >
                        <div
                          className={`p-2 rounded-lg ${
                            isActive ? "bg-white/20" : item.bgColor
                          }`}
                        >
                          <IconComponent
                            className={`flat-icon ${
                              isActive ? "text-white" : item.color
                            }`}
                          />
                        </div>
                        <span className="font-medium text-sm flex-1">
                          {item.title}
                        </span>
                        {item.badge && (
                          <Badge
                            className={`text-xs px-2 py-1 ${
                              isActive
                                ? "bg-white/20 text-white"
                                : "bg-white/80 text-gray-700"
                            }`}
                          >
                            {item.badge}
                          </Badge>
                        )}
                      </Link>
                    );
                  })}
                </div>
              </div>
            ))}
          </div>
        </ScrollArea>

        {/* Bottom Section */}
        <div className="mt-6 pt-6 border-t border-gray-200">
          <div className="flex items-center gap-3 p-3 bg-gradient-to-r from-blue-50 to-purple-50 rounded-lg">
            <div className="w-8 h-8 bg-gradient-to-br from-blue-500 to-purple-600 rounded-full flex items-center justify-center">
              <BookOpen className="w-4 h-4 text-white" />
            </div>
            <div className="flex-1">
              <p className="text-sm font-medium text-gray-900">Need Help?</p>
              <p className="text-xs text-gray-500">View documentation</p>
            </div>
          </div>
        </div>
      </div>
    </aside>
  );
};
