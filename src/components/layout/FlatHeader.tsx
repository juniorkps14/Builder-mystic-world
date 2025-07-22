import React from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { PersistenceStatus } from "@/components/ui/persistence-status";
import { useLanguage } from "@/contexts/LanguageContext";
import { useTheme } from "@/contexts/ThemeContext";
import { useROSIntegration } from "@/services/ROSIntegrationService";
import {
  Sun,
  Moon,
  Globe,
  Wifi,
  WifiOff,
  Activity,
  Cpu,
  HardDrive,
  Menu,
  User,
  Settings,
  Bell,
  Search,
} from "lucide-react";
import "../../styles/flat-vector-theme.css";

interface FlatHeaderProps {
  onMenuClick?: () => void;
}

export const FlatHeader: React.FC<FlatHeaderProps> = ({ onMenuClick }) => {
  const { language, setLanguage } = useLanguage();
  const { theme, setTheme } = useTheme();
  const { isConnected } = useROSIntegration();

  // Mock system data
  const systemData = {
    cpu: 45,
    memory: 68,
    disk: 73,
    temp: 42,
  };

  return (
    <header className="flat-header">
      {/* Left Section */}
      <div className="flex items-center gap-4">
        <Button
          variant="ghost"
          size="sm"
          onClick={onMenuClick}
          className="flat-button-outline lg:hidden"
        >
          <Menu className="flat-icon" />
        </Button>

        <div className="flex items-center gap-3">
          <div className="w-12 h-12 bg-gradient-to-br from-blue-500 to-purple-600 rounded-xl flex items-center justify-center">
            <Activity className="w-6 h-6 text-white" />
          </div>
          <div className="hidden sm:block">
            <h1 className="text-xl font-bold text-gray-900">Dino Core</h1>
            <p className="text-sm text-gray-500">Robot Control System</p>
          </div>
        </div>
      </div>

      {/* Center Section - System Status */}
      <div className="hidden md:flex items-center gap-4">
        <div className="flex items-center gap-3 bg-gray-50 rounded-full px-4 py-2">
          <div className="flex items-center gap-2">
            <Cpu className="w-4 h-4 text-blue-500" />
            <span className="text-sm font-medium">CPU</span>
            <span className="text-sm text-gray-600">{systemData.cpu}%</span>
          </div>
          <div className="w-1 h-4 bg-gray-300 rounded-full"></div>
          <div className="flex items-center gap-2">
            <HardDrive className="w-4 h-4 text-green-500" />
            <span className="text-sm font-medium">RAM</span>
            <span className="text-sm text-gray-600">{systemData.memory}%</span>
          </div>
        </div>

        {/* ROS Connection Status */}
        <div
          className={`flex items-center gap-2 px-3 py-2 rounded-full ${
            isConnected
              ? "bg-green-100 text-green-700"
              : "bg-red-100 text-red-700"
          }`}
        >
          {isConnected ? (
            <Wifi className="w-4 h-4" />
          ) : (
            <WifiOff className="w-4 h-4" />
          )}
          <span className="text-sm font-medium">
            {isConnected ? "Connected" : "Disconnected"}
          </span>
        </div>
      </div>

      {/* Right Section */}
      <div className="flex items-center gap-3">
        {/* Search Button */}
        <Button variant="ghost" size="sm" className="hidden sm:flex">
          <Search className="flat-icon" />
        </Button>

        {/* Notifications */}
        <Button variant="ghost" size="sm" className="relative">
          <Bell className="flat-icon" />
          <div className="absolute -top-1 -right-1 w-3 h-3 bg-red-500 rounded-full"></div>
        </Button>

        {/* Theme Toggle */}
        <Button
          variant="ghost"
          size="sm"
          onClick={() => setTheme(theme === "light" ? "dark" : "light")}
          className="flat-button-outline"
        >
          {theme === "light" ? (
            <Moon className="flat-icon" />
          ) : (
            <Sun className="flat-icon" />
          )}
        </Button>

        {/* Language Toggle */}
        <Button
          variant="ghost"
          size="sm"
          onClick={() => setLanguage(language === "th" ? "en" : "th")}
          className="flat-button-outline"
        >
          <Globe className="flat-icon" />
          <span className="hidden sm:inline ml-2">
            {language === "th" ? "EN" : "TH"}
          </span>
        </Button>

        {/* System Info */}
        <div className="flex items-center gap-3 border-l pl-3 ml-3">
          <div className="hidden sm:block text-right">
            <p className="text-sm font-medium text-gray-900">System Ready</p>
            <p className="text-xs text-gray-500">All Systems Operational</p>
          </div>
          <div className="w-8 h-8 bg-gradient-to-br from-green-500 to-teal-500 rounded-full flex items-center justify-center">
            <Settings className="w-4 h-4 text-white" />
          </div>
        </div>
      </div>
    </header>
  );
};
