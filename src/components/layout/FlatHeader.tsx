import React from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { PersistenceStatus } from "@/components/ui/persistence-status";
import { TeslaThemeToggle } from "@/components/ui/tesla-theme-toggle";
import { useTheme } from "@/contexts/ThemeContext";
import { useROSIntegration } from "@/services/ROSIntegrationService";
import {
  Sun,
  Moon,
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
  Zap,
  Battery,
} from "lucide-react";

interface FlatHeaderProps {
  onMenuClick?: () => void;
}

export const FlatHeader: React.FC<FlatHeaderProps> = ({ onMenuClick }) => {
  const { theme, setTheme } = useTheme();
  const { isConnected } = useROSIntegration();

  // Mock system data
  const systemData = {
    cpu: 45,
    memory: 68,
    disk: 73,
    temp: 42,
    battery: 87,
  };

  return (
    <header className="bg-white/5 backdrop-blur-xl border-b border-white/20 px-6 py-4 shadow-lg">
      <div className="flex items-center justify-between">
        {/* Left Section */}
        <div className="flex items-center gap-4">
          {/* Mobile Menu Button */}
          <Button
            variant="ghost"
            size="sm"
            onClick={onMenuClick}
            className="lg:hidden p-2 hover:bg-white/10 text-white border border-white/20 rounded-lg"
          >
            <Menu className="h-5 w-5" />
          </Button>

          {/* System Metrics */}
          <div className="hidden md:flex items-center gap-4">
            <div className="flex items-center gap-2 px-3 py-2 bg-white/10 rounded-lg border border-white/10">
              <Cpu className="h-4 w-4 text-blue-400" />
              <span className="text-sm font-mono text-white">{systemData.cpu}%</span>
            </div>
            <div className="flex items-center gap-2 px-3 py-2 bg-white/10 rounded-lg border border-white/10">
              <HardDrive className="h-4 w-4 text-purple-400" />
              <span className="text-sm font-mono text-white">{systemData.memory}%</span>
            </div>
            <div className="flex items-center gap-2 px-3 py-2 bg-white/10 rounded-lg border border-white/10">
              <Battery className="h-4 w-4 text-green-400" />
              <span className="text-sm font-mono text-white">{systemData.battery}%</span>
            </div>
          </div>

          {/* ROS Connection Status */}
          <div
            className={`flex items-center gap-2 px-3 py-2 rounded-lg border ${
              isConnected
                ? "bg-emerald-500/20 text-emerald-300 border-emerald-500/30"
                : "bg-red-500/20 text-red-300 border-red-500/30"
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

        {/* Center Section - Time */}
        <div className="hidden lg:block">
          <div className="text-center">
            <p className="text-lg font-light text-white">
              {new Date().toLocaleTimeString('en-US', { 
                hour12: false,
                hour: '2-digit',
                minute: '2-digit'
              })}
            </p>
            <p className="text-xs text-slate-400">
              {new Date().toLocaleDateString('en-US', {
                weekday: 'short',
                month: 'short',
                day: 'numeric'
              })}
            </p>
          </div>
        </div>

        {/* Right Section */}
        <div className="flex items-center gap-3">
          {/* Search Button */}
          <Button 
            variant="ghost" 
            size="sm" 
            className="hidden sm:flex bg-white/10 hover:bg-white/20 border border-white/20 text-white"
          >
            <Search className="h-4 w-4" />
          </Button>

          {/* Notifications */}
          <Button 
            variant="ghost" 
            size="sm" 
            className="relative bg-white/10 hover:bg-white/20 border border-white/20 text-white"
          >
            <Bell className="h-4 w-4" />
            <div className="absolute -top-1 -right-1 w-3 h-3 bg-gradient-to-r from-red-500 to-pink-500 rounded-full"></div>
          </Button>

          {/* Theme Toggle */}
          <Button
            variant="ghost"
            size="sm"
            onClick={() => setTheme(theme === "light" ? "dark" : "light")}
            className="bg-white/10 hover:bg-white/20 border border-white/20 text-white"
          >
            {theme === "light" ? (
              <Moon className="h-4 w-4" />
            ) : (
              <Sun className="h-4 w-4" />
            )}
          </Button>

          {/* Persistence Status */}
          <PersistenceStatus />

          {/* Tesla Theme Toggle */}
          <TeslaThemeToggle />

          {/* User Profile */}
          <div className="flex items-center gap-3 border-l border-white/20 pl-3 ml-3">
            <div className="hidden sm:block text-right">
              <p className="text-sm font-medium text-white">Robot Admin</p>
              <p className="text-xs text-slate-400">System Ready</p>
            </div>
            <div className="w-10 h-10 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-xl flex items-center justify-center shadow-lg">
              <User className="w-5 h-5 text-white" />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
};
