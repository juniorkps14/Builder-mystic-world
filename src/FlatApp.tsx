import { Toaster } from "@/components/ui/toaster";
import { Toaster as Sonner } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { FlatHeader } from "@/components/layout/FlatHeader";
import { FlatSidebar } from "@/components/layout/FlatSidebar";
import { LanguageProvider } from "@/contexts/LanguageContext";
import { ThemeProvider } from "@/contexts/ThemeContext";
import { ROSIntegrationProvider } from "@/services/ROSIntegrationService";
import { useState } from "react";

// Import new flat pages
import FlatDashboard from "./pages/FlatDashboard";
import HolonomicRobotControl from "./pages/HolonomicRobotControl";

// Import existing pages (will be updated gradually)
import Sequences from "./pages/Sequences";
import RoboticArm from "./pages/RoboticArm";
import HolonomicDrive from "./pages/HolonomicDrive";
import IOConfiguration from "./pages/IOConfiguration";
import FeatureManagement from "./pages/FeatureManagement";
import TFTree from "./pages/TFTree";
import MapViewer from "./pages/MapViewer";
import VirtualRviz from "./pages/VirtualRviz";
import Sensors from "./pages/Sensors";
import System from "./pages/System";
import Cameras from "./pages/Cameras";
import Navigation from "./pages/Navigation";
import Nodes from "./pages/Nodes";
import Topics from "./pages/Topics";
import Services from "./pages/Services";
import Parameters from "./pages/Parameters";
import Logs from "./pages/Logs";
import Terminal from "./pages/Terminal";
import About from "./pages/About";
import Settings from "./pages/Settings";
import MicrocontrollerConnections from "./pages/MicrocontrollerConnections";
import CodeDevelopment from "./pages/CodeDevelopment";
import LogViewer from "./pages/LogViewer";
import ROSTerminal from "./pages/ROSTerminal";
import SystemTerminal from "./pages/SystemTerminal";
import SystemConfiguration from "./pages/SystemConfiguration";
import ROSSetup from "./pages/ROSSetup";
import NotFound from "./pages/NotFound";

import "./styles/flat-vector-theme.css";

const queryClient = new QueryClient();

const FlatApp = () => {
  const [sidebarOpen, setSidebarOpen] = useState(false);

  return (
    <QueryClientProvider client={queryClient}>
      <ThemeProvider>
        <LanguageProvider>
          <ROSIntegrationProvider>
            <TooltipProvider>
              <Toaster />
              <Sonner />
              <BrowserRouter>
                <div className="min-h-screen bg-gray-50 flex">
                  {/* Sidebar */}
                  <FlatSidebar
                    isOpen={sidebarOpen}
                    onClose={() => setSidebarOpen(false)}
                  />

                  {/* Mobile Overlay */}
                  {sidebarOpen && (
                    <div
                      className="fixed inset-0 bg-black/50 z-40 lg:hidden"
                      onClick={() => setSidebarOpen(false)}
                    />
                  )}

                  {/* Main Content */}
                  <div className="flex-1 flex flex-col min-h-screen lg:ml-0">
                    {/* Header */}
                    <FlatHeader onMenuClick={() => setSidebarOpen(true)} />

                    {/* Page Content */}
                    <main className="flex-1 overflow-auto">
                      <Routes>
                        {/* New Flat Vector Pages */}
                        <Route path="/" element={<FlatDashboard />} />
                        <Route path="/control" element={<FlatRobotControl />} />

                        {/* Existing Pages (keeping for now, will update gradually) */}
                        <Route path="/sequences" element={<Sequences />} />
                        <Route path="/robotic-arm" element={<RoboticArm />} />
                        <Route
                          path="/holonomic-drive"
                          element={<HolonomicDrive />}
                        />
                        <Route path="/ros-setup" element={<ROSSetup />} />
                        <Route
                          path="/io-config"
                          element={<IOConfiguration />}
                        />
                        <Route
                          path="/features"
                          element={<FeatureManagement />}
                        />
                        <Route path="/tf-tree" element={<TFTree />} />
                        <Route path="/map-viewer" element={<MapViewer />} />
                        <Route path="/virtual-rviz" element={<VirtualRviz />} />
                        <Route path="/cameras" element={<Cameras />} />
                        <Route path="/sensors" element={<Sensors />} />
                        <Route path="/navigation" element={<Navigation />} />
                        <Route path="/system" element={<System />} />
                        <Route path="/nodes" element={<Nodes />} />
                        <Route path="/topics" element={<Topics />} />
                        <Route path="/services" element={<Services />} />
                        <Route path="/parameters" element={<Parameters />} />
                        <Route path="/logs" element={<LogViewer />} />
                        <Route path="/terminal" element={<ROSTerminal />} />
                        <Route
                          path="/system-terminal"
                          element={<SystemTerminal />}
                        />
                        <Route
                          path="/microcontroller-connections"
                          element={<MicrocontrollerConnections />}
                        />
                        <Route
                          path="/code-development"
                          element={<CodeDevelopment />}
                        />
                        <Route
                          path="/system-configuration"
                          element={<SystemConfiguration />}
                        />
                        <Route path="/about" element={<About />} />
                        <Route path="/settings" element={<Settings />} />
                        <Route path="*" element={<NotFound />} />
                      </Routes>
                    </main>

                    {/* Footer */}
                    <footer className="bg-white border-t border-gray-200 px-6 py-4">
                      <div className="flex items-center justify-between text-sm text-gray-500">
                        <div>
                          © 2024 Dino Core Robot Control System. Made with ❤️
                          for robotics.
                        </div>
                        <div className="flex items-center gap-4">
                          <span>Version 2.0.0</span>
                          <span>•</span>
                          <span>Status: All systems operational</span>
                        </div>
                      </div>
                    </footer>
                  </div>
                </div>
              </BrowserRouter>
            </TooltipProvider>
          </ROSIntegrationProvider>
        </LanguageProvider>
      </ThemeProvider>
    </QueryClientProvider>
  );
};

export default FlatApp;
