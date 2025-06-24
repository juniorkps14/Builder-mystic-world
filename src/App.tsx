import { Toaster } from "@/components/ui/toaster";
import { Toaster as Sonner } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Header } from "@/components/layout/Header";
import { Sidebar } from "@/components/layout/Sidebar";
import { LanguageProvider } from "@/contexts/LanguageContext";
import { ThemeProvider } from "@/contexts/ThemeContext";
import { ROSIntegrationProvider } from "@/services/ROSIntegrationService";
import Index from "./pages/Index";
import RobotControl from "./pages/RobotControl";
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
import SystemConfiguration from "./pages/SystemConfiguration";
import NotFound from "./pages/NotFound";

const queryClient = new QueryClient();

const App = () => (
  <QueryClientProvider client={queryClient}>
    <ThemeProvider>
      <LanguageProvider>
        <ROSIntegrationProvider>
          <TooltipProvider>
            <Toaster />
            <Sonner />
            <BrowserRouter>
              <div className="min-h-screen bg-background text-foreground">
                <div className="flex h-screen">
                  <Sidebar />
                  <div className="flex-1 flex flex-col overflow-hidden">
                    <Header />
                    <main className="flex-1 overflow-y-auto p-6">
                      <Routes>
                        <Route path="/" element={<Index />} />
                        <Route path="/control" element={<RobotControl />} />
                        <Route path="/sequences" element={<Sequences />} />
                        <Route path="/robotic-arm" element={<RoboticArm />} />
                        <Route
                          path="/holonomic-drive"
                          element={<HolonomicDrive />}
                        />
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
                        {/* ADD ALL CUSTOM ROUTES ABOVE THE CATCH-ALL "*" ROUTE */}
                        <Route path="*" element={<NotFound />} />
                      </Routes>
                    </main>
                  </div>
                </div>
              </div>
            </BrowserRouter>
          </TooltipProvider>
        </ROSIntegrationProvider>
      </LanguageProvider>
    </ThemeProvider>
  </QueryClientProvider>
);

export default App;
