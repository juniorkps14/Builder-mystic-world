import React from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import {
  Bot,
  Zap,
  Heart,
  Github,
  Globe,
  Mail,
  Shield,
  Users,
  Award,
  Rocket,
  Code,
  Database,
  Cpu,
  Activity,
  ExternalLink,
  Download,
  Star,
  GitBranch,
} from "lucide-react";

export default function About() {
  const features = [
    {
      icon: Bot,
      title: "Advanced Robotics Control",
      description:
        "Comprehensive robot control system with real-time monitoring and autonomous navigation capabilities.",
    },
    {
      icon: Zap,
      title: "High Performance",
      description:
        "Optimized for real-time operations with minimal latency and maximum reliability.",
    },
    {
      icon: Shield,
      title: "Enterprise Security",
      description:
        "Built with security-first principles and enterprise-grade authentication systems.",
    },
    {
      icon: Code,
      title: "Open Architecture",
      description:
        "Modular design with extensive API support and plugin architecture for customization.",
    },
    {
      icon: Database,
      title: "Data Intelligence",
      description:
        "Advanced analytics and machine learning integration for intelligent decision making.",
    },
    {
      icon: Activity,
      title: "Real-time Monitoring",
      description:
        "Comprehensive monitoring suite with predictive maintenance and diagnostic capabilities.",
    },
  ];

  const teamMembers = [
    {
      name: "Robotics Team",
      role: "Core Development",
      contribution: "System architecture and robotics integration",
    },
    {
      name: "AI Research Team",
      role: "Machine Learning",
      contribution: "Autonomous navigation and computer vision",
    },
    {
      name: "Security Team",
      role: "Cybersecurity",
      contribution: "System security and data protection",
    },
  ];

  const technologies = [
    { name: "React", version: "18.0+", type: "Frontend" },
    { name: "TypeScript", version: "5.0+", type: "Language" },
    { name: "ROS", version: "Noetic/Humble", type: "Robotics" },
    { name: "Node.js", version: "18.0+", type: "Backend" },
    { name: "Python", version: "3.8+", type: "AI/ML" },
    { name: "WebRTC", version: "Latest", type: "Streaming" },
  ];

  const statistics = [
    { label: "Lines of Code", value: "50,000+", icon: Code },
    { label: "Test Coverage", value: "95%", icon: Shield },
    { label: "Performance Score", value: "98/100", icon: Zap },
    { label: "Uptime", value: "99.9%", icon: Activity },
  ];

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
      {/* Tesla-inspired Header */}
      <div className="mb-8">
        <div className="bg-white/10 backdrop-blur-xl border border-white/20 rounded-3xl p-8 shadow-2xl text-center">
          <div className="flex justify-center mb-6">
            <div className="w-20 h-20 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-2xl flex items-center justify-center shadow-2xl">
              <Bot className="h-10 w-10 text-white" />
            </div>
          </div>
          <h1 className="text-5xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent mb-4">
            Dino Core
          </h1>
          <p className="text-xl text-slate-300 font-light mb-6 max-w-2xl mx-auto">
            Advanced Robot Control System for Next-Generation Autonomous
            Operations
          </p>
          <div className="flex items-center justify-center gap-4 mb-6">
            <Badge className="bg-gradient-to-r from-emerald-500 to-teal-500 text-white border-0 px-4 py-2">
              Version 2.0.0
            </Badge>
            <Badge className="bg-white/20 text-white border border-white/30 px-4 py-2">
              Production Ready
            </Badge>
          </div>

          <div className="flex justify-center gap-4">
            <Button className="bg-gradient-to-r from-blue-500 to-cyan-500 hover:from-blue-600 hover:to-cyan-600 text-white shadow-lg">
              <Github className="h-4 w-4 mr-2" />
              View Source
              <ExternalLink className="h-4 w-4 ml-2" />
            </Button>
            <Button className="bg-white/10 hover:bg-white/20 border border-white/20 text-white">
              <Download className="h-4 w-4 mr-2" />
              Download
            </Button>
          </div>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        {statistics.map((stat, index) => {
          const IconComponent = stat.icon;
          return (
            <Card
              key={index}
              className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300"
            >
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-slate-400 text-sm">{stat.label}</p>
                  <p className="text-2xl font-light text-white mt-1">
                    {stat.value}
                  </p>
                </div>
                <div className="h-12 w-12 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center">
                  <IconComponent className="h-6 w-6 text-blue-400" />
                </div>
              </div>
            </Card>
          );
        })}
      </div>

      {/* Features Grid */}
      <div className="mb-8">
        <h2 className="text-3xl font-light text-white mb-6 text-center">
          Key Features
        </h2>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
          {features.map((feature, index) => {
            const IconComponent = feature.icon;
            return (
              <Card
                key={index}
                className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl hover:bg-white/15 transition-all duration-300"
              >
                <div className="flex items-center gap-4 mb-4">
                  <div className="p-3 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl border border-white/20">
                    <IconComponent className="h-6 w-6 text-blue-400" />
                  </div>
                  <h3 className="text-lg font-medium text-white">
                    {feature.title}
                  </h3>
                </div>
                <p className="text-slate-300 text-sm leading-relaxed">
                  {feature.description}
                </p>
              </Card>
            );
          })}
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-8 mb-8">
        {/* Technology Stack */}
        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl">
          <h3 className="text-xl font-light text-white mb-6 flex items-center gap-2">
            <Cpu className="h-5 w-5 text-blue-400" />
            Technology Stack
          </h3>
          <div className="space-y-4">
            {technologies.map((tech, index) => (
              <div
                key={index}
                className="flex items-center justify-between p-3 bg-white/5 rounded-lg border border-white/10"
              >
                <div>
                  <p className="text-white font-medium">{tech.name}</p>
                  <p className="text-slate-400 text-sm">{tech.type}</p>
                </div>
                <Badge className="bg-white/20 text-white border border-white/30">
                  {tech.version}
                </Badge>
              </div>
            ))}
          </div>
        </Card>

        {/* Team */}
        <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-6 shadow-xl">
          <h3 className="text-xl font-light text-white mb-6 flex items-center gap-2">
            <Users className="h-5 w-5 text-blue-400" />
            Development Team
          </h3>
          <div className="space-y-4">
            {teamMembers.map((member, index) => (
              <div
                key={index}
                className="p-4 bg-white/5 rounded-lg border border-white/10"
              >
                <div className="flex items-center gap-3 mb-2">
                  <div className="w-10 h-10 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-lg flex items-center justify-center">
                    <Users className="h-5 w-5 text-white" />
                  </div>
                  <div>
                    <p className="text-white font-medium">{member.name}</p>
                    <p className="text-slate-400 text-sm">{member.role}</p>
                  </div>
                </div>
                <p className="text-slate-300 text-sm">{member.contribution}</p>
              </div>
            ))}
          </div>
        </Card>
      </div>

      {/* System Information */}
      <Card className="bg-white/10 backdrop-blur-xl border border-white/20 p-8 shadow-xl">
        <h3 className="text-2xl font-light text-white mb-6 text-center">
          System Information
        </h3>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
          <div className="text-center">
            <div className="w-16 h-16 bg-gradient-to-br from-emerald-500/20 to-teal-500/20 rounded-xl flex items-center justify-center mx-auto mb-3 border border-white/20">
              <Rocket className="h-8 w-8 text-emerald-400" />
            </div>
            <h4 className="text-white font-medium mb-2">Release</h4>
            <p className="text-slate-300 text-sm">Stable Production Build</p>
            <p className="text-slate-400 text-xs mt-1">v2.0.0 (2024)</p>
          </div>

          <div className="text-center">
            <div className="w-16 h-16 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center mx-auto mb-3 border border-white/20">
              <GitBranch className="h-8 w-8 text-blue-400" />
            </div>
            <h4 className="text-white font-medium mb-2">License</h4>
            <p className="text-slate-300 text-sm">MIT Open Source</p>
            <p className="text-slate-400 text-xs mt-1">
              Free for commercial use
            </p>
          </div>

          <div className="text-center">
            <div className="w-16 h-16 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center mx-auto mb-3 border border-white/20">
              <Globe className="h-8 w-8 text-purple-400" />
            </div>
            <h4 className="text-white font-medium mb-2">Platform</h4>
            <p className="text-slate-300 text-sm">Cross-platform Web</p>
            <p className="text-slate-400 text-xs mt-1">Works everywhere</p>
          </div>

          <div className="text-center">
            <div className="w-16 h-16 bg-gradient-to-br from-orange-500/20 to-red-500/20 rounded-xl flex items-center justify-center mx-auto mb-3 border border-white/20">
              <Heart className="h-8 w-8 text-orange-400" />
            </div>
            <h4 className="text-white font-medium mb-2">Support</h4>
            <p className="text-slate-300 text-sm">Community Driven</p>
            <p className="text-slate-400 text-xs mt-1">24/7 Documentation</p>
          </div>
        </div>

        {/* Contact */}
        <div className="mt-8 pt-6 border-t border-white/10 text-center">
          <p className="text-slate-300 mb-4">
            Built with <Heart className="inline h-4 w-4 text-red-400 mx-1" />{" "}
            for the robotics community
          </p>
          <div className="flex justify-center gap-4">
            <Button className="bg-white/10 hover:bg-white/20 border border-white/20 text-white">
              <Mail className="h-4 w-4 mr-2" />
              Contact Support
            </Button>
            <Button className="bg-white/10 hover:bg-white/20 border border-white/20 text-white">
              <Github className="h-4 w-4 mr-2" />
              Contribute
            </Button>
            <Button className="bg-white/10 hover:bg-white/20 border border-white/20 text-white">
              <Star className="h-4 w-4 mr-2" />
              Star on GitHub
            </Button>
          </div>
        </div>
      </Card>
    </div>
  );
}
