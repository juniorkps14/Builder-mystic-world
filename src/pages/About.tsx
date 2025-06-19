import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import {
  Bot,
  Cpu,
  Heart,
  Code,
  Users,
  Star,
  Github,
  Mail,
  Calendar,
} from "lucide-react";

const About = () => {
  const features = [
    {
      icon: Bot,
      title: "AI-Powered Interface",
      description: "Intelligent automation and smart decision making",
    },
    {
      icon: Cpu,
      title: "ROS Integration",
      description: "Seamless Robot Operating System connectivity",
    },
    {
      icon: Code,
      title: "Modern Architecture",
      description: "Built with React, TypeScript, and cutting-edge tools",
    },
    {
      icon: Heart,
      title: "User-Centric Design",
      description: "Intuitive interface designed for operators",
    },
  ];

  const teamMembers = [
    {
      name: "พิฆเนศ แสงสะเดาะ",
      role: "Lead Developer",
      description: "Main developer and architect of Dino Core",
      avatar: "🦕",
    },
    // Placeholder for future team members
    {
      name: "รอเพิ่มรายชื่อทีมงาน",
      role: "Team Member",
      description: "More team members will be added here",
      avatar: "👤",
      placeholder: true,
    },
  ];

  return (
    <div className="space-y-8">
      {/* Header */}
      <div className="text-center">
        <div className="flex items-center justify-center gap-3 mb-4">
          <div className="w-16 h-16 rounded-2xl bg-primary flex items-center justify-center">
            <Bot className="h-8 w-8 text-primary-foreground" />
          </div>
          <div>
            <h1 className="text-4xl font-bold">Dino Core</h1>
            <p className="text-xl text-muted-foreground">
              ROS Web Control Interface
            </p>
          </div>
        </div>
        <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
          A modern, AI-powered web interface for Robot Operating System (ROS)
          control and monitoring. Built to simplify complex robotics operations
          through an intuitive and powerful interface.
        </p>
      </div>

      {/* Creation Info */}
      <Card className="p-6">
        <div className="flex items-center gap-3 mb-4">
          <Star className="h-6 w-6 text-yellow-500" />
          <h2 className="text-2xl font-bold">About This Project</h2>
        </div>
        <div className="space-y-4">
          <div className="flex items-center gap-3">
            <Bot className="h-5 w-5 text-primary" />
            <span className="text-lg">
              <strong>Created by AI</strong> - Designed and generated using
              artificial intelligence
            </span>
          </div>
          <div className="flex items-center gap-3">
            <Code className="h-5 w-5 text-accent" />
            <span className="text-lg">
              <strong>Developed by พิฆเนศ แสงสะเดาะ</strong> - Lead developer
              and project architect
            </span>
          </div>
          <div className="flex items-center gap-3">
            <Calendar className="h-5 w-5 text-muted-foreground" />
            <span className="text-lg">
              <strong>Status:</strong> Active development - continuously
              evolving
            </span>
          </div>
        </div>
      </Card>

      {/* Features */}
      <div>
        <h2 className="text-2xl font-bold mb-6">Key Features</h2>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          {features.map((feature, index) => (
            <Card key={index} className="p-6">
              <div className="flex items-start gap-4">
                <div className="p-3 rounded-lg bg-primary/10">
                  <feature.icon className="h-6 w-6 text-primary" />
                </div>
                <div>
                  <h3 className="font-semibold text-lg mb-2">
                    {feature.title}
                  </h3>
                  <p className="text-muted-foreground">{feature.description}</p>
                </div>
              </div>
            </Card>
          ))}
        </div>
      </div>

      {/* Technology Stack */}
      <Card className="p-6">
        <h2 className="text-2xl font-bold mb-4">Technology Stack</h2>
        <div className="flex flex-wrap gap-2 mb-4">
          {[
            "React 18",
            "TypeScript",
            "Vite",
            "TailwindCSS",
            "Radix UI",
            "React Router",
            "Framer Motion",
            "Lucide Icons",
          ].map((tech, index) => (
            <Badge key={index} variant="outline">
              {tech}
            </Badge>
          ))}
        </div>
        <p className="text-muted-foreground">
          Built with modern web technologies to ensure performance,
          maintainability, and scalability.
        </p>
      </Card>

      {/* Team */}
      <div>
        <h2 className="text-2xl font-bold mb-6">Development Team</h2>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          {teamMembers.map((member, index) => (
            <Card
              key={index}
              className={`p-6 ${member.placeholder ? "border-dashed opacity-60" : ""}`}
            >
              <div className="flex items-start gap-4">
                <div className="w-12 h-12 rounded-full bg-primary/10 flex items-center justify-center text-2xl">
                  {member.avatar}
                </div>
                <div className="flex-1">
                  <h3 className="font-semibold text-lg">{member.name}</h3>
                  <Badge variant="outline" className="mb-2">
                    {member.role}
                  </Badge>
                  <p className="text-muted-foreground text-sm">
                    {member.description}
                  </p>
                </div>
              </div>
            </Card>
          ))}
        </div>
      </div>

      {/* Contact */}
      <Card className="p-6">
        <h2 className="text-2xl font-bold mb-4">Get Involved</h2>
        <p className="text-muted-foreground mb-4">
          Interested in contributing to Dino Core or have questions about the
          project? We'd love to hear from you!
        </p>
        <div className="flex flex-wrap gap-4">
          <div className="flex items-center gap-2">
            <Github className="h-4 w-4" />
            <span className="text-sm">GitHub repository coming soon</span>
          </div>
          <div className="flex items-center gap-2">
            <Mail className="h-4 w-4" />
            <span className="text-sm">Contact information will be added</span>
          </div>
        </div>
      </Card>

      {/* Version Info */}
      <Card className="p-4 bg-muted/30">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="w-8 h-8 rounded bg-primary flex items-center justify-center">
              <span className="text-xs font-bold text-primary-foreground">
                v1
              </span>
            </div>
            <div>
              <p className="font-medium">Dino Core v1.0.0</p>
              <p className="text-sm text-muted-foreground">
                Initial release with core ROS interface features
              </p>
            </div>
          </div>
          <Badge variant="default">Beta</Badge>
        </div>
      </Card>
    </div>
  );
};

export default About;
