import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Switch } from "@/components/ui/switch";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Radio,
  Play,
  Pause,
  RefreshCw,
  Search,
  Settings,
  Activity,
  Eye,
  Download,
  Filter,
  BarChart3,
  MessageSquare,
  Clock,
  Zap,
  Link,
  Users,
} from "lucide-react";

interface ROSTopic {
  name: string;
  type: string;
  publishers: string[];
  subscribers: string[];
  messageCount: number;
  frequency: number;
  lastMessage: Date;
  bandwidth: number;
  isActive: boolean;
  messageSize: number;
  latency: number;
}

interface TopicMessage {
  timestamp: Date;
  data: any;
  size: number;
}

const Topics = () => {
  const { t } = useLanguage();

  const [topics, setTopics] = useState<ROSTopic[]>([
    {
      name: "/cmd_vel",
      type: "geometry_msgs/Twist",
      publishers: ["move_base"],
      subscribers: ["robot_driver"],
      messageCount: 15420,
      frequency: 10.5,
      lastMessage: new Date(),
      bandwidth: 1.2,
      isActive: true,
      messageSize: 48,
      latency: 5.2,
    },
    {
      name: "/scan",
      type: "sensor_msgs/LaserScan",
      publishers: ["lidar_node"],
      subscribers: ["move_base", "amcl", "scan_matcher"],
      messageCount: 45600,
      frequency: 20.0,
      lastMessage: new Date(),
      bandwidth: 15.8,
      isActive: true,
      messageSize: 2048,
      latency: 8.1,
    },
    {
      name: "/camera/image_raw",
      type: "sensor_msgs/Image",
      publishers: ["camera_node"],
      subscribers: ["image_processing"],
      messageCount: 8750,
      frequency: 30.0,
      lastMessage: new Date(),
      bandwidth: 125.4,
      isActive: true,
      messageSize: 4096,
      latency: 12.5,
    },
    {
      name: "/odom",
      type: "nav_msgs/Odometry",
      publishers: ["robot_localization"],
      subscribers: ["move_base", "amcl"],
      messageCount: 23450,
      frequency: 50.0,
      lastMessage: new Date(),
      bandwidth: 3.2,
      isActive: true,
      messageSize: 192,
      latency: 3.1,
    },
    {
      name: "/amcl_pose",
      type: "geometry_msgs/PoseWithCovarianceStamped",
      publishers: ["amcl"],
      subscribers: ["move_base", "rviz"],
      messageCount: 5230,
      frequency: 5.0,
      lastMessage: new Date(),
      bandwidth: 1.8,
      isActive: true,
      messageSize: 216,
      latency: 4.5,
    },
    {
      name: "/joint_states",
      type: "sensor_msgs/JointState",
      publishers: ["joint_state_publisher"],
      subscribers: ["robot_state_publisher"],
      messageCount: 0,
      frequency: 0,
      lastMessage: new Date(Date.now() - 300000),
      bandwidth: 0,
      isActive: false,
      messageSize: 128,
      latency: 0,
    },
    {
      name: "/tf",
      type: "tf2_msgs/TFMessage",
      publishers: ["robot_state_publisher", "amcl"],
      subscribers: ["move_base", "rviz"],
      messageCount: 89750,
      frequency: 100.0,
      lastMessage: new Date(),
      bandwidth: 25.6,
      isActive: true,
      messageSize: 256,
      latency: 2.8,
    },
    {
      name: "/map",
      type: "nav_msgs/OccupancyGrid",
      publishers: ["map_server"],
      subscribers: ["move_base", "rviz"],
      messageCount: 1,
      frequency: 0.1,
      lastMessage: new Date(Date.now() - 60000),
      bandwidth: 0.8,
      isActive: true,
      messageSize: 65536,
      latency: 15.2,
    },
  ]);

  const [selectedTopic, setSelectedTopic] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState("");
  const [typeFilter, setTypeFilter] = useState("all");
  const [isMonitoring, setIsMonitoring] = useState(true);
  const [showInactive, setShowInactive] = useState(true);
  const [messageHistory, setMessageHistory] = useState<TopicMessage[]>([]);

  // Simulate real-time updates
  useEffect(() => {
    if (!isMonitoring) return;

    const interval = setInterval(() => {
      setTopics((prev) =>
        prev.map((topic) => {
          if (topic.isActive) {
            const newCount = topic.messageCount + Math.floor(Math.random() * 5);
            const newFrequency = Math.max(
              0,
              topic.frequency + (Math.random() - 0.5) * 2,
            );

            return {
              ...topic,
              messageCount: newCount,
              frequency: newFrequency,
              lastMessage: new Date(),
              bandwidth: (newFrequency * topic.messageSize) / 1024, // KB/s
              latency: Math.max(1, topic.latency + (Math.random() - 0.5) * 2),
            };
          }
          return topic;
        }),
      );

      // Simulate message history for selected topic
      if (selectedTopic) {
        const topic = topics.find((t) => t.name === selectedTopic);
        if (topic && topic.isActive) {
          setMessageHistory((prev) => {
            const newMessage: TopicMessage = {
              timestamp: new Date(),
              data: generateMockMessage(topic.type),
              size: topic.messageSize,
            };
            return [newMessage, ...prev.slice(0, 49)]; // Keep last 50 messages
          });
        }
      }
    }, 1000);

    return () => clearInterval(interval);
  }, [isMonitoring, selectedTopic, topics]);

  const generateMockMessage = (type: string) => {
    switch (type) {
      case "geometry_msgs/Twist":
        return {
          linear: { x: (Math.random() - 0.5) * 2, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: (Math.random() - 0.5) * 1 },
        };
      case "sensor_msgs/LaserScan":
        return {
          header: {
            frame_id: "laser_link",
            stamp: { sec: Math.floor(Date.now() / 1000) },
          },
          angle_min: -3.14159,
          angle_max: 3.14159,
          range_min: 0.1,
          range_max: 30.0,
          ranges: Array.from({ length: 360 }, () => Math.random() * 10),
        };
      case "nav_msgs/Odometry":
        return {
          header: {
            frame_id: "odom",
            stamp: { sec: Math.floor(Date.now() / 1000) },
          },
          pose: {
            pose: {
              position: { x: Math.random() * 10, y: Math.random() * 10, z: 0 },
              orientation: { x: 0, y: 0, z: Math.random(), w: Math.random() },
            },
          },
          twist: {
            twist: {
              linear: { x: Math.random() * 2, y: 0, z: 0 },
              angular: { x: 0, y: 0, z: Math.random() * 1 },
            },
          },
        };
      default:
        return { data: "Mock message data", timestamp: Date.now() };
    }
  };

  const filteredTopics = topics.filter((topic) => {
    const matchesSearch =
      topic.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
      topic.type.toLowerCase().includes(searchQuery.toLowerCase());
    const matchesType = typeFilter === "all" || topic.type.includes(typeFilter);
    const matchesActive = showInactive || topic.isActive;
    return matchesSearch && matchesType && matchesActive;
  });

  const selectedTopicData = topics.find(
    (topic) => topic.name === selectedTopic,
  );

  const topicTypes = Array.from(
    new Set(topics.map((t) => t.type.split("/")[0])),
  );

  const totalBandwidth = topics.reduce(
    (sum, topic) => sum + topic.bandwidth,
    0,
  );
  const activeTopics = topics.filter((t) => t.isActive).length;
  const totalMessages = topics.reduce(
    (sum, topic) => sum + topic.messageCount,
    0,
  );

  const formatBytes = (bytes: number) => {
    if (bytes < 1024) return `${bytes.toFixed(1)} B/s`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB/s`;
    return `${(bytes / 1024 / 1024).toFixed(1)} MB/s`;
  };

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Radio className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            {t("nav.topics")}
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Real-time ROS Topic Monitoring and Message Inspection
          </p>
        </div>

        <div className="flex gap-2">
          <Button
            variant={isMonitoring ? "default" : "outline"}
            onClick={() => setIsMonitoring(!isMonitoring)}
            className="gap-2"
          >
            {isMonitoring ? (
              <Pause className="h-4 w-4" />
            ) : (
              <Play className="h-4 w-4" />
            )}
            {isMonitoring ? "Pause" : "Start"} Monitoring
          </Button>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
          <Button variant="outline" className="gap-2">
            <Settings className="h-4 w-4" />
            Settings
          </Button>
        </div>
      </div>

      {/* Statistics Overview */}
      <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-primary">
              {topics.length}
            </div>
            <div className="text-sm text-muted-foreground">Total Topics</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-green-500">
              {activeTopics}
            </div>
            <div className="text-sm text-muted-foreground">Active</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-blue-500">
              {totalMessages.toLocaleString()}
            </div>
            <div className="text-sm text-muted-foreground">Messages</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-purple-500">
              {formatBytes(totalBandwidth * 1024)}
            </div>
            <div className="text-sm text-muted-foreground">Bandwidth</div>
          </div>
        </Card>
      </div>

      {/* Controls */}
      <Card className="p-4">
        <div className="flex flex-col lg:flex-row gap-4">
          <div className="flex-1">
            <div className="relative">
              <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
              <Input
                placeholder="Search topics..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="pl-10"
              />
            </div>
          </div>
          <div className="flex gap-4 items-center">
            <div className="flex items-center gap-2">
              <Switch
                checked={showInactive}
                onCheckedChange={setShowInactive}
              />
              <Label className="text-sm">Show Inactive</Label>
            </div>
            <div className="flex gap-2">
              {["all", ...topicTypes].map((type) => (
                <Button
                  key={type}
                  variant={typeFilter === type ? "default" : "outline"}
                  size="sm"
                  onClick={() => setTypeFilter(type)}
                  className="gap-1"
                >
                  <Filter className="h-3 w-3" />
                  {type === "all" ? "All" : type}
                </Button>
              ))}
            </div>
          </div>
        </div>
      </Card>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Topics List */}
        <Card className="lg:col-span-2 p-4">
          <h3 className="text-lg font-light mb-4 flex items-center gap-2">
            <MessageSquare className="h-5 w-5 text-primary" />
            Topics ({filteredTopics.length})
          </h3>

          <ScrollArea className="h-[600px]">
            <div className="space-y-2">
              {filteredTopics.map((topic, index) => {
                const isSelected = selectedTopic === topic.name;

                return (
                  <div
                    key={topic.name}
                    className={`p-4 rounded-lg border transition-all cursor-pointer hover-lift stagger-item ${
                      isSelected
                        ? "bg-primary/5 border-primary/30"
                        : "hover:bg-muted/50"
                    }`}
                    style={{ animationDelay: `${index * 0.05}s` }}
                    onClick={() =>
                      setSelectedTopic(isSelected ? null : topic.name)
                    }
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-3 flex-1">
                        <div
                          className={`h-3 w-3 rounded-full ${topic.isActive ? "bg-green-500 glow-on-hover" : "bg-muted-foreground"}`}
                        />
                        <div className="flex-1 min-w-0">
                          <div className="font-mono text-sm font-medium truncate">
                            {topic.name}
                          </div>
                          <div className="text-xs text-muted-foreground truncate">
                            {topic.type}
                          </div>
                        </div>
                      </div>

                      <div className="flex items-center gap-2">
                        <div className="text-right text-xs space-y-1">
                          <div className="flex items-center gap-1">
                            <Activity className="h-3 w-3 text-muted-foreground" />
                            <span className="font-mono">
                              {topic.frequency.toFixed(1)} Hz
                            </span>
                          </div>
                          <div className="flex items-center gap-1">
                            <BarChart3 className="h-3 w-3 text-muted-foreground" />
                            <span className="font-mono">
                              {formatBytes(topic.bandwidth * 1024)}
                            </span>
                          </div>
                        </div>

                        <div className="flex flex-col items-end gap-1">
                          <Badge
                            variant={topic.isActive ? "default" : "secondary"}
                            className="text-xs"
                          >
                            {topic.isActive ? "ACTIVE" : "INACTIVE"}
                          </Badge>
                          <div className="flex gap-1">
                            <Badge variant="outline" className="text-xs">
                              <Users className="h-2 w-2 mr-1" />
                              {topic.publishers.length}P/
                              {topic.subscribers.length}S
                            </Badge>
                          </div>
                        </div>
                      </div>
                    </div>

                    {isSelected && (
                      <div className="mt-4 pt-4 border-t space-y-3 fade-in-up">
                        <div className="grid grid-cols-1 sm:grid-cols-2 gap-4 text-xs">
                          <div>
                            <Label className="text-muted-foreground">
                              Publishers
                            </Label>
                            <div className="mt-1 space-y-1">
                              {topic.publishers.length > 0 ? (
                                topic.publishers.map((pub) => (
                                  <div
                                    key={pub}
                                    className="font-mono text-xs bg-green-500/10 text-green-700 p-1 rounded"
                                  >
                                    {pub}
                                  </div>
                                ))
                              ) : (
                                <div className="text-muted-foreground">
                                  None
                                </div>
                              )}
                            </div>
                          </div>
                          <div>
                            <Label className="text-muted-foreground">
                              Subscribers
                            </Label>
                            <div className="mt-1 space-y-1">
                              {topic.subscribers.length > 0 ? (
                                topic.subscribers.map((sub) => (
                                  <div
                                    key={sub}
                                    className="font-mono text-xs bg-blue-500/10 text-blue-700 p-1 rounded"
                                  >
                                    {sub}
                                  </div>
                                ))
                              ) : (
                                <div className="text-muted-foreground">
                                  None
                                </div>
                              )}
                            </div>
                          </div>
                        </div>

                        <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 text-xs">
                          <div>
                            <span className="text-muted-foreground">
                              Messages:
                            </span>
                            <div className="font-mono">
                              {topic.messageCount.toLocaleString()}
                            </div>
                          </div>
                          <div>
                            <span className="text-muted-foreground">Size:</span>
                            <div className="font-mono">
                              {topic.messageSize} bytes
                            </div>
                          </div>
                          <div>
                            <span className="text-muted-foreground">
                              Latency:
                            </span>
                            <div className="font-mono">
                              {topic.latency.toFixed(1)}ms
                            </div>
                          </div>
                          <div>
                            <span className="text-muted-foreground">Last:</span>
                            <div className="font-mono">
                              {topic.lastMessage.toLocaleTimeString()}
                            </div>
                          </div>
                        </div>
                      </div>
                    )}
                  </div>
                );
              })}
            </div>
          </ScrollArea>
        </Card>

        {/* Message Inspector */}
        <div className="space-y-6">
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Eye className="h-5 w-5 text-primary" />
              Message Inspector
            </h3>

            {selectedTopicData ? (
              <div className="space-y-4">
                <div>
                  <Label className="text-sm font-medium">Topic</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedTopicData.name}
                  </div>
                </div>

                <div>
                  <Label className="text-sm font-medium">Message Type</Label>
                  <div className="font-mono text-sm bg-muted p-2 rounded mt-1">
                    {selectedTopicData.type}
                  </div>
                </div>

                <Separator />

                <div className="grid grid-cols-2 gap-2 text-xs">
                  <div>
                    <span className="text-muted-foreground">Frequency:</span>
                    <div className="font-mono">
                      {selectedTopicData.frequency.toFixed(1)} Hz
                    </div>
                  </div>
                  <div>
                    <span className="text-muted-foreground">Bandwidth:</span>
                    <div className="font-mono">
                      {formatBytes(selectedTopicData.bandwidth * 1024)}
                    </div>
                  </div>
                  <div>
                    <span className="text-muted-foreground">Latency:</span>
                    <div className="font-mono">
                      {selectedTopicData.latency.toFixed(1)}ms
                    </div>
                  </div>
                  <div>
                    <span className="text-muted-foreground">Messages:</span>
                    <div className="font-mono">
                      {selectedTopicData.messageCount.toLocaleString()}
                    </div>
                  </div>
                </div>

                <Separator />

                <div>
                  <Label className="text-sm font-medium">Live Messages</Label>
                  <ScrollArea className="h-64 mt-2">
                    <div className="space-y-2">
                      {messageHistory.map((msg, index) => (
                        <div
                          key={index}
                          className="text-xs bg-muted p-2 rounded"
                        >
                          <div className="flex justify-between items-center mb-1">
                            <span className="font-mono text-muted-foreground">
                              {msg.timestamp.toLocaleTimeString()}
                            </span>
                            <Badge variant="outline" className="text-xs">
                              {msg.size}B
                            </Badge>
                          </div>
                          <pre className="font-mono text-xs overflow-x-auto">
                            {JSON.stringify(msg.data, null, 2)}
                          </pre>
                        </div>
                      ))}
                    </div>
                  </ScrollArea>
                </div>
              </div>
            ) : (
              <div className="text-center text-muted-foreground py-8">
                Select a topic to inspect messages
              </div>
            )}
          </Card>

          {/* Topic Actions */}
          <Card className="p-4">
            <h3 className="text-lg font-light mb-4 flex items-center gap-2">
              <Zap className="h-5 w-5 text-primary" />
              Topic Actions
            </h3>

            <div className="space-y-2">
              <Button
                variant="outline"
                className="w-full justify-start gap-2"
                disabled={!selectedTopicData}
              >
                <Play className="h-4 w-4" />
                Subscribe to Topic
              </Button>
              <Button
                variant="outline"
                className="w-full justify-start gap-2"
                disabled={!selectedTopicData}
              >
                <Download className="h-4 w-4" />
                Record Messages
              </Button>
              <Button
                variant="outline"
                className="w-full justify-start gap-2"
                disabled={!selectedTopicData}
              >
                <BarChart3 className="h-4 w-4" />
                View Graph
              </Button>
              <Button
                variant="outline"
                className="w-full justify-start gap-2"
                disabled={!selectedTopicData}
              >
                <Settings className="h-4 w-4" />
                Topic Info
              </Button>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
};

export default Topics;
