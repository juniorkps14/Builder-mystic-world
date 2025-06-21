import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { Switch } from "@/components/ui/switch";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { useLanguage } from "@/contexts/LanguageContext";
import {
  Layers,
  Search,
  Plus,
  Edit,
  Trash2,
  Save,
  RefreshCw,
  Download,
  Upload,
  Settings,
  FolderOpen,
  File,
  ChevronRight,
  ChevronDown,
  Copy,
  Eye,
  EyeOff,
  AlertTriangle,
  CheckCircle,
  Info,
} from "lucide-react";

interface ROSParameter {
  name: string;
  value: any;
  type: "string" | "int" | "double" | "bool" | "list" | "dict";
  namespace: string;
  description?: string;
  lastModified: Date;
  isReadOnly: boolean;
  source: string;
}

interface ParameterGroup {
  namespace: string;
  expanded: boolean;
  parameters: ROSParameter[];
  children: ParameterGroup[];
}

const Parameters = () => {
  const { t } = useLanguage();

  const [parameters, setParameters] = useState<ROSParameter[]>([
    {
      name: "/robot/max_velocity",
      value: 2.5,
      type: "double",
      namespace: "/robot",
      description: "Maximum velocity for robot movement in m/s",
      lastModified: new Date(),
      isReadOnly: false,
      source: "launch_file",
    },
    {
      name: "/robot/wheel_diameter",
      value: 0.1,
      type: "double",
      namespace: "/robot",
      description: "Diameter of robot wheels in meters",
      lastModified: new Date(),
      isReadOnly: false,
      source: "urdf",
    },
    {
      name: "/robot/base_frame",
      value: "base_link",
      type: "string",
      namespace: "/robot",
      description: "Base frame ID for the robot",
      lastModified: new Date(),
      isReadOnly: false,
      source: "tf_config",
    },
    {
      name: "/move_base/global_planner",
      value: "navfn/NavfnROS",
      type: "string",
      namespace: "/move_base",
      description: "Global path planner algorithm",
      lastModified: new Date(),
      isReadOnly: false,
      source: "navigation",
    },
    {
      name: "/move_base/local_planner",
      value: "dwa_local_planner/DWAPlannerROS",
      type: "string",
      namespace: "/move_base",
      description: "Local path planner algorithm",
      lastModified: new Date(),
      isReadOnly: false,
      source: "navigation",
    },
    {
      name: "/move_base/controller_frequency",
      value: 20.0,
      type: "double",
      namespace: "/move_base",
      description: "Frequency for the controller loop in Hz",
      lastModified: new Date(),
      isReadOnly: false,
      source: "navigation",
    },
    {
      name: "/amcl/min_particles",
      value: 500,
      type: "int",
      namespace: "/amcl",
      description: "Minimum number of particles for localization",
      lastModified: new Date(),
      isReadOnly: false,
      source: "localization",
    },
    {
      name: "/amcl/max_particles",
      value: 5000,
      type: "int",
      namespace: "/amcl",
      description: "Maximum number of particles for localization",
      lastModified: new Date(),
      isReadOnly: false,
      source: "localization",
    },
    {
      name: "/camera/frame_rate",
      value: 30,
      type: "int",
      namespace: "/camera",
      description: "Camera capture frame rate",
      lastModified: new Date(),
      isReadOnly: false,
      source: "camera_driver",
    },
    {
      name: "/camera/resolution",
      value: [1920, 1080],
      type: "list",
      namespace: "/camera",
      description: "Camera resolution [width, height]",
      lastModified: new Date(),
      isReadOnly: false,
      source: "camera_driver",
    },
    {
      name: "/safety/emergency_stop_enabled",
      value: true,
      type: "bool",
      namespace: "/safety",
      description: "Enable emergency stop functionality",
      lastModified: new Date(),
      isReadOnly: false,
      source: "safety_controller",
    },
    {
      name: "/diagnostics/publish_rate",
      value: 1.0,
      type: "double",
      namespace: "/diagnostics",
      description: "Rate for publishing diagnostic messages",
      lastModified: new Date(),
      isReadOnly: false,
      source: "diagnostic_aggregator",
    },
  ]);

  const [searchQuery, setSearchQuery] = useState("");
  const [namespaceFilter, setNamespaceFilter] = useState("all");
  const [typeFilter, setTypeFilter] = useState("all");
  const [showReadOnly, setShowReadOnly] = useState(true);
  const [selectedParameter, setSelectedParameter] = useState<string | null>(
    null,
  );
  const [isEditDialogOpen, setIsEditDialogOpen] = useState(false);
  const [isAddDialogOpen, setIsAddDialogOpen] = useState(false);
  const [expandedNamespaces, setExpandedNamespaces] = useState<Set<string>>(
    new Set(),
  );

  // New parameter form state
  const [newParameter, setNewParameter] = useState({
    name: "",
    value: "",
    type: "string" as const,
    namespace: "/",
    description: "",
  });

  // Edit parameter state
  const [editParameter, setEditParameter] = useState<ROSParameter | null>(null);

  // Simulate real-time parameter updates
  useEffect(() => {
    const interval = setInterval(() => {
      // Randomly update some parameters to simulate dynamic changes
      setParameters((prev) =>
        prev.map((param) => {
          if (Math.random() < 0.05) {
            // 5% chance to update
            let newValue = param.value;
            switch (param.type) {
              case "double":
                if (typeof param.value === "number") {
                  newValue = param.value + (Math.random() - 0.5) * 0.1;
                }
                break;
              case "int":
                if (typeof param.value === "number") {
                  newValue = Math.round(
                    param.value + (Math.random() - 0.5) * 2,
                  );
                }
                break;
            }
            return {
              ...param,
              value: newValue,
              lastModified: new Date(),
            };
          }
          return param;
        }),
      );
    }, 10000); // Update every 10 seconds

    return () => clearInterval(interval);
  }, []);

  const filteredParameters = parameters.filter((param) => {
    const matchesSearch =
      param.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
      param.description?.toLowerCase().includes(searchQuery.toLowerCase()) ||
      param.namespace.toLowerCase().includes(searchQuery.toLowerCase());

    const matchesNamespace =
      namespaceFilter === "all" || param.namespace === namespaceFilter;
    const matchesType = typeFilter === "all" || param.type === typeFilter;
    const matchesReadOnly = showReadOnly || !param.isReadOnly;

    return matchesSearch && matchesNamespace && matchesType && matchesReadOnly;
  });

  const groupParametersByNamespace = (
    params: ROSParameter[],
  ): ParameterGroup[] => {
    const namespaceMap = new Map<string, ROSParameter[]>();

    params.forEach((param) => {
      const ns = param.namespace;
      if (!namespaceMap.has(ns)) {
        namespaceMap.set(ns, []);
      }
      namespaceMap.get(ns)!.push(param);
    });

    return Array.from(namespaceMap.entries()).map(
      ([namespace, parameters]) => ({
        namespace,
        expanded: expandedNamespaces.has(namespace),
        parameters: parameters.sort((a, b) => a.name.localeCompare(b.name)),
        children: [],
      }),
    );
  };

  const toggleNamespace = (namespace: string) => {
    const newExpanded = new Set(expandedNamespaces);
    if (newExpanded.has(namespace)) {
      newExpanded.delete(namespace);
    } else {
      newExpanded.add(namespace);
    }
    setExpandedNamespaces(newExpanded);
  };

  const handleAddParameter = () => {
    const paramName =
      newParameter.namespace + "/" + newParameter.name.replace(/^\/+/, "");

    let parsedValue: any = newParameter.value;
    try {
      switch (newParameter.type) {
        case "int":
          parsedValue = parseInt(newParameter.value);
          break;
        case "double":
          parsedValue = parseFloat(newParameter.value);
          break;
        case "bool":
          parsedValue = newParameter.value.toLowerCase() === "true";
          break;
        case "list":
        case "dict":
          parsedValue = JSON.parse(newParameter.value);
          break;
      }
    } catch (error) {
      alert(`Invalid value for type ${newParameter.type}: ${error}`);
      return;
    }

    const parameter: ROSParameter = {
      name: paramName,
      value: parsedValue,
      type: newParameter.type,
      namespace: newParameter.namespace,
      description: newParameter.description,
      lastModified: new Date(),
      isReadOnly: false,
      source: "manual",
    };

    setParameters((prev) => [...prev, parameter]);
    setNewParameter({
      name: "",
      value: "",
      type: "string",
      namespace: "/",
      description: "",
    });
    setIsAddDialogOpen(false);
  };

  const handleEditParameter = (param: ROSParameter) => {
    setEditParameter({ ...param });
    setIsEditDialogOpen(true);
  };

  const handleSaveEdit = () => {
    if (!editParameter) return;

    setParameters((prev) =>
      prev.map((param) =>
        param.name === editParameter.name
          ? { ...editParameter, lastModified: new Date() }
          : param,
      ),
    );
    setEditParameter(null);
    setIsEditDialogOpen(false);
  };

  const handleDeleteParameter = (paramName: string) => {
    if (confirm(`Are you sure you want to delete parameter ${paramName}?`)) {
      setParameters((prev) => prev.filter((param) => param.name !== paramName));
    }
  };

  const getTypeIcon = (type: string) => {
    switch (type) {
      case "string":
        return "📝";
      case "int":
        return "🔢";
      case "double":
        return "📊";
      case "bool":
        return "🔘";
      case "list":
        return "📋";
      case "dict":
        return "📁";
      default:
        return "❓";
    }
  };

  const getTypeColor = (type: string) => {
    switch (type) {
      case "string":
        return "bg-blue-100 text-blue-800";
      case "int":
        return "bg-green-100 text-green-800";
      case "double":
        return "bg-purple-100 text-purple-800";
      case "bool":
        return "bg-orange-100 text-orange-800";
      case "list":
        return "bg-pink-100 text-pink-800";
      case "dict":
        return "bg-yellow-100 text-yellow-800";
      default:
        return "bg-gray-100 text-gray-800";
    }
  };

  const uniqueNamespaces = Array.from(
    new Set(parameters.map((p) => p.namespace)),
  );
  const uniqueTypes = Array.from(new Set(parameters.map((p) => p.type)));
  const parameterGroups = groupParametersByNamespace(filteredParameters);

  return (
    <div className="space-y-6 p-4 md:p-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="space-y-2">
          <h1 className="text-2xl md:text-3xl font-extralight flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 float-animation">
              <Layers className="h-6 w-6 md:h-8 w-8 text-primary" />
            </div>
            ROS Parameter Server
          </h1>
          <p className="text-muted-foreground text-sm md:text-base">
            Configure and manage ROS parameters with real-time monitoring
          </p>
        </div>

        <div className="flex gap-2">
          <Dialog open={isAddDialogOpen} onOpenChange={setIsAddDialogOpen}>
            <DialogTrigger asChild>
              <Button className="gap-2">
                <Plus className="h-4 w-4" />
                Add Parameter
              </Button>
            </DialogTrigger>
            <DialogContent className="sm:max-w-[525px]">
              <DialogHeader>
                <DialogTitle>Add New Parameter</DialogTitle>
                <DialogDescription>
                  Create a new ROS parameter with the specified configuration.
                </DialogDescription>
              </DialogHeader>
              <div className="grid gap-4 py-4">
                <div className="grid grid-cols-4 items-center gap-4">
                  <Label htmlFor="namespace" className="text-right">
                    Namespace
                  </Label>
                  <Input
                    id="namespace"
                    value={newParameter.namespace}
                    onChange={(e) =>
                      setNewParameter((prev) => ({
                        ...prev,
                        namespace: e.target.value,
                      }))
                    }
                    className="col-span-3"
                    placeholder="/robot"
                  />
                </div>
                <div className="grid grid-cols-4 items-center gap-4">
                  <Label htmlFor="name" className="text-right">
                    Name
                  </Label>
                  <Input
                    id="name"
                    value={newParameter.name}
                    onChange={(e) =>
                      setNewParameter((prev) => ({
                        ...prev,
                        name: e.target.value,
                      }))
                    }
                    className="col-span-3"
                    placeholder="parameter_name"
                  />
                </div>
                <div className="grid grid-cols-4 items-center gap-4">
                  <Label htmlFor="type" className="text-right">
                    Type
                  </Label>
                  <Select
                    value={newParameter.type}
                    onValueChange={(value: any) =>
                      setNewParameter((prev) => ({ ...prev, type: value }))
                    }
                  >
                    <SelectTrigger className="col-span-3">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="string">String</SelectItem>
                      <SelectItem value="int">Integer</SelectItem>
                      <SelectItem value="double">Double</SelectItem>
                      <SelectItem value="bool">Boolean</SelectItem>
                      <SelectItem value="list">List</SelectItem>
                      <SelectItem value="dict">Dictionary</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
                <div className="grid grid-cols-4 items-center gap-4">
                  <Label htmlFor="value" className="text-right">
                    Value
                  </Label>
                  <Input
                    id="value"
                    value={newParameter.value}
                    onChange={(e) =>
                      setNewParameter((prev) => ({
                        ...prev,
                        value: e.target.value,
                      }))
                    }
                    className="col-span-3"
                    placeholder="parameter_value"
                  />
                </div>
                <div className="grid grid-cols-4 items-center gap-4">
                  <Label htmlFor="description" className="text-right">
                    Description
                  </Label>
                  <Textarea
                    id="description"
                    value={newParameter.description}
                    onChange={(e) =>
                      setNewParameter((prev) => ({
                        ...prev,
                        description: e.target.value,
                      }))
                    }
                    className="col-span-3"
                    placeholder="Parameter description"
                  />
                </div>
              </div>
              <DialogFooter>
                <Button type="submit" onClick={handleAddParameter}>
                  Add Parameter
                </Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>

          <Button variant="outline" className="gap-2">
            <RefreshCw className="h-4 w-4" />
            Reload
          </Button>
          <Button variant="outline" className="gap-2">
            <Download className="h-4 w-4" />
            Export
          </Button>
          <Button variant="outline" className="gap-2">
            <Upload className="h-4 w-4" />
            Import
          </Button>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-primary">
              {parameters.length}
            </div>
            <div className="text-sm text-muted-foreground">
              Total Parameters
            </div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-green-500">
              {uniqueNamespaces.length}
            </div>
            <div className="text-sm text-muted-foreground">Namespaces</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-blue-500">
              {parameters.filter((p) => !p.isReadOnly).length}
            </div>
            <div className="text-sm text-muted-foreground">Editable</div>
          </div>
        </Card>
        <Card className="p-4 hover-lift">
          <div className="text-center">
            <div className="text-2xl font-extralight text-orange-500">
              {
                parameters.filter(
                  (p) => p.lastModified > new Date(Date.now() - 3600000),
                ).length
              }
            </div>
            <div className="text-sm text-muted-foreground">Recent Changes</div>
          </div>
        </Card>
      </div>

      {/* Filters */}
      <Card className="p-4">
        <div className="grid grid-cols-1 lg:grid-cols-5 gap-4">
          <div className="lg:col-span-2">
            <Label className="text-sm">Search Parameters</Label>
            <div className="relative mt-1">
              <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
              <Input
                placeholder="Search by name, namespace, or description..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="pl-10"
              />
            </div>
          </div>

          <div>
            <Label className="text-sm">Namespace</Label>
            <Select value={namespaceFilter} onValueChange={setNamespaceFilter}>
              <SelectTrigger className="mt-1">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All Namespaces</SelectItem>
                {uniqueNamespaces.map((ns) => (
                  <SelectItem key={ns} value={ns}>
                    {ns}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          <div>
            <Label className="text-sm">Type</Label>
            <Select value={typeFilter} onValueChange={setTypeFilter}>
              <SelectTrigger className="mt-1">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All Types</SelectItem>
                {uniqueTypes.map((type) => (
                  <SelectItem key={type} value={type}>
                    {type}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          <div className="flex items-end">
            <div className="flex items-center space-x-2">
              <Switch
                checked={showReadOnly}
                onCheckedChange={setShowReadOnly}
              />
              <Label className="text-sm">Show Read-Only</Label>
            </div>
          </div>
        </div>
      </Card>

      {/* Parameters Display */}
      <Card className="p-4">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-light flex items-center gap-2">
            <Settings className="h-5 w-5 text-primary" />
            Parameters ({filteredParameters.length})
          </h3>
          <Badge variant="outline">
            {filteredParameters.length} of {parameters.length}
          </Badge>
        </div>

        <ScrollArea className="h-[600px]">
          <div className="space-y-2">
            {parameterGroups.map((group, groupIndex) => (
              <div key={group.namespace} className="border rounded-lg">
                {/* Namespace Header */}
                <div
                  className="flex items-center justify-between p-3 bg-muted/20 cursor-pointer hover:bg-muted/40 transition-colors"
                  onClick={() => toggleNamespace(group.namespace)}
                >
                  <div className="flex items-center gap-2">
                    {group.expanded ? (
                      <ChevronDown className="h-4 w-4" />
                    ) : (
                      <ChevronRight className="h-4 w-4" />
                    )}
                    <FolderOpen className="h-4 w-4 text-primary" />
                    <span className="font-mono text-sm font-medium">
                      {group.namespace}
                    </span>
                  </div>
                  <Badge variant="outline" className="text-xs">
                    {group.parameters.length} params
                  </Badge>
                </div>

                {/* Parameters in Namespace */}
                {group.expanded && (
                  <div className="divide-y">
                    {group.parameters.map((param, paramIndex) => (
                      <div
                        key={param.name}
                        className={`p-4 hover:bg-muted/20 transition-colors stagger-item`}
                        style={{
                          animationDelay: `${(groupIndex * 5 + paramIndex) * 0.05}s`,
                        }}
                      >
                        <div className="flex items-start justify-between">
                          <div className="flex-1 min-w-0">
                            <div className="flex items-center gap-2 mb-2">
                              <File className="h-4 w-4 text-muted-foreground" />
                              <span className="font-mono text-sm font-medium truncate">
                                {param.name.replace(param.namespace + "/", "")}
                              </span>
                              <Badge
                                className={`text-xs ${getTypeColor(param.type)}`}
                              >
                                {getTypeIcon(param.type)} {param.type}
                              </Badge>
                              {param.isReadOnly && (
                                <Badge variant="outline" className="text-xs">
                                  Read-Only
                                </Badge>
                              )}
                            </div>

                            <div className="space-y-1">
                              <div className="flex items-center gap-2">
                                <span className="text-xs text-muted-foreground">
                                  Value:
                                </span>
                                <code className="text-xs bg-muted px-2 py-1 rounded font-mono">
                                  {typeof param.value === "object"
                                    ? JSON.stringify(param.value)
                                    : String(param.value)}
                                </code>
                              </div>

                              {param.description && (
                                <div className="flex items-start gap-2">
                                  <span className="text-xs text-muted-foreground">
                                    Description:
                                  </span>
                                  <span className="text-xs text-muted-foreground">
                                    {param.description}
                                  </span>
                                </div>
                              )}

                              <div className="flex items-center gap-4 text-xs text-muted-foreground">
                                <span>Source: {param.source}</span>
                                <span>
                                  Modified:{" "}
                                  {param.lastModified.toLocaleTimeString()}
                                </span>
                              </div>
                            </div>
                          </div>

                          <div className="flex gap-1 ml-4">
                            <Button
                              size="sm"
                              variant="ghost"
                              onClick={() =>
                                navigator.clipboard.writeText(param.name)
                              }
                              className="gap-1"
                            >
                              <Copy className="h-3 w-3" />
                            </Button>
                            {!param.isReadOnly && (
                              <>
                                <Button
                                  size="sm"
                                  variant="ghost"
                                  onClick={() => handleEditParameter(param)}
                                  className="gap-1"
                                >
                                  <Edit className="h-3 w-3" />
                                </Button>
                                <Button
                                  size="sm"
                                  variant="ghost"
                                  onClick={() =>
                                    handleDeleteParameter(param.name)
                                  }
                                  className="gap-1 text-red-500 hover:text-red-700"
                                >
                                  <Trash2 className="h-3 w-3" />
                                </Button>
                              </>
                            )}
                          </div>
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}
          </div>
        </ScrollArea>
      </Card>

      {/* Edit Parameter Dialog */}
      <Dialog open={isEditDialogOpen} onOpenChange={setIsEditDialogOpen}>
        <DialogContent className="sm:max-w-[525px]">
          <DialogHeader>
            <DialogTitle>Edit Parameter</DialogTitle>
            <DialogDescription>
              Modify the parameter value and settings.
            </DialogDescription>
          </DialogHeader>
          {editParameter && (
            <div className="grid gap-4 py-4">
              <div className="grid grid-cols-4 items-center gap-4">
                <Label className="text-right">Name</Label>
                <Input
                  value={editParameter.name}
                  disabled
                  className="col-span-3 bg-muted"
                />
              </div>
              <div className="grid grid-cols-4 items-center gap-4">
                <Label className="text-right">Type</Label>
                <Badge
                  className={`col-span-3 justify-start ${getTypeColor(editParameter.type)}`}
                >
                  {getTypeIcon(editParameter.type)} {editParameter.type}
                </Badge>
              </div>
              <div className="grid grid-cols-4 items-center gap-4">
                <Label className="text-right">Value</Label>
                <Input
                  value={
                    typeof editParameter.value === "object"
                      ? JSON.stringify(editParameter.value)
                      : String(editParameter.value)
                  }
                  onChange={(e) => {
                    if (!editParameter) return;
                    let newValue: any = e.target.value;
                    try {
                      switch (editParameter.type) {
                        case "int":
                          newValue = parseInt(e.target.value);
                          break;
                        case "double":
                          newValue = parseFloat(e.target.value);
                          break;
                        case "bool":
                          newValue = e.target.value.toLowerCase() === "true";
                          break;
                        case "list":
                        case "dict":
                          newValue = JSON.parse(e.target.value);
                          break;
                      }
                    } catch (error) {
                      // Keep as string if parsing fails
                      newValue = e.target.value;
                    }
                    setEditParameter({ ...editParameter, value: newValue });
                  }}
                  className="col-span-3"
                />
              </div>
              <div className="grid grid-cols-4 items-center gap-4">
                <Label className="text-right">Description</Label>
                <Textarea
                  value={editParameter.description || ""}
                  onChange={(e) =>
                    setEditParameter((prev) =>
                      prev ? { ...prev, description: e.target.value } : null,
                    )
                  }
                  className="col-span-3"
                />
              </div>
            </div>
          )}
          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => setIsEditDialogOpen(false)}
            >
              Cancel
            </Button>
            <Button onClick={handleSaveEdit}>
              <Save className="h-4 w-4 mr-2" />
              Save Changes
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
};

export default Parameters;
