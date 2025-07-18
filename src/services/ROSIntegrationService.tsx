import {
  createContext,
  useContext,
  useState,
  useEffect,
  ReactNode,
} from "react";

// ROS Connection Configuration
interface ROSConfig {
  rosBridgeUrl: string;
  masterUri: string;
  isConnected: boolean;
  reconnectInterval: number;
  heartbeatInterval: number;
  connectionStatus: "connecting" | "connected" | "disconnected" | "error";
  lastError?: string;
  autoReconnect: boolean;
  maxReconnectAttempts: number;
  currentReconnectAttempt: number;
}

// ROS Message Types
interface ROSMessage {
  op: string;
  topic?: string;
  service?: string;
  type?: string;
  msg?: any;
  args?: any;
  id?: string;
  result?: boolean;
  values?: any;
}

// Service Response Types
interface ServiceResponse {
  success: boolean;
  data?: any;
  error?: string;
  timestamp: Date;
}

interface ROSIntegrationContextType {
  // Connection Management
  config: ROSConfig;
  updateConfig: (newConfig: Partial<ROSConfig>) => void;
  connect: () => Promise<boolean>;
  disconnect: () => void;

  // Topic Operations
  publishTopic: (
    topic: string,
    message: any,
    messageType?: string,
  ) => Promise<ServiceResponse>;
  subscribeTopic: (
    topic: string,
    callback: (data: any) => void,
  ) => Promise<ServiceResponse>;
  unsubscribeTopic: (topic: string) => Promise<ServiceResponse>;
  getTopics: () => Promise<ServiceResponse>;

  // Service Operations
  callService: (service: string, args?: any) => Promise<ServiceResponse>;
  getServices: () => Promise<ServiceResponse>;

  // Parameter Operations
  getParameter: (param: string) => Promise<ServiceResponse>;
  setParameter: (param: string, value: any) => Promise<ServiceResponse>;
  deleteParameter: (param: string) => Promise<ServiceResponse>;
  getParameters: () => Promise<ServiceResponse>;

  // Node Operations
  getNodes: () => Promise<ServiceResponse>;
  getNodeInfo: (node: string) => Promise<ServiceResponse>;

  // Navigation Commands
  sendNavigationGoal: (
    x: number,
    y: number,
    yaw?: number,
  ) => Promise<ServiceResponse>;
  cancelNavigationGoal: () => Promise<ServiceResponse>;
  getCurrentPose: () => Promise<ServiceResponse>;

  // Robot Control Commands
  sendVelocityCommand: (
    linear: { x: number; y: number; z: number },
    angular: { x: number; y: number; z: number },
  ) => Promise<ServiceResponse>;
  emergencyStop: () => Promise<ServiceResponse>;

  // Sensor Data
  getSensorData: (sensorType: string) => Promise<ServiceResponse>;

  // System Status
  getSystemStatus: () => Promise<ServiceResponse>;

  // Real-time Data Subscriptions
  subscriptions: Map<string, (data: any) => void>;
}

const ROSIntegrationContext = createContext<ROSIntegrationContextType | null>(
  null,
);

export const useROSIntegration = () => {
  const context = useContext(ROSIntegrationContext);
  if (!context) {
    throw new Error(
      "useROSIntegration must be used within ROSIntegrationProvider",
    );
  }
  return context;
};

interface ROSIntegrationProviderProps {
  children: ReactNode;
}

export const ROSIntegrationProvider: React.FC<ROSIntegrationProviderProps> = ({
  children,
}) => {
  const [config, setConfig] = useState<ROSConfig>({
    rosBridgeUrl: "ws://localhost:9090",
    masterUri: "http://localhost:11311",
    isConnected: false,
    reconnectInterval: 5000,
    heartbeatInterval: 30000,
    connectionStatus: "disconnected",
    autoReconnect: true,
    maxReconnectAttempts: 10,
    currentReconnectAttempt: 0,
  });

  const [websocket, setWebsocket] = useState<WebSocket | null>(null);
  const [subscriptions] = useState<Map<string, (data: any) => void>>(new Map());
  const [messageId, setMessageId] = useState(0);
  const [pendingCalls] = useState<
    Map<string, { resolve: Function; reject: Function }>
  >(new Map());

  // Initialize WebSocket connection
  const connect = async (): Promise<boolean> => {
    return new Promise((resolve) => {
      try {
        // Validate URL format
        if (!config.rosBridgeUrl || !config.rosBridgeUrl.startsWith("ws")) {
          console.error("Invalid RosBridge URL:", config.rosBridgeUrl);
          setConfig((prev) => ({ ...prev, isConnected: false }));
          resolve(false);
          return;
        }

        console.log("Attempting to connect to RosBridge:", config.rosBridgeUrl);
        const ws = new WebSocket(config.rosBridgeUrl);

        ws.onopen = () => {
          console.log("ROS Bridge connected successfully");
          setConfig((prev) => ({ ...prev, isConnected: true }));
          setWebsocket(ws);

          // Start heartbeat
          startHeartbeat(ws);
          resolve(true);
        };

        ws.onmessage = (event) => {
          try {
            const message: ROSMessage = JSON.parse(event.data);
            handleIncomingMessage(message);
          } catch (error) {
            console.error("Failed to parse ROS message:", error);
          }
        };

        ws.onclose = (event) => {
          console.log("ROS Bridge connection closed", {
            code: event.code,
            reason: event.reason,
            wasClean: event.wasClean,
          });
          setConfig((prev) => ({ ...prev, isConnected: false }));
          setWebsocket(null);

          // Auto-reconnect only if it wasn't a clean close
          if (!event.wasClean && event.code !== 1000) {
            console.log(
              `Attempting reconnection in ${config.reconnectInterval}ms...`,
            );
            setTimeout(() => {
              if (!config.isConnected) {
                console.log("Reconnecting to ROS Bridge...");
                connect();
              }
            }, config.reconnectInterval);
          }
        };

        ws.onerror = (event) => {
          const errorMessage = `WebSocket connection failed to ${config.rosBridgeUrl}. Please check if RosBridge server is running.`;
          console.error("ROS Bridge connection error:", errorMessage);
          console.error("Error event details:", event);
          setConfig((prev) => ({ ...prev, isConnected: false }));
          resolve(false);
        };
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : String(error);
        console.error("Failed to create WebSocket connection:", {
          url: config.rosBridgeUrl,
          error: errorMessage,
          suggestion:
            "Check if RosBridge server is running on the specified URL",
        });
        setConfig((prev) => ({ ...prev, isConnected: false }));
        resolve(false);
      }
    });
  };

  const disconnect = () => {
    if (websocket) {
      websocket.close();
      setWebsocket(null);
      setConfig((prev) => ({ ...prev, isConnected: false }));
    }
  };

  const startHeartbeat = (ws: WebSocket) => {
    const interval = setInterval(() => {
      if (ws.readyState === WebSocket.OPEN) {
        sendMessage(ws, {
          op: "call_service",
          service: "/rosapi/get_time",
        });
      } else {
        clearInterval(interval);
      }
    }, config.heartbeatInterval);
  };

  const sendMessage = (
    ws: WebSocket,
    message: ROSMessage,
  ): Promise<ServiceResponse> => {
    return new Promise((resolve, reject) => {
      const id = (++messageId).toString();
      const messageWithId = { ...message, id };

      pendingCalls.set(id, { resolve, reject });

      ws.send(JSON.stringify(messageWithId));

      // Timeout after 10 seconds
      setTimeout(() => {
        if (pendingCalls.has(id)) {
          pendingCalls.delete(id);
          reject(
            new Error(
              `Request timeout for message: ${JSON.stringify(message)}`,
            ),
          );
        }
      }, 10000);

      // Handle WebSocket errors
      if (ws.readyState !== WebSocket.OPEN) {
        pendingCalls.delete(id);
        reject(
          new Error(`WebSocket not connected. Current state: ${ws.readyState}`),
        );
        return;
      }
    });
  };

  const handleIncomingMessage = (message: ROSMessage) => {
    // Handle service call responses
    if (message.id && pendingCalls.has(message.id)) {
      const { resolve } = pendingCalls.get(message.id)!;
      pendingCalls.delete(message.id);

      resolve({
        success: message.result !== false,
        data: message.values || message,
        timestamp: new Date(),
      });
      return;
    }

    // Handle topic subscriptions
    if (message.topic && subscriptions.has(message.topic)) {
      const callback = subscriptions.get(message.topic)!;
      callback(message.msg);
    }
  };

  // API Methods
  const updateConfig = (newConfig: Partial<ROSConfig>) => {
    setConfig((prev) => ({ ...prev, ...newConfig }));
  };

  const publishTopic = async (
    topic: string,
    message: any,
    messageType?: string,
  ): Promise<ServiceResponse> => {
    if (!websocket) {
      return {
        success: false,
        error: "Not connected to ROS",
        timestamp: new Date(),
      };
    }

    try {
      const rosMessage: ROSMessage = {
        op: "publish",
        topic,
        msg: message,
        type: messageType,
      };

      await sendMessage(websocket, rosMessage);
      return { success: true, timestamp: new Date() };
    } catch (error) {
      return { success: false, error: String(error), timestamp: new Date() };
    }
  };

  const subscribeTopic = async (
    topic: string,
    callback: (data: any) => void,
  ): Promise<ServiceResponse> => {
    if (!websocket) {
      return {
        success: false,
        error: "Not connected to ROS",
        timestamp: new Date(),
      };
    }

    try {
      subscriptions.set(topic, callback);

      const rosMessage: ROSMessage = {
        op: "subscribe",
        topic,
      };

      await sendMessage(websocket, rosMessage);
      return { success: true, timestamp: new Date() };
    } catch (error) {
      subscriptions.delete(topic);
      return { success: false, error: String(error), timestamp: new Date() };
    }
  };

  const unsubscribeTopic = async (topic: string): Promise<ServiceResponse> => {
    if (!websocket) {
      return {
        success: false,
        error: "Not connected to ROS",
        timestamp: new Date(),
      };
    }

    try {
      subscriptions.delete(topic);

      const rosMessage: ROSMessage = {
        op: "unsubscribe",
        topic,
      };

      await sendMessage(websocket, rosMessage);
      return { success: true, timestamp: new Date() };
    } catch (error) {
      return { success: false, error: String(error), timestamp: new Date() };
    }
  };

  const callService = async (
    service: string,
    args?: any,
  ): Promise<ServiceResponse> => {
    if (!websocket) {
      return {
        success: false,
        error: "Not connected to ROS",
        timestamp: new Date(),
      };
    }

    try {
      const rosMessage: ROSMessage = {
        op: "call_service",
        service,
        args: args || {},
      };

      const response = await sendMessage(websocket, rosMessage);
      return response;
    } catch (error) {
      return { success: false, error: String(error), timestamp: new Date() };
    }
  };

  const getTopics = async (): Promise<ServiceResponse> => {
    return callService("/rosapi/topics");
  };

  const getServices = async (): Promise<ServiceResponse> => {
    return callService("/rosapi/services");
  };

  const getNodes = async (): Promise<ServiceResponse> => {
    return callService("/rosapi/nodes");
  };

  const getNodeInfo = async (node: string): Promise<ServiceResponse> => {
    return callService("/rosapi/node_details", { node });
  };

  const getParameter = async (param: string): Promise<ServiceResponse> => {
    return callService("/rosapi/get_param", { name: param });
  };

  const setParameter = async (
    param: string,
    value: any,
  ): Promise<ServiceResponse> => {
    return callService("/rosapi/set_param", { name: param, value });
  };

  const deleteParameter = async (param: string): Promise<ServiceResponse> => {
    return callService("/rosapi/delete_param", { name: param });
  };

  const getParameters = async (): Promise<ServiceResponse> => {
    return callService("/rosapi/get_param_names");
  };

  // Navigation Commands
  const sendNavigationGoal = async (
    x: number,
    y: number,
    yaw: number = 0,
  ): Promise<ServiceResponse> => {
    const goal = {
      target_pose: {
        header: {
          frame_id: "map",
          stamp: { secs: Math.floor(Date.now() / 1000), nsecs: 0 },
        },
        pose: {
          position: { x, y, z: 0 },
          orientation: {
            x: 0,
            y: 0,
            z: Math.sin(yaw / 2),
            w: Math.cos(yaw / 2),
          },
        },
      },
    };

    return publishTopic(
      "/move_base_simple/goal",
      goal,
      "geometry_msgs/PoseStamped",
    );
  };

  const cancelNavigationGoal = async (): Promise<ServiceResponse> => {
    return callService("/move_base/cancel");
  };

  const getCurrentPose = async (): Promise<ServiceResponse> => {
    return new Promise((resolve) => {
      subscribeTopic("/amcl_pose", (data) => {
        resolve({ success: true, data, timestamp: new Date() });
      });
    });
  };

  // Robot Control Commands
  const sendVelocityCommand = async (
    linear: { x: number; y: number; z: number },
    angular: { x: number; y: number; z: number },
  ): Promise<ServiceResponse> => {
    const twist = { linear, angular };
    return publishTopic("/cmd_vel", twist, "geometry_msgs/Twist");
  };

  const emergencyStop = async (): Promise<ServiceResponse> => {
    const stopTwist = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    return publishTopic("/cmd_vel", stopTwist, "geometry_msgs/Twist");
  };

  // Sensor Data
  const getSensorData = async (
    sensorType: string,
  ): Promise<ServiceResponse> => {
    const topicMap: { [key: string]: string } = {
      imu: "/imu/data",
      laser: "/scan",
      camera: "/camera/image_raw",
      odom: "/odom",
      battery: "/battery_state",
    };

    const topic = topicMap[sensorType];
    if (!topic) {
      return {
        success: false,
        error: `Unknown sensor type: ${sensorType}`,
        timestamp: new Date(),
      };
    }

    return new Promise((resolve) => {
      subscribeTopic(topic, (data) => {
        resolve({ success: true, data, timestamp: new Date() });
      });
    });
  };

  // System Status
  const getSystemStatus = async (): Promise<ServiceResponse> => {
    try {
      const [topics, nodes, services] = await Promise.all([
        getTopics(),
        getNodes(),
        getServices(),
      ]);

      return {
        success: true,
        data: {
          topics: topics.data,
          nodes: nodes.data,
          services: services.data,
          connected: config.isConnected,
          timestamp: new Date(),
        },
        timestamp: new Date(),
      };
    } catch (error) {
      return { success: false, error: String(error), timestamp: new Date() };
    }
  };

  // Auto-connect on component mount
  useEffect(() => {
    connect();
    return () => disconnect();
  }, []);

  const contextValue: ROSIntegrationContextType = {
    config,
    updateConfig,
    connect,
    disconnect,
    publishTopic,
    subscribeTopic,
    unsubscribeTopic,
    getTopics,
    callService,
    getServices,
    getParameter,
    setParameter,
    deleteParameter,
    getParameters,
    getNodes,
    getNodeInfo,
    sendNavigationGoal,
    cancelNavigationGoal,
    getCurrentPose,
    sendVelocityCommand,
    emergencyStop,
    getSensorData,
    getSystemStatus,
    subscriptions,
  };

  return (
    <ROSIntegrationContext.Provider value={contextValue}>
      {children}
    </ROSIntegrationContext.Provider>
  );
};

// Helper hook for real-time topic data
export const useROSTopic = (topic: string, enabled: boolean = true) => {
  const { subscribeTopic, unsubscribeTopic } = useROSIntegration();
  const [data, setData] = useState<any>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!enabled) return;

    const handleData = (topicData: any) => {
      setData(topicData);
      setError(null);
    };

    subscribeTopic(topic, handleData).catch((err) => {
      setError(err.message || "Failed to subscribe to topic");
    });

    return () => {
      unsubscribeTopic(topic);
    };
  }, [topic, enabled, subscribeTopic, unsubscribeTopic]);

  return { data, error };
};

// Helper hook for service calls
export const useROSService = () => {
  const { callService } = useROSIntegration();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const call = async (service: string, args?: any) => {
    setLoading(true);
    setError(null);

    try {
      const response = await callService(service, args);
      if (!response.success) {
        throw new Error(response.error || "Service call failed");
      }
      return response.data;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : "Unknown error";
      setError(errorMessage);
      throw err;
    } finally {
      setLoading(false);
    }
  };

  return { call, loading, error };
};
