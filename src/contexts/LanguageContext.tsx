import { createContext, useContext, useState, ReactNode } from "react";

type Language = "th" | "en";

interface LanguageContextType {
  language: Language;
  setLanguage: (lang: Language) => void;
  t: (key: string) => string;
}

const translations = {
  en: {
    // Navigation
    "nav.dashboard": "Dashboard",
    "nav.robotControl": "Robot Control",
    "nav.sequences": "Sequences",
    "nav.cameras": "Camera Feeds",
    "nav.sensors": "Sensors",
    "nav.navigation": "Navigation",
    "nav.system": "System Monitor",
    "nav.roboticArm": "Robotic Arm",
    "nav.holonomic": "Holonomic Drive",
    "nav.ioConfig": "I/O Configuration",
    "nav.features": "Feature Management",
    "nav.tfTree": "TF Tree",
    "nav.mapView": "Map Viewer",
    "nav.nodes": "Nodes",
    "nav.topics": "Topics",
    "nav.services": "Services",
    "nav.parameters": "Parameters",
    "nav.logs": "Logs",
    "nav.terminal": "Terminal",
    "nav.microcontrollers": "Microcontrollers",
    "nav.codeDevelopment": "Code Development",
    "nav.about": "About",
    "nav.settings": "Settings",

    // Common
    "common.save": "Save",
    "common.cancel": "Cancel",
    "common.delete": "Delete",
    "common.edit": "Edit",
    "common.add": "Add",
    "common.start": "Start",
    "common.stop": "Stop",
    "common.pause": "Pause",
    "common.resume": "Resume",
    "common.status": "Status",
    "common.active": "Active",
    "common.inactive": "Inactive",
    "common.connected": "Connected",
    "common.disconnected": "Disconnected",
    "common.online": "Online",
    "common.offline": "Offline",

    // App Info
    "app.name": "Dino Core",
    "app.subtitle": "ROS Control Interface",
    "app.systemStatus": "System Status",
    "app.allSystemsOperational": "All systems operational",

    // Theme
    "theme.light": "Light Mode",
    "theme.dark": "Dark Mode",
    "theme.system": "System",

    // Language
    "language.thai": "ไทย",
    "language.english": "English",

    // Robotic Arm
    "arm.title": "Robotic Arm Control",
    "arm.joints": "Joint Control",
    "arm.endEffector": "End Effector",
    "arm.position": "Position",
    "arm.orientation": "Orientation",
    "arm.gripper": "Gripper",
    "arm.preset": "Preset Positions",
    "arm.home": "Home Position",
    "arm.emergency": "Emergency Stop",

    // Holonomic
    "holonomic.title": "Holonomic Drive Control",
    "holonomic.velocity": "Velocity Control",
    "holonomic.linear": "Linear Velocity",
    "holonomic.angular": "Angular Velocity",
    "holonomic.strafe": "Strafe Velocity",
    "holonomic.wheels": "Wheel Status",
    "holonomic.kinematics": "Kinematics",

    // I/O Configuration
    "io.title": "I/O Configuration",
    "io.pins": "Pin Configuration",
    "io.pid": "PID Parameters",
    "io.kinematics": "Kinematics Parameters",
    "io.wheels": "Wheel Configuration",
    "io.arms": "Arm Configuration",
    "io.sensors": "Sensor Configuration",
    "io.imu": "IMU Configuration",

    // Navigation
    "navigation.title": "Navigation System",
    "navigation.goalSetting": "Navigation Goal",
    "navigation.currentPosition": "Current Position",
    "navigation.status": "Navigation Status",
    "navigation.startNavigation": "Start Navigation",
    "navigation.cancelGoal": "Cancel Goal",
    "navigation.statistics": "Navigation Statistics",
    "navigation.recentGoals": "Recent Goals",

    // System Monitor
    "system.title": "System Monitor",
    "system.cpu": "CPU Usage",
    "system.memory": "Memory Usage",
    "system.storage": "Storage",
    "system.power": "Power Status",
    "system.rosSystem": "ROS System",
    "system.network": "Network",
    "system.processes": "Process Monitor",

    // Nodes
    "nodes.title": "ROS Nodes",
    "nodes.totalNodes": "Total Nodes",
    "nodes.running": "Running",
    "nodes.stopped": "Stopped",
    "nodes.error": "Error",
    "nodes.nodeDetails": "Node Details",
    "nodes.quickActions": "Quick Actions",

    // Topics
    "topics.title": "ROS Topics",
    "topics.totalTopics": "Total Topics",
    "topics.active": "Active",
    "topics.bandwidth": "Bandwidth",
    "topics.messageInspector": "Message Inspector",
    "topics.liveMessages": "Live Messages",

    // Services
    "services.title": "ROS Services",
    "services.totalServices": "Total Services",
    "services.totalCalls": "Total Calls",
    "services.successRate": "Success Rate",
    "services.serviceCaller": "Service Caller",
    "services.callHistory": "Call History",

    // Microcontroller Connections
    "mcu.title": "Microcontroller Connections",
    "mcu.totalControllers": "Total Controllers",
    "mcu.connected": "Connected",
    "mcu.powerUsage": "Power Usage",
    "mcu.addConnection": "Add Connection",
    "mcu.connectionDetails": "Connection Details",
    "mcu.quickActions": "Quick Actions",

    // Task Management
    "task.addTask": "Add Task",
    "task.editTask": "Edit Task",
    "task.deleteTask": "Delete Task",
    "task.duplicateTask": "Duplicate Task",
    "task.advancedEdit": "Advanced Edit",
    "task.quickEdit": "Quick Edit",
    "task.taskName": "Task Name",
    "task.taskType": "Task Type",
    "task.description": "Description",
    "task.timeout": "Timeout",
    "task.retries": "Max Retries",
    "task.waitForFeedback": "Wait for Feedback",
    "task.feedbackTimeout": "Feedback Timeout",
    "task.basicSettings": "Basic Settings",
    "task.parameters": "Parameters",
    "task.advanced": "Advanced",
    "task.configureTask": "Configure Task",

    // Movement Types
    "movement.robotMovement": "Robot Movement",
    "movement.moveWithObstacleCheck": "Move with Obstacle Check",
    "movement.armManipulation": "Arm Manipulation",
    "movement.visionProcessing": "Vision Processing",
    "movement.sensorReading": "Sensor Reading",
    "movement.aiProcessing": "AI Processing",
    "movement.safetyCheck": "Safety Check",
    "movement.communication": "Communication",
    "movement.voiceCommand": "Voice Command",
    "movement.dataLogging": "Data Logging",
    "movement.maintenance": "Maintenance",
    "movement.retryControl": "Retry Control",
    "movement.checkpoint": "Checkpoint",

    // Movement Modes
    "movement.moveToPosition": "Move to Position",
    "movement.manualRelative": "Manual Relative",
    "movement.patternMovement": "Pattern Movement",
    "movement.autoNav": "Auto Navigation",
    "movement.intelligentNav": "Intelligent Nav",
    "movement.adaptiveNav": "Adaptive Nav",
    "movement.safeNav": "Safe Nav",

    // Sequences
    "sequence.title": "Advanced Sequence Management",
    "sequence.subtitle":
      "Create, manage, and execute intelligent task sequences",
    "sequence.noTasks": "No Tasks",
    "sequence.addFirstTask": "Add First Task",
    "sequence.createSequence": "Create Sequence",
    "sequence.importSequence": "Import",
    "sequence.exportSequence": "Export",
    "sequence.sequenceLibrary": "Sequence Library",
    "sequence.totalTasks": "Total Tasks",
    "sequence.completed": "Completed",
    "sequence.running": "Running",
    "sequence.failed": "Failed",
    "sequence.overallProgress": "Overall Progress",

    // Obstacle Detection
    "obstacle.detectionMode": "Detection Mode",
    "obstacle.avoidanceStrategy": "Avoidance Strategy",
    "obstacle.safetyDistance": "Safety Distance",
    "obstacle.detectionRange": "Detection Range",
    "obstacle.knownRegions": "Known Obstacle Regions",
    "obstacle.loadFromCostmap": "Load from Costmap",
    "obstacle.addRegion": "Add Region",
    "obstacle.regionName": "Region Name",
    "obstacle.currentRegions": "Current Obstacle Regions",
    "obstacle.continuous": "Continuous",
    "obstacle.periodic": "Periodic",
    "obstacle.triggerBased": "Trigger Based",
    "obstacle.smartAvoid": "Smart Avoid",
    "obstacle.wait": "Wait",
    "obstacle.alternativeRoute": "Alternative Route",
    "obstacle.emergencyStop": "Emergency Stop",
    "obstacle.slowDown": "Slow Down",

    // Pattern Movement
    "pattern.configuration": "Movement Pattern Configuration",
    "pattern.enablePattern": "Enable Pattern Movement",
    "pattern.quickPresets": "Quick Pattern Presets",
    "pattern.square": "Square",
    "pattern.circle": "Circle",
    "pattern.patrol": "Patrol",
    "pattern.clear": "Clear",
    "pattern.addWaypoint": "Add Waypoint",
    "pattern.currentPattern": "Current Pattern",
    "pattern.waypoints": "waypoints",
    "pattern.patternRepeat": "Pattern Repeat",
    "pattern.waitAtWaypoint": "Wait at Waypoint",
    "pattern.waypointTolerance": "Waypoint Tolerance",
    "pattern.targetPosition": "Target Position & Orientation",
    "pattern.relativeMovement": "Relative Movement (from current position)",
    "pattern.xPosition": "X Position",
    "pattern.yPosition": "Y Position",
    "pattern.yawAngle": "Yaw Angle",
    "pattern.xDistance": "X Distance",
    "pattern.yDistance": "Y Distance",
    "pattern.yawChange": "Yaw Change",
  },
  th: {
    // Navigation
    "nav.dashboard": "แดชบอร์ด",
    "nav.robotControl": "ควบคุมหุ่นยนต์",
    "nav.sequences": "ลำดับการทำงาน",
    "nav.cameras": "ฟีดกล้อง",
    "nav.sensors": "เซ็นเซอร์",
    "nav.navigation": "การนำทาง",
    "nav.system": "ตรวจสอบระบบ",
    "nav.roboticArm": "แขนกล",
    "nav.holonomic": "ระบบล้อโฮโลโนมิก",
    "nav.ioConfig": "ตั้งค่า I/O",
    "nav.features": "จัดการฟีเจ��ร์",
    "nav.tfTree": "แผนผัง TF",
    "nav.mapView": "แสดงแผนที่",
    "nav.nodes": "โหนด",
    "nav.topics": "หัวข้อ",
    "nav.services": "บริการ",
    "nav.parameters": "พารามิเตอร์",
    "nav.logs": "บันทึก",
    "nav.terminal": "เทอร์มินัล",
    "nav.microcontrollers": "ไมโครคอนโทรลเลอร์",
    "nav.codeDevelopment": "การพัฒนาโค้ด",
    "nav.about": "เกี่ยวกับ",
    "nav.settings": "การตั้งค่า",

    // Common
    "common.save": "บันทึก",
    "common.cancel": "ยกเลิก",
    "common.delete": "ลบ",
    "common.edit": "แก้ไข",
    "common.add": "เพิ่ม",
    "common.start": "เริ่ม",
    "common.stop": "หยุด",
    "common.pause": "พัก",
    "common.resume": "ดำเนินต่อ",
    "common.status": "สถานะ",
    "common.active": "ใช้งาน",
    "common.inactive": "ไม่ใช้งาน",
    "common.connected": "เชื่อมต่อ",
    "common.disconnected": "ขาดการเชื่อมต่อ",
    "common.online": "ออนไลน์",
    "common.offline": "ออฟไลน์",

    // App Info
    "app.name": "ไดโน คอร์",
    "app.subtitle": "อินเทอร์เฟซควบคุม ROS",
    "app.systemStatus": "สถานะระบบ",
    "app.allSystemsOperational": "ระบบทั้งหมดทำงานปกติ",

    // Theme
    "theme.light": "โหมดสว่าง",
    "theme.dark": "โหมดมืด",
    "theme.system": "ตามระบบ",

    // Language
    "language.thai": "ไทย",
    "language.english": "English",

    // Robotic Arm
    "arm.title": "ควบคุมแขนกล",
    "arm.joints": "ควบคุมข้อต่อ",
    "arm.endEffector": "ปลายแขน",
    "arm.position": "ตำแหน่ง",
    "arm.orientation": "การหมุน",
    "arm.gripper": "คีมจับ",
    "arm.preset": "ตำแหน่งที่บันทึก",
    "arm.home": "ตำแหน่งบ้าน",
    "arm.emergency": "หยุดฉุกเฉิน",

    // Holonomic
    "holonomic.title": "ควบคุมระบบล้อโฮโลโนมิก",
    "holonomic.velocity": "ควบคุมความเร็ว",
    "holonomic.linear": "ความเร็วเชิงเส้น",
    "holonomic.angular": "ความเร็วเชิงมุม",
    "holonomic.strafe": "ความเร็วเคลื่อนข้าง",
    "holonomic.wheels": "สถานะล้อ",
    "holonomic.kinematics": "จลนศาสตร์",

    // I/O Configuration
    "io.title": "การตั้งค่า I/O",
    "io.pins": "การตั้งค่าพิน",
    "io.pid": "พารามิเตอร์ PID",
    "io.kinematics": "พารามิเตอร์จลนศาสตร์",
    "io.wheels": "การตั้งค่าล้อ",
    "io.arms": "การตั้งค่าแขนกล",
    "io.sensors": "การตั้งค่าเซ็นเซอร์",
    "io.imu": "การตั้งค่า IMU",

    // Navigation
    "navigation.title": "ระบบนำทาง",
    "navigation.goalSetting": "จุดหมายการนำทาง",
    "navigation.currentPosition": "ตำแหน่งปัจจุบัน",
    "navigation.status": "สถานะการนำทาง",
    "navigation.startNavigation": "เริ่มการนำทาง",
    "navigation.cancelGoal": "ยกเลิกจุดหมาย",
    "navigation.statistics": "สถิติการนำทาง",
    "navigation.recentGoals": "จุดหมายล่าสุด",

    // System Monitor
    "system.title": "ตรวจสอบระบบ",
    "system.cpu": "การใช้งาน CPU",
    "system.memory": "การใช้งานหน่วยความจำ",
    "system.storage": "พื้นที่จัดเก็บ",
    "system.power": "สถานะพลังงาน",
    "system.rosSystem": "ระบบ ROS",
    "system.network": "เครือข่าย",
    "system.processes": "ตรวจสอบกระบวนการ",

    // Nodes
    "nodes.title": "โหนด ROS",
    "nodes.totalNodes": "โหนดทั้งหมด",
    "nodes.running": "กำลังทำงาน",
    "nodes.stopped": "หยุดทำงาน",
    "nodes.error": "ข้อผิดพลาด",
    "nodes.nodeDetails": "รายละเอียดโหนด",
    "nodes.quickActions": "การดำเนินการด่วน",

    // Topics
    "topics.title": "หัวข้อ ROS",
    "topics.totalTopics": "หัวข้อทั้งหมด",
    "topics.active": "ใช้งานอยู่",
    "topics.bandwidth": "แบนด์วิดท์",
    "topics.messageInspector": "ตรวจสอบข้อความ",
    "topics.liveMessages": "ข้อความสด",

    // Services
    "services.title": "บริการ ROS",
    "services.totalServices": "บริการทั้งหมด",
    "services.totalCalls": "การเรียกทั้งหมด",
    "services.successRate": "อัตราความสำเร็จ",
    "services.serviceCaller": "เรียกบริการ",
    "services.callHistory": "ประวัติการเรียก",

    // Microcontroller Connections
    "mcu.title": "การเชื่อมต่อไมโครคอนโทรลเลอร์",
    "mcu.totalControllers": "คอนโทรลเลอร์ทั้งหมด",
    "mcu.connected": "เชื่อมต่อแล้ว",
    "mcu.powerUsage": "การใช้พลังงาน",
    "mcu.addConnection": "เพิ่มการเชื่อมต่อ",
    "mcu.connectionDetails": "รายละเอียดการเชื่อมต่อ",
    "mcu.quickActions": "การดำเนินการด่วน",

    // Task Management
    "task.addTask": "เพิ่มงาน",
    "task.editTask": "แก้ไขงาน",
    "task.deleteTask": "ลบงาน",
    "task.duplicateTask": "ทำสำเนางาน",
    "task.advancedEdit": "แก้ไขขั้นสูง",
    "task.quickEdit": "แก้ไขเร็ว",
    "task.taskName": "ชื่องาน",
    "task.taskType": "ประเภทงาน",
    "task.description": "รายละเอียด",
    "task.timeout": "เวลาหมดอายุ",
    "task.retries": "จำนวนลองใหม่สูงสุด",
    "task.waitForFeedback": "รอการตอบกลับ",
    "task.feedbackTimeout": "เวลาหมดอายุการตอบกลับ",
    "task.basicSettings": "การตั้งค่าพื้นฐาน",
    "task.parameters": "พารามิเตอร์",
    "task.advanced": "ขั้นสูง",
    "task.configureTask": "ตั้งค่างาน",

    // Movement Types
    "movement.robotMovement": "การเคลื่อนที่ของหุ่นยนต์",
    "movement.moveWithObstacleCheck": "เคลื่อนที่พร้อมตรวจสิ่งกีดขวาง",
    "movement.armManipulation": "การควบคุมแขนกล",
    "movement.visionProcessing": "การประมวลผลภาพ",
    "movement.sensorReading": "การอ่านเซ็นเซอร์",
    "movement.aiProcessing": "การประมวลผล AI",
    "movement.safetyCheck": "การตรวจสอบความปลอดภัย",
    "movement.communication": "การสื่อสาร",
    "movement.voiceCommand": "คำสั่งเสียง",
    "movement.dataLogging": "การบันทึกข้อมูล",
    "movement.maintenance": "การบำรุงรักษา",
    "movement.retryControl": "การควบคุมลองใหม่",
    "movement.checkpoint": "จุดตรวจสอบ",

    // Movement Modes
    "movement.moveToPosition": "เคลื่อนที่ไปตำแหน่ง",
    "movement.manualRelative": "เคลื่อนที่แบบสัมพัทธ์",
    "movement.patternMovement": "เคลื่อนที่ตามรูปแบบ",
    "movement.autoNav": "นำทางอัตโนมัติ",
    "movement.intelligentNav": "นำทางอัจฉริยะ",
    "movement.adaptiveNav": "นำทางปรับตัว",
    "movement.safeNav": "นำทางปลอดภัย",

    // Sequences
    "sequence.title": "การจัดการลำดับงานขั้นสูง",
    "sequence.subtitle": "สร้าง จัดการ และดำเนินการลำดับงานอัจฉริยะ",
    "sequence.noTasks": "ไม่มีงาน",
    "sequence.addFirstTask": "เพิ่มงานแรก",
    "sequence.createSequence": "สร้างลำดับงาน",
    "sequence.importSequence": "นำเข้า",
    "sequence.exportSequence": "ส่งออก",
    "sequence.sequenceLibrary": "คลังลำดับงาน",
    "sequence.totalTasks": "งานทั้งหมด",
    "sequence.completed": "เสร็จสิ้น",
    "sequence.running": "กำลังทำงาน",
    "sequence.failed": "ล้มเหลว",
    "sequence.overallProgress": "ความคืบหน้าโดยรวม",

    // Obstacle Detection
    "obstacle.detectionMode": "โหมดการตรวจจับ",
    "obstacle.avoidanceStrategy": "กลยุทธ์การห��ีกเลี่ยง",
    "obstacle.safetyDistance": "ระยะห่างปลอดภัย",
    "obstacle.detectionRange": "ระยะการตรวจจับ",
    "obstacle.knownRegions": "พื้นที่สิ่งกีดขวางที่ทราบ",
    "obstacle.loadFromCostmap": "โหลด��าก Costmap",
    "obstacle.addRegion": "เพิ่มพื้นที่",
    "obstacle.regionName": "ชื่อพื้นที่",
    "obstacle.currentRegions": "พื้นที่สิ่งกีดขวางปัจ��ุบัน",
    "obstacle.continuous": "ต่อเนื่อง",
    "obstacle.periodic": "เป็นช่วงๆ",
    "obstacle.triggerBased": "ตามสัญญาณ",
    "obstacle.smartAvoid": "หลีกเลี่ยงอัจฉริยะ",
    "obstacle.wait": "รอ",
    "obstacle.alternativeRoute": "เส้นทางทางเลือก",
    "obstacle.emergencyStop": "หยุดฉุกเฉิน",
    "obstacle.slowDown": "ลดความเร็ว",

    // Pattern Movement
    "pattern.configuration": "การตั้งค่ารูปแบบการเคลื่อนที่",
    "pattern.enablePattern": "เปิดใช้การเคลื่อนที่ตามรูปแบบ",
    "pattern.quickPresets": "รูปแบบสำเร็จรูป",
    "pattern.square": "สี่เหลี่ยม",
    "pattern.circle": "วงกลม",
    "pattern.patrol": "ลาดตระเวน",
    "pattern.clear": "ล้าง",
    "pattern.addWaypoint": "เพิ่มจุดหมาย",
    "pattern.currentPattern": "รูปแบบปัจจุบัน",
    "pattern.waypoints": "จุดหมาย",
    "pattern.patternRepeat": "ทำซ้ำรูปแบบ",
    "pattern.waitAtWaypoint": "รอที่จุดห��าย",
    "pattern.waypointTolerance": "ค่าคลาดเคลื่อนจุดหมาย",
    "pattern.targetPosition": "ตำแหน่งเป้าหมายและการหมุน",
    "pattern.relativeMovement": "การเคลื่อนที่แบบสัมพัทธ์ (จากตำแหน่งปัจจุบัน)",
    "pattern.xPosition": "ตำแหน่ง X",
    "pattern.yPosition": "ตำแหน่ง Y",
    "pattern.yawAngle": "มุม Yaw",
    "pattern.xDistance": "ระยะทาง X",
    "pattern.yDistance": "ระยะทาง Y",
    "pattern.yawChange": "การเปลี่ยนแปลง Yaw",
  },
};

const LanguageContext = createContext<LanguageContextType | undefined>(
  undefined,
);

export function LanguageProvider({ children }: { children: ReactNode }) {
  const [language, setLanguage] = useState<Language>("en");

  const t = (key: string): string => {
    return (
      translations[language][key as keyof (typeof translations)["en"]] || key
    );
  };

  return (
    <LanguageContext.Provider value={{ language, setLanguage, t }}>
      {children}
    </LanguageContext.Provider>
  );
}

export function useLanguage() {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error("useLanguage must be used within a LanguageProvider");
  }
  return context;
}
