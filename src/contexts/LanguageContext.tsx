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
    "nav.virtualRviz": "Virtual RViz",
    "nav.nodes": "Nodes",
    "nav.topics": "Topics",
    "nav.services": "Services",
    "nav.parameters": "Parameters",
    "nav.logs": "Log Viewer",
    "nav.terminal": "ROS Terminal",
    "nav.microcontrollers": "Microcontrollers",
    "nav.codeDevelopment": "Code Development",
    "nav.systemConfiguration": "System Configuration",
    "nav.about": "About",
    "nav.settings": "Settings",

    // Common
    "common.save": "Save",
    "common.cancel": "Cancel",
    "common.delete": "Delete",
    "common.edit": "Edit",
    "common.add": "Add",
    "common.create": "Create",
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
    "common.error": "Error",

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

    // Map Viewer
    "map.title": "Map Viewer",
    "map.subtitle":
      "Create, manage, and view maps from ROS system with real-time updates",
    "map.availableMaps": "Available Maps",
    "map.manageMaps": "Manage your navigation maps",
    "map.createMap": "Create Map",
    "map.createNewMap": "Create New Map",
    "map.createNewMapDescription":
      "Create a new navigation map from current ROS map server",
    "map.mapName": "Map Name",
    "map.enterMapName": "Enter map name",
    "map.description": "Description",
    "map.enterDescription": "Enter map description",
    "map.deleteMap": "Delete Map",
    "map.deleteMapConfirmation":
      "Are you sure you want to delete this map? This action cannot be undone.",
    "map.mapCreated": "Map Created",
    "map.mapCreatedSuccessfully": "Map has been created successfully",
    "map.mapDeleted": "Map Deleted",
    "map.mapDeletedSuccessfully": "Map has been deleted successfully",
    "map.createMapError": "Failed to create map",
    "map.deleteMapError": "Failed to delete map",
    "map.noMapSelected": "No Map Selected",
    "map.selectMapToView":
      "Select a map from the list to view and interact with it",
    "map.selectMapFromList":
      "Select a map from the left panel to start viewing",
    "map.resolution": "Resolution",
    "map.size": "Size",
    "map.fileSize": "File Size",
    "map.lastModified": "Last Modified",
    "map.coordinates": "Coordinates",
    "map.goalSet": "Navigation Goal Set",
    "map.displaySettings": "Display Settings",
    "map.showGrid": "Show Grid",
    "map.showRobot": "Show Robot",
    "map.showGoal": "Show Goal",
    "map.showTrajectory": "Show Trajectory",
    "map.refreshRate": "Refresh Rate",
    "map.occupancyThreshold": "Occupancy Threshold",
    "map.zoom": "Zoom",
    "map.robotPos": "Robot Position",
    "map.goalPos": "Goal Position",
    "map.playing": "Playing",
    "map.paused": "Paused",
    "map.fps": "FPS",
    "map.mapSize": "Map Size",
    "map.dimensions": "Dimensions",
    "map.origin": "Origin",

    // Virtual RViz
    "rviz.title": "Virtual RViz",
    "rviz.subtitle":
      "Professional 3D visualization interface for ROS data with real-time rendering",
    "rviz.displays": "Displays",
    "rviz.manageDisplays": "Manage visualization displays",
    "rviz.addDisplay": "Add Display",
    "rviz.addNewDisplay": "Add New Display",
    "rviz.addDisplayDescription":
      "Add a new visualization display to the 3D scene",
    "rviz.displayName": "Display Name",
    "rviz.enterDisplayName": "Enter display name",
    "rviz.displayType": "Display Type",
    "rviz.topic": "Topic",
    "rviz.selectTopic": "Select topic",
    "rviz.displayAdded": "Display Added",
    "rviz.displayAddedSuccessfully": "Display has been added successfully",
    "rviz.viewer3D": "3D Viewer",
    "rviz.interactiveVisualization":
      "Interactive 3D visualization with real-time data",
    "rviz.viewSettings": "View Settings",
    "rviz.showGrid": "Show Grid",
    "rviz.showAxes": "Show Axes",
    "rviz.backgroundColor": "Background Color",
    "rviz.gridSize": "Grid Size",
    "rviz.fps": "FPS",
    "rviz.rendering": "Rendering",
    "rviz.paused": "Paused",
    "rviz.displayProperties": "Display Properties",
    "rviz.color": "Color",
    "rviz.size": "Size",
    "rviz.alpha": "Alpha",
    "rviz.frame": "Frame",

    // System Terminal
    "terminal.title": "System Terminal",
    "terminal.subtitle":
      "Execute system commands directly on the robot with real-time feedback",
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
    "nav.features": "จัดการฟีเจอร์",
    "nav.tfTree": "แผนผัง TF",
    "nav.mapView": "แสดงแผนที่",
    "nav.virtualRviz": "Virtual RViz",
    "nav.nodes": "โหนด",
    "nav.topics": "หัวข้อ",
    "nav.services": "บริการ",
    "nav.parameters": "พารามิเตอร์",
    "nav.logs": "ดูบันทึก",
    "nav.terminal": "เทอร์มินัล ROS",
    "nav.microcontrollers": "ไมโครคอนโทรลเลอร์",
    "nav.codeDevelopment": "การพัฒนาโค้ด",
    "nav.systemConfiguration": "การตั้งค่าระบบ",
    "nav.about": "เกี่ยวกับ",
    "nav.settings": "การตั้งค่า",

    // Common
    "common.save": "บันทึก",
    "common.cancel": "ยกเลิก",
    "common.delete": "ลบ",
    "common.edit": "แก้ไข",
    "common.add": "เพิ่ม",
    "common.create": "สร้าง",
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
    "common.error": "ข้อผิดพลาด",

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
    "holonomic.velocity": "ควบคุมค��ามเร็ว",
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
    "mcu.connectionDetails": "รายละเอีย���การเชื่อมต่อ",
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
    "movement.robotMovement": "การเคลื่อนท���่ของหุ่นยนต์",
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
    "obstacle.slowDown": "ลดควา���เร็ว",

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
    "pattern.waitAtWaypoint": "รอที่จุดหมาย",
    "pattern.waypointTolerance": "ค่าคลาดเคลื่อนจุดหมาย",
    "pattern.targetPosition": "ตำแหน่งเป้าหมายและการหมุน",
    "pattern.relativeMovement": "การเคลื่อนที่แบบสัมพัทธ์ (จากตำแหน่งปัจจุบัน)",
    "pattern.xPosition": "ตำแหน่ง X",
    "pattern.yPosition": "ตำแหน่ง Y",
    "pattern.yawAngle": "มุม Yaw",
    "pattern.xDistance": "ระยะทาง X",
    "pattern.yDistance": "ระยะทาง Y",
    "pattern.yawChange": "การเปลี่ยนแปลง Yaw",

    // Map Viewer
    "map.title": "แสดงแผนที่",
    "map.subtitle":
      "สร้าง จัดการ และดูแผนที่จากระบบ ROS พร้อมการอัปเดตแบบเรียลไทม์",
    "map.availableMaps": "แผนที่ที่มีอยู่",
    "map.manageMaps": "จัดการแผนที่นำทางของคุณ",
    "map.createMap": "สร้างแผนที่",
    "map.createNewMap": "สร้างแผนที่ใหม่",
    "map.createNewMapDescription":
      "สร้างแผนที่นำทางใหม่จาก ROS map server ปัจจุบัน",
    "map.mapName": "ชื่อแผนที่",
    "map.enterMapName": "ใส่ชื่อแผนที่",
    "map.description": "คำอธิบาย",
    "map.enterDescription": "ใส่คำอธิบาย",
    "map.deleteMap": "ลบแผนที่",
    "map.deleteMapConfirmation":
      "คุณแน่ใจหรือไม่ที่จะลบแผนที่นี้? การกระทำนี้ไม่สามารถยกเลิกได้",
    "map.mapCreated": "สร้างแผนที่สำเร็จ",
    "map.mapCreatedSuccessfully": "สร้างแผนที่เรียบร้อยแล้ว",
    "map.mapDeleted": "ลบแผนที่สำเร็จ",
    "map.mapDeletedSuccessfully": "ลบแผนที่เรียบร้อยแล้ว",
    "map.createMapError": "สร้างแผนที่ไม่สำเร็จ",
    "map.deleteMapError": "ลบแผนที่ไม่สำเร็จ",
    "map.noMapSelected": "ไม่ได้เลือกแผนที่",
    "map.selectMapToView": "เลือกแผนที่จากรายการเพื่อดูและโต้ตอบ",
    "map.selectMapFromList": "เลือกแผนที่จากแผงซ้ายเพื่อเริ่มดู",
    "map.resolution": "ความละเอียด",
    "map.size": "ขนาด",
    "map.fileSize": "ขนาดไฟล์",
    "map.lastModified": "แก้ไขล่าสุด",
    "map.coordinates": "พิกัด",
    "map.goalSet": "ตั้งเป้าหมายการนำทาง",
    "map.displaySettings": "การตั้ง��่าการแสดงผล",
    "map.showGrid": "แสดงตาราง",
    "map.showRobot": "แสดงหุ่นยนต์",
    "map.showGoal": "แสดงเป้าหมาย",
    "map.showTrajectory": "แสดงเส้นทาง",
    "map.refreshRate": "อัตราการรีเฟรช",
    "map.occupancyThreshold": "เกณฑ์การครอบครอง",
    "map.zoom": "ซูม",
    "map.robotPos": "ตำแหน่งหุ่นยนต์",
    "map.goalPos": "ตำแหน่งเป้าหมาย",
    "map.playing": "กำลังเล่น",
    "map.paused": "หยุดชั่วคราว",
    "map.fps": "FPS",
    "map.mapSize": "ขนาดแผนที่",
    "map.dimensions": "มิติ",
    "map.origin": "จุดกำเนิด",

    // Virtual RViz
    "rviz.title": "Virtual RViz",
    "rviz.subtitle":
      "อินเทอร์เฟซการแสดงผลแบบ 3D สำหรับข้อมูล ROS พร้อมการเรนเดอร์แบบเรียลไทม์",
    "rviz.displays": "การแสดงผล",
    "rviz.manageDisplays": "จัดการการแ��ดงผลภาพ",
    "rviz.addDisplay": "เพิ่มการแสดงผล",
    "rviz.addNewDisplay": "เพิ่มการแสดงผลใหม่",
    "rviz.addDisplayDescription": "เพิ่มการแสดงผลใหม่ไปยังฉาก 3D",
    "rviz.displayName": "ชื่อการแสดงผล",
    "rviz.enterDisplayName": "ใส่ชื่อการแสดงผล",
    "rviz.displayType": "ประเภทการแสดงผล",
    "rviz.topic": "หัวข้อ",
    "rviz.selectTopic": "เลือกหัวข้อ",
    "rviz.displayAdded": "เพิ่มการแสดงผลสำเร็จ",
    "rviz.displayAddedSuccessfully": "เพิ่มการแสดงผลเรียบร้อยแล้ว",
    "rviz.viewer3D": "เครื่องมือดู 3D",
    "rviz.interactiveVisualization":
      "การแสดงผล 3D แบบโต้ตอบพร้อมข้อมูลเรียลไทม์",
    "rviz.viewSettings": "การตั้งค่าการดู",
    "rviz.showGrid": "แสดงตาราง",
    "rviz.showAxes": "แสดงแกน",
    "rviz.backgroundColor": "สีพื้นหลัง",
    "rviz.gridSize": "ขนาดตาราง",
    "rviz.fps": "FPS",
    "rviz.rendering": "กำลังเรนเดอร์",
    "rviz.paused": "หยุดชั่วคราว",
    "rviz.displayProperties": "คุณสมบัติการแสดงผล",
    "rviz.color": "สี",
    "rviz.size": "ขนาด",
    "rviz.alpha": "ความโปร่งใส",
    "rviz.frame": "เฟรม",

    // System Terminal
    "terminal.title": "เทอร์มินัลระบบ",
    "terminal.subtitle": "สั่งงานระบบโดยตรงบนหุ่นยนต์พร้อมผลลัพธ์แบบเรียลไทม์",
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
