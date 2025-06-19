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
    "nav.nodes": "โหนด",
    "nav.topics": "หัวข้อ",
    "nav.services": "บริการ",
    "nav.parameters": "พารามิเตอร์",
    "nav.logs": "บันทึก",
    "nav.terminal": "เทอร์มินัล",
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
