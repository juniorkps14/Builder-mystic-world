<template>
  <div class="min-h-screen tesla-ui p-6">
    <!-- Header Section -->
    <div class="mb-8">
      <div class="tesla-card p-6">
        <div class="flex items-center justify-between">
          <div>
            <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent mb-2 tesla-title">
              Robotics Control Center
            </h1>
            <p class="text-slate-400 text-lg font-light tesla-subtitle">
              Comprehensive autonomous system management and monitoring
            </p>
          </div>
          <div class="flex items-center gap-4">
            <div class="text-right">
              <div class="text-2xl font-light text-white">{{ currentTime }}</div>
              <div class="text-sm text-slate-400">{{ currentDate }}</div>
            </div>
            <div class="w-16 h-16 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-2xl flex items-center justify-center shadow-xl tesla-interactive">
              <Zap class="h-8 w-8 text-white" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- System Status Cards -->
    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
      <!-- Robot Status -->
      <n-card class="tesla-glass-card hover:scale-105 transition-all duration-300">
        <div class="flex items-center justify-between">
          <div>
            <p class="text-slate-400 text-sm font-medium">Robot Status</p>
            <p class="text-2xl font-light text-white mt-1">Online</p>
            <div class="flex items-center gap-2 mt-2">
              <div class="w-2 h-2 bg-emerald-400 rounded-full animate-pulse" />
              <span class="text-xs text-emerald-400">Connected</span>
            </div>
          </div>
          <div class="h-12 w-12 bg-gradient-to-br from-emerald-500/20 to-green-500/20 rounded-xl flex items-center justify-center">
            <Bot class="h-6 w-6 text-emerald-400" />
          </div>
        </div>
      </n-card>

      <!-- Active Tasks -->
      <n-card class="tesla-glass-card hover:scale-105 transition-all duration-300">
        <div class="flex items-center justify-between">
          <div>
            <p class="text-slate-400 text-sm font-medium">Active Tasks</p>
            <p class="text-2xl font-light text-white mt-1">{{ systemStats.activeTasks }}</p>
            <p class="text-xs text-blue-400 mt-2">{{ systemStats.completedTasks }} completed today</p>
          </div>
          <div class="h-12 w-12 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center">
            <Play class="h-6 w-6 text-blue-400" />
          </div>
        </div>
      </n-card>

      <!-- System Health -->
      <n-card class="tesla-glass-card hover:scale-105 transition-all duration-300">
        <div class="flex items-center justify-between">
          <div>
            <p class="text-slate-400 text-sm font-medium">System Health</p>
            <p class="text-2xl font-light text-white mt-1">{{ systemStats.health }}%</p>
            <div class="mt-2">
              <n-progress :percentage="systemStats.health" size="small" color="#10b981" />
            </div>
          </div>
          <div class="h-12 w-12 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center">
            <Activity class="h-6 w-6 text-purple-400" />
          </div>
        </div>
      </n-card>

      <!-- Uptime -->
      <n-card class="tesla-glass-card hover:scale-105 transition-all duration-300">
        <div class="flex items-center justify-between">
          <div>
            <p class="text-slate-400 text-sm font-medium">Uptime</p>
            <p class="text-2xl font-light text-white mt-1">{{ systemStats.uptime }}</p>
            <p class="text-xs text-slate-400 mt-2">Since last restart</p>
          </div>
          <div class="h-12 w-12 bg-gradient-to-br from-yellow-500/20 to-orange-500/20 rounded-xl flex items-center justify-center">
            <Clock class="h-6 w-6 text-yellow-400" />
          </div>
        </div>
      </n-card>
    </div>

    <!-- Main Dashboard Grid -->
    <div class="grid grid-cols-1 lg:grid-cols-3 gap-8 mb-8">
      <!-- Quick Actions -->
      <div class="lg:col-span-1">
        <n-card class="tesla-glass-card">
          <template #header>
            <h3 class="text-xl font-light text-white">Quick Actions</h3>
          </template>
          <div class="space-y-3">
            <router-link to="/sequences" class="block">
              <div class="flex items-center p-4 bg-white/5 rounded-xl hover:bg-white/10 transition-all duration-300 cursor-pointer">
                <div class="h-10 w-10 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-lg flex items-center justify-center mr-4">
                  <Play class="h-5 w-5 text-blue-400" />
                </div>
                <div>
                  <h4 class="font-medium text-white">Start Sequence</h4>
                  <p class="text-sm text-slate-400">Execute automated tasks</p>
                </div>
                <ChevronRight class="h-5 w-5 text-slate-400 ml-auto" />
              </div>
            </router-link>

            <router-link to="/robot-control" class="block">
              <div class="flex items-center p-4 bg-white/5 rounded-xl hover:bg-white/10 transition-all duration-300 cursor-pointer">
                <div class="h-10 w-10 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-lg flex items-center justify-center mr-4">
                  <Gamepad2 class="h-5 w-5 text-purple-400" />
                </div>
                <div>
                  <h4 class="font-medium text-white">Manual Control</h4>
                  <p class="text-sm text-slate-400">Direct robot operation</p>
                </div>
                <ChevronRight class="h-5 w-5 text-slate-400 ml-auto" />
              </div>
            </router-link>

            <router-link to="/cameras" class="block">
              <div class="flex items-center p-4 bg-white/5 rounded-xl hover:bg-white/10 transition-all duration-300 cursor-pointer">
                <div class="h-10 w-10 bg-gradient-to-br from-green-500/20 to-emerald-500/20 rounded-lg flex items-center justify-center mr-4">
                  <Camera class="h-5 w-5 text-green-400" />
                </div>
                <div>
                  <h4 class="font-medium text-white">Camera Feeds</h4>
                  <p class="text-sm text-slate-400">Monitor visual systems</p>
                </div>
                <ChevronRight class="h-5 w-5 text-slate-400 ml-auto" />
              </div>
            </router-link>

            <router-link to="/system-monitoring" class="block">
              <div class="flex items-center p-4 bg-white/5 rounded-xl hover:bg-white/10 transition-all duration-300 cursor-pointer">
                <div class="h-10 w-10 bg-gradient-to-br from-orange-500/20 to-red-500/20 rounded-lg flex items-center justify-center mr-4">
                  <BarChart3 class="h-5 w-5 text-orange-400" />
                </div>
                <div>
                  <h4 class="font-medium text-white">System Monitor</h4>
                  <p class="text-sm text-slate-400">View system metrics</p>
                </div>
                <ChevronRight class="h-5 w-5 text-slate-400 ml-auto" />
              </div>
            </router-link>
          </div>
        </n-card>
      </div>

      <!-- System Metrics -->
      <div class="lg:col-span-2">
        <n-card class="tesla-glass-card">
          <template #header>
            <h3 class="text-xl font-light text-white">System Performance</h3>
          </template>
          <div class="grid grid-cols-1 md:grid-cols-3 gap-6">
            <!-- CPU Usage -->
            <div class="text-center">
              <div class="relative w-24 h-24 mx-auto mb-4">
                <svg class="w-24 h-24 transform -rotate-90" viewBox="0 0 100 100">
                  <circle
                    cx="50"
                    cy="50"
                    r="40"
                    stroke="rgba(255,255,255,0.1)"
                    stroke-width="8"
                    fill="none"
                  />
                  <circle
                    cx="50"
                    cy="50"
                    r="40"
                    stroke="url(#cpuGradient)"
                    stroke-width="8"
                    stroke-linecap="round"
                    fill="none"
                    :stroke-dasharray="`${metrics.cpu * 2.51} 251`"
                    class="transition-all duration-500"
                  />
                  <defs>
                    <linearGradient id="cpuGradient" x1="0%" y1="0%" x2="100%" y2="0%">
                      <stop offset="0%" style="stop-color:#3b82f6" />
                      <stop offset="100%" style="stop-color:#06b6d4" />
                    </linearGradient>
                  </defs>
                </svg>
                <div class="absolute inset-0 flex items-center justify-center">
                  <span class="text-xl font-light text-white">{{ metrics.cpu }}%</span>
                </div>
              </div>
              <h4 class="font-medium text-white">CPU Usage</h4>
              <p class="text-sm text-slate-400">Processing Load</p>
            </div>

            <!-- Memory Usage -->
            <div class="text-center">
              <div class="relative w-24 h-24 mx-auto mb-4">
                <svg class="w-24 h-24 transform -rotate-90" viewBox="0 0 100 100">
                  <circle
                    cx="50"
                    cy="50"
                    r="40"
                    stroke="rgba(255,255,255,0.1)"
                    stroke-width="8"
                    fill="none"
                  />
                  <circle
                    cx="50"
                    cy="50"
                    r="40"
                    stroke="url(#memoryGradient)"
                    stroke-width="8"
                    stroke-linecap="round"
                    fill="none"
                    :stroke-dasharray="`${metrics.memory * 2.51} 251`"
                    class="transition-all duration-500"
                  />
                  <defs>
                    <linearGradient id="memoryGradient" x1="0%" y1="0%" x2="100%" y2="0%">
                      <stop offset="0%" style="stop-color:#8b5cf6" />
                      <stop offset="100%" style="stop-color:#d946ef" />
                    </linearGradient>
                  </defs>
                </svg>
                <div class="absolute inset-0 flex items-center justify-center">
                  <span class="text-xl font-light text-white">{{ metrics.memory }}%</span>
                </div>
              </div>
              <h4 class="font-medium text-white">Memory</h4>
              <p class="text-sm text-slate-400">RAM Usage</p>
            </div>

            <!-- Network -->
            <div class="text-center">
              <div class="relative w-24 h-24 mx-auto mb-4">
                <svg class="w-24 h-24 transform -rotate-90" viewBox="0 0 100 100">
                  <circle
                    cx="50"
                    cy="50"
                    r="40"
                    stroke="rgba(255,255,255,0.1)"
                    stroke-width="8"
                    fill="none"
                  />
                  <circle
                    cx="50"
                    cy="50"
                    r="40"
                    stroke="url(#networkGradient)"
                    stroke-width="8"
                    stroke-linecap="round"
                    fill="none"
                    :stroke-dasharray="`${(100 - metrics.latency) * 2.51} 251`"
                    class="transition-all duration-500"
                  />
                  <defs>
                    <linearGradient id="networkGradient" x1="0%" y1="0%" x2="100%" y2="0%">
                      <stop offset="0%" style="stop-color:#10b981" />
                      <stop offset="100%" style="stop-color:#06b6d4" />
                    </linearGradient>
                  </defs>
                </svg>
                <div class="absolute inset-0 flex items-center justify-center">
                  <span class="text-xl font-light text-white">{{ metrics.latency }}ms</span>
                </div>
              </div>
              <h4 class="font-medium text-white">Network</h4>
              <p class="text-sm text-slate-400">Latency</p>
            </div>
          </div>
        </n-card>
      </div>
    </div>

    <!-- Recent Activity -->
    <div class="grid grid-cols-1 lg:grid-cols-2 gap-8">
      <!-- Recent Tasks -->
      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Recent Tasks</h3>
        </template>
        <div class="space-y-3">
          <div
            v-for="task in recentTasks"
            :key="task.id"
            class="flex items-center justify-between p-3 bg-white/5 rounded-lg hover:bg-white/10 transition-colors"
          >
            <div class="flex items-center gap-3">
              <div :class="[
                'w-2 h-2 rounded-full',
                task.status === 'completed' ? 'bg-emerald-400' :
                task.status === 'running' ? 'bg-blue-400' :
                task.status === 'failed' ? 'bg-red-400' : 'bg-yellow-400'
              ]" />
              <div>
                <h4 class="font-medium text-white text-sm">{{ task.name }}</h4>
                <p class="text-xs text-slate-400">{{ task.type }}</p>
              </div>
            </div>
            <div class="text-right">
              <n-tag :type="getStatusTagType(task.status)" size="small">
                {{ task.status }}
              </n-tag>
              <p class="text-xs text-slate-400 mt-1">{{ formatTime(task.timestamp) }}</p>
            </div>
          </div>
        </div>
      </n-card>

      <!-- System Logs -->
      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">System Logs</h3>
        </template>
        <n-scrollbar style="max-height: 300px">
          <div class="space-y-2">
            <div
              v-for="(log, index) in systemLogs"
              :key="index"
              class="flex gap-3 p-2 rounded-lg hover:bg-white/5 transition-colors"
            >
              <span class="text-xs font-mono text-slate-400 w-16 flex-shrink-0">{{ log.time }}</span>
              <span :class="[
                'text-xs font-medium w-12 flex-shrink-0',
                log.level === 'INFO' ? 'text-blue-400' :
                log.level === 'WARN' ? 'text-yellow-400' :
                log.level === 'ERROR' ? 'text-red-400' : 'text-green-400'
              ]">
                {{ log.level }}
              </span>
              <span class="text-xs text-slate-300 flex-1">{{ log.message }}</span>
            </div>
          </div>
        </n-scrollbar>
      </n-card>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import {
  Zap,
  Bot,
  Play,
  Activity,
  Clock,
  Gamepad2,
  Camera,
  BarChart3,
  ChevronRight
} from 'lucide-vue-next'

const currentTime = ref('')
const currentDate = ref('')
let timeInterval: number

const systemStats = ref({
  activeTasks: 3,
  completedTasks: 12,
  health: 94,
  uptime: '5d 14h'
})

const metrics = ref({
  cpu: 45,
  memory: 62,
  latency: 23
})

const recentTasks = ref([
  {
    id: '1',
    name: 'Security Patrol',
    type: 'navigation',
    status: 'completed',
    timestamp: new Date(Date.now() - 300000)
  },
  {
    id: '2',
    name: 'Object Detection',
    type: 'ai',
    status: 'running',
    timestamp: new Date(Date.now() - 60000)
  },
  {
    id: '3',
    name: 'Sensor Calibration',
    type: 'sensor',
    status: 'completed',
    timestamp: new Date(Date.now() - 1800000)
  }
])

const systemLogs = ref([
  { time: '14:32:15', level: 'INFO', message: 'System initialization completed successfully' },
  { time: '14:31:42', level: 'INFO', message: 'ROS node connections established' },
  { time: '14:31:15', level: 'WARN', message: 'High CPU usage detected, optimizing processes' },
  { time: '14:30:58', level: 'SUCCESS', message: 'All sensors operational and calibrated' },
  { time: '14:30:12', level: 'INFO', message: 'Network connection stable, latency 23ms' }
])

const getStatusTagType = (status: string) => {
  switch (status) {
    case 'completed': return 'success'
    case 'running': return 'info'
    case 'failed': return 'error'
    case 'paused': return 'warning'
    default: return 'default'
  }
}

const formatTime = (date: Date) => {
  const now = new Date()
  const diff = now.getTime() - date.getTime()

  if (diff < 60000) {
    return 'Just now'
  } else if (diff < 3600000) {
    return `${Math.floor(diff / 60000)}m ago`
  } else {
    return `${Math.floor(diff / 3600000)}h ago`
  }
}

const updateTime = () => {
  const now = new Date()
  currentTime.value = now.toLocaleTimeString()
  currentDate.value = now.toLocaleDateString('en-US', {
    weekday: 'long',
    year: 'numeric',
    month: 'long',
    day: 'numeric'
  })
}

const updateMetrics = () => {
  // Simulate real-time metrics updates
  metrics.value.cpu = Math.floor(Math.random() * 30) + 40
  metrics.value.memory = Math.floor(Math.random() * 20) + 55
  metrics.value.latency = Math.floor(Math.random() * 20) + 15
}

onMounted(() => {
  updateTime()
  timeInterval = setInterval(() => {
    updateTime()
    updateMetrics()
  }, 1000)
})

onUnmounted(() => {
  if (timeInterval) {
    clearInterval(timeInterval)
  }
})
</script>

<style scoped>
.tesla-glass-card {
  background: rgba(255, 255, 255, 0.05) !important;
  backdrop-filter: blur(20px) !important;
  border: 1px solid rgba(255, 255, 255, 0.1) !important;
  border-radius: 20px !important;
  box-shadow: 0 25px 50px -12px rgba(0, 0, 0, 0.25) !important;
}
</style>
