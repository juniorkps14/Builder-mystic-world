<template>
  <header class="tesla-glass fixed top-0 left-20 right-0 z-40 border-b border-white/20 px-6 py-4 shadow-lg">
    <div class="flex items-center justify-between">
      <!-- Page Title & Breadcrumb -->
      <div class="flex items-center space-x-4">
        <div class="space-y-1">
          <h1 class="text-xl font-light text-white tesla-title">{{ currentPageTitle }}</h1>
          <div class="flex items-center text-sm text-slate-400">
            <Clock class="h-4 w-4 mr-2" />
            {{ currentTime }}
          </div>
        </div>
      </div>

      <!-- System Status & Controls -->
      <div class="flex items-center space-x-4">
        <!-- Connection Status -->
        <div class="tesla-status-online flex items-center gap-2 px-3 py-2 rounded-lg">
          <div class="w-2 h-2 bg-emerald-400 rounded-full animate-pulse" />
          <span class="text-sm font-medium text-white">ROS Connected</span>
        </div>

        <!-- System Metrics -->
        <div class="flex items-center gap-4">
          <!-- CPU Usage -->
          <div class="flex items-center gap-2">
            <Cpu class="h-4 w-4 text-blue-400" />
            <div class="space-y-1">
              <div class="text-xs text-slate-400">CPU</div>
              <div class="text-sm font-medium text-white">{{ systemMetrics.cpu }}%</div>
            </div>
            <div class="tesla-progress w-16">
              <div
                class="tesla-progress-bar"
                :style="{ width: systemMetrics.cpu + '%' }"
              ></div>
            </div>
          </div>

          <!-- Memory Usage -->
          <div class="flex items-center gap-2">
            <HardDrive class="h-4 w-4 text-purple-400" />
            <div class="space-y-1">
              <div class="text-xs text-slate-400">Memory</div>
              <div class="text-sm font-medium text-white">{{ systemMetrics.memory }}%</div>
            </div>
            <div class="tesla-progress w-16">
              <div
                class="tesla-progress-bar bg-gradient-to-r from-purple-500 to-pink-500"
                :style="{ width: systemMetrics.memory + '%' }"
              ></div>
            </div>
          </div>

          <!-- Network Status -->
          <div class="flex items-center gap-2">
            <Wifi class="h-4 w-4 text-emerald-400" />
            <div class="space-y-1">
              <div class="text-xs text-slate-400">Network</div>
              <div class="text-sm font-medium text-white">{{ systemMetrics.network }}ms</div>
            </div>
          </div>
        </div>

        <!-- User Menu -->
        <div class="flex items-center space-x-2">
          <div class="tesla-glass flex items-center gap-3 px-3 py-2 rounded-lg cursor-pointer hover:bg-white/20 transition-all duration-300 tesla-interactive">
            <div class="w-8 h-8 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-full flex items-center justify-center">
              <User class="h-4 w-4 text-white" />
            </div>
            <div class="text-sm">
              <div class="font-medium text-white">Admin</div>
              <div class="text-xs text-slate-400">System Administrator</div>
            </div>
            <ChevronDown class="h-4 w-4 text-slate-400" />
          </div>
        </div>
      </div>
    </div>
  </header>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useRoute } from 'vue-router'
import {
  Clock,
  Cpu,
  HardDrive,
  Wifi,
  User,
  ChevronDown
} from 'lucide-vue-next'

const route = useRoute()
const currentTime = ref('')
let timeInterval: number

// System metrics (simulated)
// System metrics (simulated)
const systemMetrics = ref({
  cpu: 45,
  memory: 62,
  network: 23
})

const currentPageTitle = computed(() => {
  const titles: Record<string, string> = {
    '/': 'Dashboard',
    '/sequences': 'Robot Sequences',
    '/robot-control': 'Robot Control',
    '/cameras': 'Camera Systems',
    '/navigation': 'Navigation',
    '/sensors': 'Sensor Management',
    '/robotic-arm': 'Robotic Arm Control',
    '/system-monitoring': 'System Monitoring',
    '/settings': 'Settings',
    '/about': 'About'
  }
  return titles[route.path] || 'Unknown Page'
})

const updateTime = () => {
  currentTime.value = new Date().toLocaleTimeString()
}

const updateMetrics = () => {
  // Simulate real-time metrics
  systemMetrics.value.cpu = Math.floor(Math.random() * 30) + 40
  systemMetrics.value.memory = Math.floor(Math.random() * 20) + 55
  systemMetrics.value.network = Math.floor(Math.random() * 20) + 15
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
