<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
    <div class="mb-8">
      <n-card class="tesla-glass-card">
        <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
          System Monitoring
        </h1>
        <p class="text-slate-400 font-light mt-2">
          Real-time system performance and health monitoring
        </p>
      </n-card>
    </div>

    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
      <n-card v-for="metric in systemMetrics" :key="metric.name" class="tesla-glass-card">
        <div class="text-center">
          <div :class="[
            'w-16 h-16 rounded-xl flex items-center justify-center mx-auto mb-4',
            metric.color
          ]">
            <component :is="metric.icon" class="h-8 w-8 text-white" />
          </div>
          <h3 class="font-medium text-white mb-2">{{ metric.name }}</h3>
          <div class="text-2xl font-light text-white mb-2">{{ metric.value }}</div>
          <n-progress
            :percentage="metric.percentage"
            size="small"
            :color="metric.progressColor"
          />
        </div>
      </n-card>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-2 gap-8">
      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Performance Graph</h3>
        </template>
        <div class="h-64 bg-black/20 rounded-lg flex items-center justify-center">
          <div class="text-center">
            <BarChart3 class="h-12 w-12 text-slate-600 mx-auto mb-4" />
            <p class="text-slate-400">Performance charts will be displayed here</p>
          </div>
        </div>
      </n-card>

      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">System Logs</h3>
        </template>
        <n-scrollbar style="max-height: 300px">
          <div class="space-y-2">
            <div
              v-for="(log, index) in logs"
              :key="index"
              class="flex gap-3 p-2 rounded-lg hover:bg-white/5 transition-colors"
            >
              <span class="text-xs font-mono text-slate-400 w-16 flex-shrink-0">{{ log.time }}</span>
              <span :class="[
                'text-xs font-medium w-12 flex-shrink-0',
                getLogLevelColor(log.level)
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
import { Cpu, HardDrive, Thermometer, Zap, BarChart3 } from 'lucide-vue-next'

const systemMetrics = ref([
  {
    name: 'CPU Usage',
    value: '45%',
    percentage: 45,
    icon: Cpu,
    color: 'bg-gradient-to-br from-blue-500/20 to-cyan-500/20',
    progressColor: '#3b82f6'
  },
  {
    name: 'Memory',
    value: '62%',
    percentage: 62,
    icon: HardDrive,
    color: 'bg-gradient-to-br from-purple-500/20 to-pink-500/20',
    progressColor: '#8b5cf6'
  },
  {
    name: 'Temperature',
    value: '58°C',
    percentage: 58,
    icon: Thermometer,
    color: 'bg-gradient-to-br from-orange-500/20 to-red-500/20',
    progressColor: '#f97316'
  },
  {
    name: 'Power',
    value: '87%',
    percentage: 87,
    icon: Zap,
    color: 'bg-gradient-to-br from-emerald-500/20 to-green-500/20',
    progressColor: '#10b981'
  }
])

const logs = ref([
  { time: '14:32:15', level: 'INFO', message: 'System health check completed successfully' },
  { time: '14:31:42', level: 'INFO', message: 'Memory optimization completed' },
  { time: '14:31:15', level: 'WARN', message: 'CPU usage spike detected, monitoring...' },
  { time: '14:30:58', level: 'INFO', message: 'Background processes optimized' },
  { time: '14:30:12', level: 'ERROR', message: 'Temporary sensor connection lost' },
  { time: '14:29:45', level: 'INFO', message: 'System startup completed' }
])

let updateInterval: number

const getLogLevelColor = (level: string) => {
  switch (level) {
    case 'INFO': return 'text-blue-400'
    case 'WARN': return 'text-yellow-400'
    case 'ERROR': return 'text-red-400'
    case 'SUCCESS': return 'text-emerald-400'
    default: return 'text-slate-400'
  }
}

const updateMetrics = () => {
  systemMetrics.value.forEach(metric => {
    // Simulate real-time updates
    const variation = (Math.random() - 0.5) * 10
    metric.percentage = Math.max(0, Math.min(100, metric.percentage + variation))
    
    if (metric.name === 'CPU Usage') {
      metric.value = `${Math.round(metric.percentage)}%`
    } else if (metric.name === 'Memory') {
      metric.value = `${Math.round(metric.percentage)}%`
    } else if (metric.name === 'Temperature') {
      metric.value = `${Math.round(metric.percentage + 20)}°C`
    } else if (metric.name === 'Power') {
      metric.value = `${Math.round(metric.percentage)}%`
    }
  })
}

onMounted(() => {
  updateInterval = setInterval(updateMetrics, 2000)
})

onUnmounted(() => {
  if (updateInterval) {
    clearInterval(updateInterval)
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
