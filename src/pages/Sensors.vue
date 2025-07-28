<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
    <div class="mb-8">
      <n-card class="tesla-glass-card">
        <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
          Sensor Management
        </h1>
        <p class="text-slate-400 font-light mt-2">
          Multi-sensor data monitoring and calibration
        </p>
      </n-card>
    </div>

    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      <n-card v-for="sensor in sensors" :key="sensor.id" class="tesla-glass-card">
        <template #header>
          <div class="flex items-center justify-between">
            <h3 class="text-lg font-light text-white">{{ sensor.name }}</h3>
            <n-tag :type="getSensorStatusType(sensor.status)" size="small">
              {{ sensor.status }}
            </n-tag>
          </div>
        </template>
        <div class="space-y-4">
          <div class="flex items-center justify-center py-8">
            <component :is="sensor.icon" class="h-12 w-12 text-slate-400" />
          </div>
          <div class="space-y-2">
            <div class="flex justify-between text-sm">
              <span class="text-slate-400">Type:</span>
              <span class="text-white">{{ sensor.type }}</span>
            </div>
            <div class="flex justify-between text-sm">
              <span class="text-slate-400">Range:</span>
              <span class="text-white">{{ sensor.range }}</span>
            </div>
            <div class="flex justify-between text-sm">
              <span class="text-slate-400">Accuracy:</span>
              <span class="text-white">{{ sensor.accuracy }}</span>
            </div>
          </div>
          <div v-if="sensor.status === 'active'" class="pt-2">
            <div class="text-xs text-slate-400 mb-1">Signal Strength</div>
            <n-progress :percentage="sensor.signal" size="small" />
          </div>
        </div>
      </n-card>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import { Radar, Camera, Thermometer, Compass, Zap, Wifi } from 'lucide-vue-next'

const sensors = ref([
  {
    id: 1,
    name: 'LIDAR Scanner',
    type: 'Distance',
    status: 'active',
    range: '0-100m',
    accuracy: '±2cm',
    signal: 95,
    icon: Radar
  },
  {
    id: 2,
    name: 'RGB Camera',
    type: 'Visual',
    status: 'active',
    range: '0-50m',
    accuracy: '1080p',
    signal: 87,
    icon: Camera
  },
  {
    id: 3,
    name: 'Temperature',
    type: 'Environmental',
    status: 'inactive',
    range: '-40°C to 85°C',
    accuracy: '±0.5°C',
    signal: 0,
    icon: Thermometer
  },
  {
    id: 4,
    name: 'IMU Sensor',
    type: 'Orientation',
    status: 'active',
    range: '360°',
    accuracy: '±0.1°',
    signal: 92,
    icon: Compass
  },
  {
    id: 5,
    name: 'Ultrasonic',
    type: 'Proximity',
    status: 'error',
    range: '0-4m',
    accuracy: '±3mm',
    signal: 0,
    icon: Zap
  },
  {
    id: 6,
    name: 'WiFi Module',
    type: 'Communication',
    status: 'active',
    range: '100m',
    accuracy: '802.11ac',
    signal: 78,
    icon: Wifi
  }
])

const getSensorStatusType = (status: string) => {
  switch (status) {
    case 'active': return 'success'
    case 'inactive': return 'warning'
    case 'error': return 'error'
    default: return 'default'
  }
}
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
