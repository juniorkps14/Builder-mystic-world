<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
    <div class="mb-8">
      <n-card class="tesla-glass-card">
        <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
          System Settings
        </h1>
        <p class="text-slate-400 font-light mt-2">
          Configure system preferences and operational parameters
        </p>
      </n-card>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-2 gap-8">
      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">General Settings</h3>
        </template>
        <div class="space-y-6">
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-2">Robot Name</label>
            <n-input v-model:value="settings.robotName" class="tesla-input" />
          </div>
          
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-2">Operation Mode</label>
            <n-select
              v-model:value="settings.operationMode"
              :options="operationModes"
              class="tesla-select"
            />
          </div>
          
          <div class="flex items-center justify-between">
            <span class="text-slate-300">Auto-start sequences</span>
            <n-switch v-model:value="settings.autoStart" />
          </div>
          
          <div class="flex items-center justify-between">
            <span class="text-slate-300">Safety mode</span>
            <n-switch v-model:value="settings.safetyMode" />
          </div>
        </div>
      </n-card>

      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Network Settings</h3>
        </template>
        <div class="space-y-6">
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-2">ROS Master URI</label>
            <n-input v-model:value="settings.rosMasterUri" class="tesla-input" />
          </div>
          
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-2">Robot IP Address</label>
            <n-input v-model:value="settings.robotIp" class="tesla-input" />
          </div>
          
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-2">Connection Timeout (ms)</label>
            <n-input-number v-model:value="settings.connectionTimeout" :min="1000" :max="30000" class="tesla-input" />
          </div>
        </div>
      </n-card>

      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Performance</h3>
        </template>
        <div class="space-y-6">
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-2">Max CPU Usage (%)</label>
            <n-slider v-model:value="settings.maxCpuUsage" :min="50" :max="100" />
            <div class="text-right text-sm text-slate-400 mt-1">{{ settings.maxCpuUsage }}%</div>
          </div>
          
          <div>
            <label class="block text-sm font-medium text-slate-300 mb-2">Log Retention (days)</label>
            <n-input-number v-model:value="settings.logRetention" :min="1" :max="30" class="tesla-input" />
          </div>
          
          <div class="flex items-center justify-between">
            <span class="text-slate-300">Debug mode</span>
            <n-switch v-model:value="settings.debugMode" />
          </div>
        </div>
      </n-card>

      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Actions</h3>
        </template>
        <div class="space-y-4">
          <n-button type="primary" block class="tesla-button-primary" @click="saveSettings">
            Save Settings
          </n-button>
          <n-button block class="tesla-button-secondary" @click="resetSettings">
            Reset to Defaults
          </n-button>
          <n-button block class="tesla-button-secondary" @click="exportSettings">
            Export Configuration
          </n-button>
          <n-button block class="tesla-button-secondary" @click="importSettings">
            Import Configuration
          </n-button>
        </div>
      </n-card>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import { useMessage } from 'naive-ui'

const message = useMessage()

const settings = ref({
  robotName: 'RoboticsSystem-01',
  operationMode: 'autonomous',
  autoStart: true,
  safetyMode: true,
  rosMasterUri: 'http://localhost:11311',
  robotIp: '192.168.1.100',
  connectionTimeout: 5000,
  maxCpuUsage: 80,
  logRetention: 7,
  debugMode: false
})

const operationModes = [
  { label: 'Autonomous', value: 'autonomous' },
  { label: 'Manual', value: 'manual' },
  { label: 'Hybrid', value: 'hybrid' },
  { label: 'Maintenance', value: 'maintenance' }
]

const saveSettings = () => {
  // Simulate saving settings
  message.success('Settings saved successfully')
}

const resetSettings = () => {
  // Reset to default values
  settings.value = {
    robotName: 'RoboticsSystem-01',
    operationMode: 'autonomous',
    autoStart: true,
    safetyMode: true,
    rosMasterUri: 'http://localhost:11311',
    robotIp: '192.168.1.100',
    connectionTimeout: 5000,
    maxCpuUsage: 80,
    logRetention: 7,
    debugMode: false
  }
  message.info('Settings reset to defaults')
}

const exportSettings = () => {
  // Simulate exporting settings
  message.info('Configuration exported')
}

const importSettings = () => {
  // Simulate importing settings
  message.info('Import configuration file')
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

.tesla-button-primary {
  background: linear-gradient(45deg, #3b82f6, #06b6d4) !important;
  border: none !important;
  color: white !important;
}

.tesla-button-secondary {
  background: rgba(255, 255, 255, 0.1) !important;
  border: 1px solid rgba(255, 255, 255, 0.2) !important;
  color: white !important;
}

.tesla-input {
  background: rgba(255, 255, 255, 0.05) !important;
  border: 1px solid rgba(255, 255, 255, 0.2) !important;
  color: white !important;
}

.tesla-select {
  background: rgba(255, 255, 255, 0.05) !important;
}
</style>
