<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
    <div class="mb-8">
      <n-card class="tesla-glass-card">
        <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
          Navigation System
        </h1>
        <p class="text-slate-400 font-light mt-2">
          Autonomous path planning and navigation control
        </p>
      </n-card>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-3 gap-8">
      <div class="lg:col-span-2">
        <n-card class="tesla-glass-card h-96">
          <template #header>
            <h3 class="text-xl font-light text-white">Map View</h3>
          </template>
          <div class="h-full bg-black/20 rounded-lg flex items-center justify-center">
            <div class="text-center">
              <Navigation class="h-16 w-16 text-slate-600 mx-auto mb-4" />
              <p class="text-slate-400">Navigation map will be displayed here</p>
            </div>
          </div>
        </n-card>
      </div>

      <div class="space-y-6">
        <n-card class="tesla-glass-card">
          <template #header>
            <h3 class="text-lg font-light text-white">Current Status</h3>
          </template>
          <div class="space-y-3">
            <div class="flex justify-between">
              <span class="text-slate-400">Position</span>
              <span class="text-white">0.0, 0.0</span>
            </div>
            <div class="flex justify-between">
              <span class="text-slate-400">Heading</span>
              <span class="text-white">0.0°</span>
            </div>
            <div class="flex justify-between">
              <span class="text-slate-400">Speed</span>
              <span class="text-white">0.0 m/s</span>
            </div>
          </div>
        </n-card>

        <n-card class="tesla-glass-card">
          <template #header>
            <h3 class="text-lg font-light text-white">Quick Actions</h3>
          </template>
          <div class="space-y-3">
            <n-button type="primary" block class="tesla-button-primary">
              Set Goal
            </n-button>
            <n-button block class="tesla-button-secondary">
              Return Home
            </n-button>
            <n-button block class="tesla-button-secondary">
              Stop Navigation
            </n-button>
          </div>
        </n-card>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { Navigation } from 'lucide-vue-next'
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
</style>
