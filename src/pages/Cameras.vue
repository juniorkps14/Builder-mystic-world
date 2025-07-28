<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
    <div class="mb-8">
      <n-card class="tesla-glass-card">
        <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
          Camera Systems
        </h1>
        <p class="text-slate-400 font-light mt-2">
          Multi-camera monitoring and visual sensor management
        </p>
      </n-card>
    </div>

    <div class="grid grid-cols-1 md:grid-cols-2 gap-6">
      <n-card v-for="camera in cameras" :key="camera.id" class="tesla-glass-card">
        <template #header>
          <div class="flex items-center justify-between">
            <h3 class="text-lg font-light text-white">{{ camera.name }}</h3>
            <n-tag :type="camera.status === 'active' ? 'success' : 'error'" size="small">
              {{ camera.status }}
            </n-tag>
          </div>
        </template>
        <div class="aspect-video bg-black/50 rounded-lg flex items-center justify-center mb-4">
          <Camera class="h-12 w-12 text-slate-600" />
        </div>
        <div class="grid grid-cols-2 gap-4 text-sm">
          <div>
            <span class="text-slate-400">Resolution:</span>
            <span class="text-white ml-2">{{ camera.resolution }}</span>
          </div>
          <div>
            <span class="text-slate-400">FPS:</span>
            <span class="text-white ml-2">{{ camera.fps }}</span>
          </div>
        </div>
      </n-card>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import { Camera } from 'lucide-vue-next'

const cameras = ref([
  { id: 1, name: 'Front Camera', status: 'active', resolution: '1920x1080', fps: '30' },
  { id: 2, name: 'Rear Camera', status: 'active', resolution: '1920x1080', fps: '30' },
  { id: 3, name: 'Left Camera', status: 'inactive', resolution: '1280x720', fps: '24' },
  { id: 4, name: 'Right Camera', status: 'active', resolution: '1280x720', fps: '24' }
])
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
