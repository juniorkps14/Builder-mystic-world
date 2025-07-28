<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
    <div class="max-w-4xl mx-auto">
      <!-- Header -->
      <div class="mb-8">
        <n-card class="tesla-glass-card">
          <div class="text-center">
            <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent mb-4">
              About Robotics Control System
            </h1>
            <p class="text-slate-400 text-lg font-light">
              Advanced autonomous robotics platform powered by modern technology
            </p>
          </div>
        </n-card>
      </div>

      <!-- System Information -->
      <div class="grid grid-cols-1 lg:grid-cols-2 gap-8 mb-8">
        <n-card class="tesla-glass-card">
          <template #header>
            <h3 class="text-xl font-light text-white">System Information</h3>
          </template>
          <div class="space-y-4">
            <div class="flex justify-between">
              <span class="text-slate-400">Version</span>
              <span class="text-white font-medium">2.1.0</span>
            </div>
            <div class="flex justify-between">
              <span class="text-slate-400">Framework</span>
              <span class="text-white font-medium">Vue.js 3.5</span>
            </div>
            <div class="flex justify-between">
              <span class="text-slate-400">ROS Version</span>
              <span class="text-white font-medium">ROS 2 Humble</span>
            </div>
            <div class="flex justify-between">
              <span class="text-slate-400">Last Updated</span>
              <span class="text-white font-medium">{{ new Date().toLocaleDateString() }}</span>
            </div>
          </div>
        </n-card>

        <n-card class="tesla-glass-card">
          <template #header>
            <h3 class="text-xl font-light text-white">Features</h3>
          </template>
          <div class="space-y-3">
            <div class="flex items-center gap-3">
              <div class="w-2 h-2 bg-emerald-400 rounded-full" />
              <span class="text-white">Autonomous Navigation</span>
            </div>
            <div class="flex items-center gap-3">
              <div class="w-2 h-2 bg-blue-400 rounded-full" />
              <span class="text-white">Task Sequencing</span>
            </div>
            <div class="flex items-center gap-3">
              <div class="w-2 h-2 bg-purple-400 rounded-full" />
              <span class="text-white">Real-time Monitoring</span>
            </div>
            <div class="flex items-center gap-3">
              <div class="w-2 h-2 bg-cyan-400 rounded-full" />
              <span class="text-white">Multi-sensor Integration</span>
            </div>
          </div>
        </n-card>
      </div>

      <!-- Technology Stack -->
      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Technology Stack</h3>
        </template>
        <div class="grid grid-cols-2 md:grid-cols-4 gap-6">
          <div class="text-center">
            <div class="w-16 h-16 bg-gradient-to-br from-green-500/20 to-emerald-500/20 rounded-xl flex items-center justify-center mx-auto mb-3">
              <span class="text-2xl">🟢</span>
            </div>
            <h4 class="font-medium text-white mb-1">Vue.js</h4>
            <p class="text-sm text-slate-400">Frontend Framework</p>
          </div>
          <div class="text-center">
            <div class="w-16 h-16 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center mx-auto mb-3">
              <span class="text-2xl">🤖</span>
            </div>
            <h4 class="font-medium text-white mb-1">ROS 2</h4>
            <p class="text-sm text-slate-400">Robotics Middleware</p>
          </div>
          <div class="text-center">
            <div class="w-16 h-16 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center mx-auto mb-3">
              <span class="text-2xl">⚡</span>
            </div>
            <h4 class="font-medium text-white mb-1">TypeScript</h4>
            <p class="text-sm text-slate-400">Type Safety</p>
          </div>
          <div class="text-center">
            <div class="w-16 h-16 bg-gradient-to-br from-orange-500/20 to-red-500/20 rounded-xl flex items-center justify-center mx-auto mb-3">
              <span class="text-2xl">🎨</span>
            </div>
            <h4 class="font-medium text-white mb-1">Tailwind</h4>
            <p class="text-sm text-slate-400">UI Styling</p>
          </div>
        </div>
      </n-card>
    </div>
  </div>
</template>

<script setup lang="ts">
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
