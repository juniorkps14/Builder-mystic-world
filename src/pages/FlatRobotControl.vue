<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
    <div class="mb-8">
      <n-card class="tesla-glass-card">
        <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
          Robot Control
        </h1>
        <p class="text-slate-400 font-light mt-2">
          Direct manual control and teleoperation interface
        </p>
      </n-card>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-2 gap-8">
      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Control Interface</h3>
        </template>
        <div class="text-center py-12">
          <div class="w-24 h-24 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-full flex items-center justify-center mx-auto mb-6">
            <Gamepad2 class="h-12 w-12 text-purple-400" />
          </div>
          <p class="text-slate-400">Robot control interface will be implemented here</p>
        </div>
      </n-card>

      <n-card class="tesla-glass-card">
        <template #header>
          <h3 class="text-xl font-light text-white">Status</h3>
        </template>
        <div class="space-y-4">
          <div class="flex justify-between">
            <span class="text-slate-400">Connection</span>
            <span class="text-emerald-400">Connected</span>
          </div>
          <div class="flex justify-between">
            <span class="text-slate-400">Mode</span>
            <span class="text-white">Manual</span>
          </div>
          <div class="flex justify-between">
            <span class="text-slate-400">Battery</span>
            <span class="text-white">87%</span>
          </div>
        </div>
      </n-card>
    </div>
  </div>
</template>

<script setup lang="ts">
import { Gamepad2 } from 'lucide-vue-next'
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
