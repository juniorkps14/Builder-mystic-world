<template>
  <aside class="fixed top-0 left-0 z-50 h-full bg-white/5 backdrop-blur-xl border-r border-white/20 shadow-2xl w-20 transition-all duration-300">
    <!-- Logo Section -->
    <div class="flex items-center justify-center h-16 border-b border-white/10">
      <div class="w-10 h-10 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-xl flex items-center justify-center shadow-lg">
        <Zap class="h-6 w-6 text-white" />
      </div>
    </div>

    <!-- Navigation -->
    <nav class="p-3 space-y-2">
      <router-link
        v-for="item in navigationItems"
        :key="item.path"
        :to="item.path"
        v-slot="{ isActive }"
        class="block"
      >
        <n-tooltip placement="right" :delay="500">
          <template #trigger>
            <div
              :class="[
                'group relative flex items-center justify-center h-12 rounded-xl transition-all duration-300 hover:bg-white/10',
                isActive
                  ? 'bg-gradient-to-r from-blue-500/20 to-cyan-500/20 border border-blue-400/50 shadow-lg'
                  : 'bg-white/5 border border-white/10 hover:border-white/20'
              ]"
            >
              <component :is="item.icon" class="h-5 w-5 text-white group-hover:scale-110 transition-transform" />
              
              <!-- Active indicator -->
              <div
                v-if="isActive"
                class="absolute -right-1 top-1/2 transform -translate-y-1/2 w-1 h-6 bg-gradient-to-b from-blue-400 to-cyan-400 rounded-l-full"
              />
            </div>
          </template>
          {{ item.name }}
        </n-tooltip>
      </router-link>
    </nav>

    <!-- Bottom Actions -->
    <div class="absolute bottom-4 left-3 right-3 space-y-2">
      <n-tooltip placement="right" :delay="500">
        <template #trigger>
          <div class="flex items-center justify-center h-12 rounded-xl bg-white/5 border border-white/10 hover:bg-white/10 transition-all duration-300 cursor-pointer">
            <Settings class="h-5 w-5 text-white" />
          </div>
        </template>
        Settings
      </n-tooltip>
      
      <n-tooltip placement="right" :delay="500">
        <template #trigger>
          <div class="flex items-center justify-center h-12 rounded-xl bg-white/5 border border-white/10 hover:bg-white/10 transition-all duration-300 cursor-pointer">
            <HelpCircle class="h-5 w-5 text-white" />
          </div>
        </template>
        Help
      </n-tooltip>
    </div>
  </aside>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import {
  Zap,
  LayoutDashboard,
  Settings,
  HelpCircle,
  Play,
  Gamepad2,
  Camera,
  Navigation,
  Radar,
  Bot,
  Activity,
  Info
} from 'lucide-vue-next'

const navigationItems = ref([
  {
    name: 'Dashboard',
    path: '/',
    icon: LayoutDashboard
  },
  {
    name: 'Robot Sequences',
    path: '/sequences',
    icon: Play
  },
  {
    name: 'Robot Control',
    path: '/robot-control',
    icon: Gamepad2
  },
  {
    name: 'Cameras',
    path: '/cameras',
    icon: Camera
  },
  {
    name: 'Navigation',
    path: '/navigation',
    icon: Navigation
  },
  {
    name: 'Sensors',
    path: '/sensors',
    icon: Radar
  },
  {
    name: 'Robotic Arm',
    path: '/robotic-arm',
    icon: Bot
  },
  {
    name: 'System Monitoring',
    path: '/system-monitoring',
    icon: Activity
  },
  {
    name: 'About',
    path: '/about',
    icon: Info
  }
])
</script>

<style scoped>
.router-link-active {
  /* Additional active styles if needed */
}
</style>
