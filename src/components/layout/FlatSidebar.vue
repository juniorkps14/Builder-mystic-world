<template>
  <aside class="tesla-glass fixed top-0 left-0 z-50 h-full w-20 border-r border-white/20 shadow-2xl">
    <!-- Logo Section -->
    <div class="flex items-center justify-center h-16 border-b border-white/10">
      <div class="w-10 h-10 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-xl flex items-center justify-center shadow-lg tesla-interactive">
        <Zap class="h-6 w-6 text-white" />
      </div>
    </div>

    <!-- Navigation -->
    <nav class="p-3 space-y-2 mt-4">
      <router-link
        v-for="item in navigationItems"
        :key="item.path"
        :to="item.path"
        v-slot="{ isActive }"
        class="block group"
      >
        <div class="relative">
          <div
            :class="[
              'group relative flex items-center justify-center h-12 rounded-xl transition-all duration-300 cursor-pointer tesla-interactive',
              isActive
                ? 'tesla-glass-intense border border-blue-400/50 shadow-lg bg-gradient-to-r from-blue-500/30 to-cyan-500/30'
                : 'tesla-glass border border-white/10 hover:border-white/30 hover:bg-white/15'
            ]"
            :title="item.name"
          >
            <component :is="item.icon" class="h-5 w-5 text-white group-hover:scale-110 transition-transform duration-300" />

            <!-- Active indicator -->
            <div
              v-if="isActive"
              class="absolute -right-1 top-1/2 transform -translate-y-1/2 w-1 h-6 bg-gradient-to-b from-blue-400 to-cyan-400 rounded-l-full shadow-lg"
            />

            <!-- Hover tooltip -->
            <div class="absolute left-full ml-3 top-1/2 transform -translate-y-1/2 tesla-glass px-3 py-2 rounded-lg text-sm font-medium text-white opacity-0 group-hover:opacity-100 transition-opacity duration-300 pointer-events-none z-50 whitespace-nowrap border border-white/20">
              {{ item.name }}
              <div class="absolute right-full top-1/2 transform -translate-y-1/2 w-0 h-0 border-t-4 border-b-4 border-r-4 border-transparent border-r-white/20"></div>
            </div>
          </div>
        </div>
      </router-link>
    </nav>

    <!-- Bottom Actions -->
    <div class="absolute bottom-4 left-3 right-3 space-y-2">
      <router-link to="/settings" class="block group">
        <div class="tesla-glass flex items-center justify-center h-12 rounded-xl border border-white/10 hover:border-white/30 hover:bg-white/15 transition-all duration-300 cursor-pointer tesla-interactive relative" title="Settings">
          <Settings class="h-5 w-5 text-white group-hover:scale-110 transition-transform duration-300" />
          <div class="absolute left-full ml-3 top-1/2 transform -translate-y-1/2 tesla-glass px-3 py-2 rounded-lg text-sm font-medium text-white opacity-0 group-hover:opacity-100 transition-opacity duration-300 pointer-events-none z-50 whitespace-nowrap border border-white/20">
            Settings
            <div class="absolute right-full top-1/2 transform -translate-y-1/2 w-0 h-0 border-t-4 border-b-4 border-r-4 border-transparent border-r-white/20"></div>
          </div>
        </div>
      </router-link>

      <div class="tesla-glass flex items-center justify-center h-12 rounded-xl border border-white/10 hover:border-white/30 hover:bg-white/15 transition-all duration-300 cursor-pointer tesla-interactive relative group" title="Help">
        <HelpCircle class="h-5 w-5 text-white group-hover:scale-110 transition-transform duration-300" />
        <div class="absolute left-full ml-3 top-1/2 transform -translate-y-1/2 tesla-glass px-3 py-2 rounded-lg text-sm font-medium text-white opacity-0 group-hover:opacity-100 transition-opacity duration-300 pointer-events-none z-50 whitespace-nowrap border border-white/20">
          Help
          <div class="absolute right-full top-1/2 transform -translate-y-1/2 w-0 h-0 border-t-4 border-b-4 border-r-4 border-transparent border-r-white/20"></div>
        </div>
      </div>
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
