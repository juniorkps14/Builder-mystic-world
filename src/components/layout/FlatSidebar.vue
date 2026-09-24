<template>
  <aside class="side-rail">
    <router-link to="/" class="brand-mark" aria-label="Dino Core dashboard">
      <span class="brand-dot"></span>
      <span class="brand-name">DINO<br /><b>CORE</b></span>
    </router-link>

    <div class="rail-label">Workspace</div>
    <nav class="rail-nav" aria-label="Main navigation">
      <router-link
        v-for="item in navigationItems"
        :key="item.path"
        :to="item.path"
        class="rail-link"
        :class="{ active: route.path === item.path }"
      >
        <component :is="item.icon" :size="17" :stroke-width="1.7" />
        <span>{{ item.name }}</span>
      </router-link>
    </nav>

    <div class="rail-footer">
      <div class="rail-label">System</div>
      <router-link to="/settings" class="rail-link" :class="{ active: route.path === '/settings' }">
        <Settings :size="17" :stroke-width="1.7" />
        <span>Settings</span>
      </router-link>
      <div class="rail-version">DINO OS <span>v2.4.1</span></div>
    </div>
  </aside>
</template>

<script setup lang="ts">
import { useRoute } from 'vue-router'
import {
  LayoutDashboard, ListTodo, Gamepad2, Camera, Navigation, Radar,
  Bot, Activity, Info, Settings
} from 'lucide-vue-next'

const route = useRoute()
const navigationItems = [
  { name: 'Overview', path: '/', icon: LayoutDashboard },
  { name: 'Sequences', path: '/sequences', icon: ListTodo },
  { name: 'Robot control', path: '/robot-control', icon: Gamepad2 },
  { name: 'Cameras', path: '/cameras', icon: Camera },
  { name: 'Navigation', path: '/navigation', icon: Navigation },
  { name: 'Sensors', path: '/sensors', icon: Radar },
  { name: 'Robotic arm', path: '/robotic-arm', icon: Bot },
  { name: 'Monitoring', path: '/system-monitoring', icon: Activity },
  { name: 'About', path: '/about', icon: Info }
]
</script>
