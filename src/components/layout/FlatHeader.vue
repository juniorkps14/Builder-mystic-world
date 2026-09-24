<template>
  <header class="top-header">
    <div class="header-context">
      <span class="eyebrow">THE SWERVE CORE BY THAI SWERVE DRIVE ROBOTS / {{ currentPageTitle }}</span>
      <h1>{{ currentPageTitle }}</h1>
    </div>

    <div class="header-tools">
      <div class="live-status"><span></span> ROS connected</div>
      <div class="header-time">{{ currentTime }}</div>
      <button class="icon-button" type="button" :aria-label="isDark ? 'Use light mode' : 'Use dark mode'" @click="$emit('toggle-theme')">
        <Sun v-if="isDark" :size="17" />
        <Moon v-else :size="17" />
      </button>
      <div class="profile-chip">
        <span class="profile-avatar">A</span>
        <span class="profile-name">Admin</span>
        <ChevronDown :size="14" />
      </div>
    </div>
  </header>
</template>

<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue'
import { useRoute } from 'vue-router'
import { ChevronDown, Moon, Sun } from 'lucide-vue-next'

defineProps<{ isDark: boolean }>()
defineEmits<{ (event: 'toggle-theme'): void }>()

const route = useRoute()
const currentTime = ref('')
let timeInterval: ReturnType<typeof setInterval> | undefined

const currentPageTitle = computed(() => ({
  '/': 'Overview',
  '/sequences': 'Sequences',
  '/robot-control': 'Robot control',
  '/cameras': 'Cameras',
  '/navigation': 'Navigation',
  '/sensors': 'Sensors',
  '/robotic-arm': 'Robotic arm',
  '/system-monitoring': 'Monitoring',
  '/settings': 'Settings',
  '/about': 'About'
}[route.path] || 'Overview'))

const updateTime = () => {
  currentTime.value = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
}

onMounted(() => {
  updateTime()
  timeInterval = setInterval(updateTime, 1000)
})

onUnmounted(() => {
  if (timeInterval) clearInterval(timeInterval)
})
</script>
