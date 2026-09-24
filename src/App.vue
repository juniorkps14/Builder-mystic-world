<template>
  <div class="app-shell" :class="{ dark: isDark }">
    <FlatSidebar />
    <div class="app-frame">
      <FlatHeader :is-dark="isDark" @toggle-theme="toggleTheme" />
      <main class="app-main">
        <router-view />
      </main>
    </div>
  </div>
</template>

<script setup lang="ts">
import { onMounted, ref, watch } from 'vue'
import FlatSidebar from '@/components/layout/FlatSidebar.vue'
import FlatHeader from '@/components/layout/FlatHeader.vue'

const isDark = ref(false)

const applyTheme = (dark: boolean) => {
  document.documentElement.style.colorScheme = dark ? 'dark' : 'light'
  localStorage.setItem('robot-theme', dark ? 'dark' : 'light')
}

const toggleTheme = () => {
  isDark.value = !isDark.value
}

watch(isDark, applyTheme)

onMounted(() => {
  isDark.value = localStorage.getItem('robot-theme') === 'dark'
})
</script>

<style>
@import './styles/tesla-ui-theme.css';

* {
  box-sizing: border-box;
}

html,
body,
#app {
  min-height: 100%;
  margin: 0;
}

body {
  font-family: 'Inter', 'Helvetica Neue', Arial, sans-serif;
  background: #f7f7f5;
  color: #191919;
}

button,
input,
select,
textarea {
  font: inherit;
}

button,
a {
  -webkit-tap-highlight-color: transparent;
}

button:focus-visible,
a:focus-visible {
  outline: 2px solid var(--accent);
  outline-offset: 3px;
}

::-webkit-scrollbar {
  width: 8px;
}

::-webkit-scrollbar-track {
  background: var(--surface);
}

::-webkit-scrollbar-thumb {
  background: var(--line-strong);
  border-radius: 10px;
}
</style>
