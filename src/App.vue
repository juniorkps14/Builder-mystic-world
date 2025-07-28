<template>
  <div class="tesla-ui min-h-screen">
    <FlatSidebar />
    <div class="pl-20">
      <FlatHeader />
      <main class="pt-16">
        <router-view />
      </main>
    </div>
  </div>
</template>

<script setup lang="ts">
import FlatSidebar from '@/components/layout/FlatSidebar.vue'
import FlatHeader from '@/components/layout/FlatHeader.vue'
</script>

<style>
/* Tesla UI Global Styles */
@import './styles/tesla-ui-theme.css';

body {
  margin: 0;
  padding: 0;
  font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
  background: linear-gradient(135deg, #1e293b 0%, #334155 50%, #1e293b 100%);
  color: white;
}

/* Tesla Scrollbar Styling */
::-webkit-scrollbar {
  width: 6px;
}

::-webkit-scrollbar-track {
  background: rgba(255, 255, 255, 0.05);
  border-radius: 3px;
}

::-webkit-scrollbar-thumb {
  background: rgba(255, 255, 255, 0.2);
  border-radius: 3px;
  transition: background 0.3s ease;
}

::-webkit-scrollbar-thumb:hover {
  background: rgba(255, 255, 255, 0.3);
}

/* Focus styles for accessibility */
*:focus {
  outline: 2px solid var(--tesla-blue);
  outline-offset: 2px;
}

/* Custom input styling */
input, textarea, select {
  font-family: inherit;
}
</style>
