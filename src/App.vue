<template>
  <n-config-provider :theme="darkTheme">
    <n-dialog-provider>
      <n-message-provider>
        <n-notification-provider>
          <n-loading-bar-provider>
            <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900">
              <FlatSidebar />
              <div class="pl-20">
                <FlatHeader />
                <main class="pt-16">
                  <router-view />
                </main>
              </div>
            </div>
          </n-loading-bar-provider>
        </n-notification-provider>
      </n-message-provider>
    </n-dialog-provider>
  </n-config-provider>
</template>

<script setup lang="ts">
import { darkTheme } from 'naive-ui'
import FlatSidebar from '@/components/layout/FlatSidebar.vue'
import FlatHeader from '@/components/layout/FlatHeader.vue'
</script>

<style>
/* Tesla UI Global Styles */
@import './styles/tesla-ui-theme.css';

body {
  margin: 0;
  padding: 0;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
  background: linear-gradient(135deg, #1e293b 0%, #334155 50%, #1e293b 100%);
  color: white;
}

/* Custom scrollbar */
::-webkit-scrollbar {
  width: 6px;
}

::-webkit-scrollbar-track {
  background: rgba(255, 255, 255, 0.05);
}

::-webkit-scrollbar-thumb {
  background: rgba(255, 255, 255, 0.2);
  border-radius: 3px;
}

::-webkit-scrollbar-thumb:hover {
  background: rgba(255, 255, 255, 0.3);
}

/* Naive UI Dark Theme Overrides */
.n-card {
  background: rgba(255, 255, 255, 0.05) !important;
  backdrop-filter: blur(20px) !important;
  border: 1px solid rgba(255, 255, 255, 0.1) !important;
  border-radius: 20px !important;
}

.n-button--primary-type {
  background: linear-gradient(45deg, #3b82f6, #06b6d4) !important;
  border: none !important;
}

.n-input {
  background: rgba(255, 255, 255, 0.05) !important;
  border: 1px solid rgba(255, 255, 255, 0.2) !important;
  color: white !important;
}

.n-select {
  background: rgba(255, 255, 255, 0.05) !important;
}

.n-progress .n-progress-graph .n-progress-graph-line-fill {
  background: linear-gradient(45deg, #3b82f6, #06b6d4) !important;
}
</style>
