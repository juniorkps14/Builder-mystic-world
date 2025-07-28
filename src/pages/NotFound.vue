<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 flex items-center justify-center p-6">
    <div class="text-center">
      <n-card class="tesla-glass-card max-w-md mx-auto">
        <div class="py-8">
          <div class="w-24 h-24 bg-gradient-to-br from-red-500/20 to-pink-500/20 rounded-full flex items-center justify-center mx-auto mb-6">
            <span class="text-4xl">🤖</span>
          </div>
          <h1 class="text-6xl font-light text-white mb-4">404</h1>
          <h2 class="text-2xl font-light text-white mb-2">Page Not Found</h2>
          <p class="text-slate-400 mb-8">
            The requested page could not be found in our robotics system.
          </p>
          <div class="space-y-3">
            <n-button type="primary" size="large" @click="$router.push('/')" class="tesla-button-primary">
              Return to Dashboard
            </n-button>
            <br>
            <n-button @click="$router.back()" class="tesla-button-secondary">
              Go Back
            </n-button>
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

.tesla-button-primary {
  background: linear-gradient(45deg, #3b82f6, #06b6d4) !important;
  border: none !important;
  color: white !important;
}

.tesla-button-secondary {
  background: rgba(255, 255, 255, 0.1) !important;
  border: 1px solid rgba(255, 255, 255, 0.2) !important;
  color: white !important;
}
</style>
