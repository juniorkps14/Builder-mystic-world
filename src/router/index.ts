import { createRouter, createWebHistory } from 'vue-router'

const router = createRouter({
  history: createWebHistory(),
  routes: [
    {
      path: '/',
      name: 'Dashboard',
      component: () => import('@/pages/FlatDashboard.vue')
    },
    {
      path: '/sequences',
      name: 'Sequences',
      component: () => import('@/pages/Sequences.vue')
    },
    {
      path: '/robot-control',
      name: 'RobotControl',
      component: () => import('@/pages/FlatRobotControl.vue')
    },
    {
      path: '/cameras',
      name: 'Cameras',
      component: () => import('@/pages/Cameras.vue')
    },
    {
      path: '/navigation',
      name: 'Navigation',
      component: () => import('@/pages/Navigation.vue')
    },
    {
      path: '/sensors',
      name: 'Sensors',
      component: () => import('@/pages/Sensors.vue')
    },
    {
      path: '/robotic-arm',
      name: 'RoboticArm',
      component: () => import('@/pages/RoboticArm.vue')
    },
    {
      path: '/system-monitoring',
      name: 'SystemMonitoring',
      component: () => import('@/pages/SystemMonitoring.vue')
    },
    {
      path: '/settings',
      name: 'Settings',
      component: () => import('@/pages/Settings.vue')
    },
    {
      path: '/about',
      name: 'About',
      component: () => import('@/pages/About.vue')
    },
    {
      path: '/:pathMatch(.*)*',
      name: 'NotFound',
      component: () => import('@/pages/NotFound.vue')
    }
  ]
})

export default router
