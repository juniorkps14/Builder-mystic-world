<template>
  <div class="dashboard-page">
    <section class="dashboard-intro">
      <div>
        <p class="eyebrow">Autonomous operations / 01</p>
        <h2>Robotics<br /><em>Control Center</em></h2>
        <p class="intro-copy">A clear view of your robot fleet, active missions and system health.</p>
      </div>
      <div class="intro-aside">
        <div class="date-label">{{ currentDate }}</div>
        <div class="intro-clock">{{ currentTime }}</div>
        <div class="status-line"><span></span> All systems operational</div>
      </div>
    </section>

    <section class="stat-grid" aria-label="System summary">
      <article v-for="stat in stats" :key="stat.label" class="stat-card">
        <div class="stat-top"><span class="stat-label">{{ stat.label }}</span><component :is="stat.icon" :size="17" /></div>
        <div class="stat-value">{{ stat.value }}</div>
        <div class="stat-foot"><span :class="stat.tone">{{ stat.change }}</span><span>{{ stat.note }}</span></div>
      </article>
    </section>

    <section class="content-grid">
      <article class="panel mission-panel">
        <div class="panel-heading">
          <div><p class="eyebrow">Live operations</p><h3>Active missions</h3></div>
          <router-link to="/sequences" class="text-link">View all <ArrowUpRight :size="15" /></router-link>
        </div>
        <div class="mission-list">
          <div v-for="mission in missions" :key="mission.name" class="mission-row">
            <div class="mission-icon" :class="mission.tone"><component :is="mission.icon" :size="17" /></div>
            <div class="mission-info"><strong>{{ mission.name }}</strong><span>{{ mission.detail }}</span></div>
            <div class="mission-progress"><div class="progress-track"><span :style="{ width: `${mission.progress}%` }"></span></div><small>{{ mission.progress }}%</small></div>
            <span class="mission-state" :class="mission.tone">{{ mission.state }}</span>
          </div>
        </div>
      </article>

      <article class="panel system-panel">
        <div class="panel-heading"><div><p class="eyebrow">Node telemetry</p><h3>System health</h3></div><span class="health-score">{{ health }}%</span></div>
        <div class="health-bar"><span :style="{ width: `${health}%` }"></span></div>
        <div class="health-list">
          <div v-for="metric in metrics" :key="metric.name" class="health-row"><span>{{ metric.name }}</span><strong>{{ metric.value }}<small>{{ metric.unit }}</small></strong></div>
        </div>
        <router-link to="/system-monitoring" class="panel-button">Open monitoring <ArrowUpRight :size="15" /></router-link>
      </article>
    </section>

    <section class="bottom-grid">
      <article class="quick-panel">
        <div class="panel-heading"><div><p class="eyebrow">Shortcuts</p><h3>Quick actions</h3></div></div>
        <div class="quick-grid">
          <router-link v-for="action in actions" :key="action.label" :to="action.path" class="quick-action"><span><component :is="action.icon" :size="18" /></span><strong>{{ action.label }}</strong><ArrowUpRight :size="15" /></router-link>
        </div>
      </article>
      <article class="activity-panel">
        <div class="panel-heading"><div><p class="eyebrow">Latest events</p><h3>Activity log</h3></div><span class="live-label"><i></i>Live</span></div>
        <div class="activity-list"><div v-for="event in events" :key="event.time" class="activity-row"><span class="activity-time">{{ event.time }}</span><span class="activity-dot" :class="event.tone"></span><span>{{ event.text }}</span></div></div>
      </article>
    </section>
  </div>
</template>

<script setup lang="ts">
import { onMounted, onUnmounted, ref } from 'vue'
import { Activity, ArrowUpRight, Bot, Camera, CircleCheck, Gamepad2, Gauge, Play, Radio, Route } from 'lucide-vue-next'

const currentTime = ref('')
const currentDate = ref('')
const health = ref(94)
let timeInterval: ReturnType<typeof setInterval> | undefined

const stats = [
  { label: 'Robot status', value: 'Online', change: 'Connected', note: 'ROS bridge active', tone: 'positive', icon: Bot },
  { label: 'Active tasks', value: '03', change: '+2 today', note: '12 completed', tone: 'neutral', icon: Play },
  { label: 'System health', value: '94%', change: 'Excellent', note: 'All nodes stable', tone: 'positive', icon: Gauge },
  { label: 'Fleet uptime', value: '5d 14h', change: '99.8%', note: 'Since last restart', tone: 'neutral', icon: Activity }
]

const missions = [
  { name: 'Security patrol', detail: 'North perimeter / Navigation', progress: 72, state: 'Running', tone: 'lime', icon: Route },
  { name: 'Object detection', detail: 'Camera cluster A / Vision', progress: 48, state: 'Processing', tone: 'blue', icon: Camera },
  { name: 'Sensor calibration', detail: 'Lidar + IMU / Diagnostics', progress: 100, state: 'Complete', tone: 'muted', icon: CircleCheck }
]

const metrics = [
  { name: 'CPU utilization', value: 45, unit: '%' },
  { name: 'Memory allocated', value: 62, unit: '%' },
  { name: 'Network latency', value: 23, unit: ' ms' }
]

const actions = [
  { label: 'Start sequence', path: '/sequences', icon: Play },
  { label: 'Manual control', path: '/robot-control', icon: Gamepad2 },
  { label: 'Camera feeds', path: '/cameras', icon: Camera },
  { label: 'View sensors', path: '/sensors', icon: Radio }
]

const events = [
  { time: '14:32', text: 'All sensors operational and calibrated', tone: 'positive' },
  { time: '14:31', text: 'ROS node connections established', tone: 'blue' },
  { time: '14:30', text: 'Security patrol sequence started', tone: 'lime' },
  { time: '14:28', text: 'Network connection stable at 23ms', tone: 'muted' }
]

const updateClock = () => {
  const now = new Date()
  currentTime.value = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' })
  currentDate.value = now.toLocaleDateString('en-US', { weekday: 'long', month: 'short', day: 'numeric' })
}

onMounted(() => {
  updateClock()
  timeInterval = setInterval(updateClock, 1000)
})

onUnmounted(() => { if (timeInterval) clearInterval(timeInterval) })
</script>

<style scoped>
.dashboard-page { max-width: 1400px; margin: 0 auto; }
.dashboard-intro { display: flex; justify-content: space-between; padding-bottom: 42px; border-bottom: 1px solid var(--line); }
.dashboard-intro h2 { margin: 13px 0 16px; font-size: clamp(38px, 5vw, 64px); line-height: .98; letter-spacing: -.07em; font-weight: 600; }
.dashboard-intro h2 em { color: var(--ink-muted); font-style: normal; font-weight: 400; }
.intro-copy { max-width: 330px; margin: 0; color: var(--ink-soft); font-size: 14px; line-height: 1.6; }
.intro-aside { min-width: 190px; padding-top: 19px; text-align: right; }
.date-label { color: var(--ink-muted); font: 11px 'DM Mono', monospace; text-transform: uppercase; }
.intro-clock { margin: 10px 0 25px; font: 28px 'DM Mono', monospace; letter-spacing: -.08em; }
.status-line, .live-label { display: flex; align-items: center; justify-content: flex-end; gap: 7px; color: var(--ink-soft); font-size: 11px; }
.status-line span, .live-label i { width: 6px; height: 6px; background: #64b93d; border-radius: 50%; }
.stat-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 1px; margin: 34px 0; background: var(--line); border: 1px solid var(--line); }
.stat-card { min-height: 144px; padding: 20px 22px; background: var(--surface); }
.stat-top, .stat-foot, .panel-heading, .mission-row, .quick-action, .activity-row { display: flex; align-items: center; }
.stat-top { justify-content: space-between; color: var(--ink-muted); }
.stat-label { color: var(--ink-soft); font-size: 12px; }
.stat-value { margin: 22px 0 10px; font-size: 25px; font-weight: 600; letter-spacing: -.04em; }
.stat-foot { gap: 8px; color: var(--ink-muted); font-size: 11px; }
.positive, .lime { color: #668d00; } .neutral { color: var(--ink-soft); } .blue { color: #5f82a4; } .muted { color: var(--ink-muted); }
.content-grid { display: grid; grid-template-columns: minmax(0, 1.5fr) minmax(300px, .8fr); gap: 1px; background: var(--line); border: 1px solid var(--line); }
.panel, .quick-panel, .activity-panel { min-width: 0; padding: 26px; background: var(--surface); }
.panel-heading { justify-content: space-between; margin-bottom: 27px; }
.panel-heading h3 { margin: 7px 0 0; font-size: 18px; letter-spacing: -.035em; }
.text-link, .panel-button { display: inline-flex; align-items: center; gap: 5px; color: var(--ink); font-size: 11px; text-decoration: none; }
.text-link:hover, .panel-button:hover { text-decoration: underline; }
.mission-list { display: grid; gap: 17px; }
.mission-row { gap: 12px; min-width: 0; padding-bottom: 17px; border-bottom: 1px solid var(--line); }
.mission-row:last-child { padding-bottom: 0; border-bottom: 0; }
.mission-icon { display: grid; width: 35px; height: 35px; flex: 0 0 35px; place-items: center; color: var(--ink); background: var(--surface-soft); }
.mission-info { display: grid; min-width: 150px; gap: 5px; }
.mission-info strong { font-size: 12px; font-weight: 600; }
.mission-info span { color: var(--ink-muted); font-size: 10px; }
.mission-progress { display: flex; flex: 1; align-items: center; gap: 9px; margin-left: auto; }
.progress-track { height: 3px; flex: 1; max-width: 135px; background: var(--surface-muted); }
.progress-track span, .health-bar span { display: block; height: 100%; background: var(--ink); }
.mission-progress small { color: var(--ink-muted); font: 10px 'DM Mono', monospace; }
.mission-state { min-width: 61px; font-size: 10px; text-align: right; }
.health-score { font: 24px 'DM Mono', monospace; letter-spacing: -.08em; }
.health-bar { height: 5px; margin: 4px 0 28px; background: var(--surface-muted); }
.health-bar span { background: var(--accent); }
.health-list { display: grid; gap: 15px; }
.health-row { display: flex; justify-content: space-between; padding-bottom: 12px; border-bottom: 1px solid var(--line); color: var(--ink-soft); font-size: 12px; }
.health-row strong { color: var(--ink); font: 13px 'DM Mono', monospace; } .health-row small { color: var(--ink-muted); font-size: 10px; }
.panel-button { margin-top: 23px; padding-top: 17px; border-top: 1px solid var(--line); }
.bottom-grid { display: grid; grid-template-columns: 1.1fr .9fr; gap: 1px; margin-top: 34px; background: var(--line); border: 1px solid var(--line); }
.quick-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }
.quick-action { justify-content: space-between; gap: 10px; min-height: 65px; padding: 13px; color: var(--ink); border: 1px solid var(--line); text-decoration: none; }
.quick-action:hover { background: var(--accent); border-color: var(--accent); }
.quick-action span { display: grid; width: 31px; height: 31px; place-items: center; background: var(--surface-soft); }
.quick-action strong { flex: 1; font-size: 11px; font-weight: 600; }
.live-label { justify-content: flex-start; gap: 6px; color: var(--ink-muted); }
.activity-list { display: grid; gap: 16px; }
.activity-row { gap: 11px; color: var(--ink-soft); font-size: 11px; }
.activity-time { width: 38px; color: var(--ink-muted); font: 10px 'DM Mono', monospace; }
.activity-dot { width: 5px; height: 5px; flex: 0 0 5px; background: var(--ink-muted); border-radius: 50%; }
.activity-dot.positive, .activity-dot.lime { background: #8dac32; } .activity-dot.blue { background: #7b9bb8; }
@media (max-width: 980px) { .stat-grid { grid-template-columns: repeat(2, 1fr); } .content-grid, .bottom-grid { grid-template-columns: 1fr; } }
@media (max-width: 640px) { .dashboard-intro { display: block; } .intro-aside { padding-top: 30px; text-align: left; } .status-line { justify-content: flex-start; } .stat-grid { grid-template-columns: 1fr; } .mission-row { flex-wrap: wrap; } .mission-progress { flex-basis: calc(100% - 47px); margin-left: 47px; } .mission-state { margin-left: auto; } .quick-grid { grid-template-columns: 1fr; } }
</style>
