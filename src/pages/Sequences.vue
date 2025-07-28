<template>
  <div class="min-h-screen tesla-ui p-6">
    <!-- Tesla-inspired Header with Glass Effect -->
    <div class="mb-8">
      <div class="tesla-card p-6">
        <div class="flex items-center justify-between">
          <div class="space-y-2">
            <h1 class="text-4xl font-light tracking-tight tesla-title">
              Robot Sequences
            </h1>
            <p class="text-slate-400 font-light tesla-subtitle">
              Advanced task orchestration and autonomous execution control
            </p>
          </div>
          <div class="flex items-center gap-3">
            <button class="tesla-btn flex items-center gap-2">
              <Upload class="h-4 w-4" />
              Import
            </button>
            <button class="tesla-btn flex items-center gap-2">
              <Download class="h-4 w-4" />
              Export
            </button>
            <button class="tesla-btn-primary flex items-center gap-2">
              <Save class="h-4 w-4" />
              Save All
            </button>
          </div>
        </div>
      </div>
    </div>

    <!-- Status Cards and Controls Grid -->
    <div class="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
      <!-- Status Cards -->
      <div class="lg:col-span-2">
        <div class="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
          <!-- Active Sequences Card -->
          <div class="tesla-card text-center hover:scale-105 transition-transform duration-300 p-4">
            <div class="h-10 w-10 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <PlayCircle class="h-5 w-5 text-blue-400" />
            </div>
            <p class="text-slate-400 text-xs font-medium">Active Sequences</p>
            <p class="text-xl font-light text-white mt-1">
              {{ sequences.filter(s => s.status === 'running').length }}
            </p>
          </div>

          <!-- Execution Status Card -->
          <div class="tesla-card text-center hover:scale-105 transition-transform duration-300 p-4">
            <div class="h-10 w-10 bg-gradient-to-br from-emerald-500/20 to-green-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <Activity class="h-5 w-5 text-emerald-400" />
            </div>
            <p class="text-slate-400 text-xs font-medium">Progress</p>
            <p class="text-xl font-light text-white mt-1">{{ executionStatus.totalProgress }}%</p>
          </div>

          <!-- Success Rate Card -->
          <div class="tesla-card text-center hover:scale-105 transition-transform duration-300 p-4">
            <div class="h-10 w-10 bg-gradient-to-br from-emerald-500/20 to-teal-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <CheckCircle2 class="h-5 w-5 text-emerald-400" />
            </div>
            <p class="text-xs text-slate-400 font-medium">Success Rate</p>
            <p class="text-xl font-light text-white mt-1">96.2%</p>
          </div>

          <!-- Active Tasks Card -->
          <div class="tesla-card text-center hover:scale-105 transition-transform duration-300 p-4">
            <div class="h-10 w-10 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <Timer class="h-5 w-5 text-purple-400" />
            </div>
            <p class="text-slate-400 text-xs font-medium">Active Tasks</p>
            <p class="text-xl font-light text-white mt-1">
              {{ getActiveTasks(selectedSequence?.id).length }}
            </p>
          </div>
        </div>

        <!-- Sequence Selection -->
        <div class="tesla-card p-6">
          <div class="flex items-center justify-between mb-4">
            <h3 class="text-lg font-light text-white">Available Sequences</h3>
            <button 
              @click="showSequenceDialog = true"
              class="tesla-btn flex items-center gap-2 text-sm"
            >
              <Plus class="h-4 w-4" />
              New Sequence
            </button>
          </div>
          <div class="grid grid-cols-1 gap-3">
            <div
              v-for="sequence in sequences"
              :key="sequence.id"
              @click="selectSequence(sequence.id)"
              :class="[
                'p-4 rounded-lg border transition-all duration-200 cursor-pointer tesla-interactive',
                selectedSequenceId === sequence.id
                  ? 'tesla-glass-intense border-blue-400'
                  : 'tesla-glass border-white/10 hover:border-white/30'
              ]"
            >
              <div class="flex items-center justify-between">
                <div class="flex-1">
                  <div class="flex items-center gap-3">
                    <FolderOpen class="h-4 w-4 text-blue-400" />
                    <h4 class="font-medium text-white">{{ sequence.name }}</h4>
                    <span :class="[
                      'tesla-badge text-xs px-2 py-1 rounded-full',
                      getStatusBadgeClass(sequence.status)
                    ]">
                      {{ sequence.status }}
                    </span>
                  </div>
                  <p class="text-sm text-slate-400 mt-1">{{ sequence.description }}</p>
                  <div class="flex items-center gap-4 mt-2 text-xs text-slate-400">
                    <span>{{ sequence.taskCount }} tasks</span>
                    <span>{{ sequence.successRate }}% success</span>
                    <span v-if="sequence.lastExecuted">
                      Last: {{ new Date(sequence.lastExecuted).toLocaleTimeString() }}
                    </span>
                  </div>
                </div>
                <button
                  @click.stop="handleRunSequence(sequence.id)"
                  class="tesla-btn-primary flex items-center gap-2 text-sm px-3 py-2"
                >
                  <Play class="h-4 w-4" />
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- Execution Logs -->
      <div class="lg:col-span-1">
        <div class="tesla-card h-full p-6">
          <div class="flex items-center justify-between mb-4">
            <h3 class="text-lg font-light text-white">Execution Logs</h3>
            <button class="tesla-btn text-sm">
              <FileText class="h-4 w-4" />
            </button>
          </div>

          <div class="tesla-scrollbar overflow-y-auto max-h-96">
            <div class="space-y-2">
              <div
                v-for="(log, index) in logs"
                :key="index"
                class="flex gap-3 p-2 rounded-lg hover:bg-white/5 transition-colors"
              >
                <span class="text-xs font-mono text-slate-400 w-16 flex-shrink-0">{{ log.time }}</span>
                <span :class="[
                  'text-xs font-medium w-12 flex-shrink-0',
                  getLogLevelClass(log.level)
                ]">
                  {{ log.level }}
                </span>
                <span class="text-xs text-slate-300 flex-1">{{ log.message }}</span>
              </div>
            </div>
          </div>

          <!-- Current execution status -->
          <div v-if="executionStatus.isRunning" class="mt-4 p-3 tesla-status-processing rounded-lg">
            <div class="flex items-center gap-2 mb-2">
              <Activity class="h-4 w-4 text-blue-400" />
              <span class="text-sm font-medium text-blue-400">Executing Sequence</span>
            </div>
            <div class="tesla-progress mb-2">
              <div 
                class="tesla-progress-bar"
                :style="{ width: executionStatus.totalProgress + '%' }"
              ></div>
            </div>
            <div class="flex justify-between text-xs text-slate-400">
              <span>{{ executionStatus.tasksCompleted }}/{{ executionStatus.totalTasks }} tasks</span>
              <span>{{ executionStatus.estimatedTimeRemaining }}s remaining</span>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Task Management Section -->
    <div class="grid grid-cols-1 lg:grid-cols-3 gap-8">
      <!-- Task List -->
      <div class="lg:col-span-2">
        <div class="tesla-card p-6">
          <div class="flex items-center justify-between mb-6">
            <h2 class="text-xl font-light text-white">Task Management</h2>
            <div class="flex items-center gap-3">
              <div class="relative">
                <Search class="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-slate-400" />
                <input 
                  v-model="searchQuery"
                  placeholder="Search tasks..."
                  class="tesla-input pl-10 w-48"
                />
              </div>
              <button class="tesla-btn text-sm">
                <Filter class="h-4 w-4" />
              </button>
              <button 
                @click="openTaskDialog()"
                class="tesla-btn-primary flex items-center gap-2 text-sm"
              >
                <Plus class="h-4 w-4" />
                Add Task
              </button>
            </div>
          </div>

          <div class="tesla-scrollbar overflow-y-auto max-h-96">
            <div class="divide-y divide-white/10">
              <div
                v-for="(task, index) in filteredTasks"
                :key="task.id"
                @click="selectTask(task.id)"
                :class="[
                  'p-4 cursor-pointer transition-all duration-200 hover:bg-white/5',
                  selectedTaskId === task.id
                    ? 'tesla-glass-intense border-l-4 border-blue-400'
                    : ''
                ]"
              >
                <div class="flex items-center gap-4">
                  <!-- Line Number -->
                  <div class="w-8 h-8 rounded-full bg-slate-700 flex items-center justify-center text-sm font-mono text-slate-300">
                    {{ String(index + 1).padStart(2, '0') }}
                  </div>

                  <!-- Task Type Indicator -->
                  <div :class="[
                    'w-3 h-3 rounded-full',
                    getTaskTypeColor(task.type)
                  ]" />

                  <!-- Priority Bar -->
                  <div :class="[
                    'w-1 h-8 rounded-full',
                    getPriorityColor(task.priority)
                  ]" />

                  <!-- Task Info -->
                  <div class="flex-1 min-w-0">
                    <div class="flex items-center gap-2">
                      <h3 class="font-medium text-white truncate">{{ task.name }}</h3>
                      <span :class="[
                        'tesla-badge text-xs px-2 py-1 rounded-full',
                        getStatusBadgeClass(task.status)
                      ]">
                        {{ task.status }}
                      </span>
                    </div>
                    <p class="text-sm text-slate-400 truncate mt-1">{{ task.description }}</p>
                    
                    <div v-if="task.status === 'running' && task.progress" class="mt-2">
                      <div class="tesla-progress">
                        <div 
                          class="tesla-progress-bar"
                          :style="{ width: task.progress + '%' }"
                        ></div>
                      </div>
                    </div>

                    <!-- Sub-tasks -->
                    <div v-if="task.subtasks && task.subtasks.length > 0" class="mt-2">
                      <div class="flex items-center gap-2 text-xs text-slate-400">
                        <FolderTree class="h-3 w-3" />
                        <span>{{ task.subtasks.length }} subtasks</span>
                        <span>{{ task.subtasks.filter(st => st.status === 'completed').length }} completed</span>
                      </div>
                    </div>
                  </div>

                  <!-- Duration & Success Rate -->
                  <div class="text-right">
                    <p class="text-sm font-mono text-white">{{ task.duration }}s</p>
                    <p v-if="task.successRate" class="text-xs text-slate-400">{{ task.successRate }}% success</p>
                  </div>

                  <!-- Action Buttons -->
                  <div class="flex items-center gap-2">
                    <button
                      v-if="task.status === 'running'"
                      class="tesla-btn text-xs px-2 py-1"
                      @click.stop="pauseTask(task.id)"
                    >
                      <PauseCircle class="h-4 w-4" />
                    </button>
                    <button
                      v-else-if="task.status === 'paused'"
                      class="tesla-btn text-xs px-2 py-1"
                      @click.stop="resumeTask(task.id)"
                    >
                      <PlayCircle class="h-4 w-4" />
                    </button>
                    <button
                      v-else
                      class="tesla-btn-success text-xs px-2 py-1"
                      @click.stop="executeTask(task.id)"
                    >
                      <Play class="h-4 w-4" />
                    </button>
                    <button
                      class="tesla-btn text-xs px-2 py-1"
                      @click.stop="openTaskDialog(task)"
                    >
                      <Edit class="h-4 w-4" />
                    </button>
                    <button
                      class="tesla-btn-danger text-xs px-2 py-1"
                      @click.stop="deleteTask(task.id)"
                    >
                      <Trash2 class="h-4 w-4" />
                    </button>
                    <ChevronRight class="h-4 w-4 text-slate-400" />
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- Task Details & Sub-tasks -->
      <div class="space-y-6">
        <!-- Selected Task Details -->
        <div v-if="selectedTask" class="tesla-card p-6">
          <div class="flex items-center justify-between mb-4">
            <h3 class="text-lg font-light text-white">Task Details</h3>
            <button class="tesla-btn text-sm" @click="openTaskDialog(selectedTask)">
              <Edit class="h-4 w-4" />
            </button>
          </div>

          <div class="space-y-4">
            <div>
              <p class="text-sm font-medium text-slate-400 tesla-label">Name</p>
              <p class="text-white">{{ selectedTask.name }}</p>
            </div>

            <div>
              <p class="text-sm font-medium text-slate-400 tesla-label">Type</p>
              <div class="flex items-center gap-2 mt-1">
                <div :class="['w-3 h-3 rounded-full', getTaskTypeColor(selectedTask.type)]" />
                <span class="text-white capitalize">{{ selectedTask.type }}</span>
              </div>
            </div>

            <div>
              <p class="text-sm font-medium text-slate-400 tesla-label">Status</p>
              <span :class="[
                'tesla-badge text-xs px-2 py-1 rounded-full mt-1 inline-block',
                getStatusBadgeClass(selectedTask.status)
              ]">
                {{ selectedTask.status }}
              </span>
            </div>

            <div>
              <p class="text-sm font-medium text-slate-400 tesla-label">Description</p>
              <p class="text-sm text-slate-300 mt-1">{{ selectedTask.description }}</p>
            </div>

            <div v-if="selectedTask.progress">
              <p class="text-sm font-medium text-slate-400 tesla-label">Progress</p>
              <div class="mt-2">
                <div class="tesla-progress">
                  <div 
                    class="tesla-progress-bar"
                    :style="{ width: selectedTask.progress + '%' }"
                  ></div>
                </div>
                <p class="text-xs text-slate-400 mt-1">{{ selectedTask.progress }}% complete</p>
              </div>
            </div>

            <!-- Sub-tasks Management -->
            <div v-if="selectedTask.subtasks">
              <div class="flex items-center justify-between">
                <p class="text-sm font-medium text-slate-400 tesla-label">Sub-tasks</p>
                <button class="tesla-btn text-xs" @click="addSubtask(selectedTask.id)">
                  <Plus class="h-3 w-3 mr-1" />
                  Add
                </button>
              </div>
              <div class="space-y-2 mt-2">
                <div
                  v-for="(subtask, index) in selectedTask.subtasks"
                  :key="subtask.id"
                  class="flex items-center justify-between p-2 tesla-glass rounded-lg"
                >
                  <div class="flex items-center gap-2">
                    <span class="text-xs font-mono text-slate-400">{{ index + 1 }}</span>
                    <span class="text-sm text-white">{{ subtask.name }}</span>
                    <span :class="[
                      'tesla-badge text-xs px-2 py-1 rounded-full',
                      getStatusBadgeClass(subtask.status)
                    ]">
                      {{ subtask.status }}
                    </span>
                  </div>
                  <div class="flex items-center gap-1">
                    <button class="tesla-btn text-xs p-1" @click="executeSubtask(selectedTask.id, subtask.id)">
                      <Play class="h-3 w-3" />
                    </button>
                    <button class="tesla-btn-danger text-xs p-1" @click="deleteSubtask(selectedTask.id, subtask.id)">
                      <Trash2 class="h-3 w-3" />
                    </button>
                  </div>
                </div>
              </div>
            </div>

            <div class="pt-4 border-t border-white/10">
              <div class="flex gap-2">
                <button class="tesla-btn-primary flex-1 flex items-center justify-center gap-2" @click="executeTask(selectedTask.id)">
                  <Play class="h-4 w-4" />
                  Execute
                </button>
                <button class="tesla-btn" @click="openTaskDialog(selectedTask)">
                  <Settings class="h-4 w-4" />
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Task Dialog -->
    <div v-if="showTaskDialog" class="fixed inset-0 z-50 flex items-center justify-center bg-black/50 backdrop-blur-sm">
      <div class="tesla-card m-6 max-w-md w-full p-6">
        <h3 class="text-xl font-light text-white mb-4">
          {{ editingTask ? 'Edit Task' : 'Create New Task' }}
        </h3>
        <p class="text-slate-400 text-sm mb-6">
          {{ editingTask ? 'Modify the task parameters below.' : 'Define a new task for your sequence.' }}
        </p>
        
        <div class="space-y-4">
          <div>
            <label class="tesla-label block mb-2">Task Name</label>
            <input
              v-model="taskForm.name"
              class="tesla-input w-full"
              placeholder="Enter task name..."
            />
          </div>
          
          <div>
            <label class="tesla-label block mb-2">Type</label>
            <select v-model="taskForm.type" class="tesla-input w-full">
              <option value="navigation">Navigation</option>
              <option value="manipulation">Manipulation</option>
              <option value="sensor">Sensor</option>
              <option value="ai">AI Processing</option>
              <option value="custom">Custom</option>
            </select>
          </div>
          
          <div>
            <label class="tesla-label block mb-2">Priority</label>
            <select v-model="taskForm.priority" class="tesla-input w-full">
              <option value="low">Low</option>
              <option value="medium">Medium</option>
              <option value="high">High</option>
              <option value="critical">Critical</option>
            </select>
          </div>
          
          <div>
            <label class="tesla-label block mb-2">Description</label>
            <textarea
              v-model="taskForm.description"
              class="tesla-input w-full"
              placeholder="Describe the task..."
              rows="3"
            ></textarea>
          </div>

          <div>
            <label class="tesla-label block mb-2">Duration (seconds)</label>
            <input
              v-model.number="taskForm.duration"
              type="number"
              min="1"
              class="tesla-input w-full"
            />
          </div>
        </div>
        
        <div class="flex justify-end gap-2 mt-6">
          <button 
            @click="showTaskDialog = false"
            class="tesla-btn"
          >
            Cancel
          </button>
          <button 
            @click="saveTask"
            class="tesla-btn-primary"
          >
            {{ editingTask ? 'Update Task' : 'Create Task' }}
          </button>
        </div>
      </div>
    </div>

    <!-- Sequence Dialog -->
    <div v-if="showSequenceDialog" class="fixed inset-0 z-50 flex items-center justify-center bg-black/50 backdrop-blur-sm">
      <div class="tesla-card m-6 max-w-md w-full p-6">
        <h3 class="text-xl font-light text-white mb-4">Create New Sequence</h3>
        <p class="text-slate-400 text-sm mb-6">
          Create a new task sequence for automated execution.
        </p>
        
        <div class="space-y-4">
          <div>
            <label class="tesla-label block mb-2">Sequence Name</label>
            <input
              v-model="sequenceForm.name"
              class="tesla-input w-full"
              placeholder="Enter sequence name..."
            />
          </div>
          
          <div>
            <label class="tesla-label block mb-2">Description</label>
            <textarea
              v-model="sequenceForm.description"
              class="tesla-input w-full"
              placeholder="Describe the sequence..."
              rows="3"
            ></textarea>
          </div>
        </div>
        
        <div class="flex justify-end gap-2 mt-6">
          <button 
            @click="showSequenceDialog = false"
            class="tesla-btn"
          >
            Cancel
          </button>
          <button 
            @click="createSequence"
            class="tesla-btn-primary"
          >
            Create Sequence
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted } from 'vue'
import { usePersistedReactive } from '@/stores/persistence'
import {
  Play,
  Pause,
  Square,
  Plus,
  Settings,
  CheckCircle2,
  Activity,
  Timer,
  Download,
  Upload,
  Save,
  Trash2,
  FileText,
  Edit,
  PlayCircle,
  PauseCircle,
  ChevronRight,
  Search,
  Filter,
  FolderOpen,
  FolderTree
} from 'lucide-vue-next'

interface Subtask {
  id: string
  name: string
  status: 'idle' | 'running' | 'completed' | 'failed' | 'paused'
  duration: number
  parameters: Record<string, any>
}

interface Task {
  id: string
  name: string
  type: 'navigation' | 'manipulation' | 'sensor' | 'ai' | 'custom'
  status: 'idle' | 'running' | 'completed' | 'failed' | 'paused'
  duration: number
  priority: 'low' | 'medium' | 'high' | 'critical'
  description: string
  parameters: Record<string, any>
  progress?: number
  estimatedTime?: number
  lastExecuted?: Date
  successRate?: number
  dependencies?: string[]
  subtasks?: Subtask[]
}

interface Sequence {
  id: string
  name: string
  description: string
  status: 'idle' | 'running' | 'completed' | 'failed' | 'paused'
  taskCount: number
  completedTasks: number
  totalDuration: number
  lastExecuted?: Date
  successRate: number
  tags: string[]
  tasks: string[] // Task IDs in this sequence
}

// Persistent state
const { state: preferences, updateField } = usePersistedReactive('sequences-preferences', {
  selectedTaskId: null as string | null,
  selectedSequenceId: null as string | null,
  viewMode: 'overview' as 'overview' | 'detailed',
  filterStatus: 'all' as string,
  sortBy: 'priority' as string,
})

// State
const searchQuery = ref('')
const showTaskDialog = ref(false)
const showSequenceDialog = ref(false)
const editingTask = ref<Task | null>(null)

const taskForm = ref({
  name: '',
  type: 'navigation' as Task['type'],
  priority: 'medium' as Task['priority'],
  description: '',
  duration: 60,
  parameters: {}
})

const sequenceForm = ref({
  name: '',
  description: ''
})

const sequences = ref<Sequence[]>([
  {
    id: 'seq_1',
    name: 'Security Patrol Route',
    description: 'Comprehensive security patrol covering all designated checkpoints',
    status: 'idle',
    taskCount: 5,
    completedTasks: 5,
    totalDuration: 12.5,
    lastExecuted: new Date(Date.now() - 3600000),
    successRate: 98.2,
    tags: ['security', 'patrol', 'autonomous'],
    tasks: ['task_1', 'task_2', 'task_3']
  },
  {
    id: 'seq_2',
    name: 'Maintenance Inspection',
    description: 'Automated maintenance and system health verification',
    status: 'running',
    taskCount: 8,
    completedTasks: 3,
    totalDuration: 25.0,
    successRate: 94.7,
    tags: ['maintenance', 'inspection', 'diagnostics'],
    tasks: ['task_4', 'task_5']
  }
])

const tasks = ref<Task[]>([
  {
    id: 'task_1',
    name: 'Navigate to Checkpoint Alpha',
    type: 'navigation',
    status: 'completed',
    duration: 180,
    priority: 'high',
    description: 'Move to security checkpoint Alpha using optimal path planning',
    parameters: { target: 'checkpoint_alpha', speed: 0.8, precision: 'high' },
    progress: 100,
    lastExecuted: new Date(Date.now() - 3600000),
    successRate: 99.1,
    subtasks: [
      { id: 'st_1', name: 'Calculate path', status: 'completed', duration: 30, parameters: {} },
      { id: 'st_2', name: 'Execute movement', status: 'completed', duration: 150, parameters: {} }
    ]
  },
  {
    id: 'task_2',
    name: 'Scan Environment',
    type: 'sensor',
    status: 'idle',
    duration: 45,
    priority: 'medium',
    description: '360-degree environmental scan using LIDAR and cameras',
    parameters: { scan_type: 'full', resolution: 'high', duration: 30 },
    successRate: 97.8,
    subtasks: [
      { id: 'st_3', name: 'Initialize sensors', status: 'idle', duration: 10, parameters: {} },
      { id: 'st_4', name: 'Perform scan', status: 'idle', duration: 30, parameters: {} },
      { id: 'st_5', name: 'Process data', status: 'idle', duration: 5, parameters: {} }
    ]
  },
  {
    id: 'task_3',
    name: 'Object Recognition',
    type: 'ai',
    status: 'idle',
    duration: 90,
    priority: 'high',
    description: 'AI-powered object detection and anomaly identification',
    parameters: { model: 'yolo_v8', confidence: 0.85, classes: ['person', 'vehicle'] },
    successRate: 94.3
  },
  {
    id: 'task_4',
    name: 'Pick and Place Operation',
    type: 'manipulation',
    status: 'running',
    duration: 240,
    priority: 'critical',
    description: 'Precise manipulation task with force feedback',
    parameters: { object: 'package_01', destination: 'drop_zone_b' },
    progress: 65,
    estimatedTime: 84,
    successRate: 91.7
  },
  {
    id: 'task_5',
    name: 'Status Report',
    type: 'custom',
    status: 'idle',
    duration: 15,
    priority: 'low',
    description: 'Generate and transmit status report to control center',
    parameters: { format: 'json', include_metrics: true },
    successRate: 99.9
  }
])

const executionStatus = ref({
  isRunning: false,
  currentSequence: null as string | null,
  totalProgress: 45,
  estimatedTimeRemaining: 127,
  tasksCompleted: 3,
  totalTasks: 8
})

const logs = ref([
  { time: '14:32:15', level: 'INFO', message: 'Task "Navigate to Checkpoint Alpha" completed successfully' },
  { time: '14:31:42', level: 'INFO', message: 'Starting environmental scan procedure' },
  { time: '14:31:15', level: 'WARN', message: 'Obstacle detected, recalculating path' },
  { time: '14:30:58', level: 'INFO', message: 'Sequence "Security Patrol Route" initiated' }
])

// Computed
const selectedSequenceId = computed(() => preferences.selectedSequenceId)
const selectedTaskId = computed(() => preferences.selectedTaskId)
const selectedSequence = computed(() => sequences.value.find(s => s.id === selectedSequenceId.value))
const selectedTask = computed(() => tasks.value.find(t => t.id === selectedTaskId.value))

const filteredTasks = computed(() => {
  let filtered = tasks.value

  // Filter by selected sequence
  if (selectedSequence.value) {
    filtered = filtered.filter(task => selectedSequence.value!.tasks.includes(task.id))
  }

  // Filter by search query
  if (searchQuery.value) {
    const query = searchQuery.value.toLowerCase()
    filtered = filtered.filter(task => 
      task.name.toLowerCase().includes(query) ||
      task.description.toLowerCase().includes(query)
    )
  }

  return filtered
})

// Helper functions
const getActiveTasks = (sequenceId?: string) => {
  if (!sequenceId) return tasks.value.filter(t => t.status === 'running')
  const sequence = sequences.value.find(s => s.id === sequenceId)
  if (!sequence) return []
  return tasks.value.filter(t => sequence.tasks.includes(t.id) && t.status === 'running')
}

const getStatusBadgeClass = (status: string) => {
  switch (status) {
    case 'completed': return 'tesla-badge-success'
    case 'running': return 'tesla-badge-info'
    case 'failed': return 'tesla-badge-error'
    case 'paused': return 'tesla-badge-warning'
    default: return 'tesla-badge'
  }
}

const getLogLevelClass = (level: string) => {
  switch (level) {
    case 'INFO': return 'text-blue-400'
    case 'WARN': return 'text-yellow-400'
    case 'ERROR': return 'text-red-400'
    default: return 'text-slate-400'
  }
}

const getTaskTypeColor = (type: string) => {
  switch (type) {
    case 'navigation': return 'bg-blue-500'
    case 'manipulation': return 'bg-purple-500'
    case 'sensor': return 'bg-green-500'
    case 'ai': return 'bg-pink-500'
    default: return 'bg-gray-400'
  }
}

const getPriorityColor = (priority: string) => {
  switch (priority) {
    case 'critical': return 'bg-red-500'
    case 'high': return 'bg-orange-500'
    case 'medium': return 'bg-yellow-500'
    default: return 'bg-gray-400'
  }
}

// Actions
const selectSequence = (sequenceId: string) => {
  updateField('selectedSequenceId', sequenceId)
  updateField('selectedTaskId', null) // Clear task selection when switching sequences
  addLog('INFO', `Selected sequence: ${sequences.value.find(s => s.id === sequenceId)?.name}`)
}

const selectTask = (taskId: string) => {
  updateField('selectedTaskId', taskId)
}

const handleRunSequence = (sequenceId: string) => {
  updateField('selectedSequenceId', sequenceId)
  executionStatus.value.isRunning = true
  executionStatus.value.currentSequence = sequenceId
  
  const sequence = sequences.value.find(s => s.id === sequenceId)
  if (sequence) {
    sequence.status = 'running'
    addLog('INFO', `Started executing sequence: ${sequence.name}`)
  }
}

const openTaskDialog = (task?: Task) => {
  if (task) {
    editingTask.value = task
    taskForm.value = { ...task }
  } else {
    editingTask.value = null
    taskForm.value = {
      name: '',
      type: 'navigation',
      priority: 'medium',
      description: '',
      duration: 60,
      parameters: {}
    }
  }
  showTaskDialog.value = true
}

const saveTask = () => {
  if (!taskForm.value.name) return
  
  if (editingTask.value) {
    // Update existing task
    const index = tasks.value.findIndex(t => t.id === editingTask.value!.id)
    if (index !== -1) {
      tasks.value[index] = { ...editingTask.value, ...taskForm.value }
      addLog('INFO', `Updated task: ${taskForm.value.name}`)
    }
  } else {
    // Create new task
    const newTask: Task = {
      id: `task_${Date.now()}`,
      status: 'idle',
      successRate: 0,
      ...taskForm.value
    }
    tasks.value.push(newTask)
    
    // Add to selected sequence if any
    if (selectedSequence.value) {
      selectedSequence.value.tasks.push(newTask.id)
      selectedSequence.value.taskCount++
    }
    
    addLog('INFO', `Created new task: ${taskForm.value.name}`)
  }
  
  showTaskDialog.value = false
}

const createSequence = () => {
  if (!sequenceForm.value.name) return
  
  const newSequence: Sequence = {
    id: `seq_${Date.now()}`,
    name: sequenceForm.value.name,
    description: sequenceForm.value.description,
    status: 'idle',
    taskCount: 0,
    completedTasks: 0,
    totalDuration: 0,
    successRate: 0,
    tags: [],
    tasks: []
  }
  
  sequences.value.push(newSequence)
  showSequenceDialog.value = false
  
  addLog('INFO', `Created new sequence: ${sequenceForm.value.name}`)
  
  // Reset form
  sequenceForm.value = { name: '', description: '' }
}

const executeTask = (taskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task) {
    task.status = 'running'
    task.progress = 0
    addLog('INFO', `Started executing task: ${task.name}`)
  }
}

const pauseTask = (taskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task) {
    task.status = 'paused'
    addLog('WARN', `Paused task: ${task.name}`)
  }
}

const resumeTask = (taskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task) {
    task.status = 'running'
    addLog('INFO', `Resumed task: ${task.name}`)
  }
}

const deleteTask = (taskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task) {
    tasks.value = tasks.value.filter(t => t.id !== taskId)
    
    // Remove from sequences
    sequences.value.forEach(seq => {
      const index = seq.tasks.indexOf(taskId)
      if (index !== -1) {
        seq.tasks.splice(index, 1)
        seq.taskCount--
      }
    })
    
    if (selectedTaskId.value === taskId) {
      updateField('selectedTaskId', null)
    }
    
    addLog('INFO', `Deleted task: ${task.name}`)
  }
}

const addSubtask = (taskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task) {
    if (!task.subtasks) task.subtasks = []
    
    const newSubtask: Subtask = {
      id: `st_${Date.now()}`,
      name: `Subtask ${task.subtasks.length + 1}`,
      status: 'idle',
      duration: 30,
      parameters: {}
    }
    
    task.subtasks.push(newSubtask)
    addLog('INFO', `Added subtask to: ${task.name}`)
  }
}

const executeSubtask = (taskId: string, subtaskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task && task.subtasks) {
    const subtask = task.subtasks.find(st => st.id === subtaskId)
    if (subtask) {
      subtask.status = 'running'
      addLog('INFO', `Executing subtask: ${subtask.name}`)
    }
  }
}

const deleteSubtask = (taskId: string, subtaskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task && task.subtasks) {
    task.subtasks = task.subtasks.filter(st => st.id !== subtaskId)
    addLog('INFO', `Deleted subtask from: ${task.name}`)
  }
}

const addLog = (level: string, message: string) => {
  const time = new Date().toLocaleTimeString()
  logs.value.unshift({ time, level, message })
  if (logs.value.length > 50) {
    logs.value = logs.value.slice(0, 50)
  }
}

onMounted(() => {
  // Initialize with first sequence if none selected
  if (!selectedSequenceId.value && sequences.value.length > 0) {
    updateField('selectedSequenceId', sequences.value[0].id)
  }
})
</script>
