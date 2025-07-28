<template>
  <div class="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6 text-white">
    <!-- Tesla-inspired Header with Glass Effect -->
    <div class="mb-8">
      <n-card class="tesla-glass-card">
        <div class="flex items-center justify-between">
          <div class="space-y-2">
            <h1 class="text-4xl font-light tracking-tight bg-gradient-to-r from-white via-blue-100 to-cyan-100 bg-clip-text text-transparent">
              Robot Sequences
            </h1>
            <p class="text-slate-400 font-light">
              Advanced task orchestration and autonomous execution control
            </p>
          </div>
          <div class="flex items-center gap-3">
            <n-button class="tesla-button-secondary">
              <template #icon>
                <Upload class="h-4 w-4" />
              </template>
              Import
            </n-button>
            <n-button class="tesla-button-secondary">
              <template #icon>
                <Download class="h-4 w-4" />
              </template>
              Export
            </n-button>
            <n-button type="primary" class="tesla-button-primary">
              <template #icon>
                <Save class="h-4 w-4" />
              </template>
              Save All
            </n-button>
          </div>
        </div>
      </n-card>
    </div>

    <!-- Status Cards and Controls Grid -->
    <div class="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
      <!-- Status Cards -->
      <div class="lg:col-span-2">
        <div class="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
          <!-- Active Sequences Card -->
          <n-card class="tesla-glass-card text-center hover:scale-105 transition-transform duration-300">
            <div class="h-10 w-10 bg-gradient-to-br from-blue-500/20 to-cyan-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <PlayCircle class="h-5 w-5 text-blue-400" />
            </div>
            <p class="text-slate-400 text-xs font-medium">Active Sequences</p>
            <p class="text-xl font-light text-white mt-1">
              {{ sequences.filter(s => s.status === 'running').length }}
            </p>
          </n-card>

          <!-- Execution Status Card -->
          <n-card class="tesla-glass-card text-center hover:scale-105 transition-transform duration-300">
            <div class="h-10 w-10 bg-gradient-to-br from-emerald-500/20 to-green-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <Activity class="h-5 w-5 text-emerald-400" />
            </div>
            <p class="text-slate-400 text-xs font-medium">Progress</p>
            <p class="text-xl font-light text-white mt-1">{{ executionStatus.totalProgress }}%</p>
          </n-card>

          <!-- Success Rate Card -->
          <n-card class="tesla-glass-card text-center hover:scale-105 transition-transform duration-300">
            <div class="h-10 w-10 bg-gradient-to-br from-emerald-500/20 to-teal-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <CheckCircle2 class="h-5 w-5 text-emerald-400" />
            </div>
            <p class="text-slate-400 text-xs font-medium">Success Rate</p>
            <p class="text-xl font-light text-white mt-1">96.2%</p>
          </n-card>

          <!-- Active Tasks Card -->
          <n-card class="tesla-glass-card text-center hover:scale-105 transition-transform duration-300">
            <div class="h-10 w-10 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-xl flex items-center justify-center mx-auto mb-2">
              <Timer class="h-5 w-5 text-purple-400" />
            </div>
            <p class="text-slate-400 text-xs font-medium">Active Tasks</p>
            <p class="text-xl font-light text-white mt-1">
              {{ getActiveTasks(selectedSequence?.id).length }}
            </p>
          </n-card>
        </div>

        <!-- Sequence Selection -->
        <n-card class="tesla-glass-card">
          <div class="flex items-center justify-between mb-4">
            <h3 class="text-lg font-light text-white">Available Sequences</h3>
            <n-button 
              @click="showSequenceDialog = true"
              class="tesla-button-secondary"
              size="small"
            >
              <template #icon>
                <Plus class="h-4 w-4" />
              </template>
              New Sequence
            </n-button>
          </div>
          <div class="grid grid-cols-1 gap-3">
            <div
              v-for="sequence in sequences"
              :key="sequence.id"
              @click="selectSequence(sequence.id)"
              :class="[
                'p-4 rounded-lg border transition-all duration-200 cursor-pointer',
                selectedSequenceId === sequence.id
                  ? 'bg-blue-500/10 border-blue-400'
                  : 'bg-white/5 border-white/10 hover:bg-white/10'
              ]"
            >
              <div class="flex items-center justify-between">
                <div class="flex-1">
                  <div class="flex items-center gap-3">
                    <FolderOpen class="h-4 w-4 text-blue-400" />
                    <h4 class="font-medium text-white">{{ sequence.name }}</h4>
                    <n-tag :type="getStatusTagType(sequence.status)" size="small">
                      {{ sequence.status }}
                    </n-tag>
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
                <n-button
                  @click.stop="handleRunSequence(sequence.id)"
                  type="primary"
                  size="small"
                  class="tesla-button-primary"
                >
                  <template #icon>
                    <Play class="h-4 w-4" />
                  </template>
                </n-button>
              </div>
            </div>
          </div>
        </n-card>
      </div>

      <!-- Execution Logs -->
      <div class="lg:col-span-1">
        <n-card class="tesla-glass-card h-full">
          <div class="flex items-center justify-between mb-4">
            <h3 class="text-lg font-light text-white">Execution Logs</h3>
            <n-button size="small" class="tesla-button-secondary">
              <template #icon>
                <FileText class="h-4 w-4" />
              </template>
            </n-button>
          </div>

          <n-scrollbar style="max-height: 400px">
            <div class="space-y-2">
              <div
                v-for="(log, index) in logs"
                :key="index"
                class="flex gap-3 p-2 rounded-lg hover:bg-white/5 transition-colors"
              >
                <span class="text-xs font-mono text-slate-400 w-16 flex-shrink-0">{{ log.time }}</span>
                <span :class="[
                  'text-xs font-medium w-12 flex-shrink-0',
                  log.level === 'INFO' ? 'text-blue-400' :
                  log.level === 'WARN' ? 'text-yellow-400' :
                  'text-red-400'
                ]">
                  {{ log.level }}
                </span>
                <span class="text-xs text-slate-300 flex-1">{{ log.message }}</span>
              </div>
            </div>
          </n-scrollbar>

          <!-- Current execution status -->
          <div v-if="executionStatus.isRunning" class="mt-4 p-3 bg-blue-500/10 border border-blue-400/30 rounded-lg">
            <div class="flex items-center gap-2 mb-2">
              <Activity class="h-4 w-4 text-blue-400" />
              <span class="text-sm font-medium text-blue-400">Executing Sequence</span>
            </div>
            <n-progress :percentage="executionStatus.totalProgress" class="mb-2" />
            <div class="flex justify-between text-xs text-slate-400">
              <span>{{ executionStatus.tasksCompleted }}/{{ executionStatus.totalTasks }} tasks</span>
              <span>{{ executionStatus.estimatedTimeRemaining }}s remaining</span>
            </div>
          </div>
        </n-card>
      </div>
    </div>

    <!-- Task Management Section -->
    <div class="grid grid-cols-1 lg:grid-cols-3 gap-8">
      <!-- Task List -->
      <div class="lg:col-span-2">
        <n-card class="tesla-glass-card">
          <div class="flex items-center justify-between mb-6">
            <h2 class="text-xl font-light text-white">Task Management</h2>
            <div class="flex items-center gap-3">
              <n-input
                v-model:value="searchQuery"
                placeholder="Search tasks..."
                class="tesla-input"
                style="width: 200px"
              >
                <template #prefix>
                  <Search class="h-4 w-4 text-slate-400" />
                </template>
              </n-input>
              <n-button size="small" class="tesla-button-secondary">
                <template #icon>
                  <Filter class="h-4 w-4" />
                </template>
              </n-button>
              <n-button 
                @click="openTaskDialog()"
                type="primary"
                size="small"
                class="tesla-button-primary"
              >
                <template #icon>
                  <Plus class="h-4 w-4" />
                </template>
                Add Task
              </n-button>
            </div>
          </div>

          <n-scrollbar style="max-height: 500px">
            <div class="divide-y divide-white/10">
              <div
                v-for="(task, index) in filteredTasks"
                :key="task.id"
                @click="selectTask(task.id)"
                :class="[
                  'p-4 cursor-pointer transition-all duration-200 hover:bg-white/5',
                  selectedTaskId === task.id
                    ? 'bg-blue-500/10 border-l-4 border-blue-400'
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
                      <n-tag :type="getStatusTagType(task.status)" size="small">
                        {{ task.status }}
                      </n-tag>
                    </div>
                    <p class="text-sm text-slate-400 truncate mt-1">{{ task.description }}</p>
                    
                    <div v-if="task.status === 'running' && task.progress" class="mt-2">
                      <n-progress :percentage="task.progress" size="small" />
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
                    <n-button
                      v-if="task.status === 'running'"
                      size="small"
                      type="warning"
                      @click.stop="pauseTask(task.id)"
                    >
                      <template #icon>
                        <PauseCircle class="h-4 w-4" />
                      </template>
                    </n-button>
                    <n-button
                      v-else-if="task.status === 'paused'"
                      size="small"
                      type="info"
                      @click.stop="resumeTask(task.id)"
                    >
                      <template #icon>
                        <PlayCircle class="h-4 w-4" />
                      </template>
                    </n-button>
                    <n-button
                      v-else
                      size="small"
                      type="success"
                      @click.stop="executeTask(task.id)"
                    >
                      <template #icon>
                        <Play class="h-4 w-4" />
                      </template>
                    </n-button>
                    <n-button
                      size="small"
                      type="info"
                      @click.stop="openTaskDialog(task)"
                    >
                      <template #icon>
                        <Edit class="h-4 w-4" />
                      </template>
                    </n-button>
                    <n-button
                      size="small"
                      type="error"
                      @click.stop="deleteTask(task.id)"
                    >
                      <template #icon>
                        <Trash2 class="h-4 w-4" />
                      </template>
                    </n-button>
                    <ChevronRight class="h-4 w-4 text-slate-400" />
                  </div>
                </div>
              </div>
            </div>
          </n-scrollbar>
        </n-card>
      </div>

      <!-- Task Details & Sub-tasks -->
      <div class="space-y-6">
        <!-- Selected Task Details -->
        <n-card v-if="selectedTask" class="tesla-glass-card">
          <div class="flex items-center justify-between mb-4">
            <h3 class="text-lg font-light text-white">Task Details</h3>
            <n-button size="small" @click="openTaskDialog(selectedTask)" class="tesla-button-secondary">
              <template #icon>
                <Edit class="h-4 w-4" />
              </template>
            </n-button>
          </div>

          <div class="space-y-4">
            <div>
              <p class="text-sm font-medium text-slate-400">Name</p>
              <p class="text-white">{{ selectedTask.name }}</p>
            </div>

            <div>
              <p class="text-sm font-medium text-slate-400">Type</p>
              <div class="flex items-center gap-2 mt-1">
                <div :class="['w-3 h-3 rounded-full', getTaskTypeColor(selectedTask.type)]" />
                <span class="text-white capitalize">{{ selectedTask.type }}</span>
              </div>
            </div>

            <div>
              <p class="text-sm font-medium text-slate-400">Status</p>
              <n-tag :type="getStatusTagType(selectedTask.status)" class="mt-1">
                {{ selectedTask.status }}
              </n-tag>
            </div>

            <div>
              <p class="text-sm font-medium text-slate-400">Description</p>
              <p class="text-sm text-slate-300 mt-1">{{ selectedTask.description }}</p>
            </div>

            <div v-if="selectedTask.progress">
              <p class="text-sm font-medium text-slate-400">Progress</p>
              <div class="mt-2">
                <n-progress :percentage="selectedTask.progress" />
                <p class="text-xs text-slate-400 mt-1">{{ selectedTask.progress }}% complete</p>
              </div>
            </div>

            <!-- Sub-tasks Management -->
            <div v-if="selectedTask.subtasks">
              <div class="flex items-center justify-between">
                <p class="text-sm font-medium text-slate-400">Sub-tasks</p>
                <n-button size="tiny" @click="addSubtask(selectedTask.id)" class="tesla-button-secondary">
                  <template #icon>
                    <Plus class="h-3 w-3" />
                  </template>
                  Add
                </n-button>
              </div>
              <div class="space-y-2 mt-2">
                <div
                  v-for="(subtask, index) in selectedTask.subtasks"
                  :key="subtask.id"
                  class="flex items-center justify-between p-2 bg-white/5 rounded-lg"
                >
                  <div class="flex items-center gap-2">
                    <span class="text-xs font-mono text-slate-400">{{ index + 1 }}</span>
                    <span class="text-sm text-white">{{ subtask.name }}</span>
                    <n-tag :type="getStatusTagType(subtask.status)" size="small">
                      {{ subtask.status }}
                    </n-tag>
                  </div>
                  <div class="flex items-center gap-1">
                    <n-button size="tiny" @click="executeSubtask(selectedTask.id, subtask.id)">
                      <template #icon>
                        <Play class="h-3 w-3" />
                      </template>
                    </n-button>
                    <n-button size="tiny" @click="deleteSubtask(selectedTask.id, subtask.id)" type="error">
                      <template #icon>
                        <Trash2 class="h-3 w-3" />
                      </template>
                    </n-button>
                  </div>
                </div>
              </div>
            </div>

            <div class="pt-4 border-t border-white/10">
              <div class="flex gap-2">
                <n-button type="primary" class="flex-1 tesla-button-primary" @click="executeTask(selectedTask.id)">
                  <template #icon>
                    <Play class="h-4 w-4" />
                  </template>
                  Execute
                </n-button>
                <n-button @click="openTaskDialog(selectedTask)" class="tesla-button-secondary">
                  <template #icon>
                    <Settings class="h-4 w-4" />
                  </template>
                </n-button>
              </div>
            </div>
          </div>
        </n-card>
      </div>
    </div>

    <!-- Task Dialog -->
    <n-modal v-model:show="showTaskDialog" :mask-closable="false">
      <n-card
        style="width: 600px"
        :title="editingTask ? 'Edit Task' : 'Create New Task'"
        :bordered="false"
        size="huge"
        role="dialog"
        aria-modal="true"
        class="tesla-glass-card"
      >
        <n-form :model="taskForm" :rules="taskFormRules" ref="taskFormRef">
          <n-form-item label="Task Name" path="name">
            <n-input v-model:value="taskForm.name" placeholder="Enter task name..." class="tesla-input" />
          </n-form-item>
          
          <n-form-item label="Type" path="type">
            <n-select v-model:value="taskForm.type" :options="taskTypeOptions" class="tesla-select" />
          </n-form-item>
          
          <n-form-item label="Priority" path="priority">
            <n-select v-model:value="taskForm.priority" :options="priorityOptions" class="tesla-select" />
          </n-form-item>
          
          <n-form-item label="Description" path="description">
            <n-input
              v-model:value="taskForm.description"
              type="textarea"
              :rows="3"
              placeholder="Describe the task..."
              class="tesla-input"
            />
          </n-form-item>

          <n-form-item label="Duration (seconds)" path="duration">
            <n-input-number v-model:value="taskForm.duration" :min="1" class="tesla-input" />
          </n-form-item>
        </n-form>
        
        <template #footer>
          <div class="flex justify-end gap-2">
            <n-button @click="showTaskDialog = false" class="tesla-button-secondary">
              Cancel
            </n-button>
            <n-button type="primary" @click="saveTask" class="tesla-button-primary">
              {{ editingTask ? 'Update Task' : 'Create Task' }}
            </n-button>
          </div>
        </template>
      </n-card>
    </n-modal>

    <!-- Sequence Dialog -->
    <n-modal v-model:show="showSequenceDialog" :mask-closable="false">
      <n-card
        style="width: 600px"
        title="Create New Sequence"
        :bordered="false"
        size="huge"
        role="dialog"
        aria-modal="true"
        class="tesla-glass-card"
      >
        <n-form :model="sequenceForm" ref="sequenceFormRef">
          <n-form-item label="Sequence Name" path="name">
            <n-input v-model:value="sequenceForm.name" placeholder="Enter sequence name..." class="tesla-input" />
          </n-form-item>
          
          <n-form-item label="Description" path="description">
            <n-input
              v-model:value="sequenceForm.description"
              type="textarea"
              :rows="3"
              placeholder="Describe the sequence..."
              class="tesla-input"
            />
          </n-form-item>
        </n-form>
        
        <template #footer>
          <div class="flex justify-end gap-2">
            <n-button @click="showSequenceDialog = false" class="tesla-button-secondary">
              Cancel
            </n-button>
            <n-button type="primary" @click="createSequence" class="tesla-button-primary">
              Create Sequence
            </n-button>
          </div>
        </template>
      </n-card>
    </n-modal>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted } from 'vue'
import { useMessage } from 'naive-ui'
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

const message = useMessage()

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
const taskFormRef = ref()
const sequenceFormRef = ref()

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

// Form options
const taskTypeOptions = [
  { label: 'Navigation', value: 'navigation' },
  { label: 'Manipulation', value: 'manipulation' },
  { label: 'Sensor', value: 'sensor' },
  { label: 'AI Processing', value: 'ai' },
  { label: 'Custom', value: 'custom' }
]

const priorityOptions = [
  { label: 'Low', value: 'low' },
  { label: 'Medium', value: 'medium' },
  { label: 'High', value: 'high' },
  { label: 'Critical', value: 'critical' }
]

const taskFormRules = {
  name: { required: true, message: 'Task name is required' },
  type: { required: true, message: 'Task type is required' },
  priority: { required: true, message: 'Priority is required' },
  duration: { required: true, type: 'number', message: 'Duration must be a number' }
}

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

const getStatusTagType = (status: string) => {
  switch (status) {
    case 'completed': return 'success'
    case 'running': return 'info'
    case 'failed': return 'error'
    case 'paused': return 'warning'
    default: return 'default'
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

const saveTask = async () => {
  try {
    await taskFormRef.value?.validate()
    
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
    message.success(editingTask.value ? 'Task updated successfully' : 'Task created successfully')
  } catch (error) {
    message.error('Please fill in all required fields')
  }
}

const createSequence = () => {
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
  message.success('Sequence created successfully')
  
  // Reset form
  sequenceForm.value = { name: '', description: '' }
}

const executeTask = (taskId: string) => {
  const task = tasks.value.find(t => t.id === taskId)
  if (task) {
    task.status = 'running'
    task.progress = 0
    addLog('INFO', `Started executing task: ${task.name}`)
    message.info(`Executing task: ${task.name}`)
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
    message.success('Task deleted successfully')
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
  backdrop-filter: blur(10px) !important;
}

.tesla-input {
  background: rgba(255, 255, 255, 0.05) !important;
  border: 1px solid rgba(255, 255, 255, 0.2) !important;
  color: white !important;
}

.tesla-select {
  background: rgba(255, 255, 255, 0.05) !important;
}
</style>
