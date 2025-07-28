import { defineStore } from 'pinia'
import { ref, reactive } from 'vue'

interface PersistenceData {
  value: any
  timestamp: number
  ttl: number | null
}

class PersistenceService {
  private static instance: PersistenceService

  static getInstance(): PersistenceService {
    if (!PersistenceService.instance) {
      PersistenceService.instance = new PersistenceService()
    }
    return PersistenceService.instance
  }

  save<T>(key: string, value: T, ttl?: number): void {
    const data: PersistenceData = {
      value,
      timestamp: Date.now(),
      ttl: ttl || null
    }
    localStorage.setItem(key, JSON.stringify(data))
  }

  load<T>(key: string, defaultValue: T): T {
    try {
      const stored = localStorage.getItem(key)
      if (!stored) return defaultValue

      const data: PersistenceData = JSON.parse(stored)
      
      if (data.ttl && Date.now() - data.timestamp > data.ttl) {
        localStorage.removeItem(key)
        return defaultValue
      }

      return data.value as T
    } catch {
      return defaultValue
    }
  }

  remove(key: string): void {
    localStorage.removeItem(key)
  }

  clear(): void {
    localStorage.clear()
  }
}

export const usePersistenceStore = defineStore('persistence', () => {
  const service = PersistenceService.getInstance()
  const appPrefix = ref('dino-core')

  const save = <T>(key: string, value: T, ttl?: number) => {
    service.save(`${appPrefix.value}-${key}`, value, ttl)
  }

  const load = <T>(key: string, defaultValue: T): T => {
    return service.load(`${appPrefix.value}-${key}`, defaultValue)
  }

  const remove = (key: string) => {
    service.remove(`${appPrefix.value}-${key}`)
  }

  const clear = () => {
    service.clear()
  }

  return {
    save,
    load,
    remove,
    clear
  }
})

export const usePersistedRef = <T>(key: string, defaultValue: T) => {
  const persistence = usePersistenceStore()
  const value = ref(persistence.load(key, defaultValue))

  const setValue = (newValue: T) => {
    value.value = newValue
    persistence.save(key, newValue)
  }

  return {
    value,
    setValue
  }
}

export const usePersistedReactive = <T extends Record<string, any>>(key: string, defaultValue: T) => {
  const persistence = usePersistenceStore()
  const state = reactive(persistence.load(key, defaultValue))

  const updateField = <K extends keyof T>(field: K, value: T[K]) => {
    state[field] = value
    persistence.save(key, state)
  }

  const updateState = (newState: Partial<T>) => {
    Object.assign(state, newState)
    persistence.save(key, state)
  }

  return {
    state,
    updateField,
    updateState
  }
}
