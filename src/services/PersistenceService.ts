export interface PersistenceConfig {
  key: string;
  defaultValue: any;
  ttl?: number; // Time to live in milliseconds
}

export class PersistenceService {
  private static instance: PersistenceService;
  private listeners: Map<string, Set<(value: any) => void>> = new Map();

  static getInstance(): PersistenceService {
    if (!PersistenceService.instance) {
      PersistenceService.instance = new PersistenceService();
    }
    return PersistenceService.instance;
  }

  save<T>(key: string, value: T, ttl?: number): void {
    try {
      const data = {
        value,
        timestamp: Date.now(),
        ttl: ttl || null
      };
      localStorage.setItem(key, JSON.stringify(data));
      this.notifyListeners(key, value);
    } catch (error) {
      console.warn(`Failed to save ${key} to localStorage:`, error);
    }
  }

  load<T>(key: string, defaultValue: T): T {
    try {
      const item = localStorage.getItem(key);
      if (!item) return defaultValue;

      const data = JSON.parse(item);
      
      // Check if data has expired
      if (data.ttl && Date.now() - data.timestamp > data.ttl) {
        localStorage.removeItem(key);
        return defaultValue;
      }

      return data.value;
    } catch (error) {
      console.warn(`Failed to load ${key} from localStorage:`, error);
      return defaultValue;
    }
  }

  remove(key: string): void {
    try {
      localStorage.removeItem(key);
      this.notifyListeners(key, null);
    } catch (error) {
      console.warn(`Failed to remove ${key} from localStorage:`, error);
    }
  }

  subscribe(key: string, callback: (value: any) => void): () => void {
    if (!this.listeners.has(key)) {
      this.listeners.set(key, new Set());
    }
    this.listeners.get(key)!.add(callback);

    // Return unsubscribe function
    return () => {
      const keyListeners = this.listeners.get(key);
      if (keyListeners) {
        keyListeners.delete(callback);
        if (keyListeners.size === 0) {
          this.listeners.delete(key);
        }
      }
    };
  }

  private notifyListeners(key: string, value: any): void {
    const keyListeners = this.listeners.get(key);
    if (keyListeners) {
      keyListeners.forEach(callback => callback(value));
    }
  }

  // Batch operations
  saveBatch(items: Array<{ key: string; value: any; ttl?: number }>): void {
    items.forEach(({ key, value, ttl }) => {
      this.save(key, value, ttl);
    });
  }

  loadBatch<T>(configs: PersistenceConfig[]): Record<string, T> {
    const result: Record<string, T> = {};
    configs.forEach(({ key, defaultValue }) => {
      result[key] = this.load(key, defaultValue);
    });
    return result;
  }

  // Clear all app data
  clearAll(prefix?: string): void {
    try {
      const keys = Object.keys(localStorage);
      keys.forEach(key => {
        if (!prefix || key.startsWith(prefix)) {
          localStorage.removeItem(key);
        }
      });
    } catch (error) {
      console.warn('Failed to clear localStorage:', error);
    }
  }

  // Get storage usage info
  getStorageInfo(): { used: number; available: number; percentage: number } {
    try {
      let used = 0;
      for (let key in localStorage) {
        if (localStorage.hasOwnProperty(key)) {
          used += localStorage[key].length + key.length;
        }
      }
      
      // Estimate available space (5MB is typical localStorage limit)
      const total = 5 * 1024 * 1024; // 5MB in bytes
      const available = total - used;
      const percentage = (used / total) * 100;

      return { used, available, percentage };
    } catch {
      return { used: 0, available: 0, percentage: 0 };
    }
  }
}

// Singleton instance
export const persistenceService = PersistenceService.getInstance();
