import React, { createContext, useContext, useEffect, useState } from 'react';
import { persistenceService, PersistenceConfig } from '../services/PersistenceService';

interface PersistenceContextType {
  save: <T>(key: string, value: T, ttl?: number) => void;
  load: <T>(key: string, defaultValue: T) => T;
  remove: (key: string) => void;
  subscribe: (key: string, callback: (value: any) => void) => () => void;
  clearAll: (prefix?: string) => void;
  getStorageInfo: () => { used: number; available: number; percentage: number };
  isLoaded: boolean;
}

const PersistenceContext = createContext<PersistenceContextType | undefined>(undefined);

export const usePersistence = () => {
  const context = useContext(PersistenceContext);
  if (!context) {
    throw new Error('usePersistence must be used within a PersistenceProvider');
  }
  return context;
};

interface PersistenceProviderProps {
  children: React.ReactNode;
  appPrefix?: string;
}

export const PersistenceProvider: React.FC<PersistenceProviderProps> = ({ 
  children, 
  appPrefix = 'dino-core' 
}) => {
  const [isLoaded, setIsLoaded] = useState(false);

  useEffect(() => {
    // Initialize persistence system
    const initPersistence = async () => {
      try {
        // Load any initial configuration
        const config = persistenceService.load(`${appPrefix}-config`, {
          version: '1.0.0',
          initialized: false
        });

        if (!config.initialized) {
          // First time setup
          persistenceService.save(`${appPrefix}-config`, {
            ...config,
            initialized: true,
            initTimestamp: Date.now()
          });
        }

        setIsLoaded(true);
      } catch (error) {
        console.error('Failed to initialize persistence:', error);
        setIsLoaded(true);
      }
    };

    initPersistence();
  }, [appPrefix]);

  const contextValue: PersistenceContextType = {
    save: <T>(key: string, value: T, ttl?: number) => {
      persistenceService.save(`${appPrefix}-${key}`, value, ttl);
    },
    load: <T>(key: string, defaultValue: T): T => {
      return persistenceService.load(`${appPrefix}-${key}`, defaultValue);
    },
    remove: (key: string) => {
      persistenceService.remove(`${appPrefix}-${key}`);
    },
    subscribe: (key: string, callback: (value: any) => void) => {
      return persistenceService.subscribe(`${appPrefix}-${key}`, callback);
    },
    clearAll: (prefix?: string) => {
      const fullPrefix = prefix ? `${appPrefix}-${prefix}` : appPrefix;
      persistenceService.clearAll(fullPrefix);
    },
    getStorageInfo: persistenceService.getStorageInfo,
    isLoaded
  };

  return (
    <PersistenceContext.Provider value={contextValue}>
      {children}
    </PersistenceContext.Provider>
  );
};

export default PersistenceContext;