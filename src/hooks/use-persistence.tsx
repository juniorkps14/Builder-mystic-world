import { useEffect, useState, useCallback, useRef } from "react";
import { usePersistence as useBasePersistence } from "../contexts/PersistenceContext";

interface UsePersistentStateOptions {
  key: string;
  defaultValue: any;
  ttl?: number;
  autoSave?: boolean;
  autoSaveDelay?: number;
}

export function usePersistentState<T>({
  key,
  defaultValue,
  ttl,
  autoSave = true,
  autoSaveDelay = 1000,
}: UsePersistentStateOptions) {
  const { save, load, subscribe } = useBasePersistence();
  const [state, setState] = useState<T>(() => load(key, defaultValue));
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const timeoutRef = useRef<NodeJS.Timeout>();
  const isInitialMount = useRef(true);

  // Load initial state
  useEffect(() => {
    const initialValue = load(key, defaultValue);
    setState(initialValue);
    setIsLoading(false);
  }, [key, defaultValue, load]);

  // Auto-save when state changes
  useEffect(() => {
    if (isInitialMount.current) {
      isInitialMount.current = false;
      return;
    }

    if (autoSave && !isLoading) {
      setIsSaving(true);

      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }

      timeoutRef.current = setTimeout(() => {
        save(key, state, ttl);
        setIsSaving(false);
      }, autoSaveDelay);
    }

    return () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, [state, key, ttl, autoSave, autoSaveDelay, isLoading, save]);

  // Subscribe to external changes
  useEffect(() => {
    const unsubscribe = subscribe(key, (newValue) => {
      if (newValue !== null && newValue !== state) {
        setState(newValue);
      }
    });

    return unsubscribe;
  }, [key, subscribe, state]);

  // Manual save function
  const saveNow = useCallback(() => {
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }
    setIsSaving(true);
    save(key, state, ttl);
    setIsSaving(false);
  }, [key, state, ttl, save]);

  // Update state function
  const updateState = useCallback((newState: T | ((prevState: T) => T)) => {
    setState((prevState) => {
      const nextState =
        typeof newState === "function"
          ? (newState as (prevState: T) => T)(prevState)
          : newState;
      return nextState;
    });
  }, []);

  return {
    state,
    setState: updateState,
    saveNow,
    isLoading,
    isSaving,
  };
}

// Hook for managing multiple persistent values
export function usePersistentStore<T extends Record<string, any>>(
  storeKey: string,
  initialState: T,
  options?: {
    ttl?: number;
    autoSave?: boolean;
    autoSaveDelay?: number;
  },
) {
  const {
    state: store,
    setState: setStore,
    saveNow,
    isLoading,
    isSaving,
  } = usePersistentState({
    key: storeKey,
    defaultValue: initialState,
    ...options,
  });

  const updateField = useCallback(
    <K extends keyof T>(key: K, value: T[K]) => {
      setStore((prevStore) => ({
        ...prevStore,
        [key]: value,
      }));
    },
    [setStore],
  );

  const updateFields = useCallback(
    (updates: Partial<T>) => {
      setStore((prevStore) => ({
        ...prevStore,
        ...updates,
      }));
    },
    [setStore],
  );

  const resetStore = useCallback(() => {
    setStore(initialState);
  }, [setStore, initialState]);

  return {
    store,
    updateField,
    updateFields,
    resetStore,
    setStore,
    saveNow,
    isLoading,
    isSaving,
  };
}

// Hook for persistent arrays (like command history)
export function usePersistentArray<T>(
  key: string,
  maxItems?: number,
  options?: {
    ttl?: number;
    autoSave?: boolean;
    autoSaveDelay?: number;
  },
) {
  const {
    state: items,
    setState: setItems,
    saveNow,
    isLoading,
    isSaving,
  } = usePersistentState({
    key,
    defaultValue: [] as T[],
    ...options,
  });

  const addItem = useCallback(
    (item: T) => {
      setItems((prevItems) => {
        const newItems = [item, ...prevItems];
        return maxItems ? newItems.slice(0, maxItems) : newItems;
      });
    },
    [setItems, maxItems],
  );

  const removeItem = useCallback(
    (index: number) => {
      setItems((prevItems) => prevItems.filter((_, i) => i !== index));
    },
    [setItems],
  );

  const clearItems = useCallback(() => {
    setItems([]);
  }, [setItems]);

  const updateItem = useCallback(
    (index: number, item: T) => {
      setItems((prevItems) =>
        prevItems.map((prevItem, i) => (i === index ? item : prevItem)),
      );
    },
    [setItems],
  );

  return {
    items,
    addItem,
    removeItem,
    clearItems,
    updateItem,
    setItems,
    saveNow,
    isLoading,
    isSaving,
  };
}
