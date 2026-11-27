import { useSyncExternalStore } from 'react';

/** A tiny observable value for state that React components render (low-rate data only). */
export class Store<T> {
  private listeners = new Set<() => void>();

  constructor(private value: T) {}

  get = (): T => this.value;

  set(next: T | ((prev: T) => T)): void {
    const value = typeof next === 'function' ? (next as (prev: T) => T)(this.value) : next;
    if (Object.is(value, this.value)) return;
    this.value = value;
    this.listeners.forEach((l) => l());
  }

  update(patch: Partial<T>): void {
    this.set((prev) => ({ ...prev, ...patch }));
  }

  subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };
}

export function useStore<T>(store: Store<T>): T {
  return useSyncExternalStore(store.subscribe, store.get, store.get);
}
