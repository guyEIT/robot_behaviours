import { create } from "zustand";
import type { RosLog, DiagnosticStatus } from "../types/ros";

const MAX_LOGS = 1000;

interface LogStoreState {
  logs: RosLog[];
  diagnostics: DiagnosticStatus[];
  addLog: (log: RosLog) => void;
  setDiagnostics: (diags: DiagnosticStatus[]) => void;
  clearLogs: () => void;
}

export const useLogStore = create<LogStoreState>((set, get) => ({
  logs: [],
  diagnostics: [],

  addLog: (log) => {
    const logs = get().logs;
    const next = logs.length >= MAX_LOGS ? logs.slice(1) : logs;
    set({ logs: [...next, log] });
  },

  setDiagnostics: (diagnostics) => set({ diagnostics }),

  clearLogs: () => set({ logs: [] }),
}));
