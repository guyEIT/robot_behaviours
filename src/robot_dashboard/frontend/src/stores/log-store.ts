import { create } from "zustand";
import type { RosLog, DiagnosticStatus, SkillLogEvent } from "../types/ros";

const MAX_LOGS = 1000;
const MAX_SKILL_LOGS = 500;

interface LogStoreState {
  logs: RosLog[];
  /** Structured skill/human log events from /skill_server/log_events */
  skillLogs: SkillLogEvent[];
  diagnostics: DiagnosticStatus[];
  addLog: (log: RosLog) => void;
  addSkillLog: (log: SkillLogEvent) => void;
  setDiagnostics: (diags: DiagnosticStatus[]) => void;
  clearLogs: () => void;
}

export const useLogStore = create<LogStoreState>((set, get) => ({
  logs: [],
  skillLogs: [],
  diagnostics: [],

  addLog: (log) => {
    const logs = get().logs;
    const next = logs.length >= MAX_LOGS ? logs.slice(1) : logs;
    set({ logs: [...next, log] });
  },

  addSkillLog: (log) => {
    const skillLogs = get().skillLogs;
    const next = skillLogs.length >= MAX_SKILL_LOGS ? skillLogs.slice(1) : skillLogs;
    set({ skillLogs: [...next, log] });
  },

  setDiagnostics: (diagnostics) => set({ diagnostics }),

  clearLogs: () => set({ logs: [], skillLogs: [] }),
}));
