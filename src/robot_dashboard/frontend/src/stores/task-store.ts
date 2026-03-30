import { create } from "zustand";
import type { TaskState } from "../types/ros";

interface TaskStoreState {
  /** Most recent TaskState from /skill_server/task_state */
  taskState: TaskState | null;
  /** BT XML for the currently running task */
  activeBtXml: string | null;
  /** History of recent task IDs for the selector */
  taskHistory: Array<{ id: string; name: string; status: string }>;

  setTaskState: (state: TaskState) => void;
  setActiveBtXml: (xml: string | null) => void;
}

export const useTaskStore = create<TaskStoreState>((set, get) => ({
  taskState: null,
  activeBtXml: null,
  taskHistory: [],

  setTaskState: (ts) => {
    const history = get().taskHistory;
    const existing = history.findIndex((h) => h.id === ts.task_id);
    const entry = { id: ts.task_id, name: ts.task_name, status: ts.status };

    const newHistory =
      existing >= 0
        ? history.map((h, i) => (i === existing ? entry : h))
        : [entry, ...history].slice(0, 20);

    set({ taskState: ts, taskHistory: newHistory });
  },

  setActiveBtXml: (xml) => set({ activeBtXml: xml }),
}));
