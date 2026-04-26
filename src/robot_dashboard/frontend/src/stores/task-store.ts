import { create } from "zustand";
import type { TaskState } from "../types/ros";

/**
 * One execution interval of a skill. The Gantt-style timeline reads these
 * directly. Segments are derived from observation of `current_skill` /
 * `completed_skills` / `failed_skills` transitions in successive TaskState
 * messages — the ROS messages don't ship intervals natively.
 *
 * `endedAt: null` means the segment is still running. Drawing code uses
 * `now` for the right-edge in that case.
 */
export interface SkillSegment {
  /** Stable id within a session (timestamp + name) for React keys */
  id: string;
  taskId: string;
  taskName: string;
  name: string;
  startedAt: number;       // ms epoch
  endedAt: number | null;  // ms epoch, null = still running
  status: "running" | "done" | "failed";
}

const SEGMENT_RETENTION_MS = 10 * 60 * 1000;  // 10 min — older drops off entirely

interface TaskStoreState {
  /** Most recent TaskState from /skill_server/task_state */
  taskState: TaskState | null;
  /** BT XML for the currently running task */
  activeBtXml: string | null;
  /** History of recent task IDs for the selector */
  taskHistory: Array<{ id: string; name: string; status: string }>;
  /** Per-skill execution intervals derived from TaskState transitions */
  skillSegments: SkillSegment[];

  setTaskState: (state: TaskState) => void;
  setActiveBtXml: (xml: string | null) => void;
  clearSkillSegments: () => void;
}

export const useTaskStore = create<TaskStoreState>((set, get) => ({
  taskState: null,
  activeBtXml: null,
  taskHistory: [],
  skillSegments: [],

  setTaskState: (ts) => {
    const prev = get().taskState;

    // Don't overwrite a terminal state with the same task's stale messages
    if (
      prev &&
      prev.task_id === ts.task_id &&
      (prev.status === "SUCCESS" || prev.status === "FAILURE" || prev.status === "CANCELLED") &&
      ts.status === "RUNNING"
    ) {
      return;
    }

    // Skip update if nothing meaningful changed (avoids 10Hz re-renders)
    if (
      prev &&
      prev.task_id === ts.task_id &&
      prev.status === ts.status &&
      prev.current_skill === ts.current_skill &&
      prev.current_bt_node === ts.current_bt_node &&
      prev.completed_skills.length === ts.completed_skills.length &&
      prev.failed_skills.length === ts.failed_skills.length &&
      Math.round(prev.progress * 100) === Math.round(ts.progress * 100)
    ) {
      return;
    }

    const history = get().taskHistory;
    const existing = history.findIndex((h) => h.id === ts.task_id);
    const entry = { id: ts.task_id, name: ts.task_name, status: ts.status };

    const newHistory =
      existing >= 0
        ? history.map((h, i) => (i === existing ? entry : h))
        : [entry, ...history].slice(0, 20);

    // ── Derive skill segments ────────────────────────────────────────────
    // Tracks which named skills are currently active and emits/closes
    // intervals as `current_skill` transitions. Future-proofed for parallel
    // skills by treating `current_skill` as a (currently 1-element) set.
    const now = Date.now();
    const prevSegments = get().skillSegments;
    const completed = new Set(ts.completed_skills);
    const failed = new Set(ts.failed_skills);
    const activeNow = ts.current_skill ? new Set([ts.current_skill]) : new Set<string>();

    // Open segments grouped by name (most recent first)
    const openByName = new Map<string, SkillSegment>();
    for (const s of prevSegments) {
      if (s.endedAt === null && s.taskId === ts.task_id) {
        openByName.set(s.name, s);
      }
    }

    let segments = [...prevSegments];

    // Close segments whose name is no longer active
    for (const [name, seg] of openByName) {
      if (!activeNow.has(name)) {
        const status = failed.has(name) ? "failed" : "done";
        segments = segments.map((s) =>
          s.id === seg.id ? { ...s, endedAt: now, status } : s,
        );
      }
    }

    // Open new segments for names that just became active
    for (const name of activeNow) {
      if (!openByName.has(name)) {
        segments.push({
          id: `${ts.task_id}:${name}:${now}`,
          taskId: ts.task_id,
          taskName: ts.task_name,
          name,
          startedAt: now,
          endedAt: null,
          status: "running",
        });
      }
    }

    // If the task itself reached a terminal state, close any still-open segments
    // attributed to it (final tick may not include them in current_skill).
    const taskTerminal =
      ts.status === "SUCCESS" || ts.status === "FAILURE" || ts.status === "CANCELLED";
    if (taskTerminal) {
      segments = segments.map((s) => {
        if (s.endedAt !== null || s.taskId !== ts.task_id) return s;
        const status = failed.has(s.name) ? "failed" : completed.has(s.name) ? "done" : "done";
        return { ...s, endedAt: now, status };
      });
    }

    // Promote previously-closed segments that retroactively show up in failed_skills
    // (e.g. a node that succeeded transiently then a parent reported failure).
    if (failed.size > 0) {
      segments = segments.map((s) => {
        if (s.endedAt !== null && s.taskId === ts.task_id && failed.has(s.name) && s.status !== "failed") {
          return { ...s, status: "failed" as const };
        }
        return s;
      });
    }

    // Drop very old segments
    const cutoff = now - SEGMENT_RETENTION_MS;
    segments = segments.filter((s) => (s.endedAt ?? now) >= cutoff);

    set({ taskState: ts, taskHistory: newHistory, skillSegments: segments });
  },

  setActiveBtXml: (xml) => set({ activeBtXml: xml }),
  clearSkillSegments: () => set({ skillSegments: [] }),
}));
