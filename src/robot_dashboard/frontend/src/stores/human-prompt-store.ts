import { create } from "zustand";
import type { HumanPrompt } from "../types/ros";

interface HumanPromptStoreState {
  /** Blocking prompts awaiting operator response (confirm/input/task) */
  activePrompts: HumanPrompt[];
  /** Recent notifications/warnings for history display */
  recentNotifications: HumanPrompt[];

  addPrompt: (prompt: HumanPrompt) => void;
  removePrompt: (promptId: string) => void;
  clearNotifications: () => void;
}

export const useHumanPromptStore = create<HumanPromptStoreState>((set, get) => ({
  activePrompts: [],
  recentNotifications: [],

  addPrompt: (prompt) => {
    if (prompt.prompt_type === "dismiss") {
      // Remove the matching active prompt
      set((s) => ({
        activePrompts: s.activePrompts.filter((p) => p.prompt_id !== prompt.prompt_id),
      }));
      return;
    }

    if (prompt.prompt_type === "notification" || prompt.prompt_type === "warning") {
      set((s) => ({
        recentNotifications: [prompt, ...s.recentNotifications].slice(0, 50),
      }));
    } else {
      // confirm, input, task — add to active
      set((s) => ({
        activePrompts: [...s.activePrompts, prompt],
      }));
    }
  },

  removePrompt: (promptId) => {
    set((s) => ({
      activePrompts: s.activePrompts.filter((p) => p.prompt_id !== promptId),
    }));
  },

  clearNotifications: () => set({ recentNotifications: [] }),
}));
