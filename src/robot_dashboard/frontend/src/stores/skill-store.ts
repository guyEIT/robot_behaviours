import { create } from "zustand";
import type { SkillDescription } from "../types/ros";

interface SkillStoreState {
  skills: SkillDescription[];
  loading: boolean;
  error: string | null;
  setSkills: (skills: SkillDescription[]) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
}

export const useSkillStore = create<SkillStoreState>((set) => ({
  skills: [],
  loading: false,
  error: null,
  setSkills: (skills) => set({ skills, loading: false, error: null }),
  setLoading: (loading) => set({ loading }),
  setError: (error) => set({ error, loading: false }),
}));
