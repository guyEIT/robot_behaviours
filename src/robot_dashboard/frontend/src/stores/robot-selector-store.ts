import { create } from "zustand";

interface RobotSelectorState {
  /** Currently active robot (e.g. "meca500" or "franka"). Null = show all / auto. */
  selectedRobotId: string | null;
  /** Robot IDs derived from registered skills. */
  availableRobots: string[];
  setSelectedRobotId: (id: string | null) => void;
  setAvailableRobots: (robots: string[]) => void;
}

export const useRobotSelectorStore = create<RobotSelectorState>((set) => ({
  selectedRobotId: null,
  availableRobots: [],
  setSelectedRobotId: (selectedRobotId) => set({ selectedRobotId }),
  setAvailableRobots: (availableRobots) =>
    set((state) => ({
      availableRobots,
      // If the currently selected robot is no longer in the list, reset to null
      selectedRobotId:
        state.selectedRobotId && availableRobots.includes(state.selectedRobotId)
          ? state.selectedRobotId
          : availableRobots.length > 0
          ? availableRobots[0]
          : null,
    })),
}));
