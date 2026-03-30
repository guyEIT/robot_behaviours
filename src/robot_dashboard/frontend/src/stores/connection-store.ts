import { create } from "zustand";
import { getRos, onConnectionChange, isConnected } from "../lib/rosbridge-client";

interface ConnectionState {
  connected: boolean;
  init: () => void;
}

export const useConnectionStore = create<ConnectionState>((set) => ({
  connected: false,
  init: () => {
    getRos(); // ensure connection is started
    set({ connected: isConnected() });
    onConnectionChange((connected) => set({ connected }));
  },
}));
