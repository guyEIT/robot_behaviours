import { create } from "zustand";
import { persist } from "zustand/middleware";
import type { LayoutNode, PanelId, SavedLayout } from "./layout-types";
import { PRESET_LAYOUTS } from "./layout-presets";

interface LayoutStoreState {
  layout: LayoutNode;
  activePresetName: string | null;
  savedLayouts: SavedLayout[];

  setLayout: (layout: LayoutNode) => void;
  applyPreset: (name: string) => void;
  saveCurrentLayout: (name: string) => void;
  deleteSavedLayout: (name: string) => void;
  loadSavedLayout: (name: string) => void;
  addPanel: (panelId: PanelId) => void;
  removePanel: (panelId: PanelId) => void;
}

function collectPanelIds(node: LayoutNode): Set<PanelId> {
  if (node.type === "leaf") return new Set([node.panelId]);
  const ids = new Set<PanelId>();
  for (const child of node.children) {
    for (const id of collectPanelIds(child)) ids.add(id);
  }
  return ids;
}

function removeFromTree(node: LayoutNode, panelId: PanelId): LayoutNode | null {
  if (node.type === "leaf") return node.panelId === panelId ? null : node;
  const remaining = node.children
    .map((c) => removeFromTree(c, panelId))
    .filter((c): c is LayoutNode => c !== null);
  if (remaining.length === 0) return null;
  if (remaining.length === 1) return remaining[0];
  const total = remaining.reduce((s, c) => s + c.size, 0);
  remaining.forEach((c) => (c.size = (c.size / total) * 100));
  return { ...node, children: remaining };
}

/** Validate a persisted layout — remove leaves referencing unknown panels */
function validateLayout(node: LayoutNode, validIds: Set<string>): LayoutNode | null {
  if (node.type === "leaf") return validIds.has(node.panelId) ? node : null;
  const children = node.children
    .map((c) => validateLayout(c, validIds))
    .filter((c): c is LayoutNode => c !== null);
  if (children.length === 0) return null;
  if (children.length === 1) return children[0];
  return { ...node, children };
}

const VALID_PANEL_IDS = new Set<string>([
  "tree", "executor", "monitor", "skills", "joints",
  "tf", "topics", "plotter", "services", "logs",
]);

export const useLayoutStore = create<LayoutStoreState>()(
  persist(
    (set, get) => ({
      layout: PRESET_LAYOUTS.Development,
      activePresetName: "Development",
      savedLayouts: [],

      setLayout: (layout) => set({ layout, activePresetName: null }),

      applyPreset: (name) => {
        const preset = PRESET_LAYOUTS[name];
        if (preset) set({ layout: structuredClone(preset), activePresetName: name });
      },

      saveCurrentLayout: (name) => {
        const saved: SavedLayout = {
          name,
          tree: structuredClone(get().layout),
          createdAt: Date.now(),
        };
        set((s) => ({
          savedLayouts: [
            ...s.savedLayouts.filter((l) => l.name !== name),
            saved,
          ],
        }));
      },

      deleteSavedLayout: (name) =>
        set((s) => ({
          savedLayouts: s.savedLayouts.filter((l) => l.name !== name),
        })),

      loadSavedLayout: (name) => {
        const saved = get().savedLayouts.find((l) => l.name === name);
        if (saved) set({ layout: structuredClone(saved.tree), activePresetName: null });
      },

      addPanel: (panelId) => {
        const layout = structuredClone(get().layout);
        if (layout.type === "split") {
          layout.children.push({ type: "leaf", panelId, size: 100 / (layout.children.length + 1) });
          const each = 100 / layout.children.length;
          layout.children.forEach((c) => (c.size = each));
        } else {
          set({
            layout: {
              type: "split",
              direction: "horizontal",
              size: 100,
              children: [layout, { type: "leaf", panelId, size: 50 }],
            },
            activePresetName: null,
          });
          return;
        }
        set({ layout, activePresetName: null });
      },

      removePanel: (panelId) => {
        const result = removeFromTree(structuredClone(get().layout), panelId);
        if (result) set({ layout: result, activePresetName: null });
      },
    }),
    {
      name: "robot-dashboard-layout",
      partialize: (state) => ({
        layout: state.layout,
        activePresetName: state.activePresetName,
        savedLayouts: state.savedLayouts,
      }),
      merge: (persisted: any, current) => {
        if (persisted?.layout) {
          const validated = validateLayout(persisted.layout, VALID_PANEL_IDS);
          if (validated) {
            return { ...current, ...persisted, layout: validated };
          }
        }
        return current;
      },
    }
  )
);

export { collectPanelIds };
