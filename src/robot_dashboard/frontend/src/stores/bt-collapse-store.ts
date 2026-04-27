import { create } from "zustand";
import { persist } from "zustand/middleware";

export type BtLayoutDirection = "TB" | "LR";

interface BtCollapseState {
  /**
   * Set of container *names* that are currently expanded. Default-empty
   * means every container starts collapsed — Sub-Trees and loop frames
   * render as a single header pill with their iteration badge until the
   * user clicks to expand.
   *
   * Keyed by name (not id) so user expansions survive XML re-parse.
   */
  expandedNames: Record<string, true>;

  /**
   * Names that were opened by the runner (via `autoExpand`), not by a
   * user click. Tracked separately so `autoCloseOnCompletion` can
   * collapse only those — user-opened containers stay open even after
   * the runner finishes them.
   */
  autoOpenedNames: Record<string, true>;

  /**
   * When true, every container is rendered expanded regardless of
   * `expandedNames`. Acts as a global override — useful when the
   * operator wants to see the full structure at once. Persistent across
   * reloads.
   */
  autoExpandAll: boolean;

  /**
   * When true, the BT graph automatically adds every ancestor of the
   * currently-running node to `expandedNames` so live execution is
   * always visible inside its enclosing loop / SubTree frames.
   * Persistent across reloads. Default true.
   */
  autoExpandRunning: boolean;

  /**
   * When true, an auto-opened container collapses again as soon as it
   * transitions to a terminal state (success/failure). Lets long
   * campaigns "ripple" through SubTrees one at a time without leaving
   * a trail of expanded frames behind. Persistent across reloads.
   * Default true.
   */
  autoCloseOnCompletion: boolean;

  /**
   * Direction of the dagre layout. "TB" (top-to-bottom) stacks
   * siblings horizontally — compact for small trees but cramped when
   * nodes have long names. "LR" (left-to-right) stacks siblings
   * vertically — better for long names and deep BT graphs. Default
   * "LR".
   */
  direction: BtLayoutDirection;

  /**
   * When true, the camera only pans to the active node when that node
   * is *outside* the current viewport, with a longer ease. Stops the
   * view from teleporting on every state tick during fast execution.
   * Default true.
   */
  smartFollow: boolean;

  /**
   * One-shot focus request. Set to a container name when the user
   * expands one — the BT graph reads this, fits the camera to the
   * container's new bounds, and clears the field. Not persisted (no
   * value to keep across reloads). Auto-expansion by the runner does
   * NOT set this — the camera-follow logic handles those.
   */
  focusName: string | null;

  isExpanded: (name: string) => boolean;
  /** User-driven toggle. Always clears `autoOpenedNames` flag. */
  toggle: (name: string) => void;
  /** User-driven open. Always clears `autoOpenedNames` flag. */
  expand: (name: string) => void;
  /** User-driven close. Always clears `autoOpenedNames` flag. */
  collapse: (name: string) => void;
  /** Runner-driven open — marks the container as auto-opened. */
  autoExpand: (name: string) => void;
  /** Runner-driven close — only acts if the container is auto-opened. */
  autoCloseIfTracked: (name: string) => void;
  expandAll: (names: readonly string[]) => void;
  collapseAll: () => void;
  setAutoExpandAll: (v: boolean) => void;
  setAutoExpandRunning: (v: boolean) => void;
  setAutoCloseOnCompletion: (v: boolean) => void;
  setDirection: (d: BtLayoutDirection) => void;
  setSmartFollow: (v: boolean) => void;
  clearFocus: () => void;
}

function withoutKey<T>(obj: Record<string, T>, key: string): Record<string, T> {
  if (!(key in obj)) return obj;
  const next = { ...obj };
  delete next[key];
  return next;
}

export const useBtCollapseStore = create<BtCollapseState>()(
  persist(
    (set, get) => ({
      expandedNames: {},
      autoOpenedNames: {},
      autoExpandAll: false,
      autoExpandRunning: true,
      autoCloseOnCompletion: true,
      direction: "LR",
      smartFollow: true,
      focusName: null,

      isExpanded: (name) => Boolean(get().expandedNames[name]),

      toggle: (name) =>
        set((s) => {
          const isOpen = Boolean(s.expandedNames[name]);
          // When opening (not closing), request a camera focus so the
          // newly-revealed children are guaranteed to be in the
          // viewport. Closing doesn't need a focus request.
          return {
            expandedNames: isOpen
              ? withoutKey(s.expandedNames, name)
              : { ...s.expandedNames, [name]: true },
            autoOpenedNames: withoutKey(s.autoOpenedNames, name),
            focusName: isOpen ? s.focusName : name,
          };
        }),

      expand: (name) =>
        set((s) => ({
          expandedNames: s.expandedNames[name]
            ? s.expandedNames
            : { ...s.expandedNames, [name]: true },
          autoOpenedNames: withoutKey(s.autoOpenedNames, name),
          focusName: name,
        })),

      collapse: (name) =>
        set((s) => ({
          expandedNames: withoutKey(s.expandedNames, name),
          autoOpenedNames: withoutKey(s.autoOpenedNames, name),
        })),

      autoExpand: (name) =>
        set((s) => {
          // Don't promote a user-opened container to auto-opened — once
          // the user clicked it open, we leave it under user control.
          const wasOpen = Boolean(s.expandedNames[name]);
          const wasAuto = Boolean(s.autoOpenedNames[name]);
          if (wasOpen && !wasAuto) return s;
          return {
            expandedNames: wasOpen
              ? s.expandedNames
              : { ...s.expandedNames, [name]: true },
            autoOpenedNames: { ...s.autoOpenedNames, [name]: true },
          };
        }),

      autoCloseIfTracked: (name) =>
        set((s) => {
          if (!s.autoOpenedNames[name]) return s;
          return {
            expandedNames: withoutKey(s.expandedNames, name),
            autoOpenedNames: withoutKey(s.autoOpenedNames, name),
          };
        }),

      expandAll: (names) =>
        set(() => {
          const next: Record<string, true> = {};
          for (const n of names) next[n] = true;
          return { expandedNames: next, autoOpenedNames: {} };
        }),

      collapseAll: () => set({ expandedNames: {}, autoOpenedNames: {} }),

      setAutoExpandAll: (v) => set({ autoExpandAll: v }),
      setAutoExpandRunning: (v) => set({ autoExpandRunning: v }),
      setAutoCloseOnCompletion: (v) => set({ autoCloseOnCompletion: v }),
      setDirection: (d) => set({ direction: d }),
      setSmartFollow: (v) => set({ smartFollow: v }),
      clearFocus: () => set({ focusName: null }),
    }),
    {
      name: "bt-collapse-store",
      version: 4,
      // `focusName` is a transient one-shot, don't persist it.
      partialize: (s) => ({
        expandedNames: s.expandedNames,
        autoOpenedNames: s.autoOpenedNames,
        autoExpandAll: s.autoExpandAll,
        autoExpandRunning: s.autoExpandRunning,
        autoCloseOnCompletion: s.autoCloseOnCompletion,
        direction: s.direction,
        smartFollow: s.smartFollow,
      }),
    },
  ),
);
