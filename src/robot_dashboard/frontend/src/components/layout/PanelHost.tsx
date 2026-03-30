import { createContext, useContext, useEffect, useRef } from "react";
import type { PanelId } from "../../stores/layout-types";
import { PANELS } from "./PanelRegistry";

type PanelRefs = Map<PanelId, HTMLDivElement | null>;

const PanelRefsContext = createContext<{
  panelRefs: React.MutableRefObject<PanelRefs>;
  poolRef: React.RefObject<HTMLDivElement | null>;
} | null>(null);

export function usePanelRefs() {
  const ctx = useContext(PanelRefsContext);
  if (!ctx) throw new Error("usePanelRefs must be used inside PanelHost");
  return ctx;
}

/**
 * Mounts all panels in a hidden offscreen pool. Individual LayoutLeafSlots
 * use DOM reparenting to move panels into visible layout slots.
 */
export default function PanelHost({ children }: { children: React.ReactNode }) {
  const panelRefs = useRef<PanelRefs>(new Map());
  const poolRef = useRef<HTMLDivElement>(null);

  return (
    <PanelRefsContext.Provider value={{ panelRefs, poolRef }}>
      {/* Offscreen panel pool — all panels always mounted */}
      <div ref={poolRef} className="fixed -left-[9999px] top-0 w-screen h-screen overflow-hidden">
        {PANELS.map((panel) => (
          <div
            key={panel.id}
            ref={(el) => { panelRefs.current.set(panel.id, el); }}
            className="w-full h-full"
          >
            {panel.component}
          </div>
        ))}
      </div>
      {/* Layout renderer */}
      {children}
    </PanelRefsContext.Provider>
  );
}

/**
 * A slot that reparents a panel's DOM node from the offscreen pool into
 * the visible layout position via appendChild.
 */
export function PanelSlot({ panelId }: { panelId: PanelId }) {
  const { panelRefs, poolRef } = usePanelRefs();
  const slotRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const panelEl = panelRefs.current.get(panelId);
    const slotEl = slotRef.current;
    if (!panelEl || !slotEl) return;

    slotEl.appendChild(panelEl);

    return () => {
      // Move back to pool on cleanup
      poolRef.current?.appendChild(panelEl);
    };
  }, [panelId, panelRefs, poolRef]);

  return <div ref={slotRef} className="w-full h-full overflow-hidden" />;
}
