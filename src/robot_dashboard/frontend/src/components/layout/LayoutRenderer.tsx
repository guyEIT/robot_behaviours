import React from "react";
import { Panel, PanelGroup, PanelResizeHandle } from "react-resizable-panels";
import type { LayoutNode } from "../../stores/layout-types";
import { useLayoutStore } from "../../stores/layout-store";
import { PANEL_MAP } from "./PanelRegistry";
import { PanelSlot } from "./PanelHost";
import { X } from "lucide-react";

/**
 * Recursively renders the layout tree as nested PanelGroup/Panel components.
 * Each leaf renders a PanelSlot which reparents the actual panel DOM node.
 */
export default function LayoutRenderer({
  node,
  path = [],
}: {
  node: LayoutNode;
  path?: number[];
}) {
  const removePanel = useLayoutStore((s) => s.removePanel);

  if (node.type === "leaf") {
    const info = PANEL_MAP[node.panelId];
    return (
      <div className="flex flex-col h-full">
        {/* Panel title bar */}
        <div className="h-7 flex items-center gap-1.5 px-2 bg-gray-900/80 border-b border-gray-800 shrink-0">
          <span className="text-gray-400">{info?.icon}</span>
          <span className="text-[10px] font-semibold text-gray-300 uppercase tracking-wide">
            {info?.label ?? node.panelId}
          </span>
          <button
            onClick={() => removePanel(node.panelId)}
            className="ml-auto p-0.5 text-gray-600 hover:text-gray-300 rounded"
            title="Close panel"
          >
            <X className="w-3 h-3" />
          </button>
        </div>
        <div className="flex-1 overflow-hidden">
          <PanelSlot panelId={node.panelId} />
        </div>
      </div>
    );
  }

  return (
    <PanelGroup direction={node.direction}>
      {node.children.map((child, i) => (
        <React.Fragment key={`${path.join("-")}-${i}`}>
          {i > 0 && (
            <PanelResizeHandle
              className={
                node.direction === "horizontal"
                  ? "w-1 hover:bg-blue-500/50 bg-gray-800 transition-colors cursor-col-resize"
                  : "h-1 hover:bg-blue-500/50 bg-gray-800 transition-colors cursor-row-resize"
              }
            />
          )}
          <Panel defaultSize={child.size} minSize={5}>
            <LayoutRenderer node={child} path={[...path, i]} />
          </Panel>
        </React.Fragment>
      ))}
    </PanelGroup>
  );
}
