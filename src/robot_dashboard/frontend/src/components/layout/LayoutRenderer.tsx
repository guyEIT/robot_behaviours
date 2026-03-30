import React, { useState } from "react";
import { Panel, Group as PanelGroup, Separator as PanelResizeHandle } from "react-resizable-panels";
import type { LayoutNode, PanelId } from "../../stores/layout-types";
import { useLayoutStore, collectPanelIds } from "../../stores/layout-store";
import { PANELS, PANEL_MAP } from "./PanelRegistry";
import { PanelSlot } from "./PanelHost";
import { X, Columns2, Rows2 } from "lucide-react";
import clsx from "clsx";

function SplitMenu({
  panelId,
  direction,
  onClose,
}: {
  panelId: PanelId;
  direction: "horizontal" | "vertical";
  onClose: () => void;
}) {
  const layout = useLayoutStore((s) => s.layout);
  const splitPanel = useLayoutStore((s) => s.splitPanel);
  const activeIds = collectPanelIds(layout);

  // Only show panels not already in the layout
  const available = PANELS.filter((p) => !activeIds.has(p.id));

  if (available.length === 0) {
    return (
      <div className="absolute top-full left-0 mt-1 w-40 bg-gray-900 border border-gray-700 rounded-lg shadow-xl z-50 p-2">
        <p className="text-[10px] text-gray-500 text-center py-1">All panels active</p>
      </div>
    );
  }

  return (
    <div className="absolute top-full left-0 mt-1 w-40 bg-gray-900 border border-gray-700 rounded-lg shadow-xl z-50 p-1">
      {available.map((p) => (
        <button
          key={p.id}
          onClick={() => {
            splitPanel(panelId, p.id, direction);
            onClose();
          }}
          className="w-full flex items-center gap-2 px-2 py-1 rounded text-[10px] text-gray-400 hover:text-gray-200 hover:bg-gray-800"
        >
          {p.icon}
          {p.label}
        </button>
      ))}
    </div>
  );
}

function PanelTitleBar({ panelId }: { panelId: PanelId }) {
  const removePanel = useLayoutStore((s) => s.removePanel);
  const [splitDir, setSplitDir] = useState<"horizontal" | "vertical" | null>(null);
  const info = PANEL_MAP[panelId];

  return (
    <div className="h-7 flex items-center gap-1.5 px-2 bg-gray-900/80 border-b border-gray-800 shrink-0 relative">
      <span className="text-gray-400">{info?.icon}</span>
      <span className="text-[10px] font-semibold text-gray-300 uppercase tracking-wide">
        {info?.label ?? panelId}
      </span>
      <div className="ml-auto flex items-center gap-0.5">
        <button
          onClick={() => setSplitDir(splitDir === "horizontal" ? null : "horizontal")}
          className={clsx(
            "p-0.5 rounded",
            splitDir === "horizontal" ? "text-blue-400 bg-blue-500/10" : "text-gray-600 hover:text-gray-300"
          )}
          title="Split left/right"
        >
          <Columns2 className="w-3 h-3" />
        </button>
        <button
          onClick={() => setSplitDir(splitDir === "vertical" ? null : "vertical")}
          className={clsx(
            "p-0.5 rounded",
            splitDir === "vertical" ? "text-blue-400 bg-blue-500/10" : "text-gray-600 hover:text-gray-300"
          )}
          title="Split top/bottom"
        >
          <Rows2 className="w-3 h-3" />
        </button>
        <button
          onClick={() => removePanel(panelId)}
          className="p-0.5 text-gray-600 hover:text-gray-300 rounded"
          title="Close panel"
        >
          <X className="w-3 h-3" />
        </button>
      </div>
      {splitDir && (
        <SplitMenu
          panelId={panelId}
          direction={splitDir}
          onClose={() => setSplitDir(null)}
        />
      )}
    </div>
  );
}

export default function LayoutRenderer({
  node,
  path = [],
}: {
  node: LayoutNode;
  path?: number[];
}) {
  if (node.type === "leaf") {
    return (
      <div className="flex flex-col h-full">
        <PanelTitleBar panelId={node.panelId} />
        <div className="flex-1 overflow-hidden">
          <PanelSlot panelId={node.panelId} />
        </div>
      </div>
    );
  }

  return (
    <PanelGroup orientation={node.direction}>
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
