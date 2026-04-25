import React, { useState } from "react";
import { Panel, Group as PanelGroup, Separator as PanelResizeHandle } from "react-resizable-panels";
import type { LayoutNode, PanelId } from "../../stores/layout-types";
import { useLayoutStore, collectPanelIds } from "../../stores/layout-store";
import { PANELS, PANEL_MAP } from "./PanelRegistry";
import { PanelSlot } from "./PanelHost";
import { X, Columns2, Rows2 } from "lucide-react";
import clsx from "clsx";
import { IconBtn } from "../ui";

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

  const available = PANELS.filter((p) => !activeIds.has(p.id));

  if (available.length === 0) {
    return (
      <div className="absolute top-full left-0 mt-1 w-44 bg-paper border border-hair rounded-none z-50 p-2">
        <p className="text-[10px] text-muted text-center py-1 font-mono uppercase tracking-[0.1em]">
          All panels active
        </p>
      </div>
    );
  }

  return (
    <div className="absolute top-full left-0 mt-1 w-48 bg-paper border border-hair rounded-none z-50 p-1">
      {available.map((p) => (
        <button
          key={p.id}
          onClick={() => {
            splitPanel(panelId, p.id, direction);
            onClose();
          }}
          className="w-full flex items-center gap-2 px-2 py-1.5 text-[12px] text-ink-soft hover:bg-cream transition-colors"
        >
          <span className="text-muted">{p.icon}</span>
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
    <div className="h-8 flex items-center gap-2 px-3 bg-cream-deep border-b border-hair shrink-0 relative">
      <span className="text-muted">{info?.icon}</span>
      <span className="font-mono text-[10px] font-semibold text-ink uppercase tracking-[0.12em]">
        {info?.label ?? panelId}
      </span>
      <div className="ml-auto flex items-center gap-0.5">
        <IconBtn
          onClick={() => setSplitDir(splitDir === "horizontal" ? null : "horizontal")}
          active={splitDir === "horizontal"}
          className="!w-6 !h-6"
          title="Split left/right"
        >
          <Columns2 className="w-3 h-3" />
        </IconBtn>
        <IconBtn
          onClick={() => setSplitDir(splitDir === "vertical" ? null : "vertical")}
          active={splitDir === "vertical"}
          className="!w-6 !h-6"
          title="Split top/bottom"
        >
          <Rows2 className="w-3 h-3" />
        </IconBtn>
        <IconBtn
          onClick={() => removePanel(panelId)}
          className="!w-6 !h-6"
          title="Close panel"
        >
          <X className="w-3 h-3" />
        </IconBtn>
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
      <div className="flex flex-col h-full bg-paper border-l border-hair-soft">
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
              className={clsx(
                "bg-hair-soft hover:bg-terracotta-tint transition-colors",
                node.direction === "horizontal"
                  ? "w-px hover:w-1 cursor-col-resize"
                  : "h-px hover:h-1 cursor-row-resize",
              )}
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
