import { memo } from "react";
import { Handle, Position, type NodeProps } from "@xyflow/react";
import {
  Repeat as RepeatIcon,
  RotateCcw,
  Infinity as InfinityIcon,
  Layers,
  ChevronDown,
  ChevronRight,
} from "lucide-react";
import clsx from "clsx";
import type { BtGraphNode, BtNodeState, BtContainerKind } from "../../types/bt";
import { useBtCollapseStore } from "../../stores/bt-collapse-store";

const STATE_PILL: Record<BtNodeState, string> = {
  idle: "bg-paper text-muted border-hair",
  running: "bg-running-soft text-running border-running",
  success: "bg-ok-soft text-ok border-ok",
  failure: "bg-err-soft text-err border-err",
};

const KIND_LABEL: Record<BtContainerKind, string> = {
  repeat: "REPEAT",
  retry: "RETRY",
  keep_until_fail: "FOREVER",
  subtree: "SUBTREE",
};

function kindIcon(kind: BtContainerKind) {
  const cls = "w-3.5 h-3.5";
  switch (kind) {
    case "repeat":
      return <RepeatIcon className={cls} />;
    case "retry":
      return <RotateCcw className={cls} />;
    case "keep_until_fail":
      return <InfinityIcon className={cls} />;
    case "subtree":
      return <Layers className={cls} />;
  }
}

function BtContainerNodeComponent({ data }: NodeProps & { data: BtGraphNode }) {
  const { state, name, btNodeType, displayParams, containerKind } = data;
  const kind = containerKind ?? "subtree";

  const isExpanded = useBtCollapseStore((s) => Boolean(s.expandedNames[name]));
  const toggle = useBtCollapseStore((s) => s.toggle);

  const onToggle = (e: React.MouseEvent) => {
    e.stopPropagation();
    toggle(name);
  };

  return (
    <div
      className={clsx(
        "relative h-full w-full border bg-cream-deep/40",
        "border-hair",
        kind === "subtree" && "border-[1.5px] border-double",
      )}
      style={{
        boxShadow:
          kind === "subtree"
            ? "inset 0 0 0 1px rgba(215,215,215,0.6)"
            : "inset 0 0 0 1px rgba(215,215,215,0.4)",
      }}
    >
      {/* Header bar — same look whether collapsed or expanded; clicking
          the chevron toggles. */}
      <div
        className={clsx(
          "absolute top-0 left-0 right-0 h-[28px] flex items-center gap-1.5 px-2 border-b bg-paper",
          isExpanded ? "border-hair" : "border-transparent",
        )}
      >
        <button
          type="button"
          onClick={onToggle}
          className="text-muted shrink-0 hover:text-ink-soft cursor-pointer"
          title={isExpanded ? "Collapse" : "Expand"}
        >
          {isExpanded ? (
            <ChevronDown className="w-3.5 h-3.5" />
          ) : (
            <ChevronRight className="w-3.5 h-3.5" />
          )}
        </button>
        <span className="text-muted shrink-0">{kindIcon(kind)}</span>
        <span className="font-mono text-[10px] font-semibold uppercase tracking-[0.08em] text-muted">
          {KIND_LABEL[kind]}
        </span>
        <span
          className="text-[11.5px] text-ink truncate font-medium"
          title={`${btNodeType} • ${name}`}
        >
          {name}
        </span>
        {displayParams && displayParams.length > 0 && (
          <span className="ml-1 flex items-center gap-1">
            {displayParams.map((p, i) => (
              <span
                key={i}
                className="font-mono text-[10px] px-1.5 py-[1px] border border-hair bg-cream text-ink-soft"
                title={p.title ?? p.label}
              >
                {p.label}
              </span>
            ))}
          </span>
        )}
        <span
          className={clsx(
            "ml-auto font-mono text-[9px] uppercase tracking-[0.08em] px-1.5 py-[1px] border",
            STATE_PILL[state],
          )}
          title={`State: ${state}`}
        >
          {state}
        </span>
      </div>

      {/* Children render here via ReactFlow group node mechanics when expanded. */}

      <Handle
        type="target"
        position={data.layoutDirection === "LR" ? Position.Left : Position.Top}
        className="!bg-paper !border !border-hair !w-2 !h-2"
      />
      <Handle
        type="source"
        position={data.layoutDirection === "LR" ? Position.Right : Position.Bottom}
        className="!bg-paper !border !border-hair !w-2 !h-2"
      />
    </div>
  );
}

export default memo(BtContainerNodeComponent);
