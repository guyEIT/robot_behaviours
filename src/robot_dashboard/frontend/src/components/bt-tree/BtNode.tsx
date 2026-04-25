import { memo } from "react";
import { Handle, Position, type NodeProps } from "@xyflow/react";
import {
  GitBranch,
  Play,
  RotateCcw,
  Eye,
  Diamond,
  Workflow,
  Cog,
} from "lucide-react";
import type { BtGraphNode, BtNodeState } from "../../types/bt";
import clsx from "clsx";

const STATE_BORDER: Record<BtNodeState, string> = {
  idle: "border-hair bg-paper",
  running: "border-running border-[1.5px] bg-running-soft",
  success: "border-ok bg-ok-soft",
  failure: "border-err bg-err-soft",
};

const STATE_DOT: Record<BtNodeState, string> = {
  idle: "bg-muted",
  running: "bg-running animate-pulse",
  success: "bg-ok",
  failure: "bg-err",
};

const STATE_TYPE_TEXT: Record<BtNodeState, string> = {
  idle: "text-muted",
  running: "text-running",
  success: "text-ok",
  failure: "text-err",
};

function nodeIcon(btNodeType: string, category: string) {
  const cls = "w-4 h-4 shrink-0";
  switch (category) {
    case "control":
      if (btNodeType === "Fallback" || btNodeType === "ReactiveFallback")
        return <GitBranch className={cls} />;
      return <Workflow className={cls} />;
    case "decorator":
      if (btNodeType.includes("Retry")) return <RotateCcw className={cls} />;
      return <Play className={cls} />;
    case "condition":
      return <Diamond className={cls} />;
    case "subtree":
      return <Eye className={cls} />;
    default:
      return <Cog className={cls} />;
  }
}

function BtNodeComponent({ data }: NodeProps & { data: BtGraphNode }) {
  const { category, btNodeType, name, state, params } = data;

  return (
    <div
      className={clsx(
        "border px-3 py-2 min-w-[160px] max-w-[220px] transition-colors duration-200",
        STATE_BORDER[state],
      )}
    >
      <Handle
        type="target"
        position={Position.Top}
        className="!bg-paper !border !border-hair !w-2 !h-2"
      />

      <div className="flex items-center gap-1.5 mb-0.5">
        <span className={STATE_TYPE_TEXT[state]}>{nodeIcon(btNodeType, category)}</span>
        <span className="font-mono text-[10px] font-semibold uppercase tracking-[0.08em] text-muted truncate">
          {btNodeType}
        </span>
        <span className={clsx("ml-auto w-2 h-2 rounded-full", STATE_DOT[state])} />
      </div>

      <div className="text-[12.5px] font-medium text-ink truncate" title={name}>
        {name}
      </div>

      {Object.keys(params).length > 0 && (
        <div
          className="mt-1 font-mono text-[10px] text-ink-2 truncate tracking-[0.04em]"
          title={JSON.stringify(params)}
        >
          {Object.entries(params)
            .slice(0, 2)
            .map(([k, v]) => `${k}=${v}`)
            .join(", ")}
        </div>
      )}

      <Handle
        type="source"
        position={Position.Bottom}
        className="!bg-paper !border !border-hair !w-2 !h-2"
      />
    </div>
  );
}

export default memo(BtNodeComponent);
