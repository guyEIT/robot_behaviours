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

const STATE_COLORS: Record<BtNodeState, string> = {
  idle: "border-gray-600 bg-gray-800",
  running: "border-blue-500 bg-blue-950 ring-2 ring-blue-500/40",
  success: "border-green-500 bg-green-950",
  failure: "border-red-500 bg-red-950",
};

const STATE_DOT: Record<BtNodeState, string> = {
  idle: "bg-gray-500",
  running: "bg-blue-400 animate-pulse",
  success: "bg-green-400",
  failure: "bg-red-400",
};

function nodeIcon(btNodeType: string, category: string) {
  const cls = "w-4 h-4 shrink-0";
  switch (category) {
    case "control":
      if (btNodeType === "Fallback" || btNodeType === "ReactiveFallback")
        return <GitBranch className={cls} />;
      return <Workflow className={cls} />;
    case "decorator":
      if (btNodeType.includes("Retry"))
        return <RotateCcw className={cls} />;
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
        "rounded-lg border px-3 py-2 min-w-[160px] max-w-[220px] shadow-lg transition-all duration-200",
        STATE_COLORS[state]
      )}
    >
      <Handle
        type="target"
        position={Position.Top}
        className="!bg-gray-500 !w-2 !h-2 !border-0"
      />

      {/* Header row */}
      <div className="flex items-center gap-1.5 mb-0.5">
        {nodeIcon(btNodeType, category)}
        <span className="text-[10px] font-medium uppercase text-gray-400 truncate">
          {btNodeType}
        </span>
        <span className={clsx("ml-auto w-2 h-2 rounded-full", STATE_DOT[state])} />
      </div>

      {/* Node name */}
      <div className="text-xs font-semibold text-gray-100 truncate" title={name}>
        {name}
      </div>

      {/* Show key params */}
      {Object.keys(params).length > 0 && (
        <div className="mt-1 text-[10px] text-gray-400 truncate" title={JSON.stringify(params)}>
          {Object.entries(params)
            .slice(0, 2)
            .map(([k, v]) => `${k}=${v}`)
            .join(", ")}
        </div>
      )}

      <Handle
        type="source"
        position={Position.Bottom}
        className="!bg-gray-500 !w-2 !h-2 !border-0"
      />
    </div>
  );
}

export default memo(BtNodeComponent);
