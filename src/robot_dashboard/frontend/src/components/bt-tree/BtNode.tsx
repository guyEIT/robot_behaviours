import { memo } from "react";
import { Handle, Position, type NodeProps } from "@xyflow/react";
import {
  ChevronsRight,
  GitFork,
  Equal,
  GitBranch,
  Diamond,
  Cog,
  Clock,
  ListOrdered,
  User,
  Move3d,
  Power,
  KeyRound,
  FileText,
  Database,
  Zap,
  Save,
} from "lucide-react";
import clsx from "clsx";
import type {
  BtGraphNode,
  BtNodeState,
  BtControlKind,
  BtLeafCategory,
} from "../../types/bt";

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

/**
 * The left strip motif encodes control-flow class. Sequence gets
 * chevrons (one direction of flow), Fallback gets a fork (alternatives),
 * Parallel gets a triple-bar (concurrent). IfThenElse / WhileDoElse get
 * a branch glyph; their children carry cond/then/else edge labels.
 */
function controlMotif(kind: BtControlKind | undefined) {
  if (!kind) return null;
  const cls = "w-3.5 h-3.5";
  switch (kind) {
    case "sequence":
      return <ChevronsRight className={cls} />;
    case "reactive_sequence":
      return (
        <span className="flex items-center gap-[1px]">
          <Zap className="w-3 h-3" />
          <ChevronsRight className={cls} />
        </span>
      );
    case "sequence_memory":
      return (
        <span className="flex items-center gap-[1px]">
          <Save className="w-3 h-3" />
          <ChevronsRight className={cls} />
        </span>
      );
    case "fallback":
      return <GitFork className={cls} />;
    case "reactive_fallback":
      return (
        <span className="flex items-center gap-[1px]">
          <Zap className="w-3 h-3" />
          <GitFork className={cls} />
        </span>
      );
    case "parallel":
      // Triple-bar — repurpose Equal rotated for visual parallel-ness.
      return (
        <span className="flex flex-col gap-[2px]">
          <Equal className="w-3.5 h-1" />
        </span>
      );
    case "if_then_else":
    case "while_do_else":
      return <GitBranch className={cls} />;
  }
}

function leafIcon(cat: BtLeafCategory | undefined, _btNodeType: string) {
  const cls = "w-3.5 h-3.5";
  switch (cat) {
    case "human":
      return <User className={cls} />;
    case "queue":
      return <ListOrdered className={cls} />;
    case "timing":
      return <Clock className={cls} />;
    case "pose-tf":
      return <Move3d className={cls} />;
    case "io":
      return <Power className={cls} />;
    case "lease":
      return <KeyRound className={cls} />;
    case "log":
      return <FileText className={cls} />;
    case "blackboard":
      return <Database className={cls} />;
    case "condition":
      return <Diamond className={cls} />;
    case "skill":
    case "default":
    default:
      return <Cog className={cls} />;
  }
}

function BtNodeComponent({ data }: NodeProps & { data: BtGraphNode }) {
  const {
    category,
    btNodeType,
    name,
    state,
    params,
    controlKind,
    leafCategory,
    displayParams,
    decorators,
  } = data;

  const isControl = category === "control";
  const isCondition = category === "condition";
  const motif = isControl ? controlMotif(controlKind) : null;
  const leaf = !isControl ? leafIcon(leafCategory, btNodeType) : null;

  return (
    <div
      className={clsx(
        "relative border min-w-[170px] max-w-[230px] transition-colors duration-200",
        STATE_BORDER[state],
        // Conditions get a slightly narrower, slanted left edge via a
        // notch — keep the standard rectangle but shift the icon column
        // to a softer tone.
        isCondition && "border-l-[3px] border-l-muted",
      )}
    >
      {/* Decorator chip strip — collapsed single-child decorators (Inverter, Timeout, etc.) */}
      {decorators && decorators.length > 0 && (
        <div className="flex items-center gap-1 px-2 pt-1.5 pb-0.5 border-b border-hair-soft">
          {decorators.map((d, i) => (
            <span
              key={i}
              className="font-mono text-[10px] px-1.5 py-[1px] border border-hair bg-cream text-ink-soft"
              title={`${d.btNodeType}${
                Object.keys(d.params).length > 0
                  ? " " + JSON.stringify(d.params)
                  : ""
              }`}
            >
              {d.label}
            </span>
          ))}
        </div>
      )}

      {/* Left motif strip for control nodes */}
      {motif && (
        <div className="absolute top-0 bottom-0 left-0 w-5 flex items-center justify-center text-muted bg-cream/50 border-r border-hair-soft">
          {motif}
        </div>
      )}

      <div className={clsx("px-3 py-2", motif && "pl-6")}>
        <Handle
          type="target"
          position={data.layoutDirection === "LR" ? Position.Left : Position.Top}
          className="!bg-paper !border !border-hair !w-2 !h-2"
        />

        <div className="flex items-center gap-1.5 mb-0.5">
          {leaf && (
            <span className={STATE_TYPE_TEXT[state]}>{leaf}</span>
          )}
          <span className="font-mono text-[10px] font-semibold uppercase tracking-[0.08em] text-muted truncate">
            {btNodeType}
          </span>
          <span className={clsx("ml-auto w-2 h-2 rounded-full", STATE_DOT[state])} />
        </div>

        <div className="text-[12.5px] font-medium text-ink truncate" title={name}>
          {name}
        </div>

        {displayParams && displayParams.length > 0 && (
          <div className="mt-1 flex items-center gap-1 flex-wrap">
            {displayParams.map((p, i) => (
              <span
                key={i}
                className="font-mono text-[10px] px-1.5 py-[1px] border border-hair bg-cream text-ink-soft"
                title={p.title ?? p.label}
              >
                {p.label}
              </span>
            ))}
          </div>
        )}

        {/*
          When there are no curated displayParams, fall back to the
          original "first two raw attrs" preview so unfamiliar atoms are
          still legible.
        */}
        {(!displayParams || displayParams.length === 0) &&
          Object.keys(params).length > 0 && (
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
          position={data.layoutDirection === "LR" ? Position.Right : Position.Bottom}
          className="!bg-paper !border !border-hair !w-2 !h-2"
        />
      </div>
    </div>
  );
}

export default memo(BtNodeComponent);
