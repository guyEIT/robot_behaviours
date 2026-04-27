import { useState } from "react";
import {
  ChevronsRight,
  GitFork,
  GitBranch,
  Equal,
  Repeat,
  Layers,
  Diamond,
  Cog,
  Info,
} from "lucide-react";
import clsx from "clsx";

const STATES = [
  { label: "Idle", color: "bg-muted" },
  { label: "Running", color: "bg-running animate-pulse" },
  { label: "Success", color: "bg-ok" },
  { label: "Failure", color: "bg-err" },
] as const;

interface ShapeRow {
  icon: React.ReactNode;
  label: string;
  hint: string;
}

const SHAPES: ShapeRow[] = [
  { icon: <ChevronsRight className="w-3.5 h-3.5" />, label: "Sequence", hint: "tick children in order; stop at first failure" },
  { icon: <GitFork className="w-3.5 h-3.5" />, label: "Fallback", hint: "try children in order until one succeeds (alternates dashed)" },
  { icon: <Equal className="w-3.5 h-1" />, label: "Parallel", hint: "run children concurrently with success/failure thresholds" },
  { icon: <GitBranch className="w-3.5 h-3.5" />, label: "If/While", hint: "edges labeled cond / then / else / do" },
  { icon: <Repeat className="w-3.5 h-3.5" />, label: "Loop frame", hint: "Repeat × N · RetryUntilSuccessful ↻ N · KeepRunningUntilFailure ∞" },
  { icon: <Layers className="w-3.5 h-3.5" />, label: "SubTree", hint: "double-bordered group with the SubTree ID in the header" },
  { icon: <Diamond className="w-3.5 h-3.5" />, label: "Condition", hint: "left-edge muted bar; multi-sibling Blackboard/Script conditions" },
  { icon: <span className="font-mono text-[10px] px-1 border border-muted bg-paper text-ink-soft">⊟ key == val</span>, label: "Gate edge", hint: "single-child BlackboardCondition collapsed onto the edge" },
  { icon: <span className="font-mono text-[10px] px-1 border border-hair bg-cream">!</span>, label: "Decorator chip", hint: "Inverter / Timeout / ForceSuccess / ForceFailure / Delay / RunOnce" },
  { icon: <Cog className="w-3.5 h-3.5" />, label: "Action leaf", hint: "skill atom; sub-icon by category (human / queue / timing / …)" },
];

export default function BtLegend() {
  const [open, setOpen] = useState(false);

  return (
    <div className="relative">
      <button
        type="button"
        onClick={() => setOpen((o) => !o)}
        className={clsx(
          "flex items-center gap-2 px-3 py-1.5 bg-paper border border-hair font-mono text-[10px] uppercase tracking-[0.08em] text-ink-soft hover:bg-cream",
          open && "bg-cream",
        )}
        title="Toggle legend"
      >
        <Info className="w-3 h-3" />
        Legend
        <span className="flex items-center gap-2 ml-1 normal-case tracking-normal">
          {STATES.map((s) => (
            <span key={s.label} className="flex items-center gap-1">
              <span className={clsx("w-1.5 h-1.5 rounded-full", s.color)} />
              {s.label}
            </span>
          ))}
        </span>
      </button>
      {open && (
        <div className="absolute right-0 top-full mt-1.5 z-50 bg-paper border border-hair shadow-md min-w-[300px]">
          <div className="px-3 py-2 border-b border-hair font-mono text-[10px] uppercase tracking-[0.08em] text-muted">
            Visual grammar
          </div>
          <div className="px-3 py-2 grid grid-cols-[auto_minmax(0,1fr)] gap-x-3 gap-y-1.5 items-center">
            {SHAPES.map((row) => (
              <Row key={row.label} {...row} />
            ))}
          </div>
        </div>
      )}
    </div>
  );
}

function Row({ icon, label, hint }: ShapeRow) {
  return (
    <>
      <div className="flex items-center gap-1.5 text-muted whitespace-nowrap">
        {icon}
        <span className="font-mono text-[10px] uppercase tracking-[0.08em] text-ink-soft">
          {label}
        </span>
      </div>
      <div className="text-[11px] text-ink-2 leading-tight">{hint}</div>
    </>
  );
}
