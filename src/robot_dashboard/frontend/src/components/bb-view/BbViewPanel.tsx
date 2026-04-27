import { useState } from "react";
import { Database, Search, Copy } from "lucide-react";
import { Chip, Eyebrow } from "../ui";
import type { ChipState } from "../ui";
import { useBlackboardSnapshot } from "../../hooks/useBlackboardSnapshot";
import { BbKeyTree } from "./BbKeyTree";

const STATUS_TO_CHIP: Record<string, ChipState> = {
  RUNNING: "running",
  AWAITING_APPROVAL: "running",
  PAUSED: "neutral",
  SUCCESS: "done",
  FAILURE: "failed",
  HALTED: "neutral",
  CANCELLED: "neutral",
  IDLE: "idle",
};

const TERMINAL = new Set(["SUCCESS", "FAILURE", "HALTED", "CANCELLED"]);

export default function BbViewPanel() {
  const snap = useBlackboardSnapshot();
  const [filter, setFilter] = useState("");

  const taskShort = snap?.task_id ? snap.task_id.slice(0, 8) : "—";
  const status = snap?.task_status ?? "IDLE";
  const isTerminal = TERMINAL.has(status);
  const updatedAgo = snap?.snapshot_at
    ? formatAgo(Date.now() / 1000 - snap.snapshot_at)
    : null;
  const keyCount = snap ? Object.keys(snap.persistent).length : 0;

  const onCopySnapshot = () => {
    if (!snap) return;
    void navigator.clipboard.writeText(JSON.stringify(snap.persistent, null, 2));
  };

  return (
    <div className="flex flex-col h-full bg-paper">
      <div className="flex items-center gap-3 px-5 py-3 border-b border-hair">
        <Database className="w-4 h-4 text-terracotta" />
        <h2 className="text-[14px] font-medium text-ink">Blackboard</h2>

        {snap && (
          <div className="ml-4 flex items-center gap-2 text-[12px] text-muted">
            <Eyebrow size="sm" tone="muted">Task</Eyebrow>
            <span className="font-mono text-ink-soft">{taskShort}</span>
            <Chip
              state={STATUS_TO_CHIP[status] ?? "idle"}
              showDot={status === "RUNNING"}
            >
              {snap.paused ? "PAUSED" : status}
            </Chip>
            {updatedAgo && (
              <span className="font-mono text-[11px] text-muted">
                · updated {updatedAgo}
              </span>
            )}
            <span className="font-mono text-[11px] text-muted">
              · {keyCount} {keyCount === 1 ? "key" : "keys"}
            </span>
          </div>
        )}

        <div className="ml-auto flex items-center gap-2">
          <button
            type="button"
            onClick={onCopySnapshot}
            disabled={!snap}
            className="flex items-center gap-1.5 px-2 py-1 bg-paper border border-hair font-mono text-[10px] uppercase tracking-[0.08em] text-ink-soft hover:bg-cream disabled:opacity-50"
            title="Copy persistent map as JSON"
          >
            <Copy className="w-3 h-3" />
            Copy
          </button>
        </div>
      </div>

      {/* Filter input */}
      <div className="px-5 py-2 border-b border-hair flex items-center gap-2">
        <Search className="w-3.5 h-3.5 text-muted" />
        <input
          type="text"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
          placeholder="Filter keys (e.g. plate, paused)…"
          className="flex-1 bg-cream-deep border border-hair px-2 py-1 font-mono text-[12px] text-ink-soft focus:outline-none focus:border-terracotta"
        />
        {filter && (
          <button
            type="button"
            onClick={() => setFilter("")}
            className="font-mono text-[10px] uppercase tracking-[0.08em] text-muted hover:text-ink-soft"
          >
            clear
          </button>
        )}
      </div>

      {/* Body */}
      <div className="flex-1 overflow-auto bg-cream-deep">
        {snap ? (
          <div className={isTerminal ? "opacity-60" : ""}>
            {isTerminal && (
              <div className="px-3 py-2 bg-stone/60 border-b border-hair text-[12px] text-muted">
                Task ended ({status}). Snapshot below is the last known state.
              </div>
            )}
            <BbKeyTree persistent={snap.persistent} filter={filter} />
          </div>
        ) : (
          <div className="h-full flex items-center justify-center">
            <div className="text-center text-muted">
              <Database className="w-8 h-8 mx-auto mb-2 opacity-30" />
              <p className="text-[14px] text-ink-soft font-medium">No snapshot yet</p>
              <p className="text-[12px]">
                Waiting for /skill_server/persistent_state. Start a task that uses persistent.* keys.
              </p>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

function formatAgo(seconds: number): string {
  if (!Number.isFinite(seconds) || seconds < 0) return "just now";
  if (seconds < 1) return "just now";
  if (seconds < 60) return `${Math.floor(seconds)}s ago`;
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m ago`;
  return `${(seconds / 3600).toFixed(1)}h ago`;
}
