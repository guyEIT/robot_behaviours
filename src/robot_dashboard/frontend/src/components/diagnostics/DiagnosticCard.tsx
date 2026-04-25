import { useState } from "react";
import type { DiagnosticStatus } from "../../types/ros";
import { DIAG_LEVELS } from "../../types/ros";
import { ChevronRight, ChevronDown } from "lucide-react";
import clsx from "clsx";

const LEVEL_DOT: Record<number, string> = {
  [DIAG_LEVELS.OK]: "bg-ok",
  [DIAG_LEVELS.WARN]: "bg-terracotta",
  [DIAG_LEVELS.ERROR]: "bg-err",
  [DIAG_LEVELS.STALE]: "bg-muted",
};

const LEVEL_LABEL: Record<number, string> = {
  [DIAG_LEVELS.OK]: "OK",
  [DIAG_LEVELS.WARN]: "WARN",
  [DIAG_LEVELS.ERROR]: "ERROR",
  [DIAG_LEVELS.STALE]: "STALE",
};

const LEVEL_TEXT: Record<number, string> = {
  [DIAG_LEVELS.OK]: "text-ok",
  [DIAG_LEVELS.WARN]: "text-terracotta",
  [DIAG_LEVELS.ERROR]: "text-err",
  [DIAG_LEVELS.STALE]: "text-muted",
};

const LEVEL_LEFT_BORDER: Record<number, string> = {
  [DIAG_LEVELS.OK]: "border-l-transparent",
  [DIAG_LEVELS.WARN]: "border-l-terracotta",
  [DIAG_LEVELS.ERROR]: "border-l-err",
  [DIAG_LEVELS.STALE]: "border-l-transparent",
};

function shortName(fullName: string): string {
  const parts = fullName.split("/").filter(Boolean);
  return parts.length > 1 ? parts.slice(1).join("/") : parts[0] || fullName;
}

export default function DiagnosticCard({
  status,
  defaultExpanded = false,
}: {
  status: DiagnosticStatus;
  defaultExpanded?: boolean;
}) {
  const [expanded, setExpanded] = useState(defaultExpanded);
  const hasValues = status.values.length > 0;
  const level = status.level ?? DIAG_LEVELS.STALE;

  return (
    <div
      className={clsx(
        "border-l-2 ml-2",
        LEVEL_LEFT_BORDER[level] ?? "border-l-transparent",
        level >= DIAG_LEVELS.ERROR && "bg-err-soft",
      )}
    >
      <button
        onClick={() => hasValues && setExpanded(!expanded)}
        className={clsx(
          "w-full flex items-center gap-2 px-3 py-2 text-left transition-colors",
          hasValues ? "hover:bg-cream cursor-pointer" : "cursor-default",
        )}
      >
        <span className="w-3 shrink-0 text-muted">
          {hasValues ? (
            expanded ? (
              <ChevronDown className="w-3 h-3" />
            ) : (
              <ChevronRight className="w-3 h-3" />
            )
          ) : null}
        </span>

        <span
          className={clsx(
            "w-1.5 h-1.5 rounded-full shrink-0",
            LEVEL_DOT[level] ?? LEVEL_DOT[DIAG_LEVELS.STALE]
          )}
        />

        <span className="font-mono text-[11.5px] text-ink truncate min-w-0 flex-1 tracking-[0.04em]">
          {shortName(status.name)}
        </span>

        <span
          className={clsx(
            "font-mono text-[10px] font-semibold uppercase shrink-0 tracking-[0.08em]",
            LEVEL_TEXT[level] ?? LEVEL_TEXT[DIAG_LEVELS.STALE]
          )}
        >
          {LEVEL_LABEL[level] ?? "??"}
        </span>
      </button>

      {status.message && (
        <div className="pl-9 pr-3 pb-1">
          <span className="text-[11px] text-muted italic">{status.message}</span>
        </div>
      )}

      {expanded && hasValues && (
        <div className="pl-9 pr-3 pb-3 space-y-0.5">
          {status.values.map((v) => (
            <div key={v.key} className="flex gap-2 text-[11px] font-mono tracking-[0.04em]">
              <span className="text-muted shrink-0">{v.key}:</span>
              <span className="text-ink-soft truncate">{v.value}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
