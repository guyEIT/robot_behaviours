import { useState } from "react";
import type { DiagnosticStatus } from "../../types/ros";
import { DIAG_LEVELS } from "../../types/ros";
import { ChevronRight, ChevronDown } from "lucide-react";
import clsx from "clsx";

const LEVEL_DOT: Record<number, string> = {
  [DIAG_LEVELS.OK]: "bg-green-500",
  [DIAG_LEVELS.WARN]: "bg-yellow-500",
  [DIAG_LEVELS.ERROR]: "bg-red-500",
  [DIAG_LEVELS.STALE]: "bg-gray-500",
};

const LEVEL_LABEL: Record<number, string> = {
  [DIAG_LEVELS.OK]: "OK",
  [DIAG_LEVELS.WARN]: "WARN",
  [DIAG_LEVELS.ERROR]: "ERROR",
  [DIAG_LEVELS.STALE]: "STALE",
};

const LEVEL_TEXT: Record<number, string> = {
  [DIAG_LEVELS.OK]: "text-green-400",
  [DIAG_LEVELS.WARN]: "text-yellow-400",
  [DIAG_LEVELS.ERROR]: "text-red-400",
  [DIAG_LEVELS.STALE]: "text-gray-500",
};

/** Short name: strip group prefix, e.g. "/Skill Atoms/GripperControl" → "GripperControl" */
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
        level === DIAG_LEVELS.ERROR
          ? "border-red-500/60"
          : level === DIAG_LEVELS.WARN
          ? "border-yellow-500/40"
          : "border-transparent"
      )}
    >
      {/* Header row */}
      <button
        onClick={() => hasValues && setExpanded(!expanded)}
        className={clsx(
          "w-full flex items-center gap-2 px-2 py-1.5 text-left transition-colors",
          hasValues ? "hover:bg-gray-800/40 cursor-pointer" : "cursor-default",
          level >= DIAG_LEVELS.ERROR && "bg-red-950/20"
        )}
      >
        {/* Expand chevron */}
        <span className="w-3 shrink-0 text-gray-600">
          {hasValues ? (
            expanded ? (
              <ChevronDown className="w-3 h-3" />
            ) : (
              <ChevronRight className="w-3 h-3" />
            )
          ) : null}
        </span>

        {/* Status dot */}
        <span
          className={clsx(
            "w-2 h-2 rounded-full shrink-0",
            LEVEL_DOT[level] ?? LEVEL_DOT[DIAG_LEVELS.STALE]
          )}
        />

        {/* Name */}
        <span className="text-[11px] text-gray-200 truncate min-w-0 flex-1 font-mono">
          {shortName(status.name)}
        </span>

        {/* Level badge */}
        <span
          className={clsx(
            "text-[9px] font-bold uppercase shrink-0",
            LEVEL_TEXT[level] ?? LEVEL_TEXT[DIAG_LEVELS.STALE]
          )}
        >
          {LEVEL_LABEL[level] ?? "??"}
        </span>
      </button>

      {/* Message (always visible if present) */}
      {status.message && (
        <div className="pl-9 pr-2 pb-0.5">
          <span className="text-[10px] text-gray-500 italic">{status.message}</span>
        </div>
      )}

      {/* Key-value pairs (expanded) */}
      {expanded && hasValues && (
        <div className="pl-9 pr-2 pb-2 space-y-0.5">
          {status.values.map((v) => (
            <div key={v.key} className="flex gap-2 text-[10px] font-mono">
              <span className="text-gray-500 shrink-0">{v.key}:</span>
              <span className="text-gray-300 truncate">{v.value}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
