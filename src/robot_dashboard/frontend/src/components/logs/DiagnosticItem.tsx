import type { DiagnosticStatus } from "../../types/ros";
import { DIAG_LEVELS } from "../../types/ros";
import clsx from "clsx";

const LEVEL_DOT = {
  [DIAG_LEVELS.OK]: "bg-ok",
  [DIAG_LEVELS.WARN]: "bg-terracotta",
  [DIAG_LEVELS.ERROR]: "bg-err",
  [DIAG_LEVELS.STALE]: "bg-muted",
} as Record<number, string>;

const LEVEL_LABELS = {
  [DIAG_LEVELS.OK]: "OK",
  [DIAG_LEVELS.WARN]: "WARN",
  [DIAG_LEVELS.ERROR]: "ERROR",
  [DIAG_LEVELS.STALE]: "STALE",
} as Record<number, string>;

const LEVEL_TEXT = {
  [DIAG_LEVELS.OK]: "text-ok",
  [DIAG_LEVELS.WARN]: "text-terracotta",
  [DIAG_LEVELS.ERROR]: "text-err",
  [DIAG_LEVELS.STALE]: "text-muted",
} as Record<number, string>;

interface Props {
  status: DiagnosticStatus;
}

export default function DiagnosticItem({ status }: Props) {
  return (
    <div className="px-4 py-2 border-b border-hair-soft hover:bg-cream transition-colors">
      <div className="flex items-center gap-2">
        <span
          className={clsx(
            "w-1.5 h-1.5 rounded-full shrink-0",
            LEVEL_DOT[status.level] ?? LEVEL_DOT[DIAG_LEVELS.STALE]
          )}
        />
        <span className="text-[13px] font-medium text-ink truncate flex-1">
          {status.name}
        </span>
        <span
          className={clsx(
            "font-mono text-[10px] uppercase tracking-[0.08em] font-semibold",
            LEVEL_TEXT[status.level] ?? LEVEL_TEXT[DIAG_LEVELS.STALE],
          )}
        >
          {LEVEL_LABELS[status.level] ?? "??"}
        </span>
      </div>
      {status.message && (
        <p className="text-[11px] text-muted ml-3.5 mt-0.5">{status.message}</p>
      )}
      {status.values.length > 0 && (
        <div className="ml-3.5 mt-1 space-y-0.5">
          {status.values.map((v) => (
            <div key={v.key} className="text-[11px] font-mono tracking-[0.04em]">
              <span className="text-muted">{v.key}:</span>{" "}
              <span className="text-ink-soft">{v.value}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
