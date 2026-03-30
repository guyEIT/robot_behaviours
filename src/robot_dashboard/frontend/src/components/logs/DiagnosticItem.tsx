import type { DiagnosticStatus } from "../../types/ros";
import { DIAG_LEVELS } from "../../types/ros";
import clsx from "clsx";

const LEVEL_STYLES = {
  [DIAG_LEVELS.OK]: "bg-green-500",
  [DIAG_LEVELS.WARN]: "bg-yellow-500",
  [DIAG_LEVELS.ERROR]: "bg-red-500",
  [DIAG_LEVELS.STALE]: "bg-gray-500",
} as Record<number, string>;

const LEVEL_LABELS = {
  [DIAG_LEVELS.OK]: "OK",
  [DIAG_LEVELS.WARN]: "WARN",
  [DIAG_LEVELS.ERROR]: "ERROR",
  [DIAG_LEVELS.STALE]: "STALE",
} as Record<number, string>;

interface Props {
  status: DiagnosticStatus;
}

export default function DiagnosticItem({ status }: Props) {
  return (
    <div className="px-3 py-2 border-b border-gray-800/50 hover:bg-gray-800/30">
      <div className="flex items-center gap-2">
        <span
          className={clsx(
            "w-2 h-2 rounded-full shrink-0",
            LEVEL_STYLES[status.level] ?? LEVEL_STYLES[DIAG_LEVELS.STALE]
          )}
        />
        <span className="text-xs font-medium text-gray-200 truncate flex-1">
          {status.name}
        </span>
        <span className="text-[10px] text-gray-500">
          {LEVEL_LABELS[status.level] ?? "??"}
        </span>
      </div>
      {status.message && (
        <p className="text-[10px] text-gray-400 ml-4 mt-0.5">{status.message}</p>
      )}
      {status.values.length > 0 && (
        <div className="ml-4 mt-1 space-y-0.5">
          {status.values.map((v) => (
            <div key={v.key} className="text-[10px] text-gray-500">
              <span className="text-gray-400">{v.key}:</span> {v.value}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
