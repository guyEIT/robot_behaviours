import { useMemo } from "react";
import { useLogStore } from "../../stores/log-store";
import DiagnosticItem from "./DiagnosticItem";
import type { DiagnosticStatus } from "../../types/ros";
import { HeartPulse } from "lucide-react";

/**
 * Group diagnostics by path prefix (e.g. "/Skill Atoms/MoveToNamedConfig").
 * The diagnostics aggregator uses "/" separated hierarchical names.
 */
function groupDiagnostics(statuses: DiagnosticStatus[]) {
  const groups = new Map<string, DiagnosticStatus[]>();
  for (const s of statuses) {
    const parts = s.name.split("/").filter(Boolean);
    const group = parts.length > 1 ? parts[0] : "Other";
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group)!.push(s);
  }
  return groups;
}

export default function DiagnosticsTree() {
  const diagnostics = useLogStore((s) => s.diagnostics);
  const groups = useMemo(() => groupDiagnostics(diagnostics), [diagnostics]);

  if (diagnostics.length === 0) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-gray-500 py-12">
        <HeartPulse className="w-6 h-6 mb-2 opacity-30" />
        <p className="text-xs">No diagnostics data</p>
        <p className="text-[10px]">Waiting for /diagnostics_agg</p>
      </div>
    );
  }

  return (
    <div className="divide-y divide-gray-800/30">
      {Array.from(groups.entries()).map(([group, statuses]) => (
        <div key={group}>
          <div className="px-3 py-1.5 bg-gray-800/30 text-[10px] font-bold uppercase text-gray-500 tracking-wider">
            {group}
          </div>
          {statuses.map((s) => (
            <DiagnosticItem key={s.name} status={s} />
          ))}
        </div>
      ))}
    </div>
  );
}
