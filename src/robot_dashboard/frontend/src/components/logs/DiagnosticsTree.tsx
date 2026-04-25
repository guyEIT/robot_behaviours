import { useMemo } from "react";
import { useLogStore } from "../../stores/log-store";
import DiagnosticItem from "./DiagnosticItem";
import type { DiagnosticStatus } from "../../types/ros";
import { HeartPulse } from "lucide-react";
import { Eyebrow } from "../ui";

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
      <div className="flex flex-col items-center justify-center h-full text-muted py-12 bg-cream-deep">
        <HeartPulse className="w-6 h-6 mb-2 opacity-30" />
        <p className="text-[13px] text-ink-soft">No diagnostics data</p>
        <p className="text-[11px] font-mono uppercase tracking-[0.08em]">Waiting for /diagnostics_agg</p>
      </div>
    );
  }

  return (
    <div className="bg-paper">
      {Array.from(groups.entries()).map(([group, statuses]) => (
        <div key={group}>
          <div className="px-4 py-2 bg-cream-deep border-b border-hair">
            <Eyebrow size="sm" tone="terracotta">{group}</Eyebrow>
          </div>
          {statuses.map((s) => (
            <DiagnosticItem key={s.name} status={s} />
          ))}
        </div>
      ))}
    </div>
  );
}
