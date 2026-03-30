import { useState, useMemo } from "react";
import { useLogStore } from "../../stores/log-store";
import type { DiagnosticStatus } from "../../types/ros";
import { DIAG_LEVELS } from "../../types/ros";
import DiagnosticCard from "./DiagnosticCard";
import {
  HeartPulse,
  ChevronDown,
  ChevronRight,
  Search,
  Filter,
} from "lucide-react";
import clsx from "clsx";

type LevelFilter = "all" | "warn" | "error";

interface DiagGroup {
  name: string;
  items: DiagnosticStatus[];
  worstLevel: number;
}

/** Group diagnostics by first path segment */
function groupDiagnostics(statuses: DiagnosticStatus[]): DiagGroup[] {
  const map = new Map<string, DiagnosticStatus[]>();
  for (const s of statuses) {
    const parts = s.name.split("/").filter(Boolean);
    const group = parts.length > 1 ? parts[0] : "Other";
    if (!map.has(group)) map.set(group, []);
    map.get(group)!.push(s);
  }

  return Array.from(map.entries()).map(([name, items]) => {
    // Sort: errors first, then warnings, then OK
    items.sort((a, b) => b.level - a.level);
    const worstLevel = items.reduce((w, i) => Math.max(w, i.level), 0);
    return { name, items, worstLevel };
  })
  // Groups with issues first
  .sort((a, b) => b.worstLevel - a.worstLevel);
}

function matchesSearch(s: DiagnosticStatus, query: string): boolean {
  const q = query.toLowerCase();
  if (s.name.toLowerCase().includes(q)) return true;
  if (s.message.toLowerCase().includes(q)) return true;
  if (s.hardware_id.toLowerCase().includes(q)) return true;
  return s.values.some(
    (v) => v.key.toLowerCase().includes(q) || v.value.toLowerCase().includes(q)
  );
}

function matchesLevel(s: DiagnosticStatus, filter: LevelFilter): boolean {
  if (filter === "all") return true;
  if (filter === "warn") return s.level >= DIAG_LEVELS.WARN;
  if (filter === "error") return s.level >= DIAG_LEVELS.ERROR;
  return true;
}

const LEVEL_DOT: Record<number, string> = {
  [DIAG_LEVELS.OK]: "bg-green-500",
  [DIAG_LEVELS.WARN]: "bg-yellow-500",
  [DIAG_LEVELS.ERROR]: "bg-red-500",
  [DIAG_LEVELS.STALE]: "bg-gray-500",
};

export default function DiagnosticsPanel() {
  const diagnostics = useLogStore((s) => s.diagnostics);
  const [search, setSearch] = useState("");
  const [levelFilter, setLevelFilter] = useState<LevelFilter>("all");
  const [collapsedGroups, setCollapsedGroups] = useState<Set<string>>(new Set());

  const filtered = useMemo(() => {
    let items = diagnostics;
    if (search) items = items.filter((s) => matchesSearch(s, search));
    if (levelFilter !== "all") items = items.filter((s) => matchesLevel(s, levelFilter));
    return items;
  }, [diagnostics, search, levelFilter]);

  const groups = useMemo(() => groupDiagnostics(filtered), [filtered]);
  const hiddenCount = diagnostics.length - filtered.length;

  // Summary counts
  const counts = useMemo(() => {
    let ok = 0, warn = 0, error = 0, stale = 0;
    for (const s of diagnostics) {
      if (s.level === DIAG_LEVELS.OK) ok++;
      else if (s.level === DIAG_LEVELS.WARN) warn++;
      else if (s.level === DIAG_LEVELS.ERROR) error++;
      else stale++;
    }
    return { ok, warn, error, stale, total: diagnostics.length };
  }, [diagnostics]);

  const toggleGroup = (name: string) => {
    setCollapsedGroups((prev) => {
      const next = new Set(prev);
      if (next.has(name)) next.delete(name);
      else next.add(name);
      return next;
    });
  };

  if (diagnostics.length === 0) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-gray-500">
        <HeartPulse className="w-8 h-8 mb-2 opacity-20" />
        <p className="text-sm">No diagnostics</p>
        <p className="text-xs">Waiting for /diagnostics_agg...</p>
      </div>
    );
  }

  return (
    <div className="flex flex-col h-full">
      {/* Summary strip */}
      <div className="px-3 py-2 border-b border-gray-800 shrink-0">
        <div className="flex items-center gap-3">
          {/* Overall health */}
          <div className="flex items-center gap-1.5">
            <HeartPulse className="w-3.5 h-3.5 text-blue-400" />
            <span className="text-[10px] font-semibold text-gray-300 uppercase">System Health</span>
          </div>

          {/* Count chips */}
          <div className="flex items-center gap-2 ml-auto">
            {counts.error > 0 && (
              <button
                onClick={() => setLevelFilter(levelFilter === "error" ? "all" : "error")}
                className={clsx(
                  "flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-bold",
                  levelFilter === "error"
                    ? "bg-red-500/20 text-red-400 ring-1 ring-red-500/40"
                    : "bg-red-950/40 text-red-400 hover:bg-red-500/20"
                )}
              >
                <span className="w-1.5 h-1.5 rounded-full bg-red-500" />
                {counts.error} ERROR
              </button>
            )}
            {counts.warn > 0 && (
              <button
                onClick={() => setLevelFilter(levelFilter === "warn" ? "all" : "warn")}
                className={clsx(
                  "flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-bold",
                  levelFilter === "warn"
                    ? "bg-yellow-500/20 text-yellow-400 ring-1 ring-yellow-500/40"
                    : "bg-yellow-950/30 text-yellow-500 hover:bg-yellow-500/20"
                )}
              >
                <span className="w-1.5 h-1.5 rounded-full bg-yellow-500" />
                {counts.warn} WARN
              </button>
            )}
            <span className="flex items-center gap-1 px-1.5 py-0.5 text-[10px] text-gray-500">
              <span className="w-1.5 h-1.5 rounded-full bg-green-500" />
              {counts.ok} OK
            </span>
            {counts.stale > 0 && (
              <span className="flex items-center gap-1 px-1.5 py-0.5 text-[10px] text-gray-600">
                <span className="w-1.5 h-1.5 rounded-full bg-gray-500" />
                {counts.stale} STALE
              </span>
            )}
          </div>
        </div>

        {/* Group chips */}
        <div className="flex gap-1.5 mt-1.5">
          {groups.map((g) => (
            <button
              key={g.name}
              onClick={() => toggleGroup(g.name)}
              className="flex items-center gap-1 px-2 py-0.5 rounded bg-gray-800/50 hover:bg-gray-800 text-[10px] text-gray-400 transition-colors"
            >
              <span
                className={clsx(
                  "w-1.5 h-1.5 rounded-full",
                  LEVEL_DOT[g.worstLevel] ?? LEVEL_DOT[DIAG_LEVELS.STALE]
                )}
              />
              {g.name}
              <span className="text-gray-600">{g.items.length}</span>
            </button>
          ))}
        </div>
      </div>

      {/* Filter bar */}
      <div className="px-3 py-1.5 border-b border-gray-800 flex items-center gap-2 shrink-0">
        <div className="relative flex-1">
          <Search className="absolute left-2 top-1/2 -translate-y-1/2 w-3 h-3 text-gray-500" />
          <input
            type="text"
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            placeholder="Filter diagnostics..."
            className="w-full pl-7 pr-2 py-1 text-[10px] bg-gray-800 border border-gray-700 rounded focus:border-blue-500 focus:outline-none text-gray-200 placeholder-gray-600"
          />
        </div>

        {/* Level filter buttons */}
        <div className="flex gap-0.5">
          {(["all", "warn", "error"] as const).map((f) => (
            <button
              key={f}
              onClick={() => setLevelFilter(f)}
              className={clsx(
                "px-1.5 py-0.5 rounded text-[9px] font-bold uppercase",
                levelFilter === f
                  ? "bg-blue-500/20 text-blue-400"
                  : "text-gray-600 hover:text-gray-400"
              )}
            >
              {f === "all" ? "All" : f === "warn" ? "Warn+" : "Error"}
            </button>
          ))}
        </div>

        <span className="text-[9px] text-gray-600 shrink-0">
          {filtered.length}{hiddenCount > 0 ? ` (${hiddenCount} hidden)` : ""}
        </span>
      </div>

      {/* Diagnostics tree */}
      <div className="flex-1 overflow-auto">
        {groups.map((group) => {
          const isCollapsed = collapsedGroups.has(group.name);
          // Auto-expand groups with errors/warnings
          const shouldShow = !isCollapsed || group.worstLevel >= DIAG_LEVELS.WARN;

          return (
            <div key={group.name}>
              {/* Group header */}
              <button
                onClick={() => toggleGroup(group.name)}
                className="w-full flex items-center gap-2 px-3 py-1.5 bg-gray-800/40 hover:bg-gray-800/60 transition-colors border-b border-gray-800/50"
              >
                {isCollapsed && group.worstLevel < DIAG_LEVELS.WARN ? (
                  <ChevronRight className="w-3 h-3 text-gray-600" />
                ) : (
                  <ChevronDown className="w-3 h-3 text-gray-600" />
                )}
                <span
                  className={clsx(
                    "w-2 h-2 rounded-full",
                    LEVEL_DOT[group.worstLevel] ?? LEVEL_DOT[DIAG_LEVELS.STALE]
                  )}
                />
                <span className="text-[11px] font-semibold text-gray-300 uppercase tracking-wide">
                  {group.name}
                </span>
                <span className="text-[10px] text-gray-600 ml-auto">
                  {group.items.length}
                </span>
              </button>

              {/* Group items */}
              {shouldShow && (
                <div className="border-b border-gray-800/30">
                  {group.items.map((s) => (
                    <DiagnosticCard
                      key={s.name}
                      status={s}
                      defaultExpanded={s.level >= DIAG_LEVELS.ERROR}
                    />
                  ))}
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}
