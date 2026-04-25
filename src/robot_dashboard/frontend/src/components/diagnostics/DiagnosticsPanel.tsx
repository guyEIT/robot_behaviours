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
} from "lucide-react";
import clsx from "clsx";
import { Eyebrow } from "../ui";

type LevelFilter = "all" | "warn" | "error";

interface DiagGroup {
  name: string;
  items: DiagnosticStatus[];
  worstLevel: number;
}

function groupDiagnostics(statuses: DiagnosticStatus[]): DiagGroup[] {
  const map = new Map<string, DiagnosticStatus[]>();
  for (const s of statuses) {
    const parts = s.name.split("/").filter(Boolean);
    const group = parts.length > 1 ? parts[0] : "Other";
    if (!map.has(group)) map.set(group, []);
    map.get(group)!.push(s);
  }

  return Array.from(map.entries())
    .map(([name, items]) => {
      items.sort((a, b) => b.level - a.level);
      const worstLevel = items.reduce((w, i) => Math.max(w, i.level), 0);
      return { name, items, worstLevel };
    })
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
  [DIAG_LEVELS.OK]: "bg-ok",
  [DIAG_LEVELS.WARN]: "bg-terracotta",
  [DIAG_LEVELS.ERROR]: "bg-err",
  [DIAG_LEVELS.STALE]: "bg-muted",
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
      <div className="flex flex-col items-center justify-center h-full text-muted bg-cream-deep">
        <HeartPulse className="w-8 h-8 mb-2 opacity-30" />
        <p className="text-[14px] text-ink-soft">No diagnostics</p>
        <p className="text-[11px] font-mono uppercase tracking-[0.08em]">Waiting for /diagnostics_agg…</p>
      </div>
    );
  }

  return (
    <div className="flex flex-col h-full bg-paper">
      {/* Summary strip */}
      <div className="px-4 py-3 border-b border-hair shrink-0">
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-1.5">
            <HeartPulse className="w-3.5 h-3.5 text-terracotta" />
            <Eyebrow size="sm">System Health</Eyebrow>
          </div>

          <div className="flex items-center gap-2 ml-auto">
            {counts.error > 0 && (
              <button
                onClick={() => setLevelFilter(levelFilter === "error" ? "all" : "error")}
                className={clsx(
                  "flex items-center gap-1 px-2 py-0.5 font-mono text-[10px] font-semibold uppercase tracking-[0.08em] border transition-colors",
                  levelFilter === "error"
                    ? "border-err bg-err-soft text-err"
                    : "border-err text-err hover:bg-err-soft",
                )}
              >
                <span className="w-1.5 h-1.5 rounded-full bg-err" />
                {counts.error} Error
              </button>
            )}
            {counts.warn > 0 && (
              <button
                onClick={() => setLevelFilter(levelFilter === "warn" ? "all" : "warn")}
                className={clsx(
                  "flex items-center gap-1 px-2 py-0.5 font-mono text-[10px] font-semibold uppercase tracking-[0.08em] border transition-colors",
                  levelFilter === "warn"
                    ? "border-terracotta bg-terracotta-tint text-terracotta"
                    : "border-terracotta text-terracotta hover:bg-terracotta-tint",
                )}
              >
                <span className="w-1.5 h-1.5 rounded-full bg-terracotta" />
                {counts.warn} Warn
              </button>
            )}
            <span className="flex items-center gap-1 px-2 py-0.5 font-mono text-[10px] uppercase tracking-[0.08em] text-ok">
              <span className="w-1.5 h-1.5 rounded-full bg-ok" />
              {counts.ok} OK
            </span>
            {counts.stale > 0 && (
              <span className="flex items-center gap-1 px-2 py-0.5 font-mono text-[10px] uppercase tracking-[0.08em] text-muted">
                <span className="w-1.5 h-1.5 rounded-full bg-muted" />
                {counts.stale} Stale
              </span>
            )}
          </div>
        </div>

        <div className="flex flex-wrap gap-1.5 mt-2">
          {groups.map((g) => (
            <button
              key={g.name}
              onClick={() => toggleGroup(g.name)}
              className="flex items-center gap-1.5 px-2 py-0.5 bg-cream-deep border border-hair hover:bg-cream font-mono text-[10px] uppercase tracking-[0.06em] text-ink-soft transition-colors"
            >
              <span
                className={clsx(
                  "w-1.5 h-1.5 rounded-full",
                  LEVEL_DOT[g.worstLevel] ?? LEVEL_DOT[DIAG_LEVELS.STALE]
                )}
              />
              {g.name}
              <span className="text-muted">{g.items.length}</span>
            </button>
          ))}
        </div>
      </div>

      {/* Filter bar */}
      <div className="px-4 py-2 border-b border-hair-soft flex items-center gap-2 shrink-0 bg-cream-deep">
        <div className="relative flex-1">
          <Search className="absolute left-2 top-1/2 -translate-y-1/2 w-3 h-3 text-muted" />
          <input
            type="text"
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            placeholder="Filter diagnostics…"
            className="w-full pl-7 pr-2 py-1 text-[11px] bg-paper border border-hair rounded-DEFAULT focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
          />
        </div>

        <div className="flex gap-0.5">
          {(["all", "warn", "error"] as const).map((f) => (
            <button
              key={f}
              onClick={() => setLevelFilter(f)}
              className={clsx(
                "px-2 py-0.5 font-mono text-[9px] font-semibold uppercase tracking-[0.08em] border transition-colors",
                levelFilter === f
                  ? "border-terracotta bg-terracotta text-paper"
                  : "border-hair text-muted hover:text-ink-soft",
              )}
            >
              {f === "all" ? "All" : f === "warn" ? "Warn+" : "Error"}
            </button>
          ))}
        </div>

        <span className="font-mono text-[10px] text-muted shrink-0 tracking-[0.06em]">
          {filtered.length}{hiddenCount > 0 ? ` (${hiddenCount} hidden)` : ""}
        </span>
      </div>

      <div className="flex-1 overflow-auto">
        {groups.map((group) => {
          const isCollapsed = collapsedGroups.has(group.name);
          const shouldShow = !isCollapsed || group.worstLevel >= DIAG_LEVELS.WARN;

          return (
            <div key={group.name}>
              <button
                onClick={() => toggleGroup(group.name)}
                className="w-full flex items-center gap-2 px-4 py-2 bg-cream-deep hover:bg-cream transition-colors border-b border-hair-soft"
              >
                {isCollapsed && group.worstLevel < DIAG_LEVELS.WARN ? (
                  <ChevronRight className="w-3 h-3 text-muted" />
                ) : (
                  <ChevronDown className="w-3 h-3 text-muted" />
                )}
                <span
                  className={clsx(
                    "w-2 h-2 rounded-full",
                    LEVEL_DOT[group.worstLevel] ?? LEVEL_DOT[DIAG_LEVELS.STALE]
                  )}
                />
                <Eyebrow size="sm">{group.name}</Eyebrow>
                <span className="text-[10px] text-muted ml-auto font-mono">
                  {group.items.length}
                </span>
              </button>

              {shouldShow && (
                <div className="border-b border-hair-soft">
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
