import { useRef, useEffect, useMemo, useState, useCallback } from "react";
import { FixedSizeList as List } from "react-window";
import { useLogStore } from "../../stores/log-store";
import LogFilterBar from "./LogFilterBar";
import {
  logLevelName,
  type RosLog,
  type SkillLogEvent,
  type LogLevelName,
} from "../../types/ros";
import { Trash2, User, Wrench } from "lucide-react";
import clsx from "clsx";

const LEVEL_COLORS: Record<LogLevelName, string> = {
  DEBUG: "text-muted",
  INFO: "text-ink-soft",
  WARN: "text-terracotta",
  ERROR: "text-err",
  FATAL: "text-err font-bold",
};

const SEV_COLORS: Record<string, string> = {
  info: "text-running",
  warn: "text-terracotta",
  warning: "text-terracotta",
  error: "text-err",
  critical: "text-err font-bold",
};

type ViewMode = "all" | "human";

export default function LogStream() {
  const logs = useLogStore((s) => s.logs);
  const skillLogs = useLogStore((s) => s.skillLogs);
  const clearLogs = useLogStore((s) => s.clearLogs);
  const [viewMode, setViewMode] = useState<ViewMode>("all");
  const [activeLevels, setActiveLevels] = useState<Set<LogLevelName>>(
    new Set(["INFO", "WARN", "ERROR", "FATAL"])
  );
  const [nodeFilter, setNodeFilter] = useState("");
  const [textFilter, setTextFilter] = useState("");
  const [autoScroll, setAutoScroll] = useState(true);
  const listRef = useRef<List>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [containerHeight, setContainerHeight] = useState(400);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const observer = new ResizeObserver((entries) => {
      const h = entries[0]?.contentRect.height;
      if (h && h > 0) setContainerHeight(h);
    });
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  const toggleLevel = useCallback((level: LogLevelName) => {
    setActiveLevels((prev) => {
      const next = new Set(prev);
      if (next.has(level)) next.delete(level);
      else next.add(level);
      return next;
    });
  }, []);

  const filteredRos = useMemo(() => {
    if (viewMode !== "all") return [];
    const nodeLower = nodeFilter.toLowerCase();
    const textLower = textFilter.toLowerCase();
    return logs.filter((log) => {
      const levelName = logLevelName(log.level);
      if (!activeLevels.has(levelName)) return false;
      if (nodeLower && !log.name.toLowerCase().includes(nodeLower)) return false;
      if (textLower && !log.msg.toLowerCase().includes(textLower)) return false;
      return true;
    });
  }, [logs, activeLevels, nodeFilter, textFilter, viewMode]);

  const filteredSkill = useMemo(() => {
    const textLower = textFilter.toLowerCase();
    return skillLogs.filter((log) => {
      if (viewMode === "human" && !log.tags.includes("human")) return false;
      if (textLower && !log.message.toLowerCase().includes(textLower) &&
          !log.event_name.toLowerCase().includes(textLower)) return false;
      return true;
    });
  }, [skillLogs, textFilter, viewMode]);

  type MergedEntry =
    | { type: "ros"; log: RosLog }
    | { type: "skill"; log: SkillLogEvent };

  const merged = useMemo((): MergedEntry[] => {
    if (viewMode === "human") {
      return filteredSkill.map((l) => ({ type: "skill" as const, log: l }));
    }
    const rosEntries: MergedEntry[] = filteredRos.map((l) => ({ type: "ros" as const, log: l }));
    const skillEntries: MergedEntry[] = filteredSkill.map((l) => ({ type: "skill" as const, log: l }));
    return [...rosEntries, ...skillEntries].sort((a, b) => {
      const aT = a.type === "ros" ? a.log.stamp.sec : a.log.stamp.sec;
      const bT = b.type === "ros" ? b.log.stamp.sec : b.log.stamp.sec;
      return aT - bT;
    });
  }, [filteredRos, filteredSkill, viewMode]);

  useEffect(() => {
    if (autoScroll && listRef.current && merged.length > 0) {
      listRef.current.scrollToItem(merged.length - 1);
    }
  }, [merged.length, autoScroll]);

  const Row = ({ index, style }: { index: number; style: React.CSSProperties }) => {
    const entry = merged[index];
    const stripeBg = index % 2 === 0 ? "bg-paper" : "bg-cream-deep";
    if (entry.type === "ros") {
      const log = entry.log;
      const levelName = logLevelName(log.level);
      return (
        <div
          style={style}
          className={clsx(
            "flex items-baseline gap-2 px-3 text-[11px] font-mono hover:bg-cream tracking-[0.02em]",
            stripeBg,
          )}
        >
          <span className="text-muted shrink-0 w-16">
            {new Date(log.stamp.sec * 1000).toLocaleTimeString()}
          </span>
          <span className={clsx("shrink-0 w-10 font-semibold", LEVEL_COLORS[levelName])}>
            {levelName}
          </span>
          <span className="text-muted shrink-0 w-28 truncate" title={log.name}>
            {log.name}
          </span>
          <span className="text-ink-soft truncate">{log.msg}</span>
        </div>
      );
    }
    const log = entry.log;
    const isHuman = log.tags.includes("human");
    const sevColor = SEV_COLORS[log.severity] || "text-ink-soft";
    return (
      <div
        style={style}
        className={clsx(
          "flex items-baseline gap-2 px-3 text-[11px] font-mono hover:bg-cream tracking-[0.02em]",
          stripeBg,
          isHuman && "border-l-2 border-l-terracotta",
        )}
      >
        <span className="text-muted shrink-0 w-16">
          {new Date(log.stamp.sec * 1000).toLocaleTimeString()}
        </span>
        <span className={clsx("shrink-0 w-10 font-semibold uppercase", sevColor)}>
          {log.severity.slice(0, 5)}
        </span>
        <span className="text-terracotta shrink-0 w-28 truncate" title={log.skill_name}>
          {isHuman ? "\u{1F464} " : ""}{log.skill_name || log.event_name}
        </span>
        <span className="text-ink truncate">{log.message}</span>
      </div>
    );
  };

  return (
    <div className="flex flex-col h-full">
      <div className="px-3 py-2 border-b border-hair-soft flex items-center gap-2 bg-cream-deep">
        <div className="flex shrink-0">
          <button
            onClick={() => setViewMode("human")}
            className={clsx(
              "px-2 py-0.5 font-mono text-[10px] font-semibold uppercase tracking-[0.08em] border transition-colors flex items-center gap-1",
              viewMode === "human"
                ? "bg-terracotta border-terracotta text-paper"
                : "border-hair text-muted hover:text-ink-soft",
            )}
            title="Show human-readable logs only"
          >
            <User className="w-2.5 h-2.5" />
            Human
          </button>
          <button
            onClick={() => setViewMode("all")}
            className={clsx(
              "px-2 py-0.5 font-mono text-[10px] font-semibold uppercase tracking-[0.08em] border-y border-r transition-colors flex items-center gap-1",
              viewMode === "all"
                ? "bg-terracotta border-terracotta text-paper"
                : "border-hair text-muted hover:text-ink-soft",
            )}
            title="Show all system logs"
          >
            <Wrench className="w-2.5 h-2.5" />
            All
          </button>
        </div>

        {viewMode === "all" && (
          <LogFilterBar
            activeLevels={activeLevels}
            onToggleLevel={toggleLevel}
            nodeFilter={nodeFilter}
            onNodeFilterChange={setNodeFilter}
            textFilter={textFilter}
            onTextFilterChange={setTextFilter}
          />
        )}
        {viewMode === "human" && (
          <input
            type="text"
            placeholder="Search…"
            value={textFilter}
            onChange={(e) => setTextFilter(e.target.value)}
            className="px-2 py-0.5 text-[11px] bg-paper border border-hair rounded-DEFAULT flex-1 min-w-[80px] focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
          />
        )}

        <label className="flex items-center gap-1 text-[11px] text-muted shrink-0 font-mono uppercase tracking-[0.08em]">
          <input
            type="checkbox"
            checked={autoScroll}
            onChange={(e) => setAutoScroll(e.target.checked)}
            className="accent-terracotta"
          />
          Auto
        </label>
        <button
          onClick={clearLogs}
          className="p-1 hover:bg-cream text-muted hover:text-terracotta shrink-0 transition-colors"
          title="Clear logs"
        >
          <Trash2 className="w-3 h-3" />
        </button>
      </div>

      <div className="flex-1 bg-paper" ref={containerRef}>
        <List
          ref={listRef}
          height={containerHeight}
          width="100%"
          itemCount={merged.length}
          itemSize={22}
          overscanCount={20}
        >
          {Row}
        </List>
      </div>
    </div>
  );
}
