import { useRef, useEffect, useMemo, useState, useCallback } from "react";
import { FixedSizeList as List } from "react-window";
import { useLogStore } from "../../stores/log-store";
import LogFilterBar from "./LogFilterBar";
import {
  logLevelName,
  LOG_LEVELS,
  type RosLog,
  type LogLevelName,
} from "../../types/ros";
import { Trash2 } from "lucide-react";
import clsx from "clsx";

const LEVEL_COLORS: Record<LogLevelName, string> = {
  DEBUG: "text-gray-500",
  INFO: "text-blue-400",
  WARN: "text-yellow-400",
  ERROR: "text-red-400",
  FATAL: "text-red-300 font-bold",
};

export default function LogStream() {
  const logs = useLogStore((s) => s.logs);
  const clearLogs = useLogStore((s) => s.clearLogs);
  const [activeLevels, setActiveLevels] = useState<Set<LogLevelName>>(
    new Set(["INFO", "WARN", "ERROR", "FATAL"])
  );
  const [nodeFilter, setNodeFilter] = useState("");
  const [textFilter, setTextFilter] = useState("");
  const [autoScroll, setAutoScroll] = useState(true);
  const listRef = useRef<List>(null);

  const toggleLevel = useCallback((level: LogLevelName) => {
    setActiveLevels((prev) => {
      const next = new Set(prev);
      if (next.has(level)) next.delete(level);
      else next.add(level);
      return next;
    });
  }, []);

  const filtered = useMemo(() => {
    const nodeLower = nodeFilter.toLowerCase();
    const textLower = textFilter.toLowerCase();
    return logs.filter((log) => {
      const levelName = logLevelName(log.level);
      if (!activeLevels.has(levelName)) return false;
      if (nodeLower && !log.name.toLowerCase().includes(nodeLower)) return false;
      if (textLower && !log.msg.toLowerCase().includes(textLower)) return false;
      return true;
    });
  }, [logs, activeLevels, nodeFilter, textFilter]);

  useEffect(() => {
    if (autoScroll && listRef.current && filtered.length > 0) {
      listRef.current.scrollToItem(filtered.length - 1);
    }
  }, [filtered.length, autoScroll]);

  const Row = ({ index, style }: { index: number; style: React.CSSProperties }) => {
    const log = filtered[index];
    const levelName = logLevelName(log.level);
    return (
      <div style={style} className="flex items-baseline gap-2 px-3 text-[10px] font-mono hover:bg-gray-800/50">
        <span className="text-gray-600 shrink-0 w-16">
          {new Date(log.stamp.sec * 1000).toLocaleTimeString()}
        </span>
        <span className={clsx("shrink-0 w-10 font-bold", LEVEL_COLORS[levelName])}>
          {levelName}
        </span>
        <span className="text-gray-500 shrink-0 w-28 truncate" title={log.name}>
          {log.name}
        </span>
        <span className="text-gray-300 truncate">{log.msg}</span>
      </div>
    );
  };

  return (
    <div className="flex flex-col h-full">
      <div className="px-3 py-2 border-b border-gray-800 flex items-center gap-2">
        <LogFilterBar
          activeLevels={activeLevels}
          onToggleLevel={toggleLevel}
          nodeFilter={nodeFilter}
          onNodeFilterChange={setNodeFilter}
          textFilter={textFilter}
          onTextFilterChange={setTextFilter}
        />
        <label className="flex items-center gap-1 text-[10px] text-gray-500 shrink-0">
          <input
            type="checkbox"
            checked={autoScroll}
            onChange={(e) => setAutoScroll(e.target.checked)}
            className="rounded-sm"
          />
          Auto
        </label>
        <button
          onClick={clearLogs}
          className="p-1 rounded hover:bg-gray-800 text-gray-500 hover:text-gray-300 shrink-0"
          title="Clear logs"
        >
          <Trash2 className="w-3 h-3" />
        </button>
      </div>

      <div className="flex-1">
        <List
          ref={listRef}
          height={600}
          width="100%"
          itemCount={filtered.length}
          itemSize={22}
          overscanCount={20}
          style={{ height: "100%" }}
        >
          {Row}
        </List>
      </div>
    </div>
  );
}
