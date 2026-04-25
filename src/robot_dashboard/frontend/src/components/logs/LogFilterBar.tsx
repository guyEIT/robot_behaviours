import { type LogLevelName } from "../../types/ros";
import clsx from "clsx";

const LEVELS: Array<{ name: LogLevelName; color: string }> = [
  { name: "DEBUG", color: "text-muted border-hair" },
  { name: "INFO", color: "text-running border-running" },
  { name: "WARN", color: "text-terracotta border-terracotta" },
  { name: "ERROR", color: "text-err border-err" },
  { name: "FATAL", color: "text-err border-err bg-err-soft" },
];

interface Props {
  activeLevels: Set<LogLevelName>;
  onToggleLevel: (level: LogLevelName) => void;
  nodeFilter: string;
  onNodeFilterChange: (val: string) => void;
  textFilter: string;
  onTextFilterChange: (val: string) => void;
}

export default function LogFilterBar({
  activeLevels,
  onToggleLevel,
  nodeFilter,
  onNodeFilterChange,
  textFilter,
  onTextFilterChange,
}: Props) {
  return (
    <div className="flex items-center gap-2 flex-wrap">
      {LEVELS.map((l) => (
        <button
          key={l.name}
          onClick={() => onToggleLevel(l.name)}
          className={clsx(
            "px-2 py-0.5 font-mono text-[10px] font-semibold border tracking-[0.08em] transition-colors",
            activeLevels.has(l.name) ? l.color : "text-muted-2 border-hair-soft hover:border-hair",
          )}
        >
          {l.name}
        </button>
      ))}

      <input
        type="text"
        placeholder="Node…"
        value={nodeFilter}
        onChange={(e) => onNodeFilterChange(e.target.value)}
        className="px-2 py-0.5 text-[11px] bg-paper border border-hair rounded-DEFAULT w-24 focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
      />

      <input
        type="text"
        placeholder="Search…"
        value={textFilter}
        onChange={(e) => onTextFilterChange(e.target.value)}
        className="px-2 py-0.5 text-[11px] bg-paper border border-hair rounded-DEFAULT flex-1 min-w-[80px] focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
      />
    </div>
  );
}
