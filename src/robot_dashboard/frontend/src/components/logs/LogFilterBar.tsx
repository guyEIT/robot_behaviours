import { type LogLevelName } from "../../types/ros";
import clsx from "clsx";

const LEVELS: Array<{ name: LogLevelName; color: string }> = [
  { name: "DEBUG", color: "text-gray-400 border-gray-600" },
  { name: "INFO", color: "text-blue-400 border-blue-600" },
  { name: "WARN", color: "text-yellow-400 border-yellow-600" },
  { name: "ERROR", color: "text-red-400 border-red-600" },
  { name: "FATAL", color: "text-red-300 border-red-500 bg-red-950/30" },
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
      {/* Severity toggles */}
      {LEVELS.map((l) => (
        <button
          key={l.name}
          onClick={() => onToggleLevel(l.name)}
          className={clsx(
            "px-1.5 py-0.5 rounded text-[10px] font-bold border transition-all",
            activeLevels.has(l.name)
              ? l.color
              : "text-gray-600 border-gray-800"
          )}
        >
          {l.name}
        </button>
      ))}

      {/* Node name filter */}
      <input
        type="text"
        placeholder="Node..."
        value={nodeFilter}
        onChange={(e) => onNodeFilterChange(e.target.value)}
        className="px-2 py-0.5 text-[10px] bg-gray-800 border border-gray-700 rounded w-24 focus:border-blue-500 focus:outline-none text-gray-200 placeholder-gray-500"
      />

      {/* Text search */}
      <input
        type="text"
        placeholder="Search..."
        value={textFilter}
        onChange={(e) => onTextFilterChange(e.target.value)}
        className="px-2 py-0.5 text-[10px] bg-gray-800 border border-gray-700 rounded flex-1 min-w-[80px] focus:border-blue-500 focus:outline-none text-gray-200 placeholder-gray-500"
      />
    </div>
  );
}
