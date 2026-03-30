import { useState, useMemo } from "react";
import {
  ChevronRight,
  ChevronDown,
  Copy,
  Check,
  Braces,
  Hash,
  Type,
  ToggleLeft,
  List,
  Clock,
} from "lucide-react";
import clsx from "clsx";

// ── Sparkline: tiny inline chart of recent numeric values ──────────────────

function Sparkline({ values, width = 60, height = 16 }: { values: number[]; width?: number; height?: number }) {
  if (values.length < 2) return null;
  const min = Math.min(...values);
  const max = Math.max(...values);
  const range = max - min || 1;
  const points = values
    .map((v, i) => {
      const x = (i / (values.length - 1)) * width;
      const y = height - ((v - min) / range) * (height - 2) - 1;
      return `${x},${y}`;
    })
    .join(" ");

  return (
    <svg width={width} height={height} className="inline-block ml-1.5 align-middle">
      <polyline
        points={points}
        fill="none"
        stroke="#3b82f6"
        strokeWidth="1.2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

// ── Type-specific renderers ────────────────────────────────────────────────

function JointStateRenderer({ msg }: { msg: any }) {
  const names: string[] = msg.name || [];
  const positions: number[] = msg.position || [];
  const velocities: number[] = msg.velocity || [];

  return (
    <div className="space-y-1">
      <SectionLabel label="Joint States" count={names.length} />
      <div className="grid grid-cols-[1fr_auto_auto_auto] gap-x-3 gap-y-0.5 text-[10px]">
        <span className="text-gray-500 font-medium">Joint</span>
        <span className="text-gray-500 font-medium text-right">Position</span>
        <span className="text-gray-500 font-medium text-right">Degrees</span>
        <span className="text-gray-500 font-medium text-right">Velocity</span>
        {names.map((name, i) => {
          const pos = positions[i] ?? 0;
          const vel = velocities[i] ?? 0;
          const deg = (pos * 180) / Math.PI;
          const barWidth = Math.min(Math.abs(pos) / 3.14 * 100, 100);
          return (
            <div key={name} className="contents">
              <span className="text-gray-300 font-mono truncate">{name}</span>
              <div className="text-right flex items-center justify-end gap-1">
                <div className="w-12 h-1.5 bg-gray-800 rounded-full overflow-hidden">
                  <div
                    className={clsx("h-full rounded-full", pos >= 0 ? "bg-blue-500" : "bg-orange-500")}
                    style={{ width: `${barWidth}%` }}
                  />
                </div>
                <span className="text-blue-300 font-mono w-16 text-right">{pos.toFixed(3)}</span>
              </div>
              <span className="text-gray-400 font-mono text-right">{deg.toFixed(1)}&deg;</span>
              <span className={clsx("font-mono text-right", Math.abs(vel) > 0.01 ? "text-yellow-400" : "text-gray-600")}>
                {vel.toFixed(3)}
              </span>
            </div>
          );
        })}
      </div>
    </div>
  );
}

function TFMessageRenderer({ msg }: { msg: any }) {
  const transforms: any[] = msg.transforms || [];
  return (
    <div className="space-y-1.5">
      <SectionLabel label="Transforms" count={transforms.length} />
      {transforms.map((tf: any, i: number) => {
        const t = tf.transform?.translation || {};
        const r = tf.transform?.rotation || {};
        return (
          <div key={i} className="p-2 rounded border border-gray-800/50 bg-gray-900/30 space-y-1">
            <div className="flex items-center gap-1 text-[10px]">
              <span className="text-gray-400 font-mono">{tf.header?.frame_id}</span>
              <span className="text-gray-600">&rarr;</span>
              <span className="text-blue-300 font-mono">{tf.child_frame_id}</span>
            </div>
            <div className="grid grid-cols-3 gap-2 text-[9px]">
              <div>
                <span className="text-gray-500">x: </span>
                <span className="text-red-400 font-mono">{(t.x ?? 0).toFixed(4)}</span>
              </div>
              <div>
                <span className="text-gray-500">y: </span>
                <span className="text-green-400 font-mono">{(t.y ?? 0).toFixed(4)}</span>
              </div>
              <div>
                <span className="text-gray-500">z: </span>
                <span className="text-blue-400 font-mono">{(t.z ?? 0).toFixed(4)}</span>
              </div>
            </div>
            <div className="grid grid-cols-4 gap-2 text-[9px]">
              <div><span className="text-gray-600">qx:</span> <span className="text-gray-400 font-mono">{(r.x ?? 0).toFixed(3)}</span></div>
              <div><span className="text-gray-600">qy:</span> <span className="text-gray-400 font-mono">{(r.y ?? 0).toFixed(3)}</span></div>
              <div><span className="text-gray-600">qz:</span> <span className="text-gray-400 font-mono">{(r.z ?? 0).toFixed(3)}</span></div>
              <div><span className="text-gray-600">qw:</span> <span className="text-gray-400 font-mono">{(r.w ?? 0).toFixed(3)}</span></div>
            </div>
          </div>
        );
      })}
    </div>
  );
}

function TaskStateRenderer({ msg }: { msg: any }) {
  const statusColors: Record<string, string> = {
    IDLE: "bg-gray-700 text-gray-300",
    RUNNING: "bg-blue-600 text-blue-100",
    SUCCESS: "bg-green-700 text-green-100",
    FAILURE: "bg-red-700 text-red-100",
    CANCELLED: "bg-yellow-700 text-yellow-100",
  };
  const pct = Math.round((msg.progress ?? 0) * 100);

  return (
    <div className="space-y-2">
      <div className="flex items-center gap-2">
        <span className="text-sm font-semibold text-gray-200">{msg.task_name || "—"}</span>
        <span className={clsx("px-1.5 py-0.5 rounded text-[9px] font-bold uppercase", statusColors[msg.status] || statusColors.IDLE)}>
          {msg.status}
        </span>
        <span className="text-[10px] text-gray-500 font-mono">{msg.task_id}</span>
      </div>

      {/* Progress bar */}
      <div>
        <div className="flex justify-between text-[10px] text-gray-400 mb-0.5">
          <span>Progress</span>
          <span>{pct}%</span>
        </div>
        <div className="h-2 bg-gray-800 rounded-full overflow-hidden">
          <div
            className={clsx("h-full rounded-full transition-all", msg.status === "FAILURE" ? "bg-red-500" : "bg-blue-500")}
            style={{ width: `${pct}%` }}
          />
        </div>
      </div>

      <div className="grid grid-cols-2 gap-2 text-[10px]">
        <KvRow label="Current skill" value={msg.current_skill} />
        <KvRow label="Current node" value={msg.current_bt_node} />
        <KvRow label="Elapsed" value={`${(msg.elapsed_sec ?? 0).toFixed(1)}s`} />
        <KvRow label="Completed" value={(msg.completed_skills ?? []).length} />
      </div>

      {(msg.completed_skills ?? []).length > 0 && (
        <div>
          <span className="text-[9px] text-gray-500 uppercase font-medium">Completed</span>
          <div className="flex flex-wrap gap-1 mt-0.5">
            {msg.completed_skills.map((s: string, i: number) => (
              <span key={i} className="text-[9px] px-1 py-0 rounded bg-green-900/30 text-green-300 border border-green-800/40">{s}</span>
            ))}
          </div>
        </div>
      )}

      {msg.error_message && (
        <div className="p-1.5 rounded bg-red-950/30 border border-red-800/40 text-[10px] text-red-300">
          {msg.error_skill && <span className="font-bold">{msg.error_skill}: </span>}
          {msg.error_message}
        </div>
      )}
    </div>
  );
}

function DiagnosticArrayRenderer({ msg }: { msg: any }) {
  const statuses: any[] = msg.status || [];
  const levelIcon = (level: number) => {
    if (level === 0) return <span className="w-2 h-2 rounded-full bg-green-500 inline-block" />;
    if (level === 1) return <span className="w-2 h-2 rounded-full bg-yellow-500 inline-block" />;
    if (level === 2) return <span className="w-2 h-2 rounded-full bg-red-500 inline-block" />;
    return <span className="w-2 h-2 rounded-full bg-gray-500 inline-block" />;
  };

  return (
    <div className="space-y-1">
      <SectionLabel label="Diagnostics" count={statuses.length} />
      {statuses.map((s: any, i: number) => (
        <div key={i} className="flex items-start gap-2 py-0.5 text-[10px]">
          {levelIcon(s.level)}
          <div className="flex-1 min-w-0">
            <span className="text-gray-300 font-medium">{s.name}</span>
            {s.message && <span className="text-gray-500 ml-1.5">— {s.message}</span>}
            {(s.values ?? []).length > 0 && (
              <div className="mt-0.5 space-y-0">
                {s.values.map((v: any, j: number) => (
                  <div key={j} className="text-[9px] text-gray-500 pl-2">
                    <span className="text-gray-400">{v.key}:</span> {v.value}
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      ))}
    </div>
  );
}

function PoseStampedRenderer({ msg }: { msg: any }) {
  const p = msg.pose?.position || msg.position || {};
  const o = msg.pose?.orientation || msg.orientation || {};
  return (
    <div className="space-y-1">
      <SectionLabel label="Pose" />
      {msg.header?.frame_id && (
        <div className="text-[10px] text-gray-500">frame: <span className="text-gray-300 font-mono">{msg.header.frame_id}</span></div>
      )}
      <div className="grid grid-cols-3 gap-2 text-[10px]">
        <div><span className="text-gray-500">x: </span><span className="text-red-400 font-mono">{(p.x ?? 0).toFixed(4)}</span></div>
        <div><span className="text-gray-500">y: </span><span className="text-green-400 font-mono">{(p.y ?? 0).toFixed(4)}</span></div>
        <div><span className="text-gray-500">z: </span><span className="text-blue-400 font-mono">{(p.z ?? 0).toFixed(4)}</span></div>
      </div>
      {(o.x !== undefined || o.w !== undefined) && (
        <div className="grid grid-cols-4 gap-2 text-[9px]">
          <div><span className="text-gray-600">qx:</span> <span className="text-gray-400 font-mono">{(o.x ?? 0).toFixed(3)}</span></div>
          <div><span className="text-gray-600">qy:</span> <span className="text-gray-400 font-mono">{(o.y ?? 0).toFixed(3)}</span></div>
          <div><span className="text-gray-600">qz:</span> <span className="text-gray-400 font-mono">{(o.z ?? 0).toFixed(3)}</span></div>
          <div><span className="text-gray-600">qw:</span> <span className="text-gray-400 font-mono">{(o.w ?? 0).toFixed(3)}</span></div>
        </div>
      )}
    </div>
  );
}

function StringRenderer({ msg }: { msg: any }) {
  const data = msg.data ?? "";
  // Try to parse as JSON for pretty display
  try {
    const parsed = JSON.parse(data);
    return (
      <div>
        <SectionLabel label="String (JSON)" />
        <StructuredTreeView data={parsed} path="" />
      </div>
    );
  } catch {
    return (
      <div>
        <SectionLabel label="String" />
        <div className="text-xs text-gray-300 bg-gray-900/50 rounded p-2 whitespace-pre-wrap break-all">{data}</div>
      </div>
    );
  }
}

// ── Structured tree view for generic messages ──────────────────────────────

function StructuredTreeView({ data, path, depth = 0 }: { data: any; path: string; depth?: number }) {
  if (data === null || data === undefined) {
    return <span className="text-gray-600 italic">null</span>;
  }

  if (typeof data === "number") {
    return <NumericValue value={data} />;
  }

  if (typeof data === "boolean") {
    return (
      <span className={clsx("font-mono text-[10px]", data ? "text-green-400" : "text-red-400")}>
        {data.toString()}
      </span>
    );
  }

  if (typeof data === "string") {
    return <span className="text-yellow-300 text-[10px] font-mono">"{data}"</span>;
  }

  if (Array.isArray(data)) {
    return <ArrayView data={data} path={path} depth={depth} />;
  }

  if (typeof data === "object") {
    return <ObjectView data={data} path={path} depth={depth} />;
  }

  return <span className="text-gray-400 text-[10px] font-mono">{String(data)}</span>;
}

function NumericValue({ value }: { value: number }) {
  const isInt = Number.isInteger(value);
  const formatted = isInt ? value.toString() : value.toFixed(6).replace(/0+$/, "0");
  return <span className="text-cyan-400 text-[10px] font-mono">{formatted}</span>;
}

function ObjectView({ data, path, depth }: { data: Record<string, any>; path: string; depth: number }) {
  const [collapsed, setCollapsed] = useState(depth > 2);
  const keys = Object.keys(data);

  // Render small objects (≤3 keys, all primitives) inline
  const isSmall = keys.length <= 3 && keys.every((k) => typeof data[k] !== "object" || data[k] === null);
  if (isSmall && depth > 0) {
    return (
      <span className="text-[10px]">
        <span className="text-gray-600">{"{ "}</span>
        {keys.map((k, i) => (
          <span key={k}>
            <span className="text-gray-500">{k}: </span>
            <StructuredTreeView data={data[k]} path={`${path}.${k}`} depth={depth + 1} />
            {i < keys.length - 1 && <span className="text-gray-600">, </span>}
          </span>
        ))}
        <span className="text-gray-600">{" }"}</span>
      </span>
    );
  }

  return (
    <div>
      <button
        onClick={() => setCollapsed(!collapsed)}
        className="flex items-center gap-0.5 text-gray-500 hover:text-gray-300"
      >
        {collapsed ? <ChevronRight className="w-3 h-3" /> : <ChevronDown className="w-3 h-3" />}
        <Braces className="w-3 h-3" />
        <span className="text-[9px]">{keys.length} fields</span>
      </button>
      {!collapsed && (
        <div className="ml-3 border-l border-gray-800 pl-2 space-y-0.5 mt-0.5">
          {keys.map((key) => (
            <div key={key} className="flex items-start gap-1">
              <span className="text-gray-400 text-[10px] font-mono shrink-0">{key}:</span>
              <StructuredTreeView data={data[key]} path={`${path}.${key}`} depth={depth + 1} />
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

function ArrayView({ data, path, depth }: { data: any[]; path: string; depth: number }) {
  const [collapsed, setCollapsed] = useState(depth > 2 && data.length > 5);

  // Render numeric arrays as a compact bar/list
  if (data.length > 0 && data.every((v) => typeof v === "number")) {
    return <NumericArrayView data={data} />;
  }

  // Render string arrays as tags
  if (data.length > 0 && data.every((v) => typeof v === "string")) {
    return (
      <div className="flex flex-wrap gap-1">
        {data.map((v, i) => (
          <span key={i} className="text-[9px] px-1 py-0 rounded bg-gray-800 text-yellow-300 font-mono">
            {v}
          </span>
        ))}
      </div>
    );
  }

  return (
    <div>
      <button
        onClick={() => setCollapsed(!collapsed)}
        className="flex items-center gap-0.5 text-gray-500 hover:text-gray-300"
      >
        {collapsed ? <ChevronRight className="w-3 h-3" /> : <ChevronDown className="w-3 h-3" />}
        <List className="w-3 h-3" />
        <span className="text-[9px]">{data.length} items</span>
      </button>
      {!collapsed && (
        <div className="ml-3 border-l border-gray-800 pl-2 space-y-0.5 mt-0.5">
          {data.map((item, i) => (
            <div key={i} className="flex items-start gap-1">
              <span className="text-gray-600 text-[9px] font-mono shrink-0">[{i}]</span>
              <StructuredTreeView data={item} path={`${path}[${i}]`} depth={depth + 1} />
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

function NumericArrayView({ data }: { data: number[] }) {
  const min = Math.min(...data);
  const max = Math.max(...data);
  const range = max - min || 1;

  return (
    <div className="space-y-0.5">
      <div className="flex items-center gap-1 text-[9px] text-gray-500">
        <span>[{data.length}]</span>
        <span>min: <span className="text-cyan-400 font-mono">{min.toFixed(3)}</span></span>
        <span>max: <span className="text-cyan-400 font-mono">{max.toFixed(3)}</span></span>
      </div>
      <div className="flex gap-px">
        {data.map((v, i) => {
          const h = Math.max(((v - min) / range) * 20 + 2, 2);
          return (
            <div key={i} className="flex flex-col items-center justify-end" style={{ height: 22 }} title={`[${i}] ${v.toFixed(4)}`}>
              <div className="w-2 rounded-t bg-blue-500/60 hover:bg-blue-400 transition-colors" style={{ height: h }} />
            </div>
          );
        })}
      </div>
    </div>
  );
}

// ── Utilities ───────────────────────────────────────────────────────────────

function SectionLabel({ label, count }: { label: string; count?: number }) {
  return (
    <div className="flex items-center gap-1.5 text-[9px] text-gray-500 font-bold uppercase tracking-wider mb-1">
      {label}
      {count !== undefined && <span className="text-gray-600">({count})</span>}
    </div>
  );
}

function KvRow({ label, value }: { label: string; value: any }) {
  return (
    <div className="text-[10px]">
      <span className="text-gray-500">{label}: </span>
      <span className="text-gray-300 font-mono">{value ?? "—"}</span>
    </div>
  );
}

// ── Main entry point: picks the best renderer for the message type ─────────

interface Props {
  type: string;
  msg: any;
  messages: any[]; // history for sparklines
}

/** Map message type to the best specialized renderer, falling back to tree view */
export default function MessageRenderer({ type, msg, messages }: Props) {
  // Match by ROS type string
  const shortType = type.split("/").pop() || type;

  if (shortType === "JointState") return <JointStateRenderer msg={msg} />;
  if (shortType === "TFMessage") return <TFMessageRenderer msg={msg} />;
  if (shortType === "TaskState") return <TaskStateRenderer msg={msg} />;
  if (shortType === "DiagnosticArray") return <DiagnosticArrayRenderer msg={msg} />;
  if (shortType === "String") return <StringRenderer msg={msg} />;

  // Check for pose-like structure
  if (msg.pose?.position || (msg.position && msg.orientation)) {
    return <PoseStampedRenderer msg={msg} />;
  }

  // Fallback: structured tree view
  return <StructuredTreeView data={msg} path="" />;
}
