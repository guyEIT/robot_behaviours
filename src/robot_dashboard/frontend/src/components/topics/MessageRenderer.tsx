import { useState } from "react";
import { ChevronRight, ChevronDown, Braces, List } from "lucide-react";
import clsx from "clsx";
import { Eyebrow, Chip } from "../ui";
import type { ChipState } from "../ui";

function JointStateRenderer({ msg }: { msg: any }) {
  const names: string[] = msg.name || [];
  const positions: number[] = msg.position || [];
  const velocities: number[] = msg.velocity || [];

  return (
    <div className="space-y-2">
      <SectionLabel label="Joint States" count={names.length} />
      <table className="w-full text-[11px] border-collapse">
        <thead>
          <tr>
            <th className="text-left px-2 py-1 bg-cream font-mono text-[10px] uppercase tracking-[0.1em] text-terracotta border-b border-hair">Joint</th>
            <th className="text-right px-2 py-1 bg-cream font-mono text-[10px] uppercase tracking-[0.1em] text-terracotta border-b border-hair">Position</th>
            <th className="text-right px-2 py-1 bg-cream font-mono text-[10px] uppercase tracking-[0.1em] text-terracotta border-b border-hair">Degrees</th>
            <th className="text-right px-2 py-1 bg-cream font-mono text-[10px] uppercase tracking-[0.1em] text-terracotta border-b border-hair">Velocity</th>
          </tr>
        </thead>
        <tbody>
          {names.map((name, i) => {
            const pos = positions[i] ?? 0;
            const vel = velocities[i] ?? 0;
            const deg = (pos * 180) / Math.PI;
            const barWidth = Math.min((Math.abs(pos) / 3.14) * 100, 100);
            return (
              <tr key={name} className="border-b border-hair-soft hover:bg-cream">
                <td className="px-2 py-1 font-mono text-ink-soft tracking-[0.04em] truncate">{name}</td>
                <td className="px-2 py-1 text-right">
                  <div className="flex items-center justify-end gap-2">
                    <div className="w-12 h-1 bg-stone overflow-hidden">
                      <div className="h-full bg-terracotta" style={{ width: `${barWidth}%` }} />
                    </div>
                    <span className="text-ink font-mono w-16 text-right tracking-[0.04em]">{pos.toFixed(3)}</span>
                  </div>
                </td>
                <td className="px-2 py-1 font-mono text-ink-soft text-right tracking-[0.04em]">{deg.toFixed(1)}°</td>
                <td className={clsx("px-2 py-1 font-mono text-right tracking-[0.04em]", Math.abs(vel) > 0.01 ? "text-terracotta" : "text-muted")}>
                  {vel.toFixed(3)}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

function TFMessageRenderer({ msg }: { msg: any }) {
  const transforms: any[] = msg.transforms || [];
  return (
    <div className="space-y-2">
      <SectionLabel label="Transforms" count={transforms.length} />
      {transforms.map((tf: any, i: number) => {
        const t = tf.transform?.translation || {};
        const r = tf.transform?.rotation || {};
        return (
          <div key={i} className="p-3 border border-hair bg-paper space-y-1">
            <div className="flex items-center gap-2 text-[11px]">
              <span className="text-ink-soft font-mono tracking-[0.04em]">{tf.header?.frame_id}</span>
              <span className="text-muted">→</span>
              <span className="text-terracotta font-mono tracking-[0.04em]">{tf.child_frame_id}</span>
            </div>
            <div className="grid grid-cols-3 gap-2 text-[10px]">
              <KvPair label="x" value={(t.x ?? 0).toFixed(4)} accent="err" />
              <KvPair label="y" value={(t.y ?? 0).toFixed(4)} accent="ok" />
              <KvPair label="z" value={(t.z ?? 0).toFixed(4)} accent="running" />
            </div>
            <div className="grid grid-cols-4 gap-2 text-[10px]">
              <KvPair label="qx" value={(r.x ?? 0).toFixed(3)} />
              <KvPair label="qy" value={(r.y ?? 0).toFixed(3)} />
              <KvPair label="qz" value={(r.z ?? 0).toFixed(3)} />
              <KvPair label="qw" value={(r.w ?? 0).toFixed(3)} />
            </div>
          </div>
        );
      })}
    </div>
  );
}

const STATUS_TO_CHIP: Record<string, ChipState> = {
  IDLE: "idle",
  RUNNING: "running",
  SUCCESS: "done",
  FAILURE: "failed",
  CANCELLED: "neutral",
};

function TaskStateRenderer({ msg }: { msg: any }) {
  const pct = Math.round((msg.progress ?? 0) * 100);
  const fill = msg.status === "FAILURE" ? "bg-err" : msg.status === "SUCCESS" ? "bg-ok" : "bg-terracotta";
  return (
    <div className="space-y-3">
      <div className="flex items-center gap-2 flex-wrap">
        <span className="text-[14px] font-medium text-ink">{msg.task_name || "—"}</span>
        <Chip state={STATUS_TO_CHIP[msg.status] ?? "idle"} showDot>
          {msg.status}
        </Chip>
        <span className="text-[10px] text-muted font-mono tracking-[0.04em]">{msg.task_id}</span>
      </div>

      <div>
        <div className="flex justify-between text-[11px] text-muted mb-1 font-mono tracking-[0.04em]">
          <span>Progress</span>
          <span>{pct}%</span>
        </div>
        <div className="h-1.5 bg-stone overflow-hidden">
          <div className={clsx("h-full transition-all", fill)} style={{ width: `${pct}%` }} />
        </div>
      </div>

      <div className="grid grid-cols-2 gap-2 text-[11px]">
        <KvRow label="Current skill" value={msg.current_skill} />
        <KvRow label="Current node" value={msg.current_bt_node} />
        <KvRow label="Elapsed" value={`${(msg.elapsed_sec ?? 0).toFixed(1)}s`} />
        <KvRow label="Completed" value={(msg.completed_skills ?? []).length} />
      </div>

      {(msg.completed_skills ?? []).length > 0 && (
        <div>
          <Eyebrow size="sm" tone="muted" className="block mb-1">Completed</Eyebrow>
          <div className="flex flex-wrap gap-1">
            {msg.completed_skills.map((s: string, i: number) => (
              <span key={i} className="font-mono text-[10px] px-1.5 py-0.5 border border-ok text-ok tracking-[0.04em]">
                {s}
              </span>
            ))}
          </div>
        </div>
      )}

      {msg.error_message && (
        <div className="p-3 border border-err bg-err-soft text-[12px] text-err">
          {msg.error_skill && <span className="font-semibold">{msg.error_skill}: </span>}
          {msg.error_message}
        </div>
      )}
    </div>
  );
}

function DiagnosticArrayRenderer({ msg }: { msg: any }) {
  const statuses: any[] = msg.status || [];
  const dot = (level: number) => {
    if (level === 0) return "bg-ok";
    if (level === 1) return "bg-terracotta";
    if (level === 2) return "bg-err";
    return "bg-muted";
  };

  return (
    <div className="space-y-1">
      <SectionLabel label="Diagnostics" count={statuses.length} />
      {statuses.map((s: any, i: number) => (
        <div key={i} className="flex items-start gap-2 py-1 text-[11px]">
          <span className={clsx("w-1.5 h-1.5 rounded-full inline-block mt-1", dot(s.level))} />
          <div className="flex-1 min-w-0">
            <span className="text-ink font-medium">{s.name}</span>
            {s.message && <span className="text-muted ml-2">— {s.message}</span>}
            {(s.values ?? []).length > 0 && (
              <div className="mt-0.5 space-y-0">
                {s.values.map((v: any, j: number) => (
                  <div key={j} className="text-[10px] text-muted pl-2 font-mono tracking-[0.04em]">
                    <span className="text-ink-soft">{v.key}:</span> {v.value}
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
    <div className="space-y-2">
      <SectionLabel label="Pose" />
      {msg.header?.frame_id && (
        <div className="text-[11px] text-muted">
          frame: <span className="text-ink-soft font-mono tracking-[0.04em]">{msg.header.frame_id}</span>
        </div>
      )}
      <div className="grid grid-cols-3 gap-2 text-[11px]">
        <KvPair label="x" value={(p.x ?? 0).toFixed(4)} accent="err" />
        <KvPair label="y" value={(p.y ?? 0).toFixed(4)} accent="ok" />
        <KvPair label="z" value={(p.z ?? 0).toFixed(4)} accent="running" />
      </div>
      {(o.x !== undefined || o.w !== undefined) && (
        <div className="grid grid-cols-4 gap-2 text-[10px]">
          <KvPair label="qx" value={(o.x ?? 0).toFixed(3)} />
          <KvPair label="qy" value={(o.y ?? 0).toFixed(3)} />
          <KvPair label="qz" value={(o.z ?? 0).toFixed(3)} />
          <KvPair label="qw" value={(o.w ?? 0).toFixed(3)} />
        </div>
      )}
    </div>
  );
}

function StringRenderer({ msg }: { msg: any }) {
  const data = msg.data ?? "";
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
      <div className="space-y-1">
        <SectionLabel label="String" />
        <div className="text-[12px] text-ink-soft bg-cream-deep border border-hair p-3 whitespace-pre-wrap break-all font-mono tracking-[0.02em]">
          {data}
        </div>
      </div>
    );
  }
}

function StructuredTreeView({ data, path, depth = 0 }: { data: any; path: string; depth?: number }) {
  if (data === null || data === undefined) {
    return <span className="text-muted italic">null</span>;
  }

  if (typeof data === "number") {
    return <NumericValue value={data} />;
  }

  if (typeof data === "boolean") {
    return (
      <span className={clsx("font-mono text-[11px] tracking-[0.04em]", data ? "text-ok" : "text-err")}>
        {data.toString()}
      </span>
    );
  }

  if (typeof data === "string") {
    return <span className="text-terracotta text-[11px] font-mono tracking-[0.04em]">"{data}"</span>;
  }

  if (Array.isArray(data)) {
    return <ArrayView data={data} path={path} depth={depth} />;
  }

  if (typeof data === "object") {
    return <ObjectView data={data} path={path} depth={depth} />;
  }

  return <span className="text-ink-soft text-[11px] font-mono">{String(data)}</span>;
}

function NumericValue({ value }: { value: number }) {
  const isInt = Number.isInteger(value);
  const formatted = isInt ? value.toString() : value.toFixed(6).replace(/0+$/, "0");
  return <span className="text-running text-[11px] font-mono tracking-[0.04em]">{formatted}</span>;
}

function ObjectView({ data, path, depth }: { data: Record<string, any>; path: string; depth: number }) {
  const [collapsed, setCollapsed] = useState(depth > 2);
  const keys = Object.keys(data);

  const isSmall = keys.length <= 3 && keys.every((k) => typeof data[k] !== "object" || data[k] === null);
  if (isSmall && depth > 0) {
    return (
      <span className="text-[11px]">
        <span className="text-muted">{"{ "}</span>
        {keys.map((k, i) => (
          <span key={k}>
            <span className="text-muted">{k}: </span>
            <StructuredTreeView data={data[k]} path={`${path}.${k}`} depth={depth + 1} />
            {i < keys.length - 1 && <span className="text-muted">, </span>}
          </span>
        ))}
        <span className="text-muted">{" }"}</span>
      </span>
    );
  }

  return (
    <div>
      <button
        onClick={() => setCollapsed(!collapsed)}
        className="flex items-center gap-1 text-muted hover:text-ink-soft transition-colors"
      >
        {collapsed ? <ChevronRight className="w-3 h-3" /> : <ChevronDown className="w-3 h-3" />}
        <Braces className="w-3 h-3" />
        <span className="text-[10px] font-mono uppercase tracking-[0.06em]">{keys.length} fields</span>
      </button>
      {!collapsed && (
        <div className="ml-3 border-l border-hair-soft pl-3 space-y-0.5 mt-1">
          {keys.map((key) => (
            <div key={key} className="flex items-start gap-1.5">
              <span className="text-ink-soft text-[11px] font-mono shrink-0 tracking-[0.04em]">{key}:</span>
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

  if (data.length > 0 && data.every((v) => typeof v === "number")) {
    return <NumericArrayView data={data} />;
  }

  if (data.length > 0 && data.every((v) => typeof v === "string")) {
    return (
      <div className="flex flex-wrap gap-1">
        {data.map((v, i) => (
          <span
            key={i}
            className="text-[10px] px-1.5 py-0.5 border border-hair text-terracotta font-mono tracking-[0.04em]"
          >
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
        className="flex items-center gap-1 text-muted hover:text-ink-soft transition-colors"
      >
        {collapsed ? <ChevronRight className="w-3 h-3" /> : <ChevronDown className="w-3 h-3" />}
        <List className="w-3 h-3" />
        <span className="text-[10px] font-mono uppercase tracking-[0.06em]">{data.length} items</span>
      </button>
      {!collapsed && (
        <div className="ml-3 border-l border-hair-soft pl-3 space-y-0.5 mt-1">
          {data.map((item, i) => (
            <div key={i} className="flex items-start gap-1.5">
              <span className="text-muted text-[10px] font-mono shrink-0 tracking-[0.04em]">[{i}]</span>
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
    <div className="space-y-1">
      <div className="flex items-center gap-2 text-[10px] text-muted font-mono tracking-[0.04em]">
        <span>[{data.length}]</span>
        <span>min: <span className="text-running">{min.toFixed(3)}</span></span>
        <span>max: <span className="text-running">{max.toFixed(3)}</span></span>
      </div>
      <div className="flex gap-px items-end" style={{ height: 24 }}>
        {data.map((v, i) => {
          const h = Math.max(((v - min) / range) * 22 + 2, 2);
          return (
            <div
              key={i}
              className="w-2 bg-terracotta hover:bg-terracotta-hover transition-colors"
              style={{ height: h }}
              title={`[${i}] ${v.toFixed(4)}`}
            />
          );
        })}
      </div>
    </div>
  );
}

function SectionLabel({ label, count }: { label: string; count?: number }) {
  return (
    <div className="flex items-center gap-1.5 mb-2">
      <Eyebrow size="sm">{label}</Eyebrow>
      {count !== undefined && (
        <span className="text-[10px] text-muted font-mono tracking-[0.04em]">({count})</span>
      )}
    </div>
  );
}

function KvRow({ label, value }: { label: string; value: any }) {
  return (
    <div className="text-[11px]">
      <span className="text-muted">{label}: </span>
      <span className="text-ink-soft font-mono tracking-[0.04em]">{value ?? "—"}</span>
    </div>
  );
}

function KvPair({
  label,
  value,
  accent,
}: {
  label: string;
  value: string;
  accent?: "err" | "ok" | "running";
}) {
  const accentClass =
    accent === "err"
      ? "text-err"
      : accent === "ok"
        ? "text-ok"
        : accent === "running"
          ? "text-running"
          : "text-ink-soft";
  return (
    <div>
      <span className="text-muted">{label}: </span>
      <span className={clsx("font-mono tracking-[0.04em]", accentClass)}>{value}</span>
    </div>
  );
}

interface Props {
  type: string;
  msg: any;
  messages: any[];
}

export default function MessageRenderer({ type, msg }: Props) {
  const shortType = type.split("/").pop() || type;

  if (shortType === "JointState") return <JointStateRenderer msg={msg} />;
  if (shortType === "TFMessage") return <TFMessageRenderer msg={msg} />;
  if (shortType === "TaskState") return <TaskStateRenderer msg={msg} />;
  if (shortType === "DiagnosticArray") return <DiagnosticArrayRenderer msg={msg} />;
  if (shortType === "String") return <StringRenderer msg={msg} />;

  if (msg.pose?.position || (msg.position && msg.orientation)) {
    return <PoseStampedRenderer msg={msg} />;
  }

  return <StructuredTreeView data={msg} path="" />;
}
