import { useState, useCallback, useMemo } from "react";
import {
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  ResponsiveContainer,
  Tooltip,
  Cell,
} from "recharts";
import { useTopicSubscription } from "../../hooks/useTopicSubscription";
import type { JointState } from "../../types/ros";
import { useRobotSelectorStore } from "../../stores/robot-selector-store";
import { Gauge, Activity } from "lucide-react";
import clsx from "clsx";
import { Eyebrow } from "../ui";

type ViewMode = "position" | "velocity" | "effort";

// Sociius warm-leaning palette for joint differentiation
const SOCIIUS_SERIES = [
  "#A96D4B", // terracotta
  "#51748C", // running
  "#2f7d5f", // ok
  "#1E1E1E", // ink-soft
  "#A17258", // terracotta-soft
  "#9a8f83", // muted
  "#9B2C2C", // err
];

function colorForJoint(name: string, idx: number): string {
  return SOCIIUS_SERIES[idx % SOCIIUS_SERIES.length];
}

export default function JointStateViewer() {
  const [jointState, setJointState] = useState<JointState | null>(null);
  const [viewMode, setViewMode] = useState<ViewMode>("position");
  const [hz, setHz] = useState(0);
  const [lastTime, setLastTime] = useState(0);

  const selectedRobotId = useRobotSelectorStore((s) => s.selectedRobotId);
  const jointStatesTopic = selectedRobotId
    ? `/${selectedRobotId}/joint_states`
    : "/joint_states";

  const handleMsg = useCallback(
    (msg: JointState) => {
      setJointState(msg);
      const now = performance.now();
      if (lastTime > 0) {
        const dt = (now - lastTime) / 1000;
        setHz((prev) => Math.round(0.8 * prev + 0.2 * (1 / dt)));
      }
      setLastTime(now);
    },
    [lastTime]
  );

  useTopicSubscription<JointState>(
    jointStatesTopic,
    "sensor_msgs/msg/JointState",
    handleMsg,
    50
  );

  const chartData = useMemo(() => {
    if (!jointState) return [];
    return jointState.name.map((name, i) => {
      const values =
        viewMode === "position"
          ? jointState.position
          : viewMode === "velocity"
            ? jointState.velocity
            : jointState.effort;
      return {
        name: name
          .replace("panda_", "")
          .replace("_joint", "J")
          .replace("finger_joint", "FJ")
          .replace("joint", "J"),
        fullName: name,
        index: i,
        value: values[i] ?? 0,
        degrees:
          viewMode === "position"
            ? ((values[i] ?? 0) * 180) / Math.PI
            : undefined,
      };
    });
  }, [jointState, viewMode]);

  if (!jointState) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-muted bg-cream-deep">
        <Gauge className="w-8 h-8 mb-2 opacity-30" />
        <p className="text-[14px] text-ink-soft">Waiting for /joint_states</p>
        <p className="text-[11px] font-mono uppercase tracking-[0.08em]">
          Joint state publisher not detected
        </p>
      </div>
    );
  }

  return (
    <div className="flex flex-col h-full bg-paper">
      <div className="px-5 py-3 border-b border-hair flex items-center gap-3">
        <Gauge className="w-4 h-4 text-terracotta" />
        <h2 className="text-[14px] font-medium text-ink">Joint States</h2>

        <div className="ml-2 flex gap-1">
          {(["position", "velocity", "effort"] as const).map((m) => (
            <button
              key={m}
              onClick={() => setViewMode(m)}
              className={clsx(
                "px-2 py-0.5 font-mono text-[10px] font-semibold uppercase tracking-[0.08em] transition-colors",
                viewMode === m
                  ? "bg-terracotta-tint text-terracotta"
                  : "text-muted hover:text-ink-soft",
              )}
            >
              {m}
            </button>
          ))}
        </div>

        <div className="ml-auto flex items-center gap-1.5 font-mono text-[10px] text-muted tracking-[0.06em]">
          <Activity className="w-3 h-3" />
          {hz} Hz
        </div>
      </div>

      <div className="flex-1 p-4 bg-cream-deep">
        <ResponsiveContainer width="100%" height="100%">
          <BarChart data={chartData} layout="vertical">
            <CartesianGrid strokeDasharray="2 4" stroke="#EEEEEE" />
            <XAxis
              type="number"
              stroke="#9a8f83"
              fontSize={10}
              tickFormatter={(v) =>
                viewMode === "position" ? `${(v * 180 / Math.PI).toFixed(0)}°` : v.toFixed(2)
              }
            />
            <YAxis
              dataKey="name"
              type="category"
              stroke="#9a8f83"
              fontSize={10}
              width={40}
            />
            <Tooltip
              contentStyle={{
                backgroundColor: "#FFFFFF",
                border: "1px solid #D7D7D7",
                borderRadius: 0,
                fontSize: 11,
                color: "#1E1E1E",
              }}
              formatter={(value: any, _name: any, props: any) => {
                const v = Number(value) || 0;
                const entry = props.payload;
                if (viewMode === "position") {
                  return [
                    `${v.toFixed(4)} rad (${((v * 180) / Math.PI).toFixed(1)}°)`,
                    entry.fullName,
                  ];
                }
                return [v.toFixed(4), entry.fullName];
              }}
            />
            <Bar dataKey="value">
              {chartData.map((entry, i) => (
                <Cell key={entry.fullName} fill={colorForJoint(entry.fullName, i)} />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </div>

      <div className="border-t border-hair overflow-auto max-h-52">
        <table className="w-full text-[12px] border-collapse">
          <thead>
            <tr>
              <th className="text-left px-4 py-2 font-mono text-[10px] uppercase tracking-[0.12em] text-terracotta font-semibold bg-cream border-b border-hair">
                Joint
              </th>
              <th className="text-right px-4 py-2 font-mono text-[10px] uppercase tracking-[0.12em] text-terracotta font-semibold bg-cream border-b border-hair">
                Position (rad)
              </th>
              <th className="text-right px-4 py-2 font-mono text-[10px] uppercase tracking-[0.12em] text-terracotta font-semibold bg-cream border-b border-hair">
                Position (deg)
              </th>
              <th className="text-right px-4 py-2 font-mono text-[10px] uppercase tracking-[0.12em] text-terracotta font-semibold bg-cream border-b border-hair">
                Velocity
              </th>
            </tr>
          </thead>
          <tbody>
            {jointState.name.map((name, i) => (
              <tr key={name} className="border-b border-hair-soft hover:bg-cream transition-colors">
                <td className="px-4 py-1.5 font-mono text-[11.5px] text-ink-soft tracking-[0.04em]">
                  {name}
                </td>
                <td className="text-right px-4 py-1.5 font-mono text-[11.5px] text-ink tracking-[0.04em]">
                  {(jointState.position[i] ?? 0).toFixed(4)}
                </td>
                <td className="text-right px-4 py-1.5 font-mono text-[11.5px] text-ink tracking-[0.04em]">
                  {(((jointState.position[i] ?? 0) * 180) / Math.PI).toFixed(1)}°
                </td>
                <td className="text-right px-4 py-1.5 font-mono text-[11.5px] text-ink tracking-[0.04em]">
                  {(jointState.velocity[i] ?? 0).toFixed(4)}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
