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
import { Gauge, Activity } from "lucide-react";
import clsx from "clsx";

type ViewMode = "position" | "velocity" | "effort";

const JOINT_COLORS: Record<string, string> = {
  panda_joint1: "#3b82f6",
  panda_joint2: "#22c55e",
  panda_joint3: "#ef4444",
  panda_joint4: "#f59e0b",
  panda_joint5: "#8b5cf6",
  panda_joint6: "#ec4899",
  panda_joint7: "#06b6d4",
  panda_finger_joint1: "#a78bfa",
  panda_finger_joint2: "#67e8f9",
};

export default function JointStateViewer() {
  const [jointState, setJointState] = useState<JointState | null>(null);
  const [viewMode, setViewMode] = useState<ViewMode>("position");
  const [hz, setHz] = useState(0);
  const [lastTime, setLastTime] = useState(0);

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
    "/joint_states",
    "sensor_msgs/msg/JointState",
    handleMsg,
    50 // 20 Hz max
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
        name: name.replace("panda_", "").replace("_joint", "J").replace("finger_joint", "FJ"),
        fullName: name,
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
      <div className="flex flex-col items-center justify-center h-full text-gray-500">
        <Gauge className="w-8 h-8 mb-2 opacity-30" />
        <p className="text-sm">Waiting for /joint_states</p>
        <p className="text-xs">Joint state publisher not detected</p>
      </div>
    );
  }

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-4 py-2 border-b border-gray-800 flex items-center gap-3">
        <Gauge className="w-4 h-4 text-blue-400" />
        <h2 className="text-sm font-semibold">Joint States</h2>

        <div className="ml-2 flex gap-1">
          {(["position", "velocity", "effort"] as const).map((m) => (
            <button
              key={m}
              onClick={() => setViewMode(m)}
              className={clsx(
                "px-2 py-0.5 rounded text-[10px] font-medium uppercase",
                viewMode === m
                  ? "bg-blue-500/20 text-blue-400"
                  : "text-gray-500 hover:text-gray-300"
              )}
            >
              {m}
            </button>
          ))}
        </div>

        <div className="ml-auto flex items-center gap-2 text-[10px] text-gray-500">
          <Activity className="w-3 h-3" />
          {hz} Hz
        </div>
      </div>

      {/* Chart */}
      <div className="flex-1 p-4">
        <ResponsiveContainer width="100%" height="100%">
          <BarChart data={chartData} layout="vertical">
            <CartesianGrid strokeDasharray="3 3" stroke="#1e1e2e" />
            <XAxis
              type="number"
              stroke="#4b5563"
              fontSize={10}
              tickFormatter={(v) =>
                viewMode === "position" ? `${(v * 180 / Math.PI).toFixed(0)}°` : v.toFixed(2)
              }
            />
            <YAxis
              dataKey="name"
              type="category"
              stroke="#4b5563"
              fontSize={10}
              width={40}
            />
            <Tooltip
              contentStyle={{
                backgroundColor: "#1e1e2e",
                border: "1px solid #313244",
                borderRadius: 8,
                fontSize: 10,
              }}
              formatter={(value: any, _name: any, props: any) => {
                const v = Number(value) || 0;
                const entry = props.payload;
                if (viewMode === "position") {
                  return [`${v.toFixed(4)} rad (${((v * 180) / Math.PI).toFixed(1)}°)`, entry.fullName];
                }
                return [v.toFixed(4), entry.fullName];
              }}
            />
            <Bar dataKey="value" radius={[0, 4, 4, 0]}>
              {chartData.map((entry) => (
                <Cell
                  key={entry.fullName}
                  fill={JOINT_COLORS[entry.fullName] || "#6b7280"}
                />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </div>

      {/* Joint table */}
      <div className="px-4 pb-3 border-t border-gray-800 overflow-auto max-h-48">
        <table className="w-full text-[10px]">
          <thead>
            <tr className="text-gray-500 uppercase">
              <th className="text-left py-1 font-medium">Joint</th>
              <th className="text-right py-1 font-medium">Position (rad)</th>
              <th className="text-right py-1 font-medium">Position (deg)</th>
              <th className="text-right py-1 font-medium">Velocity</th>
            </tr>
          </thead>
          <tbody>
            {jointState.name.map((name, i) => (
              <tr key={name} className="border-t border-gray-800/30 hover:bg-gray-800/30">
                <td className="py-0.5 font-mono text-gray-300">{name}</td>
                <td className="text-right font-mono text-gray-400">
                  {(jointState.position[i] ?? 0).toFixed(4)}
                </td>
                <td className="text-right font-mono text-gray-400">
                  {(((jointState.position[i] ?? 0) * 180) / Math.PI).toFixed(1)}°
                </td>
                <td className="text-right font-mono text-gray-400">
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
