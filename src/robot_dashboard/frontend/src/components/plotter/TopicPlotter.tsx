import { useState, useCallback, useRef, useEffect, useMemo } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
} from "recharts";
import ROSLIB from "roslib";
import { getRos } from "../../lib/rosbridge-client";
import { useConnectionStore } from "../../stores/connection-store";
import { useTopicStore } from "../../stores/topic-store";
import {
  LineChartIcon,
  Plus,
  Trash2,
  Play,
  Square,
  Settings,
} from "lucide-react";
import clsx from "clsx";

const COLORS = [
  "#3b82f6", "#22c55e", "#ef4444", "#f59e0b", "#8b5cf6",
  "#ec4899", "#06b6d4", "#f97316",
];

const MAX_POINTS = 200;

interface PlotSeries {
  id: number;
  topicName: string;
  fieldPath: string; // e.g. "position[0]" or "linear.x"
  color: string;
  active: boolean;
}

interface DataPoint {
  time: number;
  [key: string]: number;
}

export default function TopicPlotter() {
  const topics = useTopicStore((s) => s.topics);
  const connected = useConnectionStore((s) => s.connected);
  const [series, setSeries] = useState<PlotSeries[]>([]);
  const [data, setData] = useState<DataPoint[]>([]);
  const [paused, setPaused] = useState(false);
  const [windowSec, setWindowSec] = useState(10);
  const subsRef = useRef<Map<string, ROSLIB.Topic>>(new Map());
  const startTimeRef = useRef<number>(Date.now());
  const nextId = useRef(0);

  // Subscribe to topics as series are added
  useEffect(() => {
    if (!connected) return;

    const activeTopics = new Set(
      series.filter((s) => s.active).map((s) => s.topicName)
    );

    // Remove stale subscriptions
    for (const [name, topic] of subsRef.current) {
      if (!activeTopics.has(name)) {
        topic.unsubscribe();
        subsRef.current.delete(name);
      }
    }

    // Add new subscriptions
    for (const topicName of activeTopics) {
      if (subsRef.current.has(topicName)) continue;

      const topicInfo = topics.find((t) => t.name === topicName);
      if (!topicInfo) continue;

      const rosTopic = new ROSLIB.Topic({
        ros: getRos(),
        name: topicName,
        messageType: topicInfo.type,
        throttle_rate: 50, // 20 Hz
      });

      rosTopic.subscribe((msg: any) => {
        if (paused) return;

        const time = (Date.now() - startTimeRef.current) / 1000;
        const point: DataPoint = { time };

        for (const s of series) {
          if (s.topicName === topicName && s.active) {
            const val = extractField(msg, s.fieldPath);
            if (val !== null) {
              point[`s${s.id}`] = val;
            }
          }
        }

        setData((prev) => {
          const next = [...prev, point];
          // Trim to window
          const cutoff = time - windowSec;
          const trimmed = next.filter((p) => p.time >= cutoff);
          return trimmed.slice(-MAX_POINTS);
        });
      });

      subsRef.current.set(topicName, rosTopic);
    }

    return () => {
      subsRef.current.forEach((t) => t.unsubscribe());
      subsRef.current.clear();
    };
  }, [connected, series, topics, paused, windowSec]);

  const addSeries = () => {
    const id = nextId.current++;
    setSeries((prev) => [
      ...prev,
      {
        id,
        topicName: topics[0]?.name || "",
        fieldPath: "data",
        color: COLORS[id % COLORS.length],
        active: true,
      },
    ]);
  };

  const removeSeries = (id: number) => {
    setSeries((prev) => prev.filter((s) => s.id !== id));
  };

  const updateSeries = (id: number, field: keyof PlotSeries, value: any) => {
    setSeries((prev) =>
      prev.map((s) => (s.id === id ? { ...s, [field]: value } : s))
    );
  };

  const clearData = () => {
    setData([]);
    startTimeRef.current = Date.now();
  };

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-4 py-2 border-b border-gray-800 flex items-center gap-3">
        <LineChartIcon className="w-4 h-4 text-blue-400" />
        <h2 className="text-sm font-semibold">Topic Plotter</h2>

        <div className="ml-auto flex items-center gap-2">
          <label className="flex items-center gap-1 text-[10px] text-gray-400">
            Window:
            <select
              value={windowSec}
              onChange={(e) => setWindowSec(parseInt(e.target.value))}
              className="px-1 py-0 text-[10px] bg-gray-800 border border-gray-700 rounded text-gray-300"
            >
              <option value={5}>5s</option>
              <option value={10}>10s</option>
              <option value={30}>30s</option>
              <option value={60}>60s</option>
            </select>
          </label>

          <button
            onClick={() => setPaused(!paused)}
            className={clsx(
              "p-1 rounded",
              paused ? "text-yellow-400" : "text-gray-400 hover:text-gray-200"
            )}
            title={paused ? "Resume" : "Pause"}
          >
            {paused ? <Play className="w-3.5 h-3.5" /> : <Square className="w-3.5 h-3.5" />}
          </button>

          <button
            onClick={clearData}
            className="p-1 rounded text-gray-400 hover:text-gray-200"
            title="Clear"
          >
            <Trash2 className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>

      <div className="flex flex-1 overflow-hidden">
        {/* Chart */}
        <div className="flex-1 p-3">
          {series.length > 0 ? (
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={data}>
                <CartesianGrid strokeDasharray="3 3" stroke="#1e1e2e" />
                <XAxis
                  dataKey="time"
                  stroke="#4b5563"
                  fontSize={10}
                  tickFormatter={(v) => `${v.toFixed(1)}s`}
                />
                <YAxis stroke="#4b5563" fontSize={10} />
                <Tooltip
                  contentStyle={{
                    backgroundColor: "#1e1e2e",
                    border: "1px solid #313244",
                    borderRadius: 8,
                    fontSize: 10,
                  }}
                />
                <Legend wrapperStyle={{ fontSize: 10 }} />
                {series
                  .filter((s) => s.active)
                  .map((s) => (
                    <Line
                      key={s.id}
                      type="monotone"
                      dataKey={`s${s.id}`}
                      name={`${s.topicName.split("/").pop()}.${s.fieldPath}`}
                      stroke={s.color}
                      strokeWidth={1.5}
                      dot={false}
                      isAnimationActive={false}
                    />
                  ))}
              </LineChart>
            </ResponsiveContainer>
          ) : (
            <div className="flex flex-col items-center justify-center h-full text-gray-500">
              <LineChartIcon className="w-8 h-8 mb-2 opacity-30" />
              <p className="text-sm">No series configured</p>
              <p className="text-xs">Add a series to start plotting</p>
            </div>
          )}
        </div>

        {/* Series config sidebar */}
        <div className="w-72 border-l border-gray-800 overflow-auto p-3 space-y-2 shrink-0">
          <div className="flex items-center gap-1 text-[10px] text-gray-500 font-medium uppercase">
            <Settings className="w-3 h-3" />
            Series
          </div>

          {series.map((s) => (
            <div
              key={s.id}
              className="p-2 rounded border border-gray-800 bg-gray-900/50 space-y-1.5"
            >
              <div className="flex items-center gap-1">
                <span
                  className="w-2.5 h-2.5 rounded-full shrink-0"
                  style={{ backgroundColor: s.color }}
                />
                <select
                  value={s.topicName}
                  onChange={(e) => updateSeries(s.id, "topicName", e.target.value)}
                  className="flex-1 px-1 py-0.5 text-[10px] bg-gray-800 border border-gray-700 rounded text-gray-200 truncate"
                >
                  {topics.map((t) => (
                    <option key={t.name} value={t.name}>
                      {t.name}
                    </option>
                  ))}
                </select>
                <button
                  onClick={() => removeSeries(s.id)}
                  className="p-0.5 text-red-500 hover:text-red-300"
                >
                  <Trash2 className="w-3 h-3" />
                </button>
              </div>
              <input
                type="text"
                value={s.fieldPath}
                onChange={(e) => updateSeries(s.id, "fieldPath", e.target.value)}
                placeholder="field.path or position[0]"
                className="w-full px-1.5 py-0.5 text-[10px] font-mono bg-gray-800 border border-gray-700 rounded text-gray-300"
              />
              <div className="text-[8px] text-gray-600">
                Examples: data, position[0], linear.x, twist.angular.z
              </div>
            </div>
          ))}

          <button
            onClick={addSeries}
            className="w-full flex items-center justify-center gap-1 py-1.5 rounded border border-dashed border-gray-700 text-xs text-gray-500 hover:text-gray-300 hover:border-gray-500"
          >
            <Plus className="w-3 h-3" />
            Add Series
          </button>
        </div>
      </div>
    </div>
  );
}

/**
 * Extract a nested field from a ROS message object.
 * Supports dot notation ("linear.x") and array indexing ("position[2]").
 */
function extractField(obj: any, path: string): number | null {
  try {
    const parts = path.split(/[.\[\]]+/).filter(Boolean);
    let current = obj;
    for (const part of parts) {
      if (current == null) return null;
      const idx = parseInt(part);
      current = isNaN(idx) ? current[part] : current[idx];
    }
    return typeof current === "number" ? current : null;
  } catch {
    return null;
  }
}
