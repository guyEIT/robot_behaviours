import { useState, useRef, useEffect } from "react";
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
  LineChart as LineChartIcon,
  Plus,
  Trash2,
  Play,
  Square,
  Settings,
} from "lucide-react";
import clsx from "clsx";
import { Eyebrow, IconBtn, Button } from "../ui";

// Sociius warm-leaning palette for series differentiation
const COLORS = [
  "#A96D4B", // terracotta
  "#51748C", // running
  "#2f7d5f", // ok
  "#1E1E1E", // ink-soft
  "#A17258", // terracotta-soft
  "#9a8f83", // muted
  "#9B2C2C", // err
];

const MAX_POINTS = 200;

interface PlotSeries {
  id: number;
  topicName: string;
  fieldPath: string;
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

  useEffect(() => {
    if (!connected) return;

    const activeTopics = new Set(
      series.filter((s) => s.active).map((s) => s.topicName)
    );

    for (const [name, topic] of subsRef.current) {
      if (!activeTopics.has(name)) {
        topic.unsubscribe();
        subsRef.current.delete(name);
      }
    }

    for (const topicName of activeTopics) {
      if (subsRef.current.has(topicName)) continue;

      const topicInfo = topics.find((t) => t.name === topicName);
      if (!topicInfo) continue;

      const rosTopic = new ROSLIB.Topic({
        ros: getRos(),
        name: topicName,
        messageType: topicInfo.type,
        throttle_rate: 50,
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
    <div className="flex flex-col h-full bg-paper">
      <div className="px-5 py-3 border-b border-hair flex items-center gap-3">
        <LineChartIcon className="w-4 h-4 text-terracotta" />
        <h2 className="text-[14px] font-medium text-ink">Topic Plotter</h2>

        <div className="ml-auto flex items-center gap-3">
          <label className="flex items-center gap-1.5 text-[11px] text-muted font-mono uppercase tracking-[0.08em]">
            Window
            <select
              value={windowSec}
              onChange={(e) => setWindowSec(parseInt(e.target.value))}
              className="px-2 py-0.5 text-[11px] bg-paper border border-hair text-ink-soft rounded-DEFAULT focus:border-terracotta focus:outline-none"
            >
              <option value={5}>5s</option>
              <option value={10}>10s</option>
              <option value={30}>30s</option>
              <option value={60}>60s</option>
            </select>
          </label>

          <IconBtn
            onClick={() => setPaused(!paused)}
            active={paused}
            title={paused ? "Resume" : "Pause"}
          >
            {paused ? <Play className="w-3.5 h-3.5" /> : <Square className="w-3.5 h-3.5" />}
          </IconBtn>

          <IconBtn onClick={clearData} title="Clear">
            <Trash2 className="w-3.5 h-3.5" />
          </IconBtn>
        </div>
      </div>

      <div className="flex flex-1 overflow-hidden">
        <div className="flex-1 p-4 bg-cream-deep">
          {series.length > 0 ? (
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={data}>
                <CartesianGrid strokeDasharray="2 4" stroke="#EEEEEE" />
                <XAxis
                  dataKey="time"
                  stroke="#9a8f83"
                  fontSize={10}
                  tickFormatter={(v) => `${v.toFixed(1)}s`}
                />
                <YAxis stroke="#9a8f83" fontSize={10} />
                <Tooltip
                  contentStyle={{
                    backgroundColor: "#FFFFFF",
                    border: "1px solid #D7D7D7",
                    borderRadius: 0,
                    fontSize: 11,
                    color: "#1E1E1E",
                  }}
                />
                <Legend wrapperStyle={{ fontSize: 11, color: "#1E1E1E" }} />
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
            <div className="flex flex-col items-center justify-center h-full text-muted">
              <LineChartIcon className="w-8 h-8 mb-2 opacity-30" />
              <p className="text-[14px] text-ink-soft">No series configured</p>
              <p className="text-[12px]">Add a series to start plotting</p>
            </div>
          )}
        </div>

        <div className="w-72 border-l border-hair overflow-auto p-4 space-y-3 shrink-0">
          <div className="flex items-center gap-1.5 text-muted">
            <Settings className="w-3 h-3" />
            <Eyebrow size="sm" tone="muted">Series</Eyebrow>
          </div>

          {series.map((s) => (
            <div
              key={s.id}
              className="p-3 border border-hair bg-paper space-y-2"
            >
              <div className="flex items-center gap-2">
                <span
                  className="w-2.5 h-2.5 rounded-full shrink-0"
                  style={{ backgroundColor: s.color }}
                />
                <select
                  value={s.topicName}
                  onChange={(e) => updateSeries(s.id, "topicName", e.target.value)}
                  className="flex-1 px-2 py-0.5 text-[11px] bg-paper border border-hair text-ink-soft rounded-DEFAULT truncate focus:border-terracotta focus:outline-none"
                >
                  {topics.map((t) => (
                    <option key={t.name} value={t.name}>
                      {t.name}
                    </option>
                  ))}
                </select>
                <IconBtn
                  onClick={() => removeSeries(s.id)}
                  className="!w-6 !h-6 hover:!bg-err-soft hover:!text-err"
                >
                  <Trash2 className="w-3 h-3" />
                </IconBtn>
              </div>
              <input
                type="text"
                value={s.fieldPath}
                onChange={(e) => updateSeries(s.id, "fieldPath", e.target.value)}
                placeholder="field.path or position[0]"
                className="w-full px-2 py-1 text-[11px] font-mono bg-paper border border-hair text-ink-soft rounded-DEFAULT focus:border-terracotta focus:outline-none tracking-[0.02em]"
              />
              <div className="text-[10px] text-muted font-mono tracking-[0.04em]">
                Examples: data, position[0], linear.x, twist.angular.z
              </div>
            </div>
          ))}

          <Button
            onClick={addSeries}
            variant="ghost"
            size="sm"
            leftIcon={<Plus className="w-3 h-3" />}
            className="w-full !border-dashed"
          >
            Add Series
          </Button>
        </div>
      </div>
    </div>
  );
}

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
