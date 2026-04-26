import { useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { useTaskStore, type SkillSegment } from "../../stores/task-store";
import { Eyebrow } from "../ui";
import clsx from "clsx";

/**
 * Live Gantt-style timeline of skill executions.
 *
 * Reads derived `skillSegments` from the task store (see task-store.ts), each
 * a {name, startedAt, endedAt, status}. Bars are positioned on a continuously
 * advancing time axis: "now" is fixed at the right edge of the visible window,
 * older bars scroll left. Bars older than the window edge fade out; segments
 * older than `SEGMENT_RETENTION_MS` are pruned by the store.
 *
 * Loosely modelled on Temporal's workflow timeline view — horizontal bars,
 * concurrent activities stack into separate lanes.
 */

const WINDOW_MS = 30_000;     // visible time window: 30 s (doubled px-per-second vs. 60s)
const HEAD_ROOM_MS = 3_000;   // how far past "now" the right edge sits
const FADE_AT_MS = 24_000;    // bars older than this start fading
const FADE_OVER_MS = 6_000;   // fade-out duration after FADE_AT_MS
const ROW_H = 20;             // px per lane
const ROW_GAP = 4;            // px between lanes
const LABEL_GUTTER = 0;       // labels render INSIDE bars, no left gutter
const TICK_INTERVAL_MS = 10_000;
const MAX_LANES = 3;          // upper bound on lane count for legibility

const STATUS_COLOR: Record<SkillSegment["status"], string> = {
  running: "var(--running)",
  done: "var(--ok)",
  failed: "var(--err)",
};

const STATUS_FILL: Record<SkillSegment["status"], string> = {
  running: "var(--running-soft)",
  done: "var(--ok-soft)",
  failed: "var(--err-soft)",
};

/**
 * Lane assignment with staggering for legibility.
 *
 * Sequential, non-overlapping bars would naturally all collapse onto lane 0,
 * producing one dense row. Instead we round-robin across `MAX_LANES` so
 * consecutive bars land on different rows — easier to read at a glance.
 *
 * Parallel bars (true overlaps) are still respected: if the preferred lane
 * is occupied, we pick any free one. If all `MAX_LANES` are busy at once,
 * we stack on the lane that frees up soonest.
 */
function assignLanes(segments: SkillSegment[], now: number): {
  lanes: Map<string, number>;
  laneCount: number;
} {
  const sorted = [...segments].sort((a, b) => a.startedAt - b.startedAt);
  // Pre-allocate all MAX_LANES so the round-robin can land on a fresh lane
  // even before all lanes have been used. -Infinity = lane never used.
  const laneEnds: number[] = new Array(MAX_LANES).fill(-Infinity);
  const lanes = new Map<string, number>();
  let preferred = 0;

  for (const s of sorted) {
    const end = s.endedAt ?? now;
    const available: number[] = [];
    for (let i = 0; i < MAX_LANES; i++) {
      if (laneEnds[i] <= s.startedAt) available.push(i);
    }

    let lane: number;
    if (available.includes(preferred)) {
      lane = preferred;
    } else if (available.length > 0) {
      lane = available[0];
    } else {
      // All lanes occupied — stack on the one that frees up soonest.
      let best = 0;
      for (let i = 1; i < MAX_LANES; i++) {
        if (laneEnds[i] < laneEnds[best]) best = i;
      }
      lane = best;
    }

    laneEnds[lane] = end;
    lanes.set(s.id, lane);
    preferred = (lane + 1) % MAX_LANES;
  }

  // Lane count = (max used lane index) + 1, so rows collapse if only lane 0
  // is ever touched (e.g. a single-skill task).
  let maxLane = -1;
  for (const lane of lanes.values()) if (lane > maxLane) maxLane = lane;
  return { lanes, laneCount: maxLane + 1 };
}

export default function SkillTimeline(_props: {
  // legacy prop signature retained so TaskMonitorPanel doesn't need a shim
  completedSkills?: string[];
  failedSkills?: string[];
  currentSkill?: string;
}) {
  const segments = useTaskStore((s) => s.skillSegments);
  const containerRef = useRef<HTMLDivElement>(null);
  const [width, setWidth] = useState(600);
  const [now, setNow] = useState(() => Date.now());

  // Track container width via ResizeObserver
  useLayoutEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      const w = entries[0]?.contentRect.width;
      if (w && w > 0) setWidth(w);
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  // Drive the "now" clock at ~10 fps when there's anything live.
  useEffect(() => {
    const hasLive = segments.some((s) => s.endedAt === null);
    // Even with no live segments, we still need to advance time so the trail
    // scrolls off — but at a slower rate.
    const interval = hasLive ? 100 : 500;
    const id = setInterval(() => setNow(Date.now()), interval);
    return () => clearInterval(id);
  }, [segments]);

  const left = now - WINDOW_MS;
  const right = now + HEAD_ROOM_MS;
  const totalRange = right - left;
  const innerWidth = Math.max(width - LABEL_GUTTER, 100);

  // Visible (= ends after window's left edge)
  const visible = useMemo(
    () => segments.filter((s) => (s.endedAt ?? now) >= left),
    [segments, left, now],
  );

  const { lanes, laneCount } = useMemo(() => assignLanes(visible, now), [visible, now]);
  const numLanes = Math.max(1, laneCount);
  const height = numLanes * (ROW_H + ROW_GAP) - ROW_GAP;

  // Time→pixel
  const xFor = (t: number) => ((t - left) / totalRange) * innerWidth + LABEL_GUTTER;
  const wFor = (a: number, b: number) => Math.max(2, ((b - a) / totalRange) * innerWidth);

  // Tick marks every TICK_INTERVAL_MS, snapped to the nearest interval
  const ticks = useMemo(() => {
    const result: { t: number; label: string }[] = [];
    const first = Math.ceil(left / TICK_INTERVAL_MS) * TICK_INTERVAL_MS;
    for (let t = first; t <= right; t += TICK_INTERVAL_MS) {
      const ageSec = Math.round((now - t) / 1000);
      const label = ageSec === 0 ? "now" : ageSec > 0 ? `-${ageSec}s` : `+${-ageSec}s`;
      result.push({ t, label });
    }
    return result;
  }, [left, right, now]);

  if (segments.length === 0) {
    return (
      <div ref={containerRef}>
        <Eyebrow size="sm" tone="muted" className="block mb-2">
          Skill Timeline
        </Eyebrow>
        <p className="text-[12px] text-muted italic">No skills executed yet</p>
      </div>
    );
  }

  return (
    <div ref={containerRef} className="font-sans">
      <div className="flex items-center justify-between mb-1.5">
        <Eyebrow size="sm" tone="muted">
          Skill Timeline
        </Eyebrow>
        <span className="font-mono text-[10px] text-muted tracking-[0.06em]">
          {WINDOW_MS / 1000}s window · {visible.length}/{segments.length} visible
        </span>
      </div>

      <svg
        width={innerWidth}
        height={height + 18}
        style={{ display: "block" }}
        aria-label="Skill execution timeline"
      >
        {/* Tick grid */}
        {ticks.map((t) => {
          const x = xFor(t.t);
          return (
            <g key={t.t}>
              <line
                x1={x}
                x2={x}
                y1={0}
                y2={height}
                stroke="var(--hair-soft)"
                strokeWidth={1}
                strokeDasharray="2 4"
              />
              <text
                x={x}
                y={height + 12}
                fill="var(--muted)"
                fontFamily='"Source Code Pro", ui-monospace, monospace'
                fontSize={9}
                textAnchor="middle"
                style={{ letterSpacing: "0.04em" }}
              >
                {t.label}
              </text>
            </g>
          );
        })}

        {/* Bars */}
        {visible.map((seg) => {
          const lane = lanes.get(seg.id) ?? 0;
          const y = lane * (ROW_H + ROW_GAP);
          const segEnd = seg.endedAt ?? now;
          const x1 = Math.max(xFor(seg.startedAt), LABEL_GUTTER);
          const x2 = xFor(segEnd);
          const barW = Math.max(2, x2 - x1);
          const stroke = STATUS_COLOR[seg.status];
          const fill = STATUS_FILL[seg.status];

          // Fade by age past FADE_AT_MS (older bars dim out)
          const ageMs = now - segEnd;
          const fadeProgress = Math.max(0, Math.min(1, (ageMs - FADE_AT_MS) / FADE_OVER_MS));
          const opacity = 1 - 0.85 * fadeProgress;

          // Pulse marker for still-running bars at their leading edge
          const isRunning = seg.endedAt === null;

          return (
            <g key={seg.id} opacity={opacity}>
              <rect
                x={x1}
                y={y}
                width={barW}
                height={ROW_H}
                fill={fill}
                stroke={stroke}
                strokeWidth={1}
                rx={2}
                ry={2}
              />
              {/* Label inside bar (truncated by clip-path so it doesn't bleed past) */}
              <clipPath id={`clip-${seg.id}`}>
                <rect x={x1 + 4} y={y} width={Math.max(0, barW - 8)} height={ROW_H} />
              </clipPath>
              <text
                x={x1 + 6}
                y={y + ROW_H / 2}
                dominantBaseline="middle"
                fontFamily='"Source Sans 3", system-ui, sans-serif'
                fontSize={11}
                fontWeight={500}
                fill={stroke}
                clipPath={`url(#clip-${seg.id})`}
              >
                {seg.name}
              </text>
              {/* Live leading-edge pulse for running bars */}
              {isRunning && (
                <>
                  <line
                    x1={x2}
                    x2={x2}
                    y1={y - 2}
                    y2={y + ROW_H + 2}
                    stroke={stroke}
                    strokeWidth={1.5}
                  />
                  <circle cx={x2} cy={y + ROW_H / 2} r={3} fill={stroke}>
                    <animate
                      attributeName="r"
                      values="3;5;3"
                      dur="1.4s"
                      repeatCount="indefinite"
                    />
                    <animate
                      attributeName="opacity"
                      values="1;0.3;1"
                      dur="1.4s"
                      repeatCount="indefinite"
                    />
                  </circle>
                </>
              )}
              <title>
                {seg.name} · {seg.status} · {((segEnd - seg.startedAt) / 1000).toFixed(2)}s
              </title>
            </g>
          );
        })}

        {/* "now" cursor */}
        <line
          x1={xFor(now)}
          x2={xFor(now)}
          y1={-2}
          y2={height + 2}
          stroke="var(--terracotta)"
          strokeWidth={1}
          strokeDasharray="3 2"
          opacity={0.7}
        />
      </svg>

      {/* Legend */}
      <div className="flex items-center gap-3 mt-1.5 font-mono text-[10px] uppercase tracking-[0.08em] text-muted">
        <LegendDot color="var(--running)" label="Running" />
        <LegendDot color="var(--ok)" label="Done" />
        <LegendDot color="var(--err)" label="Failed" />
      </div>
    </div>
  );
}

function LegendDot({ color, label }: { color: string; label: string }) {
  return (
    <span className="flex items-center gap-1.5">
      <span
        style={{ background: color }}
        className={clsx("inline-block w-1.5 h-1.5 rounded-full")}
      />
      {label}
    </span>
  );
}
