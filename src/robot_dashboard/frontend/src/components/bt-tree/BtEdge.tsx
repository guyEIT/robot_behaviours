import { memo } from "react";
import {
  BaseEdge,
  EdgeLabelRenderer,
  getSmoothStepPath,
  type EdgeProps,
} from "@xyflow/react";

interface BtEdgeData {
  branchLabel?: "cond" | "then" | "else" | "do";
  ordinal?: number;
  siblingCount?: number;
  style?: "default" | "dashed";
  gate?: { key: string; expected: string };
}

function BtEdgeComponent(props: EdgeProps) {
  const data = (props.data ?? {}) as BtEdgeData;
  const [edgePath, labelX, labelY] = getSmoothStepPath({
    sourceX: props.sourceX,
    sourceY: props.sourceY,
    targetX: props.targetX,
    targetY: props.targetY,
    borderRadius: 8,
  });

  const dashed = data.style === "dashed";
  const stroke = props.animated ? "#A96D4B" : dashed ? "#B8A9A0" : "#D7D7D7";

  return (
    <>
      <BaseEdge
        id={props.id}
        path={edgePath}
        style={{
          stroke,
          strokeWidth: props.animated ? 1.75 : 1.25,
          strokeDasharray: dashed ? "4 3" : undefined,
        }}
        markerEnd={props.markerEnd}
      />
      <EdgeLabelRenderer>
        {/* Branch label (cond/then/else/do) — placed near source */}
        {data.branchLabel && (
          <div
            className="nodrag nopan absolute font-mono text-[9px] uppercase tracking-[0.1em] px-1.5 py-[1px] border border-hair bg-paper text-ink-soft pointer-events-none"
            style={{
              transform: `translate(-50%, -50%) translate(${
                (props.sourceX + labelX) / 2
              }px, ${(props.sourceY + labelY) / 2}px)`,
            }}
          >
            {data.branchLabel}
          </div>
        )}

        {/* BlackboardCondition gate pill — placed at midpoint */}
        {data.gate && (
          <div
            className="nodrag nopan absolute font-mono text-[10px] px-2 py-[2px] border border-muted bg-paper text-ink-soft pointer-events-auto"
            style={{
              transform: `translate(-50%, -50%) translate(${labelX}px, ${labelY}px)`,
            }}
            title={`${data.gate.key} == ${data.gate.expected}`}
          >
            <span className="text-muted">⊟</span>{" "}
            <span className="font-semibold">{data.gate.key}</span>{" "}
            <span className="text-muted">==</span>{" "}
            <span>{data.gate.expected}</span>
          </div>
        )}

        {/* Sibling ordinal numeral — only when there are 2+ siblings */}
        {data.ordinal !== undefined &&
          (data.siblingCount ?? 0) > 1 &&
          !data.branchLabel && (
            <div
              className="nodrag nopan absolute font-mono text-[9px] text-muted bg-paper border border-hair-soft w-4 h-4 flex items-center justify-center pointer-events-none"
              style={{
                transform: `translate(-50%, -50%) translate(${
                  (props.sourceX + labelX) / 2
                }px, ${(props.sourceY + labelY) / 2}px)`,
              }}
            >
              {data.ordinal}
            </div>
          )}
      </EdgeLabelRenderer>
    </>
  );
}

export default memo(BtEdgeComponent);
