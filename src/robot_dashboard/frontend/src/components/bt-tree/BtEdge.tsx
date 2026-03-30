import { memo } from "react";
import { BaseEdge, getSmoothStepPath, type EdgeProps } from "@xyflow/react";

function BtEdgeComponent(props: EdgeProps) {
  const [edgePath] = getSmoothStepPath({
    sourceX: props.sourceX,
    sourceY: props.sourceY,
    targetX: props.targetX,
    targetY: props.targetY,
    borderRadius: 8,
  });

  return (
    <BaseEdge
      id={props.id}
      path={edgePath}
      style={{
        stroke: props.animated ? "#3b82f6" : "#4b5563",
        strokeWidth: props.animated ? 2 : 1.5,
      }}
      markerEnd={props.markerEnd}
    />
  );
}

export default memo(BtEdgeComponent);
