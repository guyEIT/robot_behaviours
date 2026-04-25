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
        stroke: props.animated ? "#A96D4B" : "#D7D7D7",
        strokeWidth: props.animated ? 1.75 : 1.25,
      }}
      markerEnd={props.markerEnd}
    />
  );
}

export default memo(BtEdgeComponent);
