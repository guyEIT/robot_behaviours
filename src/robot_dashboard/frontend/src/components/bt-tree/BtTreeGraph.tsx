import { useMemo, useCallback, useEffect } from "react";
import {
  ReactFlow,
  ReactFlowProvider,
  Background,
  Controls,
  MiniMap,
  useNodesState,
  useEdgesState,
  useReactFlow,
  type Node,
  type Edge,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";

import BtNodeComponent from "./BtNode";
import BtEdgeComponent from "./BtEdge";
import { parseBtXml, buildNameIndex } from "../../lib/bt-parser";
import { layoutBtGraph } from "../../lib/bt-layout";
import type { BtGraphNode } from "../../types/bt";

const nodeTypes = { btNode: BtNodeComponent };
const edgeTypes = { btEdge: BtEdgeComponent };

interface Props {
  xml: string;
  activeNodeName: string | null;
  completedNodeNames?: readonly string[];
  failedNodeNames?: readonly string[];
  followActive?: boolean;
}

function BtTreeGraphInner({
  xml,
  activeNodeName,
  completedNodeNames = [],
  failedNodeNames = [],
  followActive = false,
}: Props) {
  const { setCenter } = useReactFlow();

  const { layoutNodes, layoutEdges, nameIndex } = useMemo(() => {
    try {
      const parsed = parseBtXml(xml);
      const layout = layoutBtGraph(parsed.nodes, parsed.edges);
      const nameIndex = buildNameIndex(parsed.nodes);
      return {
        layoutNodes: layout.nodes,
        layoutEdges: layout.edges,
        nameIndex,
      };
    } catch {
      return { layoutNodes: [], layoutEdges: [], nameIndex: new Map() };
    }
  }, [xml]);

  const [nodes, setNodes, onNodesChange] = useNodesState(layoutNodes as Node[]);
  const [edges, setEdges, onEdgesChange] = useEdgesState(layoutEdges as Edge[]);

  useEffect(() => {
    const activeId = activeNodeName ? nameIndex.get(activeNodeName) ?? null : null;
    const completedIds = new Set(
      completedNodeNames.map((n) => nameIndex.get(n)).filter(Boolean) as string[],
    );
    const failedIds = new Set(
      failedNodeNames.map((n) => nameIndex.get(n)).filter(Boolean) as string[],
    );

    let activeNodePosition: { x: number; y: number; w: number; h: number } | null = null;
    setNodes((prev) =>
      prev.map((node) => {
        const data = node.data as unknown as BtGraphNode;
        let newState: BtGraphNode["state"] = "idle";
        if (failedIds.has(node.id)) newState = "failure";
        if (completedIds.has(node.id)) newState = "success";
        if (node.id === activeId) {
          newState = "running";
          activeNodePosition = {
            x: node.position?.x ?? 0,
            y: node.position?.y ?? 0,
            w: node.measured?.width ?? 120,
            h: node.measured?.height ?? 40,
          };
        }
        if (data.state === newState) return node;
        return { ...node, data: { ...data, state: newState } };
      }),
    );

    setEdges((prev) =>
      prev.map((edge) => ({
        ...edge,
        animated: edge.target === activeId,
      })),
    );

    if (followActive && activeNodePosition) {
      const p: { x: number; y: number; w: number; h: number } = activeNodePosition;
      setCenter(p.x + p.w / 2, p.y + p.h / 2, { duration: 300, zoom: 1.2 });
    }
  }, [
    activeNodeName,
    completedNodeNames,
    failedNodeNames,
    nameIndex,
    setNodes,
    setEdges,
    followActive,
    setCenter,
  ]);

  useEffect(() => {
    setNodes(layoutNodes as Node[]);
    setEdges(layoutEdges as Edge[]);
  }, [layoutNodes, layoutEdges, setNodes, setEdges]);

  const onInit = useCallback((instance: any) => {
    instance.fitView({ padding: 0.1 });
  }, []);

  return (
    <ReactFlow
      nodes={nodes}
      edges={edges}
      onNodesChange={onNodesChange}
      onEdgesChange={onEdgesChange}
      nodeTypes={nodeTypes}
      edgeTypes={edgeTypes}
      onInit={onInit}
      fitView
      minZoom={0.1}
      maxZoom={2}
      proOptions={{ hideAttribution: true }}
    >
      <Background color="#D7D7D7" gap={24} size={1} />
      <Controls showInteractive={false} />
      <MiniMap
        nodeColor={() => "#9a8f83"}
        maskColor="rgba(246,244,236,0.6)"
        style={{ background: "#FBF9F5", border: "1px solid #D7D7D7" }}
      />
    </ReactFlow>
  );
}

export default function BtTreeGraph(props: Props) {
  return (
    <ReactFlowProvider>
      <BtTreeGraphInner {...props} />
    </ReactFlowProvider>
  );
}
