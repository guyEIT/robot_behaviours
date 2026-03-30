import { useMemo, useCallback, useEffect } from "react";
import {
  ReactFlow,
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
  followActive?: boolean;
}

function BtTreeGraphInner({ xml, activeNodeName, followActive = false }: Props) {
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

  // Update node states when activeNodeName changes
  useEffect(() => {
    setNodes((prev) =>
      prev.map((node) => {
        const data = node.data as unknown as BtGraphNode;
        const isActive =
          activeNodeName != null && nameIndex.get(activeNodeName) === node.id;
        const newState = isActive ? "running" : "idle";
        if (data.state === newState) return node;
        return {
          ...node,
          data: { ...data, state: newState },
        };
      })
    );

    // Animate edges leading to the active node
    const activeNodeId = activeNodeName
      ? nameIndex.get(activeNodeName) ?? null
      : null;
    setEdges((prev) =>
      prev.map((edge) => ({
        ...edge,
        animated: edge.target === activeNodeId,
      }))
    );

    // Follow the active node
    if (followActive && activeNodeName) {
      const activeNodeId = nameIndex.get(activeNodeName);
      if (activeNodeId) {
        const activeNode = nodes.find((n) => n.id === activeNodeId);
        if (activeNode?.position) {
          const x = activeNode.position.x + (activeNode.measured?.width ?? 120) / 2;
          const y = activeNode.position.y + (activeNode.measured?.height ?? 40) / 2;
          setCenter(x, y, { duration: 300, zoom: 1.2 });
        }
      }
    }
  }, [activeNodeName, nameIndex, setNodes, setEdges, followActive, nodes, setCenter]);

  // Re-layout when XML changes
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
      <Background color="#1e1e2e" gap={20} />
      <Controls showInteractive={false} />
      <MiniMap
        nodeColor={() => "#4b5563"}
        maskColor="rgba(0,0,0,0.6)"
        style={{ background: "#111118" }}
      />
    </ReactFlow>
  );
}

export default function BtTreeGraph(props: Props) {
  return <BtTreeGraphInner {...props} />;
}
