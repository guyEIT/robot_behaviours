import dagre from "@dagrejs/dagre";
import type { BtGraphNode, BtGraphEdge } from "../types/bt";

const NODE_WIDTH = 200;
const NODE_HEIGHT = 60;

export interface LayoutResult {
  nodes: Array<{
    id: string;
    position: { x: number; y: number };
    data: BtGraphNode;
    type: string;
  }>;
  edges: Array<{
    id: string;
    source: string;
    target: string;
    type: string;
    animated: boolean;
  }>;
}

/**
 * Layout BT graph nodes using dagre (top-to-bottom tree layout).
 */
export function layoutBtGraph(
  btNodes: BtGraphNode[],
  btEdges: BtGraphEdge[]
): LayoutResult {
  const g = new dagre.graphlib.Graph();
  g.setDefaultEdgeLabel(() => ({}));
  g.setGraph({
    rankdir: "TB",
    nodesep: 30,
    ranksep: 70,
    marginx: 20,
    marginy: 20,
  });

  for (const node of btNodes) {
    g.setNode(node.id, { width: NODE_WIDTH, height: NODE_HEIGHT });
  }

  for (const edge of btEdges) {
    g.setEdge(edge.source, edge.target);
  }

  dagre.layout(g);

  const nodes = btNodes.map((node) => {
    const pos = g.node(node.id);
    return {
      id: node.id,
      position: {
        x: (pos?.x ?? 0) - NODE_WIDTH / 2,
        y: (pos?.y ?? 0) - NODE_HEIGHT / 2,
      },
      data: node,
      type: "btNode",
    };
  });

  const edges = btEdges.map((edge) => ({
    id: edge.id,
    source: edge.source,
    target: edge.target,
    type: "btEdge",
    animated: false,
  }));

  return { nodes, edges };
}
