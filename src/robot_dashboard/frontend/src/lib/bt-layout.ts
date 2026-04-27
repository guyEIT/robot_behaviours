import dagre from "@dagrejs/dagre";
import type { BtGraphNode, BtGraphEdge } from "../types/bt";

export type LayoutDirection = "TB" | "LR";

const LEAF_WIDTH = 200;
const LEAF_HEIGHT = 64;
/** Header height inside container (room for title + iteration badge). */
const CONTAINER_HEADER = 32;
/** Padding from the inside of the container border to its children. */
const CONTAINER_PAD = 14;
/**
 * Size of a container when collapsed. In TB mode it's wide-and-short
 * (just the header bar); in LR mode the header still reads the same so
 * we keep the same dimensions.
 */
const COLLAPSED_WIDTH = 240;
const COLLAPSED_HEIGHT = CONTAINER_HEADER;

const NODESEP_TIGHT = 30;
const NODESEP_LOOSE = 60;
const RANKSEP_TB_TIGHT = 60;
const RANKSEP_TB_LOOSE = 110;
const RANKSEP_LR_TIGHT = 80;
const RANKSEP_LR_LOOSE = 150;

export interface LayoutResult {
  nodes: Array<{
    id: string;
    position: { x: number; y: number };
    data: BtGraphNode;
    type: string;
    parentId?: string;
    extent?: "parent";
    width?: number;
    height?: number;
    style?: Record<string, string | number>;
    hidden?: boolean;
  }>;
  edges: Array<{
    id: string;
    source: string;
    target: string;
    type: string;
    animated: boolean;
    hidden?: boolean;
    data: {
      branchLabel?: BtGraphEdge["branchLabel"];
      ordinal?: number;
      siblingCount?: number;
      style?: BtGraphEdge["style"];
      gate?: BtGraphEdge["gate"];
      collapsed?: boolean;
    };
  }>;
}

/**
 * Layout BT graph nodes with nested groups, honouring a `collapsedIds`
 * set: collapsed containers render as a single header bar (no children
 * laid out), and their descendants are marked `hidden` so ReactFlow
 * skips drawing them.
 *
 * Strategy: build a parent-id map, recurse bottom-up sizing each
 * container by laying out its direct children with their own dagre run,
 * then place each container as a sized box in its parent's run.
 */
export function layoutBtGraph(
  btNodes: BtGraphNode[],
  btEdges: BtGraphEdge[],
  collapsedIds: ReadonlySet<string> = new Set(),
  direction: LayoutDirection = "TB",
): LayoutResult {
  // Index nodes by id and by parentId.
  const byId = new Map<string, BtGraphNode>();
  const childrenByParent = new Map<string | "_root", BtGraphNode[]>();
  for (const n of btNodes) {
    byId.set(n.id, n);
    const key = n.parentId ?? "_root";
    if (!childrenByParent.has(key)) childrenByParent.set(key, []);
    childrenByParent.get(key)!.push(n);
  }

  // Compute the set of node ids that live inside any collapsed
  // container at any depth — these are hidden.
  const hiddenIds = new Set<string>();
  function markHidden(parentId: string) {
    const children = childrenByParent.get(parentId);
    if (!children) return;
    for (const c of children) {
      hiddenIds.add(c.id);
      markHidden(c.id);
    }
  }
  for (const id of collapsedIds) markHidden(id);

  const edgesByContainer = new Map<string | "_root", BtGraphEdge[]>();
  for (const e of btEdges) {
    const sParent = byId.get(e.source)?.parentId ?? "_root";
    const tParent = byId.get(e.target)?.parentId ?? "_root";
    // Edges should be drawn at the target's container scope (the
    // container's *own* incoming edge lives in its parent's scope).
    const scope = tParent === sParent ? sParent : tParent;
    if (!edgesByContainer.has(scope)) edgesByContainer.set(scope, []);
    edgesByContainer.get(scope)!.push(e);
  }

  // Container size cache, computed bottom-up.
  const sizes = new Map<string, { width: number; height: number }>();
  // Position of each node relative to its parent container (or
  // absolute if at root).
  const positions = new Map<string, { x: number; y: number }>();

  function sizeOf(nodeId: string): { width: number; height: number } {
    const cached = sizes.get(nodeId);
    if (cached) return cached;
    const node = byId.get(nodeId)!;
    if (node.containerKind) {
      if (collapsedIds.has(nodeId)) {
        const size = { width: COLLAPSED_WIDTH, height: COLLAPSED_HEIGHT };
        sizes.set(nodeId, size);
        return size;
      }
      const size = layoutContainer(nodeId);
      sizes.set(nodeId, size);
      return size;
    }
    const size = { width: LEAF_WIDTH, height: LEAF_HEIGHT };
    sizes.set(nodeId, size);
    return size;
  }

  function layoutContainer(containerId: string | "_root"): {
    width: number;
    height: number;
  } {
    const children = childrenByParent.get(containerId) ?? [];
    if (children.length === 0) {
      return { width: LEAF_WIDTH, height: LEAF_HEIGHT };
    }

    // If any child in this scope is an expanded container, loosen
    // node and rank separation for the whole scope so the expanded
    // frame doesn't crowd its siblings (and its own children get room
    // to breathe relative to the parent above). Collapsed containers
    // and plain leaves keep tight spacing.
    const hasExpandedChild = children.some(
      (c) => c.containerKind && !collapsedIds.has(c.id),
    );
    const nodesep = hasExpandedChild ? NODESEP_LOOSE : NODESEP_TIGHT;
    const ranksep = hasExpandedChild
      ? direction === "LR"
        ? RANKSEP_LR_LOOSE
        : RANKSEP_TB_LOOSE
      : direction === "LR"
        ? RANKSEP_LR_TIGHT
        : RANKSEP_TB_TIGHT;

    const g = new dagre.graphlib.Graph();
    g.setDefaultEdgeLabel(() => ({}));
    g.setGraph({
      rankdir: direction,
      nodesep,
      ranksep,
      marginx: 0,
      marginy: 0,
    });

    for (const child of children) {
      const s = sizeOf(child.id);
      g.setNode(child.id, { width: s.width, height: s.height });
    }

    const localEdges = edgesByContainer.get(containerId) ?? [];
    for (const e of localEdges) {
      const sourceParent = byId.get(e.source)?.parentId ?? "_root";
      if (
        sourceParent === containerId &&
        (byId.get(e.target)?.parentId ?? "_root") === containerId
      ) {
        g.setEdge(e.source, e.target);
      }
    }

    dagre.layout(g);

    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const child of children) {
      const pos = g.node(child.id);
      const s = sizeOf(child.id);
      const left = (pos?.x ?? 0) - s.width / 2;
      const top = (pos?.y ?? 0) - s.height / 2;
      const right = left + s.width;
      const bottom = top + s.height;
      if (left < minX) minX = left;
      if (top < minY) minY = top;
      if (right > maxX) maxX = right;
      if (bottom > maxY) maxY = bottom;
    }
    if (!Number.isFinite(minX)) {
      minX = 0;
      minY = 0;
      maxX = LEAF_WIDTH;
      maxY = LEAF_HEIGHT;
    }

    const offsetX =
      (containerId === "_root" ? 0 : CONTAINER_PAD) - minX;
    const offsetY =
      (containerId === "_root" ? 0 : CONTAINER_HEADER + CONTAINER_PAD) - minY;

    for (const child of children) {
      const pos = g.node(child.id);
      const s = sizeOf(child.id);
      positions.set(child.id, {
        x: (pos?.x ?? 0) - s.width / 2 + offsetX,
        y: (pos?.y ?? 0) - s.height / 2 + offsetY,
      });
    }

    if (containerId === "_root") {
      return { width: maxX - minX, height: maxY - minY };
    }
    return {
      width: maxX - minX + CONTAINER_PAD * 2,
      height: maxY - minY + CONTAINER_HEADER + CONTAINER_PAD * 2,
    };
  }

  layoutContainer("_root");

  const reactFlowNodes: LayoutResult["nodes"] = btNodes.map((node) => {
    const pos = positions.get(node.id) ?? { x: 0, y: 0 };
    const isContainer = !!node.containerKind;
    const isCollapsed = isContainer && collapsedIds.has(node.id);
    const size = isContainer ? sizes.get(node.id) : undefined;
    const data: BtGraphNode = isContainer
      ? { ...node, width: size?.width, height: size?.height, layoutDirection: direction }
      : { ...node, layoutDirection: direction };
    return {
      id: node.id,
      position: pos,
      data,
      type: isContainer ? "btContainer" : "btNode",
      parentId: node.parentId,
      extent: node.parentId ? "parent" : undefined,
      width: size?.width,
      height: size?.height,
      style: size ? { width: size.width, height: size.height } : undefined,
      hidden: hiddenIds.has(node.id),
      // Stash a small marker for the container component so it can show
      // a different header style when collapsed.
      ...(isCollapsed ? { className: "bt-container-collapsed" } : {}),
    };
  });

  reactFlowNodes.sort((a, b) => {
    const ad = depthOf(a.parentId, byId);
    const bd = depthOf(b.parentId, byId);
    return ad - bd;
  });

  const reactFlowEdges: LayoutResult["edges"] = btEdges.map((edge) => {
    const srcHidden = hiddenIds.has(edge.source);
    const tgtHidden = hiddenIds.has(edge.target);
    return {
      id: edge.id,
      source: edge.source,
      target: edge.target,
      type: "btEdge",
      animated: false,
      hidden: srcHidden || tgtHidden,
      data: {
        branchLabel: edge.branchLabel,
        ordinal: edge.ordinal,
        siblingCount: edge.siblingCount,
        style: edge.style,
        gate: edge.gate,
      },
    };
  });

  return { nodes: reactFlowNodes, edges: reactFlowEdges };
}

function depthOf(
  parentId: string | undefined,
  byId: Map<string, BtGraphNode>,
): number {
  let d = 0;
  let cur = parentId;
  while (cur) {
    d++;
    cur = byId.get(cur)?.parentId;
  }
  return d;
}
