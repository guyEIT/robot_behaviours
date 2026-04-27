import { useMemo, useCallback, useEffect, useRef } from "react";
import {
  ReactFlow,
  ReactFlowProvider,
  Background,
  Controls,
  MiniMap,
  useNodesState,
  useEdgesState,
  useReactFlow,
  useViewport,
  type Node,
  type Edge,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";

import BtNodeComponent from "./BtNode";
import BtContainerNodeComponent from "./BtContainerNode";
import BtEdgeComponent from "./BtEdge";
import { parseBtXml, buildNameIndex } from "../../lib/bt-parser";
import { layoutBtGraph } from "../../lib/bt-layout";
import type { BtGraphNode } from "../../types/bt";
import { useBtCollapseStore } from "../../stores/bt-collapse-store";

const nodeTypes = {
  btNode: BtNodeComponent,
  btContainer: BtContainerNodeComponent,
};
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
  const { setCenter, fitBounds } = useReactFlow();
  // Subscribe to viewport so the smart-follow logic re-evaluates after
  // the user pans or zooms.
  const viewport = useViewport();
  const wrapperRef = useRef<HTMLDivElement | null>(null);
  // Track the previously-active node id so we can detect when the
  // runner has *left* a container — that's our trigger for
  // auto-closing. Container completion isn't reliably reported via
  // `task_state.completed_skills` (which carries leaf skill names),
  // so leave-detection is the unambiguous signal.
  const prevActiveIdRef = useRef<string | null>(null);
  const expandedNames = useBtCollapseStore((s) => s.expandedNames);
  const autoExpandAll = useBtCollapseStore((s) => s.autoExpandAll);
  const autoExpandRunning = useBtCollapseStore((s) => s.autoExpandRunning);
  const autoCloseOnCompletion = useBtCollapseStore(
    (s) => s.autoCloseOnCompletion,
  );
  const autoExpand = useBtCollapseStore((s) => s.autoExpand);
  const autoCloseIfTracked = useBtCollapseStore((s) => s.autoCloseIfTracked);
  const direction = useBtCollapseStore((s) => s.direction);
  const smartFollow = useBtCollapseStore((s) => s.smartFollow);
  const focusName = useBtCollapseStore((s) => s.focusName);
  const clearFocus = useBtCollapseStore((s) => s.clearFocus);

  const {
    layoutNodes,
    layoutEdges,
    nameIndex,
    parentById,
    nameById,
    absPositionById,
  } = useMemo(() => {
      try {
        const parsed = parseBtXml(xml);
        // Build the set of collapsed container *ids*. `autoExpandAll`
        // overrides everything (no containers collapsed). Otherwise:
        // any container whose name is NOT in `expandedNames` is
        // collapsed (default-collapsed).
        const collapsedIds = new Set<string>();
        if (!autoExpandAll) {
          for (const n of parsed.nodes) {
            if (n.containerKind && !expandedNames[n.name]) {
              collapsedIds.add(n.id);
            }
          }
        }
        const layout = layoutBtGraph(
          parsed.nodes,
          parsed.edges,
          collapsedIds,
          direction,
        );
        const nameIndex = buildNameIndex(parsed.nodes);
        // Pre-compute parent ids and absolute positions from the layout,
        // independent of the ReactFlow node state — these are stable
        // across state-only updates and avoid effect re-fire loops.
        const parentById = new Map<string, string | undefined>();
        const nameById = new Map<string, string>();
        const sizeById = new Map<string, { w: number; h: number }>();
        const relPosById = new Map<string, { x: number; y: number }>();
        for (const n of layout.nodes) {
          parentById.set(n.id, n.parentId);
          nameById.set(n.id, (n.data as unknown as BtGraphNode).name);
          sizeById.set(n.id, {
            w: (n.width as number | undefined) ?? 200,
            h: (n.height as number | undefined) ?? 64,
          });
          relPosById.set(n.id, { x: n.position.x, y: n.position.y });
        }
        const absPositionById = new Map<
          string,
          { x: number; y: number; w: number; h: number }
        >();
        for (const id of relPosById.keys()) {
          let { x, y } = relPosById.get(id)!;
          let parent = parentById.get(id);
          while (parent) {
            const p = relPosById.get(parent);
            if (!p) break;
            x += p.x;
            y += p.y;
            parent = parentById.get(parent);
          }
          const s = sizeById.get(id)!;
          absPositionById.set(id, { x, y, w: s.w, h: s.h });
        }
        return {
          layoutNodes: layout.nodes,
          layoutEdges: layout.edges,
          nameIndex,
          parentById,
          nameById,
          absPositionById,
        };
      } catch {
        return {
          layoutNodes: [],
          layoutEdges: [],
          nameIndex: new Map<string, string>(),
          parentById: new Map<string, string | undefined>(),
          nameById: new Map<string, string>(),
          absPositionById: new Map<
            string,
            { x: number; y: number; w: number; h: number }
          >(),
        };
      }
    }, [xml, expandedNames, autoExpandAll, direction]);

  const [nodes, setNodes, onNodesChange] = useNodesState(layoutNodes as unknown as Node[]);
  const [edges, setEdges, onEdgesChange] = useEdgesState(layoutEdges as Edge[]);

  useEffect(() => {
    const activeId = activeNodeName ? nameIndex.get(activeNodeName) ?? null : null;
    const completedIds = new Set(
      completedNodeNames.map((n) => nameIndex.get(n)).filter(Boolean) as string[],
    );
    const failedIds = new Set(
      failedNodeNames.map((n) => nameIndex.get(n)).filter(Boolean) as string[],
    );

    setNodes((prev) =>
      prev.map((node) => {
        const data = node.data as unknown as BtGraphNode;
        let newState: BtGraphNode["state"] = "idle";
        if (failedIds.has(node.id)) newState = "failure";
        if (completedIds.has(node.id)) newState = "success";
        if (node.id === activeId) newState = "running";

        // Bubble running state up through containers when a descendant
        // is the active node. A container shows "running" while any
        // descendant ticks; success/failure is only set if the
        // container's own name appears in completed/failed.
        if (data.containerKind && newState === "idle" && activeId) {
          let cur: string | undefined = activeId;
          while (cur) {
            if (cur === node.id) {
              newState = "running";
              break;
            }
            cur = parentById.get(cur);
          }
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

    // Auto-expand ancestors of the running node — sticky, so the
    // animation is visible inside its enclosing frames as the runner
    // moves through them. Independent of `followActive`: the camera
    // toggle (followActive) is for centering, the structure toggle
    // (autoExpandRunning) is for visibility.
    if (autoExpandRunning && activeId) {
      let cur: string | undefined = parentById.get(activeId);
      while (cur) {
        const aname = nameById.get(cur);
        if (aname && !expandedNames[aname]) autoExpand(aname);
        cur = parentById.get(cur);
      }
    }

    // Close on completion: any auto-opened container whose runner has
    // just *left* (i.e., was an ancestor of the previous active node
    // but is not an ancestor of the current one) gets collapsed. This
    // works regardless of whether the container's name appears in
    // `task_state.completed_skills` — the runner moving past it is
    // the unambiguous signal. We also still check the completed/
    // failed lists for the rare case where a container's name *does*
    // appear there (covers nested-iteration edge cases).
    //
    // Skipped entirely while `autoExpandAll` is on — "show all"
    // overrides any close logic.
    const prevActiveId = prevActiveIdRef.current;
    if (autoCloseOnCompletion && !autoExpandAll) {
      const ancestorsOf = (id: string | null): Set<string> => {
        const out = new Set<string>();
        let cur = id ? parentById.get(id) : undefined;
        while (cur) {
          out.add(cur);
          cur = parentById.get(cur);
        }
        return out;
      };
      const prevAncestors = ancestorsOf(prevActiveId);
      const currAncestors = ancestorsOf(activeId);
      for (const ancestorId of prevAncestors) {
        if (!currAncestors.has(ancestorId)) {
          const aname = nameById.get(ancestorId);
          if (aname) autoCloseIfTracked(aname);
        }
      }
      for (const completedName of completedNodeNames) {
        autoCloseIfTracked(completedName);
      }
      for (const failedName of failedNodeNames) {
        autoCloseIfTracked(failedName);
      }
    }
    // Update the previous-active marker AFTER computing closes.
    prevActiveIdRef.current = activeId;

    if (followActive && activeId) {
      const p = absPositionById.get(activeId);
      if (p) {
        // Smart follow: only pan when the active node has crept
        // outside the visible viewport. With a longer ease, this
        // gives the eye time to adjust during fast execution.
        const wrapper = wrapperRef.current;
        let needsPan = true;
        if (smartFollow && wrapper) {
          const rect = wrapper.getBoundingClientRect();
          const { x: vx, y: vy, zoom } = viewport;
          // Convert flow-space node bounds to pixel-space inside the wrapper.
          const px = p.x * zoom + vx;
          const py = p.y * zoom + vy;
          const pw = p.w * zoom;
          const ph = p.h * zoom;
          // Padding from the edges before we consider the node "off".
          const pad = 60;
          const onScreen =
            px >= pad &&
            py >= pad &&
            px + pw <= rect.width - pad &&
            py + ph <= rect.height - pad;
          needsPan = !onScreen;
        }
        if (needsPan) {
          // Hold the user's current zoom (don't snap to 1.2 every tick).
          const targetZoom = Math.max(viewport.zoom, 0.6);
          // Smart-follow already gates how often the pan fires (only
          // when the active node has crept offscreen), so the pan
          // itself can be brisk — slow enough to track, quick enough
          // to keep up with fast execution. Without smart-follow,
          // every state tick triggers a pan, so we keep that brisk
          // too rather than queuing long overlapping eases.
          setCenter(p.x + p.w / 2, p.y + p.h / 2, {
            duration: 280,
            zoom: targetZoom,
          });
        }
      }
    }
  }, [
    activeNodeName,
    completedNodeNames,
    failedNodeNames,
    nameIndex,
    parentById,
    nameById,
    absPositionById,
    setNodes,
    setEdges,
    followActive,
    setCenter,
    autoExpandRunning,
    autoExpandAll,
    autoCloseOnCompletion,
    autoExpand,
    autoCloseIfTracked,
    expandedNames,
    smartFollow,
    viewport,
  ]);

  useEffect(() => {
    setNodes(layoutNodes as unknown as Node[]);
    setEdges(layoutEdges as Edge[]);
  }, [layoutNodes, layoutEdges, setNodes, setEdges]);

  // After a user-driven expand, fit the camera to the container's new
  // bounds so the newly-revealed children land in the viewport — no
  // manual scrolling required. This runs after the layout effect
  // above so absPositionById already reflects the post-expansion
  // positions. We also gate on the container actually being in the
  // current layout (focusName might refer to a SubTree from a tree
  // we're no longer viewing).
  useEffect(() => {
    if (!focusName) return;
    const id = nameIndex.get(focusName);
    if (!id) {
      clearFocus();
      return;
    }
    const b = absPositionById.get(id);
    if (!b) {
      clearFocus();
      return;
    }
    // Pad ~10% so children aren't flush with the viewport edge. The
    // ease is short — this is a deliberate user action, not a
    // background pan, so it should feel responsive.
    fitBounds(
      { x: b.x, y: b.y, width: b.w, height: b.h },
      { padding: 0.1, duration: 320 },
    );
    clearFocus();
  }, [focusName, nameIndex, absPositionById, fitBounds, clearFocus]);

  const onInit = useCallback((instance: any) => {
    instance.fitView({ padding: 0.1 });
  }, []);

  return (
    <div ref={wrapperRef} className="h-full w-full">
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
    </div>
  );
}

export default function BtTreeGraph(props: Props) {
  return (
    <ReactFlowProvider>
      <BtTreeGraphInner {...props} />
    </ReactFlowProvider>
  );
}
