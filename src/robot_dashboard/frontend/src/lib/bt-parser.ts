import type {
  BtGraphNode,
  BtGraphEdge,
  BtParsedTree,
  BtNodeCategory,
  BtTreeIR,
  BtDecoratorChip,
  BtDisplayParam,
} from "../types/bt";
import {
  CONTROL_NODES,
  DECORATOR_NODES,
  CONDITION_NODES,
  CONTAINER_LOOP_NODES,
  CONTROL_KIND_BY_TAG,
} from "../types/bt";
import { leafCategoryFor } from "./bt-categorize";

/**
 * Parse BehaviorTree.CPP v4 XML into a flat ReactFlow-ready graph plus
 * derived metadata (decorator chips, edge gates, container groups, curated
 * display params). The pipeline is:
 *
 *   XML  ──▶  BtTreeIR (recursive)  ──▶  flat nodes + edges with parentIds
 *
 * The IR pass is what enables container detection and chip/gate collapse;
 * the flatten pass is what ReactFlow consumes.
 */
export function parseBtXml(xml: string): BtParsedTree {
  const ir = parseBtXmlToIR(xml);
  return flattenIRForReactFlow(ir);
}

interface IRBundle {
  mainTreeId: string;
  rootIRs: BtTreeIR[];
}

/**
 * Phase 1: produce a recursive IR tree. SubTrees are inlined via the same
 * mechanism as before (the referenced tree's children become children of
 * the SubTree node), so consumers don't need to chase cross-references.
 */
export function parseBtXmlToIR(xml: string): IRBundle {
  const parser = new DOMParser();
  const doc = parser.parseFromString(xml, "text/xml");

  const rootEl = doc.querySelector("root");
  if (!rootEl) throw new Error("Invalid BT XML: missing <root>");

  const mainTreeId = rootEl.getAttribute("main_tree_to_execute") ?? "MainTree";

  const treeDefs = new Map<string, Element>();
  rootEl.querySelectorAll("BehaviorTree").forEach((bt) => {
    const id = bt.getAttribute("ID");
    if (id) treeDefs.set(id, bt);
  });

  let nextId = 0;
  const makeId = () => `bt_${nextId++}`;

  function getParams(el: Element): Record<string, string> {
    const params: Record<string, string> = {};
    for (const attr of Array.from(el.attributes)) {
      if (attr.name === "name" || attr.name === "ID") continue;
      params[attr.name] = attr.value;
    }
    return params;
  }

  function buildIR(el: Element, treeId: string): BtTreeIR {
    const tagName = el.tagName;
    const name = el.getAttribute("name") || el.getAttribute("ID") || tagName;

    const node: BtTreeIR = {
      id: makeId(),
      tagName,
      name,
      treeId,
      params: getParams(el),
      children: [],
    };

    if (tagName === "SubTree") {
      const refId = el.getAttribute("ID");
      if (refId && treeDefs.has(refId)) {
        const subtreeDef = treeDefs.get(refId)!;
        for (const child of Array.from(subtreeDef.children)) {
          node.children.push(buildIR(child, refId));
        }
      }
      return node;
    }

    for (const child of Array.from(el.children)) {
      node.children.push(buildIR(child, treeId));
    }
    return node;
  }

  const rootIRs: BtTreeIR[] = [];
  const mainTree = treeDefs.get(mainTreeId);
  if (mainTree) {
    for (const child of Array.from(mainTree.children)) {
      rootIRs.push(buildIR(child, mainTreeId));
    }
  }

  return { mainTreeId, rootIRs };
}

/**
 * Phase 2: walk the IR and emit ReactFlow-shape nodes/edges, applying the
 * collapse rules:
 *   - Loop decorators (Repeat / RetryUntilSuccessful / KeepRunningUntilFailure)
 *     and SubTree become container/group nodes; their children get parentId.
 *   - Single-child decorators (Inverter / Timeout / ForceSuccess /
 *     ForceFailure / Delay / RunOnce) are dropped from the graph and
 *     attached as `decorators` chips on the wrapped child.
 *   - BlackboardCondition with exactly one child (the gating idiom) is
 *     dropped from the graph; its `key == expected` becomes an `edgeGate`
 *     on the edge from the parent to that child. Multi-child or no-child
 *     usages stay as regular nodes.
 *   - Curated `displayParams` per type are attached for header rendering.
 */
function flattenIRForReactFlow(bundle: IRBundle): BtParsedTree {
  const nodes: BtGraphNode[] = [];
  const edges: BtGraphEdge[] = [];

  /**
   * Recursively emit ReactFlow nodes for an IR subtree.
   * Returns the id of the *visible* root of the subtree (or null if the
   * whole subtree was collapsed away — should not happen in practice).
   */
  function emit(
    ir: BtTreeIR,
    parentNodeId: string | null,
    parentContainerId: string | null,
    decoratorStack: BtDecoratorChip[],
    edgeMeta: Partial<BtGraphEdge> = {},
  ): string | null {
    const { tagName, params } = ir;

    // Collapse single-child decorators into a chip on the wrapped child.
    if (DECORATOR_NODES.has(tagName) && ir.children.length === 1) {
      const chip = decoratorChipFor(tagName, params);
      return emit(
        ir.children[0],
        parentNodeId,
        parentContainerId,
        [...decoratorStack, chip],
        edgeMeta,
      );
    }

    // Collapse single-child BlackboardCondition into an edge gate.
    if (
      tagName === "BlackboardCondition" &&
      ir.children.length === 1 &&
      params.key !== undefined
    ) {
      const gateMeta: Partial<BtGraphEdge> = {
        ...edgeMeta,
        gate: { key: params.key, expected: params.expected ?? "true" },
      };
      return emit(
        ir.children[0],
        parentNodeId,
        parentContainerId,
        decoratorStack,
        gateMeta,
      );
    }

    const category = classify(tagName);
    const containerKind = CONTAINER_LOOP_NODES[tagName] ??
      (tagName === "SubTree" ? "subtree" : undefined);
    const isContainer = containerKind !== undefined;

    const node: BtGraphNode = {
      id: ir.id,
      category: isContainer ? "container" : category,
      btNodeType: tagName,
      name: ir.name,
      treeId: ir.treeId,
      params,
      state: "idle",
      containerKind,
      controlKind: CONTROL_KIND_BY_TAG[tagName],
      leafCategory:
        category === "action" || category === "condition"
          ? leafCategoryFor(tagName)
          : undefined,
      displayParams: buildDisplayParams(tagName, params, ir.name),
      decorators: decoratorStack.length > 0 ? decoratorStack : undefined,
      parentId: parentContainerId ?? undefined,
    };
    nodes.push(node);

    if (parentNodeId) {
      edges.push({
        id: `e_${parentNodeId}_${node.id}`,
        source: parentNodeId,
        target: node.id,
        ...edgeMeta,
      });
    }

    // Recurse into children. For containers, children get a parentId of
    // this node (ReactFlow grouping). Non-containers pass through their
    // own parent's container id.
    const childContainerId = isContainer ? node.id : parentContainerId;
    const visibleChildren: string[] = [];
    const childCount = ir.children.length;

    ir.children.forEach((child, idx) => {
      const childEdgeMeta = childEdgeMetaFor(tagName, idx, childCount);
      const childId = emit(
        child,
        node.id,
        childContainerId,
        [],
        childEdgeMeta,
      );
      if (childId) visibleChildren.push(childId);
    });

    return node.id;
  }

  for (const root of bundle.rootIRs) {
    emit(root, null, null, [], {});
  }

  return { mainTreeId: bundle.mainTreeId, nodes, edges };
}

function classify(tagName: string): BtNodeCategory {
  if (tagName === "SubTree") return "subtree";
  if (tagName in CONTAINER_LOOP_NODES) return "container";
  if (CONTROL_NODES.has(tagName)) return "control";
  if (DECORATOR_NODES.has(tagName)) return "decorator";
  if (CONDITION_NODES.has(tagName) || tagName.includes("Condition"))
    return "condition";
  return "action";
}

function decoratorChipFor(
  tagName: string,
  params: Record<string, string>,
): BtDecoratorChip {
  switch (tagName) {
    case "Inverter":
      return { kind: "inverter", label: "!", btNodeType: tagName, params };
    case "ForceSuccess":
      return { kind: "force_success", label: "✓→", btNodeType: tagName, params };
    case "ForceFailure":
      return { kind: "force_failure", label: "✗→", btNodeType: tagName, params };
    case "Timeout": {
      const ms = params.msec ?? params.timeout_ms ?? params.seconds;
      const label = ms ? `⏱ ${ms}` : "⏱";
      return { kind: "timeout", label, btNodeType: tagName, params };
    }
    case "Delay": {
      const ms = params.delay_msec ?? params.msec ?? params.seconds;
      const label = ms ? `⏲ ${ms}` : "⏲";
      return { kind: "delay", label, btNodeType: tagName, params };
    }
    case "RunOnce":
      return { kind: "run_once", label: "1×", btNodeType: tagName, params };
    default:
      return { kind: "inverter", label: tagName, btNodeType: tagName, params };
  }
}

/**
 * Decorate the edge from a parent control node to its idx-th child.
 * Sequence/Fallback get sibling ordinals; Fallback gets dashed style on
 * non-first children; IfThenElse / WhileDoElse get cond/then/else labels.
 */
function childEdgeMetaFor(
  parentTag: string,
  idx: number,
  total: number,
): Partial<BtGraphEdge> {
  if (parentTag === "IfThenElse") {
    if (idx === 0) return { branchLabel: "cond" };
    if (idx === 1) return { branchLabel: "then" };
    return { branchLabel: "else" };
  }
  if (parentTag === "WhileDoElse") {
    if (idx === 0) return { branchLabel: "cond" };
    if (idx === 1) return { branchLabel: "do" };
    return { branchLabel: "else" };
  }
  const isSequence =
    parentTag === "Sequence" ||
    parentTag === "ReactiveSequence" ||
    parentTag === "SequenceWithMemory";
  const isFallback = parentTag === "Fallback" || parentTag === "ReactiveFallback";
  if (isSequence || isFallback) {
    return {
      ordinal: idx + 1,
      siblingCount: total,
      style: isFallback && idx > 0 ? "dashed" : "default",
    };
  }
  return {};
}

/**
 * Header-bar params: small, type-specific labels. Returns undefined for
 * types where the existing all-params hover tooltip is enough.
 */
function buildDisplayParams(
  tagName: string,
  params: Record<string, string>,
  _name: string,
): BtDisplayParam[] | undefined {
  const out: BtDisplayParam[] = [];
  switch (tagName) {
    case "Repeat":
      if (params.num_cycles) out.push({ label: `× ${params.num_cycles}` });
      break;
    case "RetryUntilSuccessful":
      if (params.num_attempts) out.push({ label: `↻ ${params.num_attempts}` });
      break;
    case "KeepRunningUntilFailure":
      out.push({ label: "∞" });
      break;
    case "Parallel":
      if (params.success_count)
        out.push({ label: `success ≥ ${params.success_count}` });
      if (params.failure_count)
        out.push({ label: `failure ≥ ${params.failure_count}` });
      break;
    case "BlackboardCondition":
      if (params.key !== undefined) {
        const expected = params.expected ?? "true";
        const truncated = expected.length > 24 ? expected.slice(0, 22) + "…" : expected;
        out.push({
          label: `${params.key} == ${truncated}`,
          title: `${params.key} == ${expected}`,
        });
      }
      break;
    case "WaitForDuration": {
      const s = params.seconds ?? params.duration ?? params.msec;
      if (s) out.push({ label: `${s}s` });
      break;
    }
    case "WaitUntil": {
      if (params.timestamp) {
        out.push({
          label: `until ${shortenTimestamp(params.timestamp)}`,
          title: params.timestamp,
        });
      }
      break;
    }
    case "Timeout": {
      const ms = params.msec ?? params.timeout_ms;
      if (ms) out.push({ label: `⏱ ${ms}ms` });
      break;
    }
    case "Delay": {
      const ms = params.delay_msec ?? params.msec;
      if (ms) out.push({ label: `⏲ ${ms}ms` });
      break;
    }
    case "PopFromQueue":
    case "PushToQueue": {
      const q = params.key ?? params._q ?? params.queue;
      if (q) out.push({ label: `q: ${q}` });
      break;
    }
    case "SubTree":
      // SubTree's container header displays the ID via name; nothing extra here.
      break;
  }
  return out.length > 0 ? out : undefined;
}

function shortenTimestamp(ts: string): string {
  // BB references like "{persistent.current_plate.next_due_at}" → keep as-is.
  if (ts.startsWith("{")) return ts;
  // Numeric epoch → render as a short relative-ish hint.
  const n = Number(ts);
  if (!Number.isFinite(n)) return ts;
  return `${n}`;
}

/**
 * Build a lookup map from node name -> node id for fast state updates.
 * The `name` axis is what `task_state.current_bt_node` and
 * `completed_skills` / `failed_skills` use to refer to a node.
 */
export function buildNameIndex(
  nodes: BtGraphNode[],
): Map<string, string> {
  const index = new Map<string, string>();
  for (const node of nodes) {
    if (!index.has(node.name)) {
      index.set(node.name, node.id);
    }
  }
  return index;
}
