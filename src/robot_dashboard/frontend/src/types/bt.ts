export type BtNodeCategory =
  | "control"
  | "decorator"
  | "action"
  | "condition"
  | "subtree"
  | "container";

export type BtNodeState = "idle" | "running" | "success" | "failure";

/** Visually distinct leaf sub-categories for actions and condition nodes. */
export type BtLeafCategory =
  | "skill"
  | "human"
  | "queue"
  | "timing"
  | "pose-tf"
  | "io"
  | "lease"
  | "log"
  | "blackboard"
  | "condition"
  | "default";

/** Container kinds wrap children in a framed group with a header bar. */
export type BtContainerKind =
  | "repeat"
  | "retry"
  | "keep_until_fail"
  | "subtree";

/** Sub-classification of control nodes — drives header motif. */
export type BtControlKind =
  | "sequence"
  | "reactive_sequence"
  | "sequence_memory"
  | "fallback"
  | "reactive_fallback"
  | "parallel"
  | "if_then_else"
  | "while_do_else";

/** A single decorator collapsed onto its child as a chip. */
export interface BtDecoratorChip {
  kind: "inverter" | "timeout" | "force_success" | "force_failure" | "delay" | "run_once";
  /** Display label, e.g. "!", "⏱ 5s", "✓→", "✗→". */
  label: string;
  /** Original BT XML node type, for the hover tooltip. */
  btNodeType: string;
  /** Raw params for tooltip detail. */
  params: Record<string, string>;
}

/**
 * BlackboardCondition collapsed onto the edge to its single gated child.
 * `key == expected` rendered as a pill on the edge.
 */
export interface BtEdgeGate {
  key: string;
  expected: string;
}

/** A curated key/value pair surfaced in the node header. */
export interface BtDisplayParam {
  /** Short label or symbol, e.g. "× 3", "↻ 2", "∞", "until +30s". */
  label: string;
  /** Optional title/tooltip for hover. */
  title?: string;
}

/** A node in the rendered ReactFlow graph. */
export interface BtGraphNode {
  id: string;
  category: BtNodeCategory;
  btNodeType: string; // e.g. "Sequence", "MoveToNamedConfig"
  name: string; // name="" attribute from XML
  treeId: string; // which <BehaviorTree ID="..."> this belongs to
  params: Record<string, string>;
  state: BtNodeState;
  /** Set on group/container nodes (Repeat / KeepRunningUntilFailure / RetryUntilSuccessful / SubTree). */
  containerKind?: BtContainerKind;
  /** Set on control leaves (Sequence/Fallback/Parallel/etc.) — drives header motif. */
  controlKind?: BtControlKind;
  /** Set on action/condition leaves — drives leaf icon and tint. */
  leafCategory?: BtLeafCategory;
  /** Curated display params shown in the header (compact, type-specific). */
  displayParams?: BtDisplayParam[];
  /** Single-child decorators collapsed onto this node. */
  decorators?: BtDecoratorChip[];
  /** ReactFlow parent grouping — present when the node lives inside a container. */
  parentId?: string;
  /** Computed by layout for container nodes. */
  width?: number;
  height?: number;
  /** Layout direction propagated from `layoutBtGraph` so node components
   *  can pick the right handle positions ("TB" → top/bottom, "LR" → left/right). */
  layoutDirection?: "TB" | "LR";
}

export interface BtGraphEdge {
  id: string;
  source: string;
  target: string;
  /** Optional decoration: `cond`/`then`/`else`/`do` label for IfThenElse / WhileDoElse. */
  branchLabel?: "cond" | "then" | "else" | "do";
  /** 1-based ordinal among siblings; rendered as a small numeral on the edge. */
  ordinal?: number;
  /** Total number of siblings. Used to suppress ordinal display for single-child edges. */
  siblingCount?: number;
  /** Style hint: dashed for Fallback "alternate" edges. */
  style?: "default" | "dashed";
  /** BlackboardCondition gate collapsed onto this edge. */
  gate?: BtEdgeGate;
}

export interface BtParsedTree {
  mainTreeId: string;
  nodes: BtGraphNode[];
  edges: BtGraphEdge[];
}

/** Recursive intermediate representation produced by the parser before flattening. */
export interface BtTreeIR {
  id: string;
  tagName: string;
  name: string;
  treeId: string;
  params: Record<string, string>;
  children: BtTreeIR[];
}

/** Known BT node types by category. */
export const CONTROL_NODES = new Set([
  "Sequence",
  "Fallback",
  "Parallel",
  "ReactiveSequence",
  "ReactiveFallback",
  "SequenceWithMemory",
  "IfThenElse",
  "WhileDoElse",
]);

/** Decorators that *can* collapse onto a single child as a chip. */
export const DECORATOR_NODES = new Set([
  "Inverter",
  "ForceSuccess",
  "ForceFailure",
  "Timeout",
  "Delay",
  "RunOnce",
]);

/** Decorators that get a framed container treatment instead of a chip. */
export const CONTAINER_LOOP_NODES: Record<string, BtContainerKind> = {
  Repeat: "repeat",
  RetryUntilSuccessful: "retry",
  KeepRunningUntilFailure: "keep_until_fail",
};

export const CONDITION_NODES = new Set([
  "ScriptCondition",
  "Condition",
  "BlackboardCondition",
]);

/** Map XML tag name to its sub-classification for header motifs. */
export const CONTROL_KIND_BY_TAG: Record<string, BtControlKind> = {
  Sequence: "sequence",
  ReactiveSequence: "reactive_sequence",
  SequenceWithMemory: "sequence_memory",
  Fallback: "fallback",
  ReactiveFallback: "reactive_fallback",
  Parallel: "parallel",
  IfThenElse: "if_then_else",
  WhileDoElse: "while_do_else",
};
