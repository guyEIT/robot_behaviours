export type BtNodeCategory =
  | "control"
  | "decorator"
  | "action"
  | "condition"
  | "subtree";

export type BtNodeState = "idle" | "running" | "success" | "failure";

export interface BtGraphNode {
  id: string;
  category: BtNodeCategory;
  btNodeType: string; // e.g. "Sequence", "MoveToNamedConfig"
  name: string; // name="" attribute from XML
  treeId: string; // which <BehaviorTree ID="..."> this belongs to
  params: Record<string, string>;
  state: BtNodeState;
}

export interface BtGraphEdge {
  id: string;
  source: string;
  target: string;
}

export interface BtParsedTree {
  mainTreeId: string;
  nodes: BtGraphNode[];
  edges: BtGraphEdge[];
}

/** Known BT node types by category */
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

export const DECORATOR_NODES = new Set([
  "RetryUntilSuccessful",
  "Repeat",
  "Inverter",
  "ForceSuccess",
  "ForceFailure",
  "Timeout",
  "Delay",
  "KeepRunningUntilFailure",
  "RunOnce",
]);

export const CONDITION_NODES = new Set([
  "ScriptCondition",
  "Condition",
]);
