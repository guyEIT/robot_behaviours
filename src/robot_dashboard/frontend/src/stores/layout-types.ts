export type PanelId =
  | "tree"
  | "executor"
  | "monitor"
  | "skills"
  | "joints"
  | "tf"
  | "topics"
  | "plotter"
  | "services"
  | "logs"
  | "diagnostics"
  | "intervention"
  | "human-prompts"
  | "campaign";

export interface LayoutLeaf {
  type: "leaf";
  panelId: PanelId;
  size: number;
}

export interface LayoutSplit {
  type: "split";
  direction: "horizontal" | "vertical";
  size: number;
  children: LayoutNode[];
}

export type LayoutNode = LayoutLeaf | LayoutSplit;

export interface SavedLayout {
  name: string;
  tree: LayoutNode;
  createdAt: number;
}
