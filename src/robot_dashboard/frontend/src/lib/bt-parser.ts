import type {
  BtGraphNode,
  BtGraphEdge,
  BtParsedTree,
  BtNodeCategory,
} from "../types/bt";
import { CONTROL_NODES, DECORATOR_NODES, CONDITION_NODES } from "../types/bt";

/**
 * Parse BehaviorTree.CPP v4 XML into a flat graph of nodes and edges
 * suitable for rendering with @xyflow/react.
 */
export function parseBtXml(xml: string): BtParsedTree {
  const parser = new DOMParser();
  const doc = parser.parseFromString(xml, "text/xml");

  const rootEl = doc.querySelector("root");
  if (!rootEl) throw new Error("Invalid BT XML: missing <root>");

  const mainTreeId =
    rootEl.getAttribute("main_tree_to_execute") ?? "MainTree";

  // Collect all <BehaviorTree> definitions
  const treeDefs = new Map<string, Element>();
  rootEl.querySelectorAll("BehaviorTree").forEach((bt) => {
    const id = bt.getAttribute("ID");
    if (id) treeDefs.set(id, bt);
  });

  const nodes: BtGraphNode[] = [];
  const edges: BtGraphEdge[] = [];
  let nextId = 0;

  function makeId(): string {
    return `bt_${nextId++}`;
  }

  function classifyNode(tagName: string): BtNodeCategory {
    if (tagName === "SubTree") return "subtree";
    if (CONTROL_NODES.has(tagName)) return "control";
    if (DECORATOR_NODES.has(tagName)) return "decorator";
    if (CONDITION_NODES.has(tagName) || tagName.includes("Condition"))
      return "condition";
    return "action";
  }

  function getParams(el: Element): Record<string, string> {
    const params: Record<string, string> = {};
    for (const attr of Array.from(el.attributes)) {
      if (attr.name === "name" || attr.name === "ID") continue;
      params[attr.name] = attr.value;
    }
    return params;
  }

  /**
   * Recursively walk a BT element and its children.
   * For SubTree nodes, inline-expand the referenced tree.
   */
  function walkElement(
    el: Element,
    treeId: string,
    parentId: string | null
  ): string | null {
    const tagName = el.tagName;

    // Skip non-BT elements (comments, text nodes handled by querySelectorAll)
    if (tagName === "root" || tagName === "BehaviorTree" || tagName === "TreeNodesModel") {
      // Process children of container elements
      for (const child of Array.from(el.children)) {
        walkElement(child, treeId, parentId);
      }
      return null;
    }

    const nodeId = makeId();
    const name =
      el.getAttribute("name") || el.getAttribute("ID") || tagName;
    const category = classifyNode(tagName);

    nodes.push({
      id: nodeId,
      category,
      btNodeType: tagName,
      name,
      treeId,
      params: getParams(el),
      state: "idle",
    });

    if (parentId) {
      edges.push({
        id: `e_${parentId}_${nodeId}`,
        source: parentId,
        target: nodeId,
      });
    }

    // For SubTree, inline-expand the referenced tree
    if (tagName === "SubTree") {
      const refId = el.getAttribute("ID");
      if (refId && treeDefs.has(refId)) {
        const subtreeDef = treeDefs.get(refId)!;
        for (const child of Array.from(subtreeDef.children)) {
          walkElement(child, refId, nodeId);
        }
      }
      return nodeId;
    }

    // Process children for control/decorator nodes
    for (const child of Array.from(el.children)) {
      walkElement(child, treeId, nodeId);
    }

    return nodeId;
  }

  // Start from the main tree
  const mainTree = treeDefs.get(mainTreeId);
  if (mainTree) {
    for (const child of Array.from(mainTree.children)) {
      walkElement(child, mainTreeId, null);
    }
  }

  return { mainTreeId, nodes, edges };
}

/**
 * Build a lookup map from node name -> node id for fast state updates.
 */
export function buildNameIndex(
  nodes: BtGraphNode[]
): Map<string, string> {
  const index = new Map<string, string>();
  for (const node of nodes) {
    // Use the first occurrence of each name
    if (!index.has(node.name)) {
      index.set(node.name, node.id);
    }
  }
  return index;
}
