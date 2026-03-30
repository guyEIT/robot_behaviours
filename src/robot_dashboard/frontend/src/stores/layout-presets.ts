import type { LayoutNode } from "./layout-types";

export const PRESET_LAYOUTS: Record<string, LayoutNode> = {
  Development: {
    type: "split",
    direction: "vertical",
    size: 100,
    children: [
      {
        type: "split",
        direction: "horizontal",
        size: 70,
        children: [
          { type: "leaf", panelId: "tree", size: 50 },
          { type: "leaf", panelId: "executor", size: 50 },
        ],
      },
      {
        type: "split",
        direction: "horizontal",
        size: 30,
        children: [
          { type: "leaf", panelId: "skills", size: 40 },
          { type: "leaf", panelId: "logs", size: 60 },
        ],
      },
    ],
  },

  Execution: {
    type: "split",
    direction: "horizontal",
    size: 100,
    children: [
      {
        type: "split",
        direction: "vertical",
        size: 55,
        children: [
          { type: "leaf", panelId: "tree", size: 60 },
          { type: "leaf", panelId: "monitor", size: 40 },
        ],
      },
      {
        type: "split",
        direction: "vertical",
        size: 45,
        children: [
          { type: "leaf", panelId: "executor", size: 25 },
          { type: "leaf", panelId: "tf", size: 40 },
          { type: "leaf", panelId: "logs", size: 35 },
        ],
      },
    ],
  },

  Monitoring: {
    type: "split",
    direction: "horizontal",
    size: 100,
    children: [
      {
        type: "split",
        direction: "vertical",
        size: 50,
        children: [
          { type: "leaf", panelId: "topics", size: 50 },
          { type: "leaf", panelId: "services", size: 50 },
        ],
      },
      {
        type: "split",
        direction: "vertical",
        size: 50,
        children: [
          { type: "leaf", panelId: "plotter", size: 50 },
          { type: "leaf", panelId: "logs", size: 50 },
        ],
      },
    ],
  },
};
