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

  Campaign: {
    type: "split",
    direction: "horizontal",
    size: 100,
    children: [
      {
        type: "split",
        direction: "vertical",
        size: 50,
        children: [
          { type: "leaf", panelId: "campaign", size: 60 },
          { type: "leaf", panelId: "monitor", size: 40 },
        ],
      },
      {
        type: "split",
        direction: "vertical",
        size: 25,
        children: [
          { type: "leaf", panelId: "tree", size: 60 },
          { type: "leaf", panelId: "blackboard", size: 40 },
        ],
      },
      {
        type: "split",
        direction: "vertical",
        size: 25,
        children: [
          { type: "leaf", panelId: "executor", size: 50 },
          { type: "leaf", panelId: "logs", size: 50 },
        ],
      },
    ],
  },

  Debug: {
    type: "split",
    direction: "horizontal",
    size: 100,
    children: [
      { type: "leaf", panelId: "tree", size: 50 },
      {
        type: "split",
        direction: "vertical",
        size: 50,
        children: [
          { type: "leaf", panelId: "blackboard", size: 60 },
          { type: "leaf", panelId: "logs", size: 40 },
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
        size: 35,
        children: [
          { type: "leaf", panelId: "diagnostics", size: 60 },
          { type: "leaf", panelId: "services", size: 40 },
        ],
      },
      {
        type: "split",
        direction: "vertical",
        size: 35,
        children: [
          { type: "leaf", panelId: "topics", size: 50 },
          { type: "leaf", panelId: "plotter", size: 50 },
        ],
      },
      {
        type: "split",
        direction: "vertical",
        size: 30,
        children: [
          { type: "leaf", panelId: "joints", size: 50 },
          { type: "leaf", panelId: "logs", size: 50 },
        ],
      },
    ],
  },
};
