import {
  TreePine,
  Activity,
  Puzzle,
  ScrollText,
  Radio,
  Rocket,
  Box,
  LineChart,
  PhoneCall,
  Gauge,
  HeartPulse,
} from "lucide-react";
import type { PanelId } from "../../stores/layout-types";

import BtTreePanel from "../bt-tree/BtTreePanel";
import TaskMonitorPanel from "../task-monitor/TaskMonitorPanel";
import SkillBrowserPanel from "../skill-browser/SkillBrowserPanel";
import LogPanel from "../logs/LogPanel";
import TopicListPanel from "../topics/TopicListPanel";
import BehaviorExecutorPanel from "../executor/BehaviorExecutorPanel";
import TfFrameViewer from "../tf-viewer/TfFrameViewer";
import TopicPlotter from "../plotter/TopicPlotter";
import ServiceCallerPanel from "../service-caller/ServiceCallerPanel";
import JointStateViewer from "../joint-viewer/JointStateViewer";
import DiagnosticsPanel from "../diagnostics/DiagnosticsPanel";

export interface PanelInfo {
  id: PanelId;
  label: string;
  icon: React.ReactNode;
  group: "behavior" | "monitor" | "tools";
  component: React.ReactNode;
}

export const PANELS: PanelInfo[] = [
  { id: "tree", label: "BT Tree", icon: <TreePine className="w-3.5 h-3.5" />, group: "behavior", component: <BtTreePanel /> },
  { id: "executor", label: "Execute", icon: <Rocket className="w-3.5 h-3.5" />, group: "behavior", component: <BehaviorExecutorPanel /> },
  { id: "monitor", label: "Task", icon: <Activity className="w-3.5 h-3.5" />, group: "behavior", component: <TaskMonitorPanel /> },
  { id: "skills", label: "Skills", icon: <Puzzle className="w-3.5 h-3.5" />, group: "behavior", component: <SkillBrowserPanel /> },
  { id: "joints", label: "Joints", icon: <Gauge className="w-3.5 h-3.5" />, group: "monitor", component: <JointStateViewer /> },
  { id: "tf", label: "TF", icon: <Box className="w-3.5 h-3.5" />, group: "monitor", component: <TfFrameViewer /> },
  { id: "topics", label: "Topics", icon: <Radio className="w-3.5 h-3.5" />, group: "tools", component: <TopicListPanel /> },
  { id: "plotter", label: "Plot", icon: <LineChart className="w-3.5 h-3.5" />, group: "tools", component: <TopicPlotter /> },
  { id: "services", label: "Services", icon: <PhoneCall className="w-3.5 h-3.5" />, group: "tools", component: <ServiceCallerPanel /> },
  { id: "logs", label: "Logs", icon: <ScrollText className="w-3.5 h-3.5" />, group: "monitor", component: <LogPanel /> },
  { id: "diagnostics", label: "Diagnostics", icon: <HeartPulse className="w-3.5 h-3.5" />, group: "monitor", component: <DiagnosticsPanel /> },
];

export const PANEL_MAP = Object.fromEntries(PANELS.map((p) => [p.id, p])) as Record<PanelId, PanelInfo>;
