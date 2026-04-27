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
  ShieldAlert,
  MessageSquareMore,
  FlaskConical,
  Database,
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
import InterventionPanel from "../intervention/InterventionPanel";
import HumanPromptPanel from "../human-prompts/HumanPromptPanel";
import CampaignPanel from "../campaign/CampaignPanel";
import BbViewPanel from "../bb-view/BbViewPanel";

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
  { id: "campaign", label: "Campaign", icon: <FlaskConical className="w-3.5 h-3.5" />, group: "behavior", component: <CampaignPanel /> },
  { id: "blackboard", label: "Blackboard", icon: <Database className="w-3.5 h-3.5" />, group: "behavior", component: <BbViewPanel /> },
  { id: "joints", label: "Joints", icon: <Gauge className="w-3.5 h-3.5" />, group: "monitor", component: <JointStateViewer /> },
  { id: "tf", label: "TF", icon: <Box className="w-3.5 h-3.5" />, group: "monitor", component: <TfFrameViewer /> },
  { id: "topics", label: "Topics", icon: <Radio className="w-3.5 h-3.5" />, group: "tools", component: <TopicListPanel /> },
  { id: "plotter", label: "Plot", icon: <LineChart className="w-3.5 h-3.5" />, group: "tools", component: <TopicPlotter /> },
  { id: "services", label: "Services", icon: <PhoneCall className="w-3.5 h-3.5" />, group: "tools", component: <ServiceCallerPanel /> },
  { id: "logs", label: "Logs", icon: <ScrollText className="w-3.5 h-3.5" />, group: "monitor", component: <LogPanel /> },
  { id: "diagnostics", label: "Diagnostics", icon: <HeartPulse className="w-3.5 h-3.5" />, group: "monitor", component: <DiagnosticsPanel /> },
  { id: "intervention", label: "Intervene", icon: <ShieldAlert className="w-3.5 h-3.5" />, group: "behavior", component: <InterventionPanel /> },
  { id: "human-prompts", label: "Human", icon: <MessageSquareMore className="w-3.5 h-3.5" />, group: "behavior", component: <HumanPromptPanel /> },
];

export const PANEL_MAP = Object.fromEntries(PANELS.map((p) => [p.id, p])) as Record<PanelId, PanelInfo>;
