import { useState, type ReactNode } from "react";
import {
  TreePine,
  Activity,
  Puzzle,
  ScrollText,
  Radio,
  Wifi,
  WifiOff,
  Rocket,
  Box,
  LineChart,
  PhoneCall,
  Gauge,
} from "lucide-react";
import clsx from "clsx";

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

type TabId =
  | "tree"
  | "executor"
  | "monitor"
  | "skills"
  | "joints"
  | "tf"
  | "topics"
  | "plotter"
  | "services"
  | "logs";

const TABS: Array<{
  id: TabId;
  label: string;
  icon: React.ReactNode;
  group: "behavior" | "monitor" | "tools";
}> = [
  // Behavior group
  { id: "tree", label: "BT Tree", icon: <TreePine className="w-4 h-4" />, group: "behavior" },
  { id: "executor", label: "Execute", icon: <Rocket className="w-4 h-4" />, group: "behavior" },
  { id: "monitor", label: "Task", icon: <Activity className="w-4 h-4" />, group: "behavior" },
  { id: "skills", label: "Skills", icon: <Puzzle className="w-4 h-4" />, group: "behavior" },
  // Monitor group
  { id: "joints", label: "Joints", icon: <Gauge className="w-4 h-4" />, group: "monitor" },
  { id: "tf", label: "TF", icon: <Box className="w-4 h-4" />, group: "monitor" },
  { id: "logs", label: "Logs", icon: <ScrollText className="w-4 h-4" />, group: "monitor" },
  // Tools group
  { id: "topics", label: "Topics", icon: <Radio className="w-4 h-4" />, group: "tools" },
  { id: "plotter", label: "Plot", icon: <LineChart className="w-4 h-4" />, group: "tools" },
  { id: "services", label: "Services", icon: <PhoneCall className="w-4 h-4" />, group: "tools" },
];

const PANEL_MAP: Record<TabId, ReactNode> = {
  tree: <BtTreePanel />,
  executor: <BehaviorExecutorPanel />,
  monitor: <TaskMonitorPanel />,
  skills: <SkillBrowserPanel />,
  joints: <JointStateViewer />,
  tf: <TfFrameViewer />,
  topics: <TopicListPanel />,
  plotter: <TopicPlotter />,
  services: <ServiceCallerPanel />,
  logs: <LogPanel />,
};

interface Props {
  connected: boolean;
}

export default function DashboardShell({ connected }: Props) {
  const [activeTab, setActiveTab] = useState<TabId>("tree");

  const groups = [
    { key: "behavior", label: "Behavior" },
    { key: "monitor", label: "Monitor" },
    { key: "tools", label: "Tools" },
  ] as const;

  return (
    <div className="h-screen flex flex-col bg-gray-950 text-gray-100">
      {/* Top bar */}
      <header className="h-10 flex items-center px-4 border-b border-gray-800 bg-gray-900/50 shrink-0">
        <div className="flex items-center gap-2 text-sm font-bold text-gray-200">
          <TreePine className="w-4 h-4 text-blue-400" />
          Robot Dashboard
        </div>

        {/* Connection indicator */}
        <div className="ml-auto flex items-center gap-1.5 text-xs">
          {connected ? (
            <>
              <Wifi className="w-3.5 h-3.5 text-green-400" />
              <span className="text-green-400">Connected</span>
            </>
          ) : (
            <>
              <WifiOff className="w-3.5 h-3.5 text-red-400" />
              <span className="text-red-400">Disconnected</span>
            </>
          )}
        </div>
      </header>

      <div className="flex flex-1 overflow-hidden">
        {/* Sidebar with grouped tabs */}
        <nav className="w-16 border-r border-gray-800 bg-gray-900/30 flex flex-col items-center py-1 gap-0 shrink-0 overflow-y-auto">
          {groups.map((group) => (
            <div key={group.key} className="w-full flex flex-col items-center">
              <div className="text-[7px] uppercase text-gray-600 font-bold tracking-widest mt-1.5 mb-0.5">
                {group.label}
              </div>
              {TABS.filter((t) => t.group === group.key).map((tab) => (
                <button
                  key={tab.id}
                  onClick={() => setActiveTab(tab.id)}
                  className={clsx(
                    "w-12 h-11 flex flex-col items-center justify-center rounded-lg text-[8px] transition-all gap-0.5",
                    activeTab === tab.id
                      ? "bg-blue-500/20 text-blue-400"
                      : "text-gray-500 hover:text-gray-300 hover:bg-gray-800/50"
                  )}
                  title={tab.label}
                >
                  {tab.icon}
                  {tab.label}
                </button>
              ))}
            </div>
          ))}
        </nav>

        {/* Main content — panels stay mounted (hidden via CSS) to preserve state */}
        <main className="flex-1 overflow-hidden">
          {TABS.map((tab) => (
            <div
              key={tab.id}
              className={activeTab === tab.id ? "h-full" : "hidden"}
            >
              {PANEL_MAP[tab.id]}
            </div>
          ))}
        </main>
      </div>
    </div>
  );
}
