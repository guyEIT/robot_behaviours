import { useMemo, useState } from "react";
import { useTaskStore } from "../../stores/task-store";
import { useBtCollapseStore } from "../../stores/bt-collapse-store";
import BtTreeGraph from "./BtTreeGraph";
import BtLegend from "./BtLegend";
import BtXmlViewer from "./BtXmlViewer";
import {
  TreePine,
  Locate,
  ChevronsDown,
  ChevronsUp,
  Maximize2,
  Activity,
  PackageOpen,
  ArrowDown,
  ArrowRight,
} from "lucide-react";
import { Chip, Eyebrow, IconBtn } from "../ui";
import type { ChipState } from "../ui";
import { parseBtXml } from "../../lib/bt-parser";

const EMPTY_LIST: readonly string[] = Object.freeze([]);

const DEMO_XML = `<root BTCPP_format="4" main_tree_to_execute="Demo">
  <BehaviorTree ID="Demo">
    <Sequence name="demo_sequence">
      <MoveToNamedConfig name="go_home" config_name="home"/>
      <GripperControl name="open_gripper" command="open"/>
    </Sequence>
  </BehaviorTree>
</root>`;

const STATUS_TO_CHIP: Record<string, ChipState> = {
  RUNNING: "running",
  SUCCESS: "done",
  FAILURE: "failed",
  IDLE: "idle",
  CANCELLED: "neutral",
};

export default function BtTreePanel() {
  const taskName = useTaskStore((s) => s.taskState?.task_name ?? null);
  const taskStatus = useTaskStore((s) => s.taskState?.status ?? null);
  const activeNodeName = useTaskStore((s) =>
    s.taskState?.status === "RUNNING" ? s.taskState.current_bt_node : null
  );
  const completedNodeNames = useTaskStore(
    (s) => s.taskState?.completed_skills ?? EMPTY_LIST,
  );
  const failedNodeNames = useTaskStore(
    (s) => s.taskState?.failed_skills ?? EMPTY_LIST,
  );
  const activeBtXml = useTaskStore((s) => s.activeBtXml);
  const hasHistory = useTaskStore((s) => s.taskHistory.length > 0);
  const [followActive, setFollowActive] = useState(true);

  const xml = activeBtXml || DEMO_XML;

  const containerNames = useMemo(() => {
    try {
      const parsed = parseBtXml(xml);
      return parsed.nodes.filter((n) => n.containerKind).map((n) => n.name);
    } catch {
      return [];
    }
  }, [xml]);

  const expandAll = useBtCollapseStore((s) => s.expandAll);
  const collapseAll = useBtCollapseStore((s) => s.collapseAll);
  const autoExpandAll = useBtCollapseStore((s) => s.autoExpandAll);
  const setAutoExpandAll = useBtCollapseStore((s) => s.setAutoExpandAll);
  const autoExpandRunning = useBtCollapseStore((s) => s.autoExpandRunning);
  const setAutoExpandRunning = useBtCollapseStore((s) => s.setAutoExpandRunning);
  const autoCloseOnCompletion = useBtCollapseStore(
    (s) => s.autoCloseOnCompletion,
  );
  const setAutoCloseOnCompletion = useBtCollapseStore(
    (s) => s.setAutoCloseOnCompletion,
  );
  const direction = useBtCollapseStore((s) => s.direction);
  const setDirection = useBtCollapseStore((s) => s.setDirection);
  const smartFollow = useBtCollapseStore((s) => s.smartFollow);
  const setSmartFollow = useBtCollapseStore((s) => s.setSmartFollow);

  return (
    <div className="flex flex-col h-full bg-paper">
      <div className="flex items-center gap-3 px-5 py-3 border-b border-hair">
        <TreePine className="w-4 h-4 text-terracotta" />
        <h2 className="text-[14px] font-medium text-ink">Behavior Tree</h2>

        {hasHistory && taskName && (
          <div className="ml-4 flex items-center gap-2 text-[12px] text-muted">
            <Eyebrow size="sm" tone="muted">Task</Eyebrow>
            <span className="text-ink-soft font-medium">{taskName}</span>
            {taskStatus && (
              <Chip state={STATUS_TO_CHIP[taskStatus] ?? "idle"} showDot>
                {taskStatus}
              </Chip>
            )}
          </div>
        )}

        <div className="ml-auto flex items-center gap-2">
          {/* Layout direction toggle: vertical (LR, children flow right
              and stack downward — better for long names) vs horizontal
              (TB, children stack across — compact for shallow trees). */}
          <IconBtn
            onClick={() => setDirection(direction === "LR" ? "TB" : "LR")}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title={
              direction === "LR"
                ? "Layout: vertical stacks (LR). Click for horizontal stacks (TB)."
                : "Layout: horizontal stacks (TB). Click for vertical stacks (LR)."
            }
          >
            {direction === "LR" ? (
              <ArrowRight className="w-3 h-3" />
            ) : (
              <ArrowDown className="w-3 h-3" />
            )}
            {direction === "LR" ? "Vertical" : "Horizontal"}
          </IconBtn>
          {/* Sticky toggle: every container always rendered expanded. */}
          <IconBtn
            onClick={() => setAutoExpandAll(!autoExpandAll)}
            active={autoExpandAll}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Show all SubTrees and loop frames expanded"
          >
            <Maximize2 className="w-3 h-3" />
            Show all
          </IconBtn>
          {/* Sticky toggle: ancestors of the running node auto-open. */}
          <IconBtn
            onClick={() => setAutoExpandRunning(!autoExpandRunning)}
            active={autoExpandRunning}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Auto-open SubTrees as the runner enters them"
          >
            <Activity className="w-3 h-3" />
            Auto-open
          </IconBtn>
          {/* Sticky toggle: auto-opened containers re-collapse on completion. */}
          <IconBtn
            onClick={() => setAutoCloseOnCompletion(!autoCloseOnCompletion)}
            active={autoCloseOnCompletion}
            disabled={autoExpandAll}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Re-collapse auto-opened SubTrees once they finish"
          >
            <PackageOpen className="w-3 h-3" />
            Auto-close
          </IconBtn>
          {/* Momentary one-shot expand/collapse. Disabled while
              autoExpandAll is on (everything's already open). */}
          <IconBtn
            onClick={() => expandAll(containerNames)}
            disabled={containerNames.length === 0 || autoExpandAll}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Expand every container once"
          >
            <ChevronsDown className="w-3 h-3" />
            Expand
          </IconBtn>
          <IconBtn
            onClick={() => collapseAll()}
            disabled={containerNames.length === 0 || autoExpandAll}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Collapse every container once"
          >
            <ChevronsUp className="w-3 h-3" />
            Collapse
          </IconBtn>
          <IconBtn
            onClick={() => setFollowActive(!followActive)}
            active={followActive}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Auto-center camera on the active node"
          >
            <Locate className="w-3 h-3" />
            Follow
          </IconBtn>
          <IconBtn
            onClick={() => setSmartFollow(!smartFollow)}
            active={smartFollow}
            disabled={!followActive}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Only pan the camera when the active node leaves the viewport (slower, easier to follow)"
          >
            <Locate className="w-3 h-3 opacity-60" />
            Smart
          </IconBtn>
          <BtLegend />
        </div>
      </div>

      <div className="flex-1 relative bg-cream-deep">
        {!activeBtXml && (
          <div className="absolute inset-0 flex items-center justify-center z-10 pointer-events-none">
            <div className="text-center text-muted">
              <TreePine className="w-8 h-8 mx-auto mb-2 opacity-30" />
              <p className="text-[14px] text-ink-soft font-medium">No active task</p>
              <p className="text-[12px]">
                Showing demo tree. Execute a behavior tree to see live visualisation.
              </p>
            </div>
          </div>
        )}
        <BtTreeGraph
          xml={xml}
          activeNodeName={activeNodeName}
          completedNodeNames={completedNodeNames}
          failedNodeNames={failedNodeNames}
          followActive={followActive}
        />
      </div>

      <BtXmlViewer xml={activeBtXml} />
    </div>
  );
}
