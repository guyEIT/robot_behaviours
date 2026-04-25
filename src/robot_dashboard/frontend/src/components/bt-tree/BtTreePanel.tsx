import { useState } from "react";
import { useTaskStore } from "../../stores/task-store";
import BtTreeGraph from "./BtTreeGraph";
import BtLegend from "./BtLegend";
import BtXmlViewer from "./BtXmlViewer";
import { TreePine, Locate } from "lucide-react";
import { Chip, Eyebrow, IconBtn } from "../ui";
import type { ChipState } from "../ui";

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
          <IconBtn
            onClick={() => setFollowActive(!followActive)}
            active={followActive}
            className="!w-auto !h-7 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
            title="Auto-follow active node during execution"
          >
            <Locate className="w-3 h-3" />
            Follow
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
