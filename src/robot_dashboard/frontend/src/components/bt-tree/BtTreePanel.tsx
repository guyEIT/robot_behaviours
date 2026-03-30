import { useState } from "react";
import { useTaskStore } from "../../stores/task-store";
import BtTreeGraph from "./BtTreeGraph";
import BtLegend from "./BtLegend";
import BtXmlViewer from "./BtXmlViewer";
import { TreePine, Locate } from "lucide-react";
import clsx from "clsx";

// Demo tree for when no task is running
const DEMO_XML = `<root BTCPP_format="4" main_tree_to_execute="Demo">
  <BehaviorTree ID="Demo">
    <Sequence name="demo_sequence">
      <MoveToNamedConfig name="go_home" config_name="home"/>
      <GripperControl name="open_gripper" command="open"/>
    </Sequence>
  </BehaviorTree>
</root>`;

export default function BtTreePanel() {
  const taskName = useTaskStore((s) => s.taskState?.task_name ?? null);
  const taskStatus = useTaskStore((s) => s.taskState?.status ?? null);
  const activeNodeName = useTaskStore((s) =>
    s.taskState?.status === "RUNNING" ? s.taskState.current_bt_node : null
  );
  const activeBtXml = useTaskStore((s) => s.activeBtXml);
  const hasHistory = useTaskStore((s) => s.taskHistory.length > 0);
  const [followActive, setFollowActive] = useState(true);

  const xml = activeBtXml || DEMO_XML;

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="flex items-center gap-3 px-4 py-2 border-b border-gray-800">
        <TreePine className="w-4 h-4 text-blue-400" />
        <h2 className="text-sm font-semibold">Behavior Tree</h2>

        {hasHistory && taskName && (
          <div className="ml-4 text-xs text-gray-400">
            Task:{" "}
            <span className="text-gray-200">{taskName}</span>
            {taskStatus && (
              <span
                className={
                  taskStatus === "RUNNING"
                    ? "ml-2 text-blue-400"
                    : taskStatus === "SUCCESS"
                    ? "ml-2 text-green-400"
                    : taskStatus === "FAILURE"
                    ? "ml-2 text-red-400"
                    : "ml-2 text-gray-400"
                }
              >
                [{taskStatus}]
              </span>
            )}
          </div>
        )}

        <div className="ml-auto flex items-center gap-2">
          <button
            onClick={() => setFollowActive(!followActive)}
            className={clsx(
              "flex items-center gap-1 px-2 py-0.5 rounded text-[10px] font-medium transition-colors",
              followActive
                ? "bg-blue-500/20 text-blue-400"
                : "text-gray-500 hover:text-gray-300"
            )}
            title="Auto-follow active node during execution"
          >
            <Locate className="w-3 h-3" />
            Follow
          </button>
          <BtLegend />
        </div>
      </div>

      {/* Tree graph */}
      <div className="flex-1 relative">
        {!activeBtXml && (
          <div className="absolute inset-0 flex items-center justify-center z-10 pointer-events-none">
            <div className="text-center text-gray-500 text-sm">
              <TreePine className="w-8 h-8 mx-auto mb-2 opacity-30" />
              <p>No active task</p>
              <p className="text-xs">Showing demo tree. Execute a behavior tree to see live visualization.</p>
            </div>
          </div>
        )}
        <BtTreeGraph xml={xml} activeNodeName={activeNodeName} followActive={followActive} />
      </div>

      {/* XML viewer */}
      <BtXmlViewer xml={activeBtXml} />
    </div>
  );
}
