import { useTaskStore } from "../../stores/task-store";
import BtTreeGraph from "./BtTreeGraph";
import BtLegend from "./BtLegend";
import BtXmlViewer from "./BtXmlViewer";
import { TreePine } from "lucide-react";

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
  const taskState = useTaskStore((s) => s.taskState);
  const activeBtXml = useTaskStore((s) => s.activeBtXml);
  const taskHistory = useTaskStore((s) => s.taskHistory);

  const xml = activeBtXml || DEMO_XML;
  const activeNodeName = taskState?.status === "RUNNING" ? taskState.current_bt_node : null;

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="flex items-center gap-3 px-4 py-2 border-b border-gray-800">
        <TreePine className="w-4 h-4 text-blue-400" />
        <h2 className="text-sm font-semibold">Behavior Tree</h2>

        {taskHistory.length > 0 && (
          <div className="ml-4 text-xs text-gray-400">
            Task:{" "}
            <span className="text-gray-200">
              {taskState?.task_name || "none"}
            </span>
            {taskState?.status && (
              <span
                className={
                  taskState.status === "RUNNING"
                    ? "ml-2 text-blue-400"
                    : taskState.status === "SUCCESS"
                    ? "ml-2 text-green-400"
                    : taskState.status === "FAILURE"
                    ? "ml-2 text-red-400"
                    : "ml-2 text-gray-400"
                }
              >
                [{taskState.status}]
              </span>
            )}
          </div>
        )}

        <div className="ml-auto">
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
        <BtTreeGraph xml={xml} activeNodeName={activeNodeName} />
      </div>

      {/* XML viewer */}
      <BtXmlViewer xml={activeBtXml} />
    </div>
  );
}
