import { useCallback } from "react";
import { useTopicSubscription } from "./useTopicSubscription";
import { useTaskStore } from "../stores/task-store";

/**
 * Subscribe to /skill_server/active_bt_xml for live tree visualization.
 * Task state (current node, progress) comes via useTaskState hook.
 */
export function useBtRunnerStatus() {
  const setActiveBtXml = useTaskStore((s) => s.setActiveBtXml);

  const handleXml = useCallback(
    (msg: { data: string }) => {
      setActiveBtXml(msg.data || null);
    },
    [setActiveBtXml]
  );

  useTopicSubscription<{ data: string }>(
    "/skill_server/active_bt_xml",
    "std_msgs/msg/String",
    handleXml
  );
}
