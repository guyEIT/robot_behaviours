import { useCallback } from "react";
import { useTopicSubscription } from "./useTopicSubscription";
import { useTaskStore } from "../stores/task-store";
import type { TaskState } from "../types/ros";

/**
 * Dynamically subscribe to /skill_server/bt_runner_status/<task_id>
 * for real-time BT node tracking. Also subscribes to active_bt_xml.
 */
export function useBtRunnerStatus() {
  const taskState = useTaskStore((s) => s.taskState);
  const setTaskState = useTaskStore((s) => s.setTaskState);
  const setActiveBtXml = useTaskStore((s) => s.setActiveBtXml);

  // Dynamic topic based on active task
  const taskId = taskState?.task_id;
  const isRunning = taskState?.status === "RUNNING";
  const topicName =
    isRunning && taskId
      ? `/skill_server/bt_runner_status/${taskId}`
      : null;

  const handleStatus = useCallback(
    (msg: TaskState) => setTaskState(msg),
    [setTaskState]
  );

  useTopicSubscription<TaskState>(topicName, "robot_skills_msgs/msg/TaskState", handleStatus);

  // Subscribe to active BT XML (latched topic)
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
