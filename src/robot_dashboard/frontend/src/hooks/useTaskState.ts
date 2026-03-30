import { useCallback } from "react";
import { useTopicSubscription } from "./useTopicSubscription";
import { useTaskStore } from "../stores/task-store";
import type { TaskState } from "../types/ros";

/** Subscribe to /skill_server/task_state and update the task store. */
export function useTaskState() {
  const setTaskState = useTaskStore((s) => s.setTaskState);

  const handleMsg = useCallback(
    (msg: TaskState) => setTaskState(msg),
    [setTaskState]
  );

  useTopicSubscription<TaskState>(
    "/skill_server/task_state",
    "robot_skills_msgs/msg/TaskState",
    handleMsg
  );
}
