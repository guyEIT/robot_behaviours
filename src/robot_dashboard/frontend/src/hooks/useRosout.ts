import { useCallback } from "react";
import { useTopicSubscription } from "./useTopicSubscription";
import { useLogStore } from "../stores/log-store";
import type { RosLog, SkillLogEvent } from "../types/ros";

/** Subscribe to /rosout and add logs to the store ring buffer. */
export function useRosout() {
  const addLog = useLogStore((s) => s.addLog);
  const addSkillLog = useLogStore((s) => s.addSkillLog);

  const handleMsg = useCallback(
    (msg: RosLog) => addLog(msg),
    [addLog]
  );

  const handleSkillLog = useCallback(
    (msg: SkillLogEvent) => addSkillLog(msg),
    [addSkillLog]
  );

  useTopicSubscription<RosLog>(
    "/rosout",
    "rcl_interfaces/msg/Log",
    handleMsg,
    100 // throttle to 10Hz max
  );

  useTopicSubscription<SkillLogEvent>(
    "/skill_server/log_events",
    "robot_skills_msgs/msg/LogEvent",
    handleSkillLog
  );
}
