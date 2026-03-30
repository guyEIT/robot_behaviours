import { useCallback } from "react";
import { useTopicSubscription } from "./useTopicSubscription";
import { useLogStore } from "../stores/log-store";
import type { RosLog } from "../types/ros";

/** Subscribe to /rosout and add logs to the store ring buffer. */
export function useRosout() {
  const addLog = useLogStore((s) => s.addLog);

  const handleMsg = useCallback(
    (msg: RosLog) => addLog(msg),
    [addLog]
  );

  useTopicSubscription<RosLog>(
    "/rosout",
    "rcl_interfaces/msg/Log",
    handleMsg,
    100 // throttle to 10Hz max
  );
}
