import { useCallback } from "react";
import { useTopicSubscription } from "./useTopicSubscription";
import { useLogStore } from "../stores/log-store";
import type { DiagnosticArray } from "../types/ros";

/** Subscribe to /diagnostics_agg and update the log store. */
export function useDiagnostics() {
  const setDiagnostics = useLogStore((s) => s.setDiagnostics);

  const handleMsg = useCallback(
    (msg: DiagnosticArray) => setDiagnostics(msg.status),
    [setDiagnostics]
  );

  useTopicSubscription<DiagnosticArray>(
    "/diagnostics_agg",
    "diagnostic_msgs/msg/DiagnosticArray",
    handleMsg,
    1000 // throttle to 1Hz
  );
}
