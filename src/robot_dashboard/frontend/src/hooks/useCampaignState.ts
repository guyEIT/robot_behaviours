import { useState } from "react";
import { useTopicSubscription } from "./useTopicSubscription";
import type { CampaignSnapshot } from "../components/campaign/types";

interface RawStringMsg {
  data: string;
}

/**
 * Subscribe to /skill_server/persistent_state — a latched JSON snapshot of
 * the active task's persistent.* blackboard, plus a couple of derived
 * fields (task_id, task_status, paused). Published by bb_operator on every
 * service handler completion + a 1 Hz timer when a task is active.
 */
export function useCampaignState(): CampaignSnapshot | null {
  const [snap, setSnap] = useState<CampaignSnapshot | null>(null);

  useTopicSubscription<RawStringMsg>(
    "/skill_server/persistent_state",
    "std_msgs/msg/String",
    (msg) => {
      try {
        const parsed = JSON.parse(msg.data) as CampaignSnapshot;
        setSnap(parsed);
      } catch {
        // Bad JSON — ignore. Could surface a toast but the publisher is
        // ours and well-formed; a parse failure indicates an env issue
        // worth investigating in logs, not in the UI.
      }
    }
  );

  return snap;
}
