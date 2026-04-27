import { useState } from "react";
import { useTopicSubscription } from "./useTopicSubscription";

interface RawStringMsg {
  data: string;
}

/**
 * Shape of /skill_server/persistent_state JSON snapshots published by
 * bb_operator. Keys in `persistent` are the active task's `persistent.*`
 * blackboard keys; values are arbitrary JSON.
 */
export interface BlackboardSnapshot {
  task_id: string;
  task_status: string;
  paused: boolean;
  snapshot_at: number;
  persistent: Record<string, unknown>;
}

/**
 * Subscribe to the latched persistent-blackboard snapshot topic. Both
 * the Campaign panel and the Blackboard panel share this single
 * subscription — the topic is latched so late mounts still receive the
 * most recent state, and centralizing the parse means we only do JSON
 * decoding once per publish even when both panels are mounted.
 */
export function useBlackboardSnapshot(): BlackboardSnapshot | null {
  const [snap, setSnap] = useState<BlackboardSnapshot | null>(null);

  useTopicSubscription<RawStringMsg>(
    "/skill_server/persistent_state",
    "std_msgs/msg/String",
    (msg) => {
      try {
        setSnap(JSON.parse(msg.data) as BlackboardSnapshot);
      } catch {
        // Bad JSON — ignore. Publisher is ours and well-formed; a parse
        // failure is an env-level issue worth investigating in logs.
      }
    },
  );

  return snap;
}
