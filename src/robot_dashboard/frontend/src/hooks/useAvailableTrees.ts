import { useState, useCallback } from "react";
import { useTopicSubscription } from "./useTopicSubscription";

export interface AvailableTree {
  name: string;
  label: string;
  filename: string;
  xml: string;
}

/**
 * Subscribe to /skill_server/available_trees (latched JSON topic).
 * Returns the list of behavior tree files discovered on disk.
 * Updates live when files are added/changed/removed.
 */
export function useAvailableTrees(): AvailableTree[] {
  const [trees, setTrees] = useState<AvailableTree[]>([]);

  const onMessage = useCallback((msg: { data: string }) => {
    try {
      const parsed = JSON.parse(msg.data) as AvailableTree[];
      setTrees(parsed);
    } catch {
      // ignore parse errors
    }
  }, []);

  useTopicSubscription(
    "/skill_server/available_trees",
    "std_msgs/msg/String",
    onMessage
  );

  return trees;
}
