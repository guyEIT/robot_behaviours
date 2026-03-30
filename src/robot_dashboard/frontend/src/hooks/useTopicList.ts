import { useEffect } from "react";
import ROSLIB from "roslib";
import { getRos } from "../lib/rosbridge-client";
import { useConnectionStore } from "../stores/connection-store";
import { useTopicStore } from "../stores/topic-store";

/**
 * Periodically fetch the list of all active ROS2 topics via rosbridge.
 */
export function useTopicList() {
  const connected = useConnectionStore((s) => s.connected);
  const setTopics = useTopicStore((s) => s.setTopics);

  useEffect(() => {
    if (!connected) return;

    const ros = getRos();
    let cancelled = false;

    function fetch() {
      ros.getTopics(
        (result: { topics: string[]; types: string[] }) => {
          if (cancelled) return;
          const topics = result.topics.map((name, i) => ({
            name,
            type: result.types[i] || "unknown",
          }));
          // Sort: skill_server topics first, then alphabetical
          topics.sort((a, b) => {
            const aSk = a.name.includes("skill_server") ? 0 : 1;
            const bSk = b.name.includes("skill_server") ? 0 : 1;
            if (aSk !== bSk) return aSk - bSk;
            return a.name.localeCompare(b.name);
          });
          setTopics(topics);
        },
        (err: string) => {
          console.warn("Failed to fetch topics:", err);
        }
      );
    }

    fetch();
    const interval = setInterval(fetch, 5000);

    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, [connected, setTopics]);
}
