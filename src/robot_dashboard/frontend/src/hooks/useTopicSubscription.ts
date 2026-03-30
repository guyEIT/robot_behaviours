import { useEffect, useRef } from "react";
import ROSLIB from "roslib";
import { getRos } from "../lib/rosbridge-client";
import { useConnectionStore } from "../stores/connection-store";

/**
 * Subscribe to a ROS2 topic via rosbridge.
 * Automatically resubscribes on reconnection.
 *
 * @param topicName - e.g. "/skill_server/task_state"
 * @param messageType - e.g. "robot_skills_msgs/msg/TaskState"
 * @param callback - called with each message
 * @param throttleMs - optional throttle interval in ms
 */
export function useTopicSubscription<T>(
  topicName: string | null,
  messageType: string,
  callback: (msg: T) => void,
  throttleMs?: number
) {
  const connected = useConnectionStore((s) => s.connected);
  const callbackRef = useRef(callback);
  callbackRef.current = callback;

  useEffect(() => {
    if (!connected || !topicName) return;

    const ros = getRos();
    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType,
      throttle_rate: throttleMs ?? 0,
    });

    const handler = (msg: any) => callbackRef.current(msg as T);
    topic.subscribe(handler);

    return () => {
      topic.unsubscribe(handler);
    };
  }, [connected, topicName, messageType, throttleMs]);
}
