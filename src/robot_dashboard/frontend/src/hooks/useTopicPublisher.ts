import { useRef, useCallback } from "react";
import ROSLIB from "roslib";
import { getRos } from "../lib/rosbridge-client";

/**
 * Publishes messages to a ROS2 topic via rosbridge.
 * Lazily creates and caches the ROSLIB.Topic instance.
 */
export function useTopicPublisher<T>(topicName: string, messageType: string) {
  const topicRef = useRef<ROSLIB.Topic | null>(null);

  const publish = useCallback(
    (msg: T) => {
      if (!topicRef.current) {
        topicRef.current = new ROSLIB.Topic({
          ros: getRos(),
          name: topicName,
          messageType,
        });
      }
      topicRef.current.publish(new ROSLIB.Message(msg as any));
    },
    [topicName, messageType]
  );

  return { publish };
}
