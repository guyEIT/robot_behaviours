import { useCallback, useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { toast } from "sonner";
import { getRos } from "../lib/rosbridge-client";

interface ActionState<F> {
  status: "idle" | "pending" | "active" | "succeeded" | "failed" | "cancelled";
  feedback: F | null;
  error: string | null;
}

let actionRoutingPatched = false;

/**
 * Patch the roslibjs Ros instance to route action_feedback and action_result
 * WebSocket messages by their `id` field.  roslibjs v1.x only handles
 * publish / service_response / call_service / status natively — this adds
 * the missing action routing so rosbridge's ROS2 action protocol works.
 */
function ensureActionRouting(ros: ROSLIB.Ros) {
  if (actionRoutingPatched) return;
  actionRoutingPatched = true;

  const attachToSocket = () => {
    const socket = (ros as any).socket as WebSocket | undefined;
    if (!socket) return;
    socket.addEventListener("message", (event: MessageEvent) => {
      try {
        const msg = JSON.parse(event.data);
        if (
          (msg.op === "action_feedback" || msg.op === "action_result") &&
          msg.id
        ) {
          (ros as any).emit(msg.id, msg);
        }
      } catch {
        // ignore parse errors
      }
    });
  };

  // Attach now (if already connected) and on every reconnection
  attachToSocket();
  ros.on("connection", attachToSocket);
}

/**
 * Hook for calling a ROS2 action via rosbridge's native action protocol.
 *
 * Uses send_action_goal / cancel_action_goal / action_feedback / action_result
 * WebSocket operations instead of the ROS1 actionlib topic-based protocol
 * that roslibjs's built-in ActionClient implements.
 */
export function useActionClient<G, F, R>(
  actionName: string,
  actionType: string
) {
  const [state, setState] = useState<ActionState<F>>({
    status: "idle",
    feedback: null,
    error: null,
  });

  const callIdRef = useRef<string | null>(null);
  const cleanupRef = useRef<(() => void) | null>(null);

  // Clean up on unmount
  useEffect(() => {
    return () => {
      cleanupRef.current?.();
    };
  }, []);

  const sendGoal = useCallback(
    (goal: G): Promise<R> => {
      // Clean up any previous in-flight goal
      cleanupRef.current?.();

      setState({ status: "pending", feedback: null, error: null });

      const ros = getRos();
      ensureActionRouting(ros);

      const id = `action:${Date.now()}:${Math.random().toString(36).slice(2)}`;
      callIdRef.current = id;

      return new Promise<R>((resolve, reject) => {
        let settled = false;

        const settle = () => {
          if (settled) return false;
          settled = true;
          (ros as any).off(id, handler);
          callIdRef.current = null;
          cleanupRef.current = null;
          return true;
        };

        const handler = (message: any) => {
          if (settled) return;

          if (message.op === "action_feedback") {
            setState((prev) => ({
              ...prev,
              status: "active",
              feedback: message.values as F,
            }));
          } else if (message.op === "action_result") {
            if (!settle()) return;

            if (message.result === true) {
              setState({ status: "succeeded", feedback: null, error: null });
              resolve(message.values as R);
            } else {
              const errMsg =
                message.values?.message || "Action failed";
              setState({ status: "failed", feedback: null, error: errMsg });
              toast.error("Action failed", {
                description: `${actionName}: ${errMsg}`,
                duration: 6000,
              });
              reject(new Error(errMsg));
            }
          }
        };

        cleanupRef.current = () => {
          settle();
        };

        ros.on(id, handler);

        // Send action goal via rosbridge's ROS2 action protocol
        (ros as any).callOnConnection({
          op: "send_action_goal",
          id,
          action: actionName,
          action_type: actionType,
          args: goal,
          feedback: true,
        });
      });
    },
    [actionName, actionType]
  );

  const cancel = useCallback(() => {
    const id = callIdRef.current;
    if (!id) return;

    setState({ status: "cancelled", feedback: null, error: null });

    const ros = getRos();
    (ros as any).callOnConnection({
      op: "cancel_action_goal",
      id,
      action: actionName,
    });
  }, [actionName]);

  return { ...state, sendGoal, cancel };
}
