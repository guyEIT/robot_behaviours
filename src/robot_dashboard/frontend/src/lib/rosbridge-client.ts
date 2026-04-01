import ROSLIB from "roslib";

const ROSBRIDGE_PORT =
  (window as any).ENV_CONFIG?.ROSBRIDGE_PORT || "9090";
const ROSBRIDGE_URL =
  (typeof import.meta !== "undefined" && (import.meta as any).env?.VITE_ROSBRIDGE_URL) ||
  `ws://${window.location.hostname}:${ROSBRIDGE_PORT}`;

let ros: ROSLIB.Ros | null = null;
let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
let attempt = 0;

type Listener = (connected: boolean) => void;
const listeners = new Set<Listener>();

function notify(connected: boolean) {
  listeners.forEach((l) => l(connected));
}

export function getRos(): ROSLIB.Ros {
  if (!ros) {
    ros = new ROSLIB.Ros({});

    ros.on("connection", () => {
      attempt = 0;
      notify(true);
    });

    ros.on("close", () => {
      notify(false);
      scheduleReconnect();
    });

    ros.on("error", () => {
      // error fires before close; close handler will reconnect
    });

    ros.connect(ROSBRIDGE_URL);
  }
  return ros;
}

function scheduleReconnect() {
  if (reconnectTimer) return;
  const delay = Math.min(1000 * 2 ** attempt, 30000);
  attempt++;
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    if (ros) {
      try {
        ros.connect(ROSBRIDGE_URL);
      } catch {
        scheduleReconnect();
      }
    }
  }, delay);
}

export function onConnectionChange(listener: Listener): () => void {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

export function isConnected(): boolean {
  return ros?.isConnected ?? false;
}
