import { useEffect, useRef } from "react";
import { toast } from "sonner";
import { useTaskStore } from "../stores/task-store";

/**
 * Plays a short synthesized tone via the Web Audio API.
 * No external audio files needed.
 */
function playTone(frequency: number, duration: number, type: OscillatorType = "sine") {
  try {
    const ctx = new AudioContext();
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    osc.type = type;
    osc.frequency.value = frequency;
    gain.gain.value = 0.15;
    gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + duration);
    osc.connect(gain);
    gain.connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + duration);
  } catch {
    // Audio not available (e.g., no user interaction yet)
  }
}

function playSuccessSound() {
  // Two ascending tones
  playTone(523, 0.15, "sine"); // C5
  setTimeout(() => playTone(659, 0.2, "sine"), 150); // E5
}

function playFailureSound() {
  // Two descending tones
  playTone(440, 0.15, "square"); // A4
  setTimeout(() => playTone(330, 0.25, "square"), 150); // E4
}

function playCancelSound() {
  playTone(392, 0.2, "triangle"); // G4
}

/**
 * Watches the task store for terminal state transitions and fires
 * toast notifications + audio cues.
 */
export function useTaskNotifications() {
  const prevRef = useRef<{ taskId: string; status: string } | null>(null);

  useEffect(() => {
    const unsub = useTaskStore.subscribe((state) => {
      const ts = state.taskState;
      if (!ts) return;

      const prev = prevRef.current;
      const isNewTransition =
        !prev || prev.taskId !== ts.task_id || prev.status !== ts.status;

      if (!isNewTransition) return;

      prevRef.current = { taskId: ts.task_id, status: ts.status };

      const name = ts.task_name || ts.task_id;

      if (ts.status === "SUCCESS") {
        const elapsed = ts.elapsed_sec
          ? ` in ${ts.elapsed_sec.toFixed(1)}s`
          : "";
        toast.success(`Task completed${elapsed}`, {
          description: name,
          duration: 5000,
        });
        playSuccessSound();
      } else if (ts.status === "FAILURE") {
        const detail = ts.error_message
          ? `${name}: ${ts.error_message}`
          : name;
        toast.error("Task failed", {
          description: detail,
          duration: 8000,
        });
        playFailureSound();
      } else if (ts.status === "CANCELLED") {
        toast.warning("Task cancelled", {
          description: name,
          duration: 4000,
        });
        playCancelSound();
      }
    });

    return unsub;
  }, []);
}
