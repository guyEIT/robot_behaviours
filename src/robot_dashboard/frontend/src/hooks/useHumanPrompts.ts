import { toast } from "sonner";
import { useTopicSubscription } from "./useTopicSubscription";
import { useHumanPromptStore } from "../stores/human-prompt-store";
import type { HumanPrompt } from "../types/ros";

function playAttentionSound() {
  try {
    const ctx = new AudioContext();
    // Three quick ascending tones
    [660, 880, 1100].forEach((freq, i) => {
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.type = "sine";
      osc.frequency.value = freq;
      gain.gain.value = 0.12;
      gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.15 + i * 0.12);
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.start(ctx.currentTime + i * 0.12);
      osc.stop(ctx.currentTime + 0.15 + i * 0.12);
    });
  } catch {
    // Audio not available
  }
}

function playNotificationSound() {
  try {
    const ctx = new AudioContext();
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    osc.type = "sine";
    osc.frequency.value = 880;
    gain.gain.value = 0.08;
    gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.15);
    osc.connect(gain);
    gain.connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.15);
  } catch {
    // Audio not available
  }
}

/**
 * Subscribes to /skill_server/human_prompts and routes to the store + toasts.
 */
export function useHumanPrompts() {
  const addPrompt = useHumanPromptStore((s) => s.addPrompt);

  useTopicSubscription<HumanPrompt>(
    "/skill_server/human_prompts",
    "robot_skills_msgs/msg/HumanPrompt",
    (msg) => {
      addPrompt(msg);

      switch (msg.prompt_type) {
        case "notification":
          toast.info(msg.title, { description: msg.message, duration: 5000 });
          playNotificationSound();
          break;
        case "warning": {
          const toastFn =
            msg.severity === "error" || msg.severity === "critical"
              ? toast.error
              : toast.warning;
          toastFn(msg.title, { description: msg.message, duration: 8000 });
          playNotificationSound();
          break;
        }
        case "confirm":
          toast.info(`Action required: ${msg.title}`, {
            description: msg.message,
            duration: 15000,
          });
          playAttentionSound();
          break;
        case "input":
          toast.info(`Input needed: ${msg.title}`, {
            description: msg.message,
            duration: 15000,
          });
          playAttentionSound();
          break;
        case "task":
          toast.info(`Task assigned: ${msg.title}`, {
            description: msg.message,
            duration: 15000,
          });
          playAttentionSound();
          break;
        // "dismiss" handled by store
      }
    }
  );
}
