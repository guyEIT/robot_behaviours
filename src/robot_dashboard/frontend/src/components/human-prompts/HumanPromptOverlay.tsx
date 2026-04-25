import { useHumanPromptStore } from "../../stores/human-prompt-store";
import { useTopicPublisher } from "../../hooks/useTopicPublisher";
import type { HumanResponse } from "../../types/ros";
import { useState } from "react";
import {
  AlertTriangle,
  CheckCircle,
  XCircle,
  Send,
  ClipboardCheck,
  MessageSquareMore,
} from "lucide-react";
import clsx from "clsx";
import { Button, Eyebrow, Input } from "../ui";

export default function HumanPromptOverlay() {
  const activePrompts = useHumanPromptStore((s) => s.activePrompts);

  if (activePrompts.length === 0) return null;

  const prompt = activePrompts[0];

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-ink-soft/45">
      <OverlayCard prompt={prompt} queueCount={activePrompts.length} />
    </div>
  );
}

function OverlayCard({
  prompt,
  queueCount,
}: {
  prompt: ReturnType<typeof useHumanPromptStore.getState>["activePrompts"][0];
  queueCount: number;
}) {
  const removePrompt = useHumanPromptStore((s) => s.removePrompt);
  const { publish } = useTopicPublisher<HumanResponse>(
    "/skill_server/human_responses",
    "robot_skills_msgs/msg/HumanResponse"
  );
  const [inputValue, setInputValue] = useState(prompt.default_value || "");
  const [notes, setNotes] = useState("");

  const respond = (accepted: boolean, value = "") => {
    publish({
      stamp: { sec: 0, nanosec: 0 },
      prompt_id: prompt.prompt_id,
      accepted,
      value,
    });
    removePrompt(prompt.prompt_id);
  };

  const eyebrowLabel =
    prompt.prompt_type === "task"
      ? "Task Assigned"
      : prompt.prompt_type === "confirm"
        ? "Confirmation Required"
        : "Input Required";

  return (
    <div className="w-full max-w-md mx-4 bg-paper border border-hair border-l-2 border-l-terracotta p-7 space-y-4">
      <div className="flex items-center gap-3">
        {prompt.prompt_type === "task" && <ClipboardCheck className="w-6 h-6 text-terracotta" />}
        {prompt.prompt_type === "confirm" && <AlertTriangle className="w-6 h-6 text-running" />}
        {prompt.prompt_type === "input" && <MessageSquareMore className="w-6 h-6 text-terracotta" />}
        <div>
          <Eyebrow size="sm" className="block mb-1">
            {eyebrowLabel}
          </Eyebrow>
          <h3 className="text-[20px] font-medium text-ink leading-tight">{prompt.title}</h3>
        </div>
      </div>

      <p className="text-[14.5px] text-ink-soft leading-relaxed">{prompt.message}</p>

      {prompt.prompt_type === "confirm" && (
        <div className="flex gap-3 pt-2">
          <Button
            onClick={() => respond(true)}
            variant="primary"
            leftIcon={<CheckCircle className="w-4 h-4" />}
            className="flex-1"
          >
            Confirm
          </Button>
          <Button
            onClick={() => respond(false)}
            variant="ghost"
            leftIcon={<XCircle className="w-4 h-4" />}
            className="flex-1"
          >
            Reject
          </Button>
        </div>
      )}

      {prompt.prompt_type === "input" && (
        <div className="space-y-3 pt-2">
          {prompt.input_type === "choice" && prompt.choices.length > 0 ? (
            <div className="space-y-2">
              {prompt.choices.map((c) => (
                <label
                  key={c}
                  className={clsx(
                    "flex items-center gap-3 p-3 border-l-2 border cursor-pointer transition-colors text-[14px]",
                    inputValue === c
                      ? "border-l-terracotta border-hair bg-terracotta-tint text-ink"
                      : "border-l-transparent border-hair text-ink-soft hover:bg-cream",
                  )}
                >
                  <input
                    type="radio"
                    name={`overlay-choice-${prompt.prompt_id}`}
                    value={c}
                    checked={inputValue === c}
                    onChange={() => setInputValue(c)}
                    className="accent-terracotta"
                  />
                  {c}
                </label>
              ))}
            </div>
          ) : (
            <Input
              type={prompt.input_type === "number" ? "number" : "text"}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Enter value…"
              autoFocus
            />
          )}
          <div className="flex gap-3">
            <Button
              onClick={() => respond(true, inputValue)}
              disabled={!inputValue}
              variant="primary"
              leftIcon={<Send className="w-4 h-4" />}
              className="flex-1"
            >
              Submit
            </Button>
            <Button onClick={() => respond(false)} variant="ghost">
              Cancel
            </Button>
          </div>
        </div>
      )}

      {prompt.prompt_type === "task" && (
        <div className="space-y-3 pt-2">
          <Input
            type="text"
            value={notes}
            onChange={(e) => setNotes(e.target.value)}
            placeholder="Optional notes…"
          />
          <div className="flex gap-3">
            <Button
              onClick={() => respond(true, notes)}
              variant="primary"
              leftIcon={<CheckCircle className="w-4 h-4" />}
              className="flex-1"
            >
              Done
            </Button>
            <Button
              onClick={() => respond(false, notes)}
              variant="ghost"
              leftIcon={<XCircle className="w-4 h-4" />}
              className="flex-1"
            >
              Failed
            </Button>
          </div>
        </div>
      )}

      {queueCount > 1 && (
        <div className="text-center font-mono text-[10px] text-muted uppercase tracking-[0.1em]">
          +{queueCount - 1} more prompt{queueCount > 2 ? "s" : ""} waiting
        </div>
      )}

      <div className="text-center font-mono text-[10px] text-muted tracking-[0.06em] pt-1 border-t border-hair-soft">
        From BT node: {prompt.bt_node_name}
      </div>
    </div>
  );
}
