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

/**
 * Full-screen overlay that appears when blocking prompts are active.
 * Ensures the operator can't miss a confirm/input/task request.
 */
export default function HumanPromptOverlay() {
  const activePrompts = useHumanPromptStore((s) => s.activePrompts);

  if (activePrompts.length === 0) return null;

  // Show the oldest prompt first
  const prompt = activePrompts[0];

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 backdrop-blur-sm">
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

  const accentColor =
    prompt.prompt_type === "task" ? "orange" :
    prompt.prompt_type === "confirm" ? "blue" : "purple";

  const borderClass =
    accentColor === "orange" ? "border-orange-500" :
    accentColor === "blue" ? "border-blue-500" : "border-purple-500";

  return (
    <div className={clsx(
      "w-full max-w-md mx-4 rounded-2xl border-2 bg-gray-900 shadow-2xl p-6 space-y-4",
      borderClass
    )}>
      {/* Header */}
      <div className="flex items-center gap-3">
        {prompt.prompt_type === "task" && <ClipboardCheck className="w-6 h-6 text-orange-400" />}
        {prompt.prompt_type === "confirm" && <AlertTriangle className="w-6 h-6 text-blue-400" />}
        {prompt.prompt_type === "input" && <MessageSquareMore className="w-6 h-6 text-purple-400" />}
        <div>
          <div className="text-[10px] text-gray-500 uppercase font-bold">
            {prompt.prompt_type === "task" ? "Task Assigned" :
             prompt.prompt_type === "confirm" ? "Confirmation Required" :
             "Input Required"}
          </div>
          <h3 className="text-lg font-bold text-gray-100">{prompt.title}</h3>
        </div>
      </div>

      {/* Message */}
      <p className="text-sm text-gray-300 leading-relaxed">{prompt.message}</p>

      {/* Confirm buttons */}
      {prompt.prompt_type === "confirm" && (
        <div className="flex gap-3 pt-2">
          <button
            onClick={() => respond(true)}
            className="flex-1 py-3 rounded-xl bg-green-600 hover:bg-green-500 text-white font-bold text-sm flex items-center justify-center gap-2 transition-all"
          >
            <CheckCircle className="w-4 h-4" />
            Confirm
          </button>
          <button
            onClick={() => respond(false)}
            className="flex-1 py-3 rounded-xl bg-red-600 hover:bg-red-500 text-white font-bold text-sm flex items-center justify-center gap-2 transition-all"
          >
            <XCircle className="w-4 h-4" />
            Reject
          </button>
        </div>
      )}

      {/* Input form */}
      {prompt.prompt_type === "input" && (
        <div className="space-y-3 pt-2">
          {prompt.input_type === "choice" && prompt.choices.length > 0 ? (
            <div className="space-y-2">
              {prompt.choices.map((c) => (
                <label
                  key={c}
                  className={clsx(
                    "flex items-center gap-3 p-2.5 rounded-lg border cursor-pointer transition-all text-sm",
                    inputValue === c
                      ? "border-purple-500 bg-purple-950/30 text-white"
                      : "border-gray-700 text-gray-400 hover:border-gray-500"
                  )}
                >
                  <input
                    type="radio"
                    name={`overlay-choice-${prompt.prompt_id}`}
                    value={c}
                    checked={inputValue === c}
                    onChange={() => setInputValue(c)}
                    className="accent-purple-500"
                  />
                  {c}
                </label>
              ))}
            </div>
          ) : (
            <input
              type={prompt.input_type === "number" ? "number" : "text"}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Enter value..."
              autoFocus
              className="w-full px-3 py-2.5 text-sm bg-gray-800 border border-gray-600 rounded-lg focus:border-purple-500 focus:outline-none text-gray-200"
            />
          )}
          <div className="flex gap-3">
            <button
              onClick={() => respond(true, inputValue)}
              disabled={!inputValue}
              className={clsx(
                "flex-1 py-2.5 rounded-xl font-bold text-sm flex items-center justify-center gap-2 transition-all",
                inputValue
                  ? "bg-purple-600 hover:bg-purple-500 text-white"
                  : "bg-gray-800 text-gray-600 cursor-not-allowed"
              )}
            >
              <Send className="w-4 h-4" />
              Submit
            </button>
            <button
              onClick={() => respond(false)}
              className="px-5 py-2.5 rounded-xl bg-gray-700 hover:bg-gray-600 text-sm text-gray-300 font-medium"
            >
              Cancel
            </button>
          </div>
        </div>
      )}

      {/* Task buttons */}
      {prompt.prompt_type === "task" && (
        <div className="space-y-3 pt-2">
          <input
            type="text"
            value={notes}
            onChange={(e) => setNotes(e.target.value)}
            placeholder="Optional notes..."
            className="w-full px-3 py-2 text-sm bg-gray-800 border border-gray-700 rounded-lg focus:border-orange-500 focus:outline-none text-gray-300"
          />
          <div className="flex gap-3">
            <button
              onClick={() => respond(true, notes)}
              className="flex-1 py-3 rounded-xl bg-green-600 hover:bg-green-500 text-white font-bold text-sm flex items-center justify-center gap-2 transition-all"
            >
              <CheckCircle className="w-4 h-4" />
              Done
            </button>
            <button
              onClick={() => respond(false, notes)}
              className="flex-1 py-3 rounded-xl bg-red-600 hover:bg-red-500 text-white font-bold text-sm flex items-center justify-center gap-2 transition-all"
            >
              <XCircle className="w-4 h-4" />
              Failed
            </button>
          </div>
        </div>
      )}

      {/* Queue indicator */}
      {queueCount > 1 && (
        <div className="text-center text-[10px] text-gray-600">
          +{queueCount - 1} more prompt{queueCount > 2 ? "s" : ""} waiting
        </div>
      )}

      {/* Source node */}
      <div className="text-[10px] text-gray-600 text-center">
        From BT node: {prompt.bt_node_name}
      </div>
    </div>
  );
}
