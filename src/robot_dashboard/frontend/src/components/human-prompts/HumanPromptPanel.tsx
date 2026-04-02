import { useState } from "react";
import { useHumanPromptStore } from "../../stores/human-prompt-store";
import { useTopicPublisher } from "../../hooks/useTopicPublisher";
import type { HumanPrompt, HumanResponse } from "../../types/ros";
import {
  MessageSquareMore,
  CheckCircle,
  XCircle,
  Send,
  ClipboardCheck,
  AlertTriangle,
  Info,
  Bell,
  Trash2,
} from "lucide-react";
import clsx from "clsx";

export default function HumanPromptPanel() {
  const activePrompts = useHumanPromptStore((s) => s.activePrompts);
  const recentNotifications = useHumanPromptStore((s) => s.recentNotifications);
  const clearNotifications = useHumanPromptStore((s) => s.clearNotifications);

  return (
    <div className="flex flex-col h-full">
      <div className="px-4 py-2 border-b border-gray-800 flex items-center gap-2">
        <MessageSquareMore className="w-4 h-4 text-purple-400" />
        <h2 className="text-sm font-semibold">Human Prompts</h2>
        {activePrompts.length > 0 && (
          <span className="ml-auto px-1.5 py-0.5 rounded-full bg-purple-500/20 text-purple-400 text-[10px] font-bold animate-pulse">
            {activePrompts.length} awaiting
          </span>
        )}
      </div>

      <div className="flex-1 overflow-auto p-3 space-y-3">
        {/* Active prompts */}
        {activePrompts.length > 0 && (
          <div>
            <div className="text-[10px] text-gray-500 font-medium uppercase mb-2">
              Awaiting Response
            </div>
            {activePrompts.map((p) => (
              <ActivePromptCard key={p.prompt_id} prompt={p} />
            ))}
          </div>
        )}

        {activePrompts.length === 0 && (
          <div className="text-center text-gray-600 text-xs py-8">
            No active prompts — the robot will request input here when needed
          </div>
        )}

        {/* Notification history */}
        {recentNotifications.length > 0 && (
          <div>
            <div className="flex items-center justify-between mb-2">
              <div className="text-[10px] text-gray-500 font-medium uppercase">
                Recent Notifications
              </div>
              <button
                onClick={clearNotifications}
                className="text-[10px] text-gray-600 hover:text-gray-400 flex items-center gap-0.5"
              >
                <Trash2 className="w-2.5 h-2.5" />
                Clear
              </button>
            </div>
            <div className="space-y-1">
              {recentNotifications.map((n) => (
                <NotificationRow key={n.prompt_id} notification={n} />
              ))}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

function ActivePromptCard({ prompt }: { prompt: HumanPrompt }) {
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

  const borderColor =
    prompt.prompt_type === "task" ? "border-orange-500/50" :
    prompt.prompt_type === "confirm" ? "border-blue-500/50" :
    "border-purple-500/50";

  return (
    <div className={clsx("p-3 rounded-lg border bg-gray-900/80 space-y-2 mb-2", borderColor)}>
      <div className="flex items-center gap-2">
        {prompt.prompt_type === "task" && <ClipboardCheck className="w-4 h-4 text-orange-400" />}
        {prompt.prompt_type === "confirm" && <AlertTriangle className="w-4 h-4 text-blue-400" />}
        {prompt.prompt_type === "input" && <MessageSquareMore className="w-4 h-4 text-purple-400" />}
        <span className="text-xs font-bold text-gray-200">{prompt.title}</span>
        <span className="ml-auto text-[9px] text-gray-600 uppercase">{prompt.prompt_type}</span>
      </div>

      <p className="text-xs text-gray-300">{prompt.message}</p>

      {prompt.bt_node_name && (
        <div className="text-[9px] text-gray-600">Node: {prompt.bt_node_name}</div>
      )}

      {/* Confirm: two buttons */}
      {prompt.prompt_type === "confirm" && (
        <div className="flex gap-2 pt-1">
          <button
            onClick={() => respond(true)}
            className="flex-1 py-2 rounded-lg bg-green-700 hover:bg-green-600 text-white text-xs font-semibold flex items-center justify-center gap-1.5"
          >
            <CheckCircle className="w-3.5 h-3.5" />
            Confirm
          </button>
          <button
            onClick={() => respond(false)}
            className="flex-1 py-2 rounded-lg bg-red-700 hover:bg-red-600 text-white text-xs font-semibold flex items-center justify-center gap-1.5"
          >
            <XCircle className="w-3.5 h-3.5" />
            Reject
          </button>
        </div>
      )}

      {/* Input: field + submit */}
      {prompt.prompt_type === "input" && (
        <div className="space-y-2 pt-1">
          {prompt.input_type === "choice" && prompt.choices.length > 0 ? (
            <div className="space-y-1">
              {prompt.choices.map((c) => (
                <label key={c} className="flex items-center gap-2 text-xs text-gray-300 cursor-pointer">
                  <input
                    type="radio"
                    name={`choice-${prompt.prompt_id}`}
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
              className="w-full px-2 py-1.5 text-xs bg-gray-800 border border-gray-700 rounded focus:border-purple-500 focus:outline-none text-gray-200"
            />
          )}
          <div className="flex gap-2">
            <button
              onClick={() => respond(true, inputValue)}
              disabled={!inputValue}
              className={clsx(
                "flex-1 py-1.5 rounded-lg text-xs font-semibold flex items-center justify-center gap-1.5",
                inputValue
                  ? "bg-purple-600 hover:bg-purple-500 text-white"
                  : "bg-gray-800 text-gray-600 cursor-not-allowed"
              )}
            >
              <Send className="w-3 h-3" />
              Submit
            </button>
            <button
              onClick={() => respond(false)}
              className="px-3 py-1.5 rounded-lg bg-gray-700 hover:bg-gray-600 text-xs text-gray-300"
            >
              Cancel
            </button>
          </div>
        </div>
      )}

      {/* Task: done/failed + notes */}
      {prompt.prompt_type === "task" && (
        <div className="space-y-2 pt-1">
          <input
            type="text"
            value={notes}
            onChange={(e) => setNotes(e.target.value)}
            placeholder="Optional notes..."
            className="w-full px-2 py-1 text-[11px] bg-gray-800 border border-gray-700 rounded focus:border-orange-500 focus:outline-none text-gray-300"
          />
          <div className="flex gap-2">
            <button
              onClick={() => respond(true, notes)}
              className="flex-1 py-2 rounded-lg bg-green-700 hover:bg-green-600 text-white text-xs font-semibold flex items-center justify-center gap-1.5"
            >
              <CheckCircle className="w-3.5 h-3.5" />
              Done
            </button>
            <button
              onClick={() => respond(false, notes)}
              className="flex-1 py-2 rounded-lg bg-red-700 hover:bg-red-600 text-white text-xs font-semibold flex items-center justify-center gap-1.5"
            >
              <XCircle className="w-3.5 h-3.5" />
              Failed
            </button>
          </div>
        </div>
      )}

      {prompt.timeout_sec > 0 && (
        <CountdownTimer startTime={prompt.stamp} timeoutSec={prompt.timeout_sec} />
      )}
    </div>
  );
}

function CountdownTimer({ startTime, timeoutSec }: { startTime: { sec: number; nanosec: number }; timeoutSec: number }) {
  const [, forceUpdate] = useState(0);

  // Re-render every second
  setTimeout(() => forceUpdate((n) => n + 1), 1000);

  const elapsed = Date.now() / 1000 - startTime.sec;
  const remaining = Math.max(0, timeoutSec - elapsed);
  const pct = (remaining / timeoutSec) * 100;

  return (
    <div className="space-y-0.5">
      <div className="h-1 bg-gray-800 rounded-full overflow-hidden">
        <div
          className={clsx(
            "h-full rounded-full transition-all duration-1000",
            remaining < 10 ? "bg-red-500" : remaining < 30 ? "bg-yellow-500" : "bg-blue-500"
          )}
          style={{ width: `${pct}%` }}
        />
      </div>
      <div className="text-[9px] text-gray-600 text-right">
        {Math.ceil(remaining)}s remaining
      </div>
    </div>
  );
}

function NotificationRow({ notification }: { notification: HumanPrompt }) {
  const icon =
    notification.severity === "error" || notification.severity === "critical"
      ? <AlertTriangle className="w-3 h-3 text-red-400" />
      : notification.prompt_type === "warning"
      ? <AlertTriangle className="w-3 h-3 text-yellow-400" />
      : notification.severity === "info"
      ? <Info className="w-3 h-3 text-blue-400" />
      : <Bell className="w-3 h-3 text-gray-400" />;

  return (
    <div className="flex items-start gap-2 px-2 py-1.5 rounded bg-gray-900/40 border border-gray-800/50">
      <div className="mt-0.5 shrink-0">{icon}</div>
      <div className="min-w-0">
        <div className="text-[11px] font-medium text-gray-300 truncate">{notification.title}</div>
        {notification.message && (
          <div className="text-[10px] text-gray-500 truncate">{notification.message}</div>
        )}
      </div>
      <div className="ml-auto text-[9px] text-gray-600 shrink-0">
        {notification.bt_node_name}
      </div>
    </div>
  );
}
