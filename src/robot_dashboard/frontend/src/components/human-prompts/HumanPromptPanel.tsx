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
import { Button, Chip, Eyebrow, Input, IconBtn } from "../ui";

export default function HumanPromptPanel() {
  const activePrompts = useHumanPromptStore((s) => s.activePrompts);
  const recentNotifications = useHumanPromptStore((s) => s.recentNotifications);
  const clearNotifications = useHumanPromptStore((s) => s.clearNotifications);

  return (
    <div className="flex flex-col h-full bg-paper">
      <div className="px-5 py-3 border-b border-hair flex items-center gap-2">
        <MessageSquareMore className="w-4 h-4 text-terracotta" />
        <h2 className="text-[14px] font-medium text-ink">Human Prompts</h2>
        {activePrompts.length > 0 && (
          <span className="ml-auto">
            <Chip state="running" showDot>
              {activePrompts.length} awaiting
            </Chip>
          </span>
        )}
      </div>

      <div className="flex-1 overflow-auto p-5 space-y-4">
        {activePrompts.length > 0 && (
          <div>
            <Eyebrow size="sm" className="block mb-2">
              Awaiting Response
            </Eyebrow>
            {activePrompts.map((p) => (
              <ActivePromptCard key={p.prompt_id} prompt={p} />
            ))}
          </div>
        )}

        {activePrompts.length === 0 && (
          <div className="text-center text-muted text-[12px] py-8">
            No active prompts — the robot will request input here when needed
          </div>
        )}

        {recentNotifications.length > 0 && (
          <div>
            <div className="flex items-center justify-between mb-2">
              <Eyebrow size="sm" tone="muted">
                Recent Notifications
              </Eyebrow>
              <IconBtn
                onClick={clearNotifications}
                className="!w-auto !h-6 px-2 gap-1 font-mono text-[10px] uppercase tracking-[0.08em]"
              >
                <Trash2 className="w-2.5 h-2.5" />
                Clear
              </IconBtn>
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

  return (
    <div className="p-4 border border-hair border-l-2 border-l-terracotta bg-paper space-y-3 mb-2">
      <div className="flex items-center gap-2">
        {prompt.prompt_type === "task" && <ClipboardCheck className="w-4 h-4 text-terracotta" />}
        {prompt.prompt_type === "confirm" && <AlertTriangle className="w-4 h-4 text-running" />}
        {prompt.prompt_type === "input" && <MessageSquareMore className="w-4 h-4 text-terracotta" />}
        <span className="text-[14px] font-medium text-ink">{prompt.title}</span>
        <span className="ml-auto font-mono text-[10px] text-muted uppercase tracking-[0.1em]">
          {prompt.prompt_type}
        </span>
      </div>

      <p className="text-[13px] text-ink-soft leading-relaxed">{prompt.message}</p>

      {prompt.bt_node_name && (
        <div className="font-mono text-[10px] text-muted tracking-[0.06em]">
          Node: {prompt.bt_node_name}
        </div>
      )}

      {prompt.prompt_type === "confirm" && (
        <div className="flex gap-2 pt-1">
          <Button
            onClick={() => respond(true)}
            variant="primary"
            size="sm"
            leftIcon={<CheckCircle className="w-3.5 h-3.5" />}
            className="flex-1"
          >
            Confirm
          </Button>
          <Button
            onClick={() => respond(false)}
            variant="ghost"
            size="sm"
            leftIcon={<XCircle className="w-3.5 h-3.5" />}
            className="flex-1"
          >
            Reject
          </Button>
        </div>
      )}

      {prompt.prompt_type === "input" && (
        <div className="space-y-2 pt-1">
          {prompt.input_type === "choice" && prompt.choices.length > 0 ? (
            <div className="space-y-1">
              {prompt.choices.map((c) => (
                <label
                  key={c}
                  className="flex items-center gap-2 text-[13px] text-ink-soft cursor-pointer py-1"
                >
                  <input
                    type="radio"
                    name={`choice-${prompt.prompt_id}`}
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
              className="!py-2"
            />
          )}
          <div className="flex gap-2">
            <Button
              onClick={() => respond(true, inputValue)}
              disabled={!inputValue}
              variant="primary"
              size="sm"
              leftIcon={<Send className="w-3 h-3" />}
              className="flex-1"
            >
              Submit
            </Button>
            <Button onClick={() => respond(false)} variant="ghost" size="sm">
              Cancel
            </Button>
          </div>
        </div>
      )}

      {prompt.prompt_type === "task" && (
        <div className="space-y-2 pt-1">
          <Input
            type="text"
            value={notes}
            onChange={(e) => setNotes(e.target.value)}
            placeholder="Optional notes…"
            className="!py-1.5"
          />
          <div className="flex gap-2">
            <Button
              onClick={() => respond(true, notes)}
              variant="primary"
              size="sm"
              leftIcon={<CheckCircle className="w-3.5 h-3.5" />}
              className="flex-1"
            >
              Done
            </Button>
            <Button
              onClick={() => respond(false, notes)}
              variant="ghost"
              size="sm"
              leftIcon={<XCircle className="w-3.5 h-3.5" />}
              className="flex-1"
            >
              Failed
            </Button>
          </div>
        </div>
      )}

      {prompt.timeout_sec > 0 && (
        <CountdownTimer startTime={prompt.stamp} timeoutSec={prompt.timeout_sec} />
      )}
    </div>
  );
}

function CountdownTimer({
  startTime,
  timeoutSec,
}: {
  startTime: { sec: number; nanosec: number };
  timeoutSec: number;
}) {
  const [, forceUpdate] = useState(0);

  setTimeout(() => forceUpdate((n) => n + 1), 1000);

  const elapsed = Date.now() / 1000 - startTime.sec;
  const remaining = Math.max(0, timeoutSec - elapsed);
  const pct = (remaining / timeoutSec) * 100;

  const fill = remaining < 10 ? "bg-err" : remaining < 30 ? "bg-terracotta" : "bg-running";

  return (
    <div className="space-y-1 pt-1">
      <div className="h-1 bg-stone overflow-hidden">
        <div
          className={clsx("h-full transition-all duration-1000", fill)}
          style={{ width: `${pct}%` }}
        />
      </div>
      <div className="text-[10px] font-mono text-muted text-right tracking-[0.06em]">
        {Math.ceil(remaining)}s remaining
      </div>
    </div>
  );
}

function NotificationRow({ notification }: { notification: HumanPrompt }) {
  const icon =
    notification.severity === "error" || notification.severity === "critical" ? (
      <AlertTriangle className="w-3 h-3 text-err" />
    ) : notification.prompt_type === "warning" ? (
      <AlertTriangle className="w-3 h-3 text-terracotta" />
    ) : notification.severity === "info" ? (
      <Info className="w-3 h-3 text-running" />
    ) : (
      <Bell className="w-3 h-3 text-muted" />
    );

  return (
    <div className="flex items-start gap-2 px-3 py-2 border-l-2 border-hair-soft hover:border-terracotta hover:bg-cream transition-colors">
      <div className="mt-0.5 shrink-0">{icon}</div>
      <div className="min-w-0 flex-1">
        <div className="text-[12px] font-medium text-ink-soft truncate">{notification.title}</div>
        {notification.message && (
          <div className="text-[11px] text-muted truncate">{notification.message}</div>
        )}
      </div>
      <div className="font-mono text-[10px] text-muted shrink-0 tracking-[0.06em]">
        {notification.bt_node_name}
      </div>
    </div>
  );
}
