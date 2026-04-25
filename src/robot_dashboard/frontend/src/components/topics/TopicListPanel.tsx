import { useState, useMemo, useEffect, useRef, useCallback } from "react";
import ROSLIB from "roslib";
import { getRos } from "../../lib/rosbridge-client";
import { useTopicStore, type TopicInfo, type TopicSubscription } from "../../stores/topic-store";
import { useConnectionStore } from "../../stores/connection-store";
import MessageRenderer from "./MessageRenderer";
import {
  Radio,
  Search,
  Eye,
  EyeOff,
  Gauge,
  ChevronRight,
  Braces,
} from "lucide-react";
import clsx from "clsx";
import { Eyebrow, IconBtn, TabBar } from "../ui";

export default function TopicListPanel() {
  const topics = useTopicStore((s) => s.topics);
  const subscriptions = useTopicStore((s) => s.subscriptions);
  const selectedTopic = useTopicStore((s) => s.selectedTopic);
  const setSelectedTopic = useTopicStore((s) => s.setSelectedTopic);
  const addSubscription = useTopicStore((s) => s.addSubscription);
  const removeSubscription = useTopicStore((s) => s.removeSubscription);
  const addMessage = useTopicStore((s) => s.addMessage);
  const connected = useConnectionStore((s) => s.connected);

  const [search, setSearch] = useState("");
  const rosSubsRef = useRef<Map<string, ROSLIB.Topic>>(new Map());

  const filtered = useMemo(() => {
    if (!search) return topics;
    const q = search.toLowerCase();
    return topics.filter(
      (t) => t.name.toLowerCase().includes(q) || t.type.toLowerCase().includes(q)
    );
  }, [topics, search]);

  const toggleSubscription = useCallback(
    (topic: TopicInfo) => {
      if (subscriptions.has(topic.name)) {
        const rosTopic = rosSubsRef.current.get(topic.name);
        if (rosTopic) {
          rosTopic.unsubscribe();
          rosSubsRef.current.delete(topic.name);
        }
        removeSubscription(topic.name);
      } else {
        addSubscription(topic.name, topic.type);
        const rosTopic = new ROSLIB.Topic({
          ros: getRos(),
          name: topic.name,
          messageType: topic.type,
          throttle_rate: 200,
        });
        rosTopic.subscribe((msg: any) => {
          addMessage(topic.name, msg);
        });
        rosSubsRef.current.set(topic.name, rosTopic);
      }
    },
    [subscriptions, addSubscription, removeSubscription, addMessage]
  );

  useEffect(() => {
    return () => {
      rosSubsRef.current.forEach((t) => t.unsubscribe());
      rosSubsRef.current.clear();
    };
  }, []);

  const selectedSub = selectedTopic ? subscriptions.get(selectedTopic) : null;

  return (
    <div className="flex h-full bg-paper">
      {/* Left: topic list */}
      <div className="w-80 border-r border-hair flex flex-col shrink-0">
        <div className="px-4 py-3 border-b border-hair">
          <div className="flex items-center gap-2 mb-2">
            <Radio className="w-4 h-4 text-terracotta" />
            <h2 className="text-[14px] font-medium text-ink">Topics</h2>
            <Eyebrow size="sm" tone="muted">
              {topics.length}
            </Eyebrow>
          </div>
          <div className="relative">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-3.5 h-3.5 text-muted" />
            <input
              type="text"
              placeholder="Filter topics…"
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              className="w-full pl-9 pr-3 py-1.5 text-[12px] bg-paper border border-hair rounded-DEFAULT focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
            />
          </div>
        </div>

        <div className="flex-1 overflow-auto">
          {filtered.map((topic) => {
            const isSubscribed = subscriptions.has(topic.name);
            const sub = subscriptions.get(topic.name);
            const isSelected = selectedTopic === topic.name;

            return (
              <div
                key={topic.name}
                className={clsx(
                  "px-3 py-1.5 border-b border-hair-soft border-l-2 flex items-center gap-2 cursor-pointer hover:bg-cream transition-colors",
                  isSelected ? "border-l-terracotta bg-terracotta-tint" : "border-l-transparent",
                )}
                onClick={() => {
                  if (!isSubscribed) toggleSubscription(topic);
                  setSelectedTopic(topic.name);
                }}
              >
                <button
                  onClick={(e) => {
                    e.stopPropagation();
                    toggleSubscription(topic);
                  }}
                  className={clsx(
                    "p-0.5",
                    isSubscribed ? "text-terracotta" : "text-muted hover:text-ink-soft",
                  )}
                  title={isSubscribed ? "Unsubscribe" : "Subscribe"}
                >
                  {isSubscribed ? <Eye className="w-3.5 h-3.5" /> : <EyeOff className="w-3.5 h-3.5" />}
                </button>

                <div className="flex-1 min-w-0">
                  <div className="text-[11.5px] text-ink truncate font-mono tracking-[0.04em]">
                    {topic.name}
                  </div>
                  <div className="text-[10px] text-muted truncate">{topic.type}</div>
                </div>

                {isSubscribed && sub && (
                  <div className="flex items-center gap-1 text-[10px] text-muted shrink-0 font-mono tracking-[0.04em]">
                    <Gauge className="w-3 h-3" />
                    {sub.hz} Hz
                  </div>
                )}

                <ChevronRight className="w-3 h-3 text-muted shrink-0" />
              </div>
            );
          })}

          {filtered.length === 0 && (
            <p className="text-[12px] text-muted text-center py-8">
              {connected ? "No topics found" : "Not connected"}
            </p>
          )}
        </div>
      </div>

      {/* Right: message viewer */}
      <div className="flex-1 flex flex-col overflow-hidden">
        {selectedSub ? (
          <TopicMessageViewer sub={selectedSub} />
        ) : (
          <div className="flex flex-col items-center justify-center h-full text-muted bg-cream-deep">
            <Braces className="w-8 h-8 mb-2 opacity-30" />
            <p className="text-[14px] text-ink-soft">Select a topic</p>
            <p className="text-[12px]">Click a topic to subscribe and view messages</p>
          </div>
        )}
      </div>
    </div>
  );
}

function TopicMessageViewer({ sub }: { sub: TopicSubscription }) {
  const [viewMode, setViewMode] = useState<"rich" | "json">("rich");
  const lastMsg = sub.messages[sub.messages.length - 1];

  return (
    <div className="flex flex-col h-full">
      <div className="px-5 py-3 border-b border-hair">
        <div className="flex items-center gap-3">
          <div className="flex-1 min-w-0">
            <div className="text-[13px] font-mono text-ink truncate tracking-[0.04em]">{sub.name}</div>
            <div className="flex items-center gap-3 text-[11px] text-muted mt-1 font-mono tracking-[0.04em]">
              <span>{sub.type}</span>
              <span>{sub.hz} Hz</span>
              <span>{sub.msgCount} msgs</span>
            </div>
          </div>
        </div>
      </div>
      <TabBar
        size="sm"
        tabs={[
          { id: "rich", label: "Rich" },
          { id: "json", label: "JSON" },
        ]}
        active={viewMode}
        onSelect={(id) => setViewMode(id as "rich" | "json")}
      />

      <div className="flex-1 overflow-auto p-4">
        {lastMsg ? (
          viewMode === "rich" ? (
            <MessageRenderer type={sub.type} msg={lastMsg} messages={sub.messages} />
          ) : (
            <pre className="sociius-code whitespace-pre-wrap">
              {JSON.stringify(lastMsg, null, 2)}
            </pre>
          )
        ) : (
          <p className="text-[12px] text-muted text-center py-8">Waiting for messages…</p>
        )}
      </div>
    </div>
  );
}
