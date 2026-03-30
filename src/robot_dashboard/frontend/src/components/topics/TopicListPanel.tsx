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
  Braces as BracesIcon,
} from "lucide-react";
import clsx from "clsx";

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

  // Manage actual rosbridge subscriptions
  const toggleSubscription = useCallback(
    (topic: TopicInfo) => {
      if (subscriptions.has(topic.name)) {
        // Unsubscribe
        const rosTopic = rosSubsRef.current.get(topic.name);
        if (rosTopic) {
          rosTopic.unsubscribe();
          rosSubsRef.current.delete(topic.name);
        }
        removeSubscription(topic.name);
      } else {
        // Subscribe
        addSubscription(topic.name, topic.type);
        const rosTopic = new ROSLIB.Topic({
          ros: getRos(),
          name: topic.name,
          messageType: topic.type,
          throttle_rate: 200, // 5 Hz max for UI
        });
        rosTopic.subscribe((msg: any) => {
          addMessage(topic.name, msg);
        });
        rosSubsRef.current.set(topic.name, rosTopic);
      }
    },
    [subscriptions, addSubscription, removeSubscription, addMessage]
  );

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      rosSubsRef.current.forEach((t) => t.unsubscribe());
      rosSubsRef.current.clear();
    };
  }, []);

  const selectedSub = selectedTopic ? subscriptions.get(selectedTopic) : null;

  return (
    <div className="flex h-full">
      {/* Left: topic list */}
      <div className="w-80 border-r border-gray-800 flex flex-col shrink-0">
        <div className="px-3 py-2 border-b border-gray-800">
          <div className="flex items-center gap-2 mb-2">
            <Radio className="w-4 h-4 text-blue-400" />
            <h2 className="text-sm font-semibold">Topics</h2>
            <span className="text-xs text-gray-500">{topics.length}</span>
          </div>
          <div className="relative">
            <Search className="absolute left-2 top-1/2 -translate-y-1/2 w-3.5 h-3.5 text-gray-500" />
            <input
              type="text"
              placeholder="Filter topics..."
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              className="w-full pl-7 pr-3 py-1 text-xs bg-gray-800 border border-gray-700 rounded focus:border-blue-500 focus:outline-none text-gray-200 placeholder-gray-500"
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
                  "px-3 py-1.5 border-b border-gray-800/50 flex items-center gap-2 cursor-pointer hover:bg-gray-800/50 transition-colors",
                  isSelected && "bg-gray-800/80"
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
                    "p-0.5 rounded",
                    isSubscribed ? "text-blue-400" : "text-gray-600 hover:text-gray-400"
                  )}
                  title={isSubscribed ? "Unsubscribe" : "Subscribe"}
                >
                  {isSubscribed ? <Eye className="w-3.5 h-3.5" /> : <EyeOff className="w-3.5 h-3.5" />}
                </button>

                <div className="flex-1 min-w-0">
                  <div className="text-[11px] text-gray-200 truncate font-mono">
                    {topic.name}
                  </div>
                  <div className="text-[9px] text-gray-500 truncate">
                    {topic.type}
                  </div>
                </div>

                {isSubscribed && sub && (
                  <div className="flex items-center gap-1 text-[9px] text-gray-500 shrink-0">
                    <Gauge className="w-3 h-3" />
                    {sub.hz} Hz
                  </div>
                )}

                <ChevronRight className="w-3 h-3 text-gray-700 shrink-0" />
              </div>
            );
          })}

          {filtered.length === 0 && (
            <p className="text-xs text-gray-600 text-center py-8">
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
          <div className="flex flex-col items-center justify-center h-full text-gray-500">
            <Braces className="w-8 h-8 mb-2 opacity-30" />
            <p className="text-sm">Select a topic</p>
            <p className="text-xs">Click a topic to subscribe and view messages</p>
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
      {/* Header */}
      <div className="px-4 py-2 border-b border-gray-800">
        <div className="flex items-center gap-2">
          <div className="flex-1 min-w-0">
            <div className="text-xs font-mono text-gray-200 truncate">{sub.name}</div>
            <div className="flex items-center gap-3 text-[10px] text-gray-500 mt-0.5">
              <span>{sub.type}</span>
              <span>{sub.hz} Hz</span>
              <span>{sub.msgCount} msgs</span>
            </div>
          </div>
          <div className="flex gap-0.5 shrink-0">
            <button
              onClick={() => setViewMode("rich")}
              className={clsx(
                "px-1.5 py-0.5 rounded text-[9px] font-medium",
                viewMode === "rich" ? "bg-blue-500/20 text-blue-400" : "text-gray-500 hover:text-gray-300"
              )}
            >
              Rich
            </button>
            <button
              onClick={() => setViewMode("json")}
              className={clsx(
                "px-1.5 py-0.5 rounded text-[9px] font-medium",
                viewMode === "json" ? "bg-blue-500/20 text-blue-400" : "text-gray-500 hover:text-gray-300"
              )}
            >
              JSON
            </button>
          </div>
        </div>
      </div>

      {/* Latest message */}
      <div className="flex-1 overflow-auto p-3">
        {lastMsg ? (
          viewMode === "rich" ? (
            <MessageRenderer type={sub.type} msg={lastMsg} messages={sub.messages} />
          ) : (
            <pre className="text-[10px] font-mono text-gray-300 leading-relaxed whitespace-pre-wrap">
              {JSON.stringify(lastMsg, null, 2)}
            </pre>
          )
        ) : (
          <p className="text-xs text-gray-500 text-center py-8">
            Waiting for messages...
          </p>
        )}
      </div>
    </div>
  );
}
