import { create } from "zustand";

export interface TopicInfo {
  name: string;
  type: string;
}

export interface TopicSubscription {
  name: string;
  type: string;
  messages: any[];
  hz: number;
  lastTime: number;
  msgCount: number;
}

const MAX_MESSAGES = 50;

interface TopicStoreState {
  /** All discovered topics */
  topics: TopicInfo[];
  /** Active subscriptions */
  subscriptions: Map<string, TopicSubscription>;
  /** Currently selected topic for detail view */
  selectedTopic: string | null;

  setTopics: (topics: TopicInfo[]) => void;
  addSubscription: (name: string, type: string) => void;
  removeSubscription: (name: string) => void;
  addMessage: (topicName: string, msg: any) => void;
  setSelectedTopic: (name: string | null) => void;
}

export const useTopicStore = create<TopicStoreState>((set, get) => ({
  topics: [],
  subscriptions: new Map(),
  selectedTopic: null,

  setTopics: (topics) => set({ topics }),

  addSubscription: (name, type) => {
    const subs = new Map(get().subscriptions);
    if (!subs.has(name)) {
      subs.set(name, { name, type, messages: [], hz: 0, lastTime: 0, msgCount: 0 });
      set({ subscriptions: subs });
    }
  },

  removeSubscription: (name) => {
    const subs = new Map(get().subscriptions);
    subs.delete(name);
    set({ subscriptions: subs });
  },

  addMessage: (topicName, msg) => {
    const subs = new Map(get().subscriptions);
    const sub = subs.get(topicName);
    if (!sub) return;

    const now = performance.now();
    const dt = sub.lastTime > 0 ? (now - sub.lastTime) / 1000 : 0;
    const hz = dt > 0 ? 0.7 * sub.hz + 0.3 * (1 / dt) : sub.hz;

    const messages =
      sub.messages.length >= MAX_MESSAGES
        ? [...sub.messages.slice(1), msg]
        : [...sub.messages, msg];

    subs.set(topicName, {
      ...sub,
      messages,
      hz: Math.round(hz * 10) / 10,
      lastTime: now,
      msgCount: sub.msgCount + 1,
    });
    set({ subscriptions: subs });
  },

  setSelectedTopic: (name) => set({ selectedTopic: name }),
}));
