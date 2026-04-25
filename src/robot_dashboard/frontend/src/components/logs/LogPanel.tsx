import { useState } from "react";
import LogStream from "./LogStream";
import DiagnosticsTree from "./DiagnosticsTree";
import { ScrollText, HeartPulse } from "lucide-react";
import { TabBar } from "../ui";

type Tab = "logs" | "diagnostics";

export default function LogPanel() {
  const [tab, setTab] = useState<Tab>("logs");

  return (
    <div className="flex flex-col h-full bg-paper">
      <TabBar
        tabs={[
          { id: "logs", label: <span className="inline-flex items-center gap-1.5"><ScrollText className="w-3.5 h-3.5" />Logs</span> },
          { id: "diagnostics", label: <span className="inline-flex items-center gap-1.5"><HeartPulse className="w-3.5 h-3.5" />Diagnostics</span> },
        ]}
        active={tab}
        onSelect={(id) => setTab(id as Tab)}
      />

      <div className="flex-1 overflow-hidden">
        {tab === "logs" ? <LogStream /> : <DiagnosticsTree />}
      </div>
    </div>
  );
}
