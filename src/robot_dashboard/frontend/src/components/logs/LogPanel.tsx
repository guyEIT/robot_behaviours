import { useState } from "react";
import LogStream from "./LogStream";
import DiagnosticsTree from "./DiagnosticsTree";
import { ScrollText, HeartPulse } from "lucide-react";
import clsx from "clsx";

type Tab = "logs" | "diagnostics";

export default function LogPanel() {
  const [tab, setTab] = useState<Tab>("logs");

  return (
    <div className="flex flex-col h-full">
      {/* Tab bar */}
      <div className="flex border-b border-gray-800">
        <TabButton
          active={tab === "logs"}
          onClick={() => setTab("logs")}
          icon={<ScrollText className="w-3.5 h-3.5" />}
          label="Logs"
        />
        <TabButton
          active={tab === "diagnostics"}
          onClick={() => setTab("diagnostics")}
          icon={<HeartPulse className="w-3.5 h-3.5" />}
          label="Diagnostics"
        />
      </div>

      {/* Content */}
      <div className="flex-1 overflow-hidden">
        {tab === "logs" ? <LogStream /> : <DiagnosticsTree />}
      </div>
    </div>
  );
}

function TabButton({
  active,
  onClick,
  icon,
  label,
}: {
  active: boolean;
  onClick: () => void;
  icon: React.ReactNode;
  label: string;
}) {
  return (
    <button
      onClick={onClick}
      className={clsx(
        "flex items-center gap-1.5 px-4 py-2 text-xs font-medium border-b-2 transition-all",
        active
          ? "text-blue-400 border-blue-400"
          : "text-gray-500 border-transparent hover:text-gray-300"
      )}
    >
      {icon}
      {label}
    </button>
  );
}
