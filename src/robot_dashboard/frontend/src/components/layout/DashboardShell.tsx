import { Bot } from "lucide-react";
import { useLayoutStore } from "../../stores/layout-store";
import { useRobotSelectorStore } from "../../stores/robot-selector-store";
import PanelHost from "./PanelHost";
import LayoutRenderer from "./LayoutRenderer";
import LayoutToolbar from "./LayoutToolbar";
import { Chip, Eyebrow, IconBtn } from "../ui";
import clsx from "clsx";

interface Props {
  connected: boolean;
}

function BrandMark() {
  return (
    <div className="flex items-center gap-2.5 shrink-0">
      <span
        aria-hidden
        className="inline-flex items-center justify-center w-[26px] h-[26px] rounded-full bg-terracotta text-paper font-mono font-bold text-[12px]"
      >
        S
      </span>
      <span className="text-[14px] font-medium text-ink tracking-[0.02em]">Sociius</span>
      <span className="text-[12px] font-mono text-muted uppercase tracking-[0.1em] hidden sm:inline">
        Robot Dashboard
      </span>
    </div>
  );
}

export default function DashboardShell({ connected }: Props) {
  const layout = useLayoutStore((s) => s.layout);
  const { availableRobots, selectedRobotId, setSelectedRobotId } =
    useRobotSelectorStore();

  return (
    <div className="h-screen flex flex-col bg-cream text-ink-soft">
      {/* Top bar */}
      <header className="h-12 flex items-center px-5 border-b border-hair bg-paper shrink-0">
        <BrandMark />

        <div className="ml-6 flex-1 flex items-center">
          <LayoutToolbar />
        </div>

        {availableRobots.length > 0 && (
          <div className="ml-4 flex items-center gap-2">
            <Bot className="w-3.5 h-3.5 text-muted shrink-0" />
            <Eyebrow size="sm" tone="muted">Robot</Eyebrow>
            <div className="flex items-center gap-1">
              {availableRobots.map((robotId) => (
                <IconBtn
                  key={robotId}
                  active={selectedRobotId === robotId}
                  onClick={() => setSelectedRobotId(robotId)}
                  className="!w-auto px-2.5 !h-7 font-mono text-[11px] uppercase tracking-[0.06em]"
                  title={robotId}
                >
                  {robotId}
                </IconBtn>
              ))}
            </div>
          </div>
        )}

        {/* Connection indicator */}
        <div className={clsx("ml-auto shrink-0")}>
          <Chip state={connected ? "done" : "failed"} showDot>
            {connected ? "Connected" : "Offline"}
          </Chip>
        </div>
      </header>

      {/* Tiling layout */}
      <PanelHost>
        <main className="flex-1 overflow-hidden">
          <LayoutRenderer node={layout} />
        </main>
      </PanelHost>
    </div>
  );
}
