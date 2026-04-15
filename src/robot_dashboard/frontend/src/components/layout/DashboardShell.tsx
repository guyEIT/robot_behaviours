import { TreePine, Wifi, WifiOff, Bot } from "lucide-react";
import { useLayoutStore } from "../../stores/layout-store";
import { useRobotSelectorStore } from "../../stores/robot-selector-store";
import PanelHost from "./PanelHost";
import LayoutRenderer from "./LayoutRenderer";
import LayoutToolbar from "./LayoutToolbar";

interface Props {
  connected: boolean;
}

export default function DashboardShell({ connected }: Props) {
  const layout = useLayoutStore((s) => s.layout);
  const { availableRobots, selectedRobotId, setSelectedRobotId } =
    useRobotSelectorStore();

  return (
    <div className="h-screen flex flex-col bg-gray-950 text-gray-100">
      {/* Top bar */}
      <header className="h-10 flex items-center px-4 border-b border-gray-800 bg-gray-900/50 shrink-0">
        <div className="flex items-center gap-2 text-sm font-bold text-gray-200 shrink-0">
          <TreePine className="w-4 h-4 text-blue-400" />
          Robot Dashboard
        </div>

        {/* Layout controls */}
        <div className="ml-6 flex-1 flex items-center">
          <LayoutToolbar />
        </div>

        {/* Robot selector — only shown when multiple robots are registered */}
        {availableRobots.length > 0 && (
          <div className="ml-4 flex items-center gap-1">
            <Bot className="w-3.5 h-3.5 text-gray-400 shrink-0" />
            <div className="flex items-center gap-0.5">
              {availableRobots.map((robotId) => (
                <button
                  key={robotId}
                  onClick={() => setSelectedRobotId(robotId)}
                  className={`px-2 py-0.5 rounded text-xs font-medium transition-colors ${
                    selectedRobotId === robotId
                      ? "bg-blue-600 text-white"
                      : "bg-gray-800 text-gray-400 hover:bg-gray-700 hover:text-gray-200"
                  }`}
                >
                  {robotId}
                </button>
              ))}
            </div>
          </div>
        )}

        {/* Connection indicator */}
        <div className="ml-auto flex items-center gap-1.5 text-xs shrink-0">
          {connected ? (
            <>
              <Wifi className="w-3.5 h-3.5 text-green-400" />
              <span className="text-green-400">Connected</span>
            </>
          ) : (
            <>
              <WifiOff className="w-3.5 h-3.5 text-red-400" />
              <span className="text-red-400">Disconnected</span>
            </>
          )}
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
