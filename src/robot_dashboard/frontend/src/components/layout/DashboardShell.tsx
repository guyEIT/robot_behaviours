import { TreePine, Wifi, WifiOff } from "lucide-react";
import { useLayoutStore } from "../../stores/layout-store";
import PanelHost from "./PanelHost";
import LayoutRenderer from "./LayoutRenderer";
import LayoutToolbar from "./LayoutToolbar";

interface Props {
  connected: boolean;
}

export default function DashboardShell({ connected }: Props) {
  const layout = useLayoutStore((s) => s.layout);

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
