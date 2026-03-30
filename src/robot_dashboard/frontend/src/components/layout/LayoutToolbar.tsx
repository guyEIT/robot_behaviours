import { useState } from "react";
import { useLayoutStore, collectPanelIds } from "../../stores/layout-store";
import { PRESET_LAYOUTS } from "../../stores/layout-presets";
import { PANELS } from "./PanelRegistry";
import type { PanelId } from "../../stores/layout-types";
import {
  LayoutGrid,
  Plus,
  Save,
  Trash2,
  FolderOpen,
  ChevronDown,
} from "lucide-react";
import clsx from "clsx";

export default function LayoutToolbar() {
  const layout = useLayoutStore((s) => s.layout);
  const activePresetName = useLayoutStore((s) => s.activePresetName);
  const savedLayouts = useLayoutStore((s) => s.savedLayouts);
  const applyPreset = useLayoutStore((s) => s.applyPreset);
  const addPanel = useLayoutStore((s) => s.addPanel);
  const removePanel = useLayoutStore((s) => s.removePanel);
  const saveCurrentLayout = useLayoutStore((s) => s.saveCurrentLayout);
  const loadSavedLayout = useLayoutStore((s) => s.loadSavedLayout);
  const deleteSavedLayout = useLayoutStore((s) => s.deleteSavedLayout);

  const [showPanels, setShowPanels] = useState(false);
  const [showSave, setShowSave] = useState(false);
  const [saveName, setSaveName] = useState("");

  const activePanelIds = collectPanelIds(layout);

  const groups = [
    { key: "behavior" as const, label: "Behavior" },
    { key: "monitor" as const, label: "Monitor" },
    { key: "tools" as const, label: "Tools" },
  ];

  return (
    <div className="flex items-center gap-1.5">
      {/* Preset buttons */}
      {Object.keys(PRESET_LAYOUTS).map((name) => (
        <button
          key={name}
          onClick={() => applyPreset(name)}
          className={clsx(
            "px-2 py-0.5 rounded text-[10px] font-medium transition-colors",
            activePresetName === name
              ? "bg-blue-500/20 text-blue-400"
              : "text-gray-500 hover:text-gray-300 hover:bg-gray-800"
          )}
        >
          {name}
        </button>
      ))}

      <div className="w-px h-4 bg-gray-700 mx-1" />

      {/* Panel selector */}
      <div className="relative">
        <button
          onClick={() => { setShowPanels(!showPanels); setShowSave(false); }}
          className="flex items-center gap-1 px-2 py-0.5 rounded text-[10px] text-gray-400 hover:text-gray-200 hover:bg-gray-800"
        >
          <Plus className="w-3 h-3" />
          Panels
          <ChevronDown className="w-2.5 h-2.5" />
        </button>

        {showPanels && (
          <div className="absolute top-full right-0 mt-1 w-64 bg-gray-900 border border-gray-700 rounded-lg shadow-xl z-50 p-2">
            {groups.map((group) => (
              <div key={group.key} className="mb-2">
                <div className="text-[9px] text-gray-500 font-bold uppercase tracking-wider px-1 mb-1">
                  {group.label}
                </div>
                {PANELS.filter((p) => p.group === group.key).map((panel) => {
                  const active = activePanelIds.has(panel.id);
                  return (
                    <button
                      key={panel.id}
                      onClick={() => {
                        if (active) removePanel(panel.id);
                        else addPanel(panel.id);
                      }}
                      className={clsx(
                        "w-full flex items-center gap-2 px-2 py-1 rounded text-[10px] transition-colors",
                        active
                          ? "bg-blue-500/10 text-blue-400"
                          : "text-gray-500 hover:text-gray-300 hover:bg-gray-800"
                      )}
                    >
                      <span className={clsx(
                        "w-3 h-3 rounded border flex items-center justify-center text-[8px]",
                        active ? "border-blue-400 bg-blue-500/20" : "border-gray-600"
                      )}>
                        {active && "✓"}
                      </span>
                      {panel.icon}
                      {panel.label}
                    </button>
                  );
                })}
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Save/Load */}
      <div className="relative">
        <button
          onClick={() => { setShowSave(!showSave); setShowPanels(false); }}
          className="flex items-center gap-1 px-2 py-0.5 rounded text-[10px] text-gray-400 hover:text-gray-200 hover:bg-gray-800"
        >
          <LayoutGrid className="w-3 h-3" />
          Layouts
          <ChevronDown className="w-2.5 h-2.5" />
        </button>

        {showSave && (
          <div className="absolute top-full right-0 mt-1 w-56 bg-gray-900 border border-gray-700 rounded-lg shadow-xl z-50 p-2">
            {/* Save form */}
            <div className="flex gap-1 mb-2">
              <input
                type="text"
                value={saveName}
                onChange={(e) => setSaveName(e.target.value)}
                placeholder="Layout name..."
                className="flex-1 px-2 py-1 text-[10px] bg-gray-800 border border-gray-700 rounded text-gray-200 placeholder-gray-600 focus:border-blue-500 focus:outline-none"
              />
              <button
                onClick={() => {
                  if (saveName.trim()) {
                    saveCurrentLayout(saveName.trim());
                    setSaveName("");
                  }
                }}
                disabled={!saveName.trim()}
                className="px-2 py-1 rounded bg-blue-600 hover:bg-blue-500 text-[10px] text-white disabled:bg-gray-700 disabled:text-gray-500"
              >
                <Save className="w-3 h-3" />
              </button>
            </div>

            {/* Saved layouts list */}
            {savedLayouts.length > 0 ? (
              <div className="space-y-0.5">
                <div className="text-[9px] text-gray-500 font-bold uppercase tracking-wider px-1 mb-1">
                  Saved
                </div>
                {savedLayouts.map((saved) => (
                  <div
                    key={saved.name}
                    className="flex items-center gap-1 px-1.5 py-1 rounded hover:bg-gray-800 group"
                  >
                    <button
                      onClick={() => { loadSavedLayout(saved.name); setShowSave(false); }}
                      className="flex items-center gap-1.5 flex-1 text-[10px] text-gray-300"
                    >
                      <FolderOpen className="w-3 h-3 text-gray-500" />
                      {saved.name}
                    </button>
                    <button
                      onClick={() => deleteSavedLayout(saved.name)}
                      className="p-0.5 text-gray-600 hover:text-red-400 opacity-0 group-hover:opacity-100"
                    >
                      <Trash2 className="w-3 h-3" />
                    </button>
                  </div>
                ))}
              </div>
            ) : (
              <p className="text-[10px] text-gray-600 text-center py-2">No saved layouts</p>
            )}
          </div>
        )}
      </div>
    </div>
  );
}
