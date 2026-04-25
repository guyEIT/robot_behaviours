import { useState } from "react";
import { useLayoutStore, collectPanelIds } from "../../stores/layout-store";
import { PRESET_LAYOUTS } from "../../stores/layout-presets";
import { PANELS } from "./PanelRegistry";
import {
  LayoutGrid,
  Plus,
  Save,
  Trash2,
  FolderOpen,
  ChevronDown,
} from "lucide-react";
import clsx from "clsx";
import { Eyebrow, Input } from "../ui";

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
      {Object.keys(PRESET_LAYOUTS).map((name) => (
        <button
          key={name}
          onClick={() => applyPreset(name)}
          className={clsx(
            "px-2.5 py-1 rounded-sm font-mono text-[10px] uppercase tracking-[0.08em] font-semibold transition-colors",
            activePresetName === name
              ? "bg-terracotta-tint text-terracotta"
              : "text-muted hover:text-ink-soft hover:bg-cream",
          )}
        >
          {name}
        </button>
      ))}

      <div className="w-px h-4 bg-hair mx-1.5" />

      {/* Panel selector */}
      <div className="relative">
        <button
          onClick={() => { setShowPanels(!showPanels); setShowSave(false); }}
          className="flex items-center gap-1 px-2 py-1 rounded-sm font-mono text-[10px] uppercase tracking-[0.08em] text-muted hover:text-ink-soft hover:bg-cream transition-colors"
        >
          <Plus className="w-3 h-3" />
          Panels
          <ChevronDown className="w-2.5 h-2.5" />
        </button>

        {showPanels && (
          <div className="absolute top-full left-0 mt-1 w-64 bg-paper border border-hair z-50 p-3">
            {groups.map((group) => (
              <div key={group.key} className="mb-3 last:mb-0">
                <Eyebrow size="sm" tone="muted" className="block mb-1.5">
                  {group.label}
                </Eyebrow>
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
                        "w-full flex items-center gap-2 px-1.5 py-1 text-[12px] transition-colors",
                        active
                          ? "text-terracotta"
                          : "text-ink-soft hover:bg-cream",
                      )}
                    >
                      <span
                        className={clsx(
                          "w-3.5 h-3.5 rounded-sm border flex items-center justify-center text-[9px] font-mono font-bold shrink-0",
                          active
                            ? "border-terracotta bg-terracotta text-paper"
                            : "border-hair",
                        )}
                      >
                        {active && "✓"}
                      </span>
                      <span className="text-muted shrink-0">{panel.icon}</span>
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
          className="flex items-center gap-1 px-2 py-1 rounded-sm font-mono text-[10px] uppercase tracking-[0.08em] text-muted hover:text-ink-soft hover:bg-cream transition-colors"
        >
          <LayoutGrid className="w-3 h-3" />
          Layouts
          <ChevronDown className="w-2.5 h-2.5" />
        </button>

        {showSave && (
          <div className="absolute top-full left-0 mt-1 w-64 bg-paper border border-hair z-50 p-3">
            <div className="flex gap-1.5 mb-3">
              <Input
                type="text"
                value={saveName}
                onChange={(e) => setSaveName(e.target.value)}
                placeholder="Layout name…"
                className="!py-1.5 !px-2 text-[12px]"
              />
              <button
                onClick={() => {
                  if (saveName.trim()) {
                    saveCurrentLayout(saveName.trim());
                    setSaveName("");
                  }
                }}
                disabled={!saveName.trim()}
                className={clsx(
                  "px-2 rounded-sm border transition-colors",
                  saveName.trim()
                    ? "bg-terracotta border-terracotta text-paper hover:bg-terracotta-hover"
                    : "bg-stone border-hair text-muted cursor-not-allowed",
                )}
              >
                <Save className="w-3.5 h-3.5" />
              </button>
            </div>

            {savedLayouts.length > 0 ? (
              <div className="space-y-0.5">
                <Eyebrow size="sm" tone="muted" className="block mb-1.5">
                  Saved
                </Eyebrow>
                {savedLayouts.map((saved) => (
                  <div
                    key={saved.name}
                    className="flex items-center gap-1 px-1.5 py-1 hover:bg-cream group transition-colors"
                  >
                    <button
                      onClick={() => { loadSavedLayout(saved.name); setShowSave(false); }}
                      className="flex items-center gap-2 flex-1 text-[12px] text-ink-soft text-left"
                    >
                      <FolderOpen className="w-3 h-3 text-muted" />
                      {saved.name}
                    </button>
                    <button
                      onClick={() => deleteSavedLayout(saved.name)}
                      className="p-0.5 text-muted hover:text-err opacity-0 group-hover:opacity-100 transition-opacity"
                    >
                      <Trash2 className="w-3 h-3" />
                    </button>
                  </div>
                ))}
              </div>
            ) : (
              <p className="text-[11px] text-muted text-center py-2 font-mono uppercase tracking-[0.1em]">
                No saved layouts
              </p>
            )}
          </div>
        )}
      </div>
    </div>
  );
}
