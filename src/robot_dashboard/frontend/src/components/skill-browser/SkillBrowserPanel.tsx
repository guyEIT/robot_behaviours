import { useState, useMemo } from "react";
import { useSkillStore } from "../../stores/skill-store";
import { useRobotSelectorStore } from "../../stores/robot-selector-store";
import CategoryFilter from "./CategoryFilter";
import SkillCard from "./SkillCard";
import SkillDetailDrawer from "./SkillDetailDrawer";
import { Puzzle, Search, Loader2 } from "lucide-react";
import type { SkillDescription } from "../../types/ros";

export default function SkillBrowserPanel() {
  const { skills, loading, error } = useSkillStore();
  const selectedRobotId = useRobotSelectorStore((s) => s.selectedRobotId);
  const [selectedCategories, setSelectedCategories] = useState<Set<string>>(new Set());
  const [search, setSearch] = useState("");
  const [selectedSkill, setSelectedSkill] = useState<SkillDescription | null>(null);

  const filtered = useMemo(() => {
    return skills.filter((s) => {
      // Filter by active robot when one is selected
      if (selectedRobotId && s.robot_id && s.robot_id !== selectedRobotId)
        return false;
      if (selectedCategories.size > 0 && !selectedCategories.has(s.category))
        return false;
      if (search) {
        const q = search.toLowerCase();
        return (
          s.name.toLowerCase().includes(q) ||
          s.display_name.toLowerCase().includes(q) ||
          s.description.toLowerCase().includes(q) ||
          s.tags.some((t) => t.toLowerCase().includes(q))
        );
      }
      return true;
    });
  }, [skills, selectedRobotId, selectedCategories, search]);

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-4 py-3 border-b border-gray-800 space-y-2">
        <div className="flex items-center gap-2">
          <Puzzle className="w-4 h-4 text-blue-400" />
          <h2 className="text-sm font-semibold">Skill Registry</h2>
          <span className="text-xs text-gray-500">{skills.length} skills</span>
          {loading && <Loader2 className="w-3.5 h-3.5 text-blue-400 animate-spin ml-auto" />}
        </div>

        {/* Search */}
        <div className="relative">
          <Search className="absolute left-2 top-1/2 -translate-y-1/2 w-3.5 h-3.5 text-gray-500" />
          <input
            type="text"
            placeholder="Search skills..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="w-full pl-7 pr-3 py-1.5 text-xs bg-gray-800 border border-gray-700 rounded focus:border-blue-500 focus:outline-none text-gray-200 placeholder-gray-500"
          />
        </div>

        {/* Category filter */}
        <CategoryFilter
          selected={selectedCategories}
          onChange={setSelectedCategories}
        />
      </div>

      {/* Error */}
      {error && (
        <div className="px-4 py-2 text-xs text-red-400 bg-red-950/30 border-b border-gray-800">
          {error}
        </div>
      )}

      {/* Skill list */}
      <div className="flex-1 overflow-auto p-3 space-y-2">
        {filtered.length === 0 && !loading && (
          <p className="text-xs text-gray-500 text-center py-8">
            {skills.length === 0
              ? "No skills registered. Start the skill server."
              : "No skills match your filters."}
          </p>
        )}
        {filtered.map((skill) => (
          <SkillCard
            key={skill.name}
            skill={skill}
            onClick={() => setSelectedSkill(skill)}
          />
        ))}
      </div>

      {/* Detail drawer */}
      {selectedSkill && (
        <SkillDetailDrawer
          skill={selectedSkill}
          onClose={() => setSelectedSkill(null)}
        />
      )}
    </div>
  );
}
