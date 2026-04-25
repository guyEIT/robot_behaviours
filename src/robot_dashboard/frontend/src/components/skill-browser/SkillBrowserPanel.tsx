import { useState, useMemo } from "react";
import { useSkillStore } from "../../stores/skill-store";
import { useRobotSelectorStore } from "../../stores/robot-selector-store";
import CategoryFilter from "./CategoryFilter";
import SkillCard from "./SkillCard";
import SkillDetailDrawer from "./SkillDetailDrawer";
import { Puzzle, Search, Loader2 } from "lucide-react";
import type { SkillDescription } from "../../types/ros";
import { Eyebrow, Banner } from "../ui";

export default function SkillBrowserPanel() {
  const { skills, loading, error } = useSkillStore();
  const selectedRobotId = useRobotSelectorStore((s) => s.selectedRobotId);
  const [selectedCategories, setSelectedCategories] = useState<Set<string>>(new Set());
  const [search, setSearch] = useState("");
  const [selectedSkill, setSelectedSkill] = useState<SkillDescription | null>(null);

  const filtered = useMemo(() => {
    return skills.filter((s) => {
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
    <div className="flex flex-col h-full bg-paper">
      <div className="px-5 py-3 border-b border-hair space-y-3">
        <div className="flex items-center gap-2">
          <Puzzle className="w-4 h-4 text-terracotta" />
          <h2 className="text-[14px] font-medium text-ink">Skill Registry</h2>
          <Eyebrow size="sm" tone="muted" className="ml-1">
            {skills.length} skills
          </Eyebrow>
          {loading && <Loader2 className="w-3.5 h-3.5 text-running animate-spin ml-auto" />}
        </div>

        <div className="relative">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-3.5 h-3.5 text-muted pointer-events-none" />
          <input
            type="text"
            placeholder="Search skills…"
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="w-full pl-9 pr-3 py-2 text-[13px] bg-paper border border-hair rounded-DEFAULT focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
          />
        </div>

        <CategoryFilter
          selected={selectedCategories}
          onChange={setSelectedCategories}
        />
      </div>

      {error && (
        <div className="px-5 py-3 border-b border-hair-soft">
          <Banner tone="err">{error}</Banner>
        </div>
      )}

      <div className="flex-1 overflow-auto p-4 space-y-2 bg-cream-deep">
        {filtered.length === 0 && !loading && (
          <p className="text-[12px] text-muted text-center py-8">
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

      {selectedSkill && (
        <SkillDetailDrawer
          skill={selectedSkill}
          onClose={() => setSelectedSkill(null)}
        />
      )}
    </div>
  );
}
