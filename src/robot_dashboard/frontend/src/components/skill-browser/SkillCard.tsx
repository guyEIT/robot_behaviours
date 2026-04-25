import type { SkillDescription } from "../../types/ros";
import { Puzzle, Tag } from "lucide-react";
import { Eyebrow } from "../ui";

interface Props {
  skill: SkillDescription;
  onClick: () => void;
}

export default function SkillCard({ skill, onClick }: Props) {
  return (
    <button
      onClick={onClick}
      className="w-full text-left p-4 border border-hair border-l-2 border-l-transparent hover:border-l-terracotta hover:bg-cream bg-paper transition-colors"
    >
      <div className="flex items-start gap-2 mb-1.5">
        <Puzzle className="w-4 h-4 text-muted shrink-0 mt-0.5" />
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-2 flex-wrap">
            <span className="text-[14px] font-medium text-ink truncate">
              {skill.display_name || skill.name}
            </span>
            {skill.is_compound && (
              <span className="font-mono text-[9px] px-1.5 py-0.5 border border-ok text-ok uppercase tracking-[0.1em]">
                compound
              </span>
            )}
          </div>
          <Eyebrow size="sm" className="block mt-1">
            {skill.category}
          </Eyebrow>
        </div>
      </div>

      <p className="text-[12.5px] text-ink-soft line-clamp-2 mb-2 ml-6">
        {skill.description}
      </p>

      {skill.tags.length > 0 && (
        <div className="flex items-center gap-1 flex-wrap ml-6">
          <Tag className="w-3 h-3 text-muted" />
          {skill.tags.slice(0, 4).map((tag) => (
            <span
              key={tag}
              className="font-mono text-[10px] px-1.5 py-0.5 bg-cream-deep text-muted tracking-[0.04em]"
            >
              {tag}
            </span>
          ))}
        </div>
      )}
    </button>
  );
}
