import type { SkillDescription } from "../../types/ros";
import { CATEGORY_COLORS } from "./CategoryFilter";
import { Puzzle, Tag } from "lucide-react";
import clsx from "clsx";

interface Props {
  skill: SkillDescription;
  onClick: () => void;
}

export default function SkillCard({ skill, onClick }: Props) {
  return (
    <button
      onClick={onClick}
      className="w-full text-left p-3 rounded-lg border border-gray-800 hover:border-gray-600 bg-gray-900/50 hover:bg-gray-900 transition-all"
    >
      <div className="flex items-start gap-2 mb-1.5">
        <Puzzle className="w-4 h-4 text-gray-500 shrink-0 mt-0.5" />
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-2">
            <span className="text-sm font-semibold text-gray-100 truncate">
              {skill.display_name || skill.name}
            </span>
            {skill.is_compound && (
              <span className="text-[9px] px-1 py-0 rounded bg-green-900 text-green-300 uppercase">
                compound
              </span>
            )}
          </div>
          <span
            className={clsx(
              "inline-block mt-0.5 px-1.5 py-0 rounded text-[9px] font-medium uppercase border",
              CATEGORY_COLORS[skill.category] ?? CATEGORY_COLORS.utility
            )}
          >
            {skill.category}
          </span>
        </div>
      </div>

      <p className="text-xs text-gray-400 line-clamp-2 mb-1.5">
        {skill.description}
      </p>

      {skill.tags.length > 0 && (
        <div className="flex items-center gap-1 flex-wrap">
          <Tag className="w-3 h-3 text-gray-600" />
          {skill.tags.slice(0, 4).map((tag) => (
            <span
              key={tag}
              className="text-[9px] px-1 py-0 rounded bg-gray-800 text-gray-500"
            >
              {tag}
            </span>
          ))}
        </div>
      )}
    </button>
  );
}
