import { CheckCircle, XCircle } from "lucide-react";

interface Props {
  completedSkills: string[];
  failedSkills: string[];
  currentSkill: string;
}

export default function SkillTimeline({
  completedSkills,
  failedSkills,
  currentSkill,
}: Props) {
  const allSkills = [
    ...completedSkills.map((s) => ({ name: s, status: "completed" as const })),
    ...failedSkills.map((s) => ({ name: s, status: "failed" as const })),
  ];

  return (
    <div className="space-y-1">
      <h4 className="text-xs font-medium text-gray-400 uppercase tracking-wide mb-2">
        Skill Timeline
      </h4>

      {currentSkill && (
        <div className="flex items-center gap-2 text-xs">
          <div className="w-4 h-4 rounded-full border-2 border-blue-500 flex items-center justify-center">
            <div className="w-1.5 h-1.5 rounded-full bg-blue-500 animate-pulse" />
          </div>
          <span className="text-blue-300">{currentSkill}</span>
          <span className="text-[10px] text-gray-500">running</span>
        </div>
      )}

      {allSkills.map((skill, i) => (
        <div key={`${skill.name}-${i}`} className="flex items-center gap-2 text-xs">
          {skill.status === "completed" ? (
            <CheckCircle className="w-4 h-4 text-green-500 shrink-0" />
          ) : (
            <XCircle className="w-4 h-4 text-red-500 shrink-0" />
          )}
          <span
            className={
              skill.status === "completed" ? "text-gray-300" : "text-red-300"
            }
          >
            {skill.name}
          </span>
        </div>
      ))}

      {allSkills.length === 0 && !currentSkill && (
        <p className="text-xs text-gray-600 italic">No skills executed yet</p>
      )}
    </div>
  );
}
