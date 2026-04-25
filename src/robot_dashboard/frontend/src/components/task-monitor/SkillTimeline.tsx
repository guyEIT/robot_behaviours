import { StepDot } from "../ui";
import { Eyebrow } from "../ui";

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

  let stepCount = 0;
  return (
    <div>
      <Eyebrow size="sm" tone="muted" className="block mb-3">
        Skill Timeline
      </Eyebrow>

      <div className="space-y-2">
        {allSkills.map((skill, i) => {
          stepCount++;
          return (
            <div key={`${skill.name}-${i}`} className="flex items-center gap-3">
              <StepDot
                size="sm"
                state={skill.status === "completed" ? "done" : "failed"}
              >
                {stepCount}
              </StepDot>
              <span
                className={
                  skill.status === "completed"
                    ? "text-[13px] text-ink-soft"
                    : "text-[13px] text-err"
                }
              >
                {skill.name}
              </span>
            </div>
          );
        })}

        {currentSkill && (
          <div className="flex items-center gap-3">
            <StepDot size="sm" state="running">
              {stepCount + 1}
            </StepDot>
            <span className="text-[13px] text-running">{currentSkill}</span>
            <span className="text-[10px] text-muted font-mono uppercase tracking-[0.1em]">
              running
            </span>
          </div>
        )}
      </div>

      {allSkills.length === 0 && !currentSkill && (
        <p className="text-[12px] text-muted italic">No skills executed yet</p>
      )}
    </div>
  );
}
