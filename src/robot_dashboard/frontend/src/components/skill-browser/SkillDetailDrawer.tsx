import type { SkillDescription } from "../../types/ros";
import { X, Server, FileCode, Shield, Zap } from "lucide-react";
import { Eyebrow, IconBtn } from "../ui";

interface Props {
  skill: SkillDescription;
  onClose: () => void;
}

export default function SkillDetailDrawer({ skill, onClose }: Props) {
  let paramsSchema: any = null;
  try {
    if (skill.parameters_schema) {
      paramsSchema = JSON.parse(skill.parameters_schema);
    }
  } catch {}

  return (
    <div className="fixed inset-y-0 right-0 w-[440px] bg-paper border-l-2 border-l-terracotta border border-hair z-50 overflow-auto">
      <div className="flex items-center justify-between px-5 py-3 border-b border-hair sticky top-0 bg-paper">
        <h3 className="text-[16px] font-medium text-ink truncate">
          {skill.display_name || skill.name}
        </h3>
        <IconBtn onClick={onClose} title="Close">
          <X className="w-4 h-4" />
        </IconBtn>
      </div>

      <div className="p-5 space-y-5">
        <p className="text-[14px] text-ink-soft leading-relaxed">
          {skill.description}
        </p>

        <Section icon={<Server className="w-3.5 h-3.5" />} title="Action Server">
          <code className="block font-mono text-[11px] text-terracotta break-all tracking-[0.04em]">
            {skill.action_server_name}
          </code>
          <div className="font-mono text-[10px] text-muted mt-1 tracking-[0.04em]">
            {skill.action_type}
          </div>
        </Section>

        {paramsSchema?.properties && (
          <Section icon={<FileCode className="w-3.5 h-3.5" />} title="Parameters">
            <table className="w-full border-collapse border border-hair text-[12px]">
              <thead>
                <tr>
                  <th className="text-left bg-cream font-mono text-[10px] uppercase tracking-[0.12em] text-terracotta font-semibold px-3 py-2 border-b border-hair">
                    Name
                  </th>
                  <th className="text-left bg-cream font-mono text-[10px] uppercase tracking-[0.12em] text-terracotta font-semibold px-3 py-2 border-b border-hair">
                    Type
                  </th>
                </tr>
              </thead>
              <tbody>
                {Object.entries(paramsSchema.properties).map(([key, val]: [string, any]) => (
                  <tr key={key} className="border-b border-hair-soft last:border-b-0">
                    <td className="px-3 py-2 align-top">
                      <div className="font-mono text-[11px] text-ink-soft tracking-[0.04em]">
                        {key}
                      </div>
                      {val.description && (
                        <div className="text-[11px] text-muted mt-0.5">{val.description}</div>
                      )}
                    </td>
                    <td className="px-3 py-2 font-mono text-[11px] text-muted tracking-[0.04em] align-top whitespace-nowrap">
                      {val.type || "any"}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </Section>
        )}

        {skill.preconditions.length > 0 && (
          <Section icon={<Shield className="w-3.5 h-3.5" />} title="Preconditions">
            <div className="flex flex-wrap gap-1.5">
              {skill.preconditions.map((p) => (
                <span
                  key={p}
                  className="font-mono text-[10px] px-2 py-0.5 border border-terracotta text-terracotta tracking-[0.06em]"
                >
                  {p}
                </span>
              ))}
            </div>
          </Section>
        )}

        {skill.postconditions.length > 0 && (
          <Section icon={<Zap className="w-3.5 h-3.5" />} title="Postconditions">
            <div className="flex flex-wrap gap-1.5">
              {skill.postconditions.map((p) => (
                <span
                  key={p}
                  className="font-mono text-[10px] px-2 py-0.5 border border-ok text-ok tracking-[0.06em]"
                >
                  {p}
                </span>
              ))}
            </div>
          </Section>
        )}

        {skill.pddl_action && (
          <Section icon={<FileCode className="w-3.5 h-3.5" />} title="PDDL Action">
            <pre className="sociius-code max-h-40 whitespace-pre-wrap">{skill.pddl_action}</pre>
          </Section>
        )}

        {skill.bt_xml && (
          <Section icon={<FileCode className="w-3.5 h-3.5" />} title="BT XML">
            <pre className="sociius-code max-h-60 whitespace-pre-wrap">{skill.bt_xml}</pre>
          </Section>
        )}

        {skill.component_skills.length > 0 && (
          <Section icon={<Zap className="w-3.5 h-3.5" />} title="Component Skills">
            <div className="flex flex-wrap gap-1.5">
              {skill.component_skills.map((s) => (
                <span
                  key={s}
                  className="font-mono text-[10px] px-2 py-0.5 bg-cream-deep border border-hair text-ink-soft tracking-[0.04em]"
                >
                  {s}
                </span>
              ))}
            </div>
          </Section>
        )}
      </div>
    </div>
  );
}

function Section({
  icon,
  title,
  children,
}: {
  icon: React.ReactNode;
  title: string;
  children: React.ReactNode;
}) {
  return (
    <div>
      <div className="flex items-center gap-1.5 mb-2 text-muted">
        {icon}
        <Eyebrow size="sm">{title}</Eyebrow>
      </div>
      {children}
    </div>
  );
}
