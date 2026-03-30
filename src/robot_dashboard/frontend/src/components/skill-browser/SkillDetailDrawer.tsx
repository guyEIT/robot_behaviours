import type { SkillDescription } from "../../types/ros";
import { X, Server, FileCode, Shield, Zap } from "lucide-react";

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
    <div className="fixed inset-y-0 right-0 w-[420px] bg-gray-900 border-l border-gray-700 shadow-2xl z-50 overflow-auto">
      {/* Header */}
      <div className="flex items-center justify-between px-4 py-3 border-b border-gray-800 sticky top-0 bg-gray-900">
        <h3 className="text-sm font-bold text-gray-100">
          {skill.display_name || skill.name}
        </h3>
        <button
          onClick={onClose}
          className="p-1 rounded hover:bg-gray-800 text-gray-400 hover:text-gray-200"
        >
          <X className="w-4 h-4" />
        </button>
      </div>

      <div className="p-4 space-y-4">
        {/* Description */}
        <p className="text-xs text-gray-300 leading-relaxed">
          {skill.description}
        </p>

        {/* Action server */}
        <Section icon={<Server className="w-3.5 h-3.5" />} title="Action Server">
          <code className="text-[10px] text-blue-300 break-all">
            {skill.action_server_name}
          </code>
          <div className="text-[10px] text-gray-500 mt-0.5">
            {skill.action_type}
          </div>
        </Section>

        {/* Parameters */}
        {paramsSchema?.properties && (
          <Section icon={<FileCode className="w-3.5 h-3.5" />} title="Parameters">
            <div className="space-y-1.5">
              {Object.entries(paramsSchema.properties).map(([key, val]: [string, any]) => (
                <div key={key} className="text-[10px]">
                  <span className="font-mono text-blue-300">{key}</span>
                  <span className="text-gray-500 ml-1">
                    ({val.type || "any"})
                  </span>
                  {val.description && (
                    <span className="text-gray-400 ml-1">
                      — {val.description}
                    </span>
                  )}
                </div>
              ))}
            </div>
          </Section>
        )}

        {/* Preconditions / Postconditions */}
        {skill.preconditions.length > 0 && (
          <Section icon={<Shield className="w-3.5 h-3.5" />} title="Preconditions">
            <div className="flex flex-wrap gap-1">
              {skill.preconditions.map((p) => (
                <span key={p} className="text-[10px] px-1.5 py-0.5 rounded bg-yellow-900/30 text-yellow-300 border border-yellow-800/40">
                  {p}
                </span>
              ))}
            </div>
          </Section>
        )}

        {skill.postconditions.length > 0 && (
          <Section icon={<Zap className="w-3.5 h-3.5" />} title="Postconditions">
            <div className="flex flex-wrap gap-1">
              {skill.postconditions.map((p) => (
                <span key={p} className="text-[10px] px-1.5 py-0.5 rounded bg-green-900/30 text-green-300 border border-green-800/40">
                  {p}
                </span>
              ))}
            </div>
          </Section>
        )}

        {/* PDDL */}
        {skill.pddl_action && (
          <Section icon={<FileCode className="w-3.5 h-3.5" />} title="PDDL Action">
            <pre className="text-[10px] text-gray-300 bg-gray-950 rounded p-2 overflow-auto max-h-40 whitespace-pre-wrap">
              {skill.pddl_action}
            </pre>
          </Section>
        )}

        {/* BT XML for compound skills */}
        {skill.bt_xml && (
          <Section icon={<FileCode className="w-3.5 h-3.5" />} title="BT XML">
            <pre className="text-[10px] text-gray-300 bg-gray-950 rounded p-2 overflow-auto max-h-60 whitespace-pre-wrap">
              {skill.bt_xml}
            </pre>
          </Section>
        )}

        {/* Component skills */}
        {skill.component_skills.length > 0 && (
          <Section icon={<Zap className="w-3.5 h-3.5" />} title="Component Skills">
            <div className="flex flex-wrap gap-1">
              {skill.component_skills.map((s) => (
                <span key={s} className="text-[10px] px-1.5 py-0.5 rounded bg-gray-800 text-gray-300">
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
      <div className="flex items-center gap-1.5 text-[10px] font-medium uppercase text-gray-500 mb-1.5">
        {icon}
        {title}
      </div>
      {children}
    </div>
  );
}
