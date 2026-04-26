import { useTaskStore } from "../../stores/task-store";
import { useConnectionStore } from "../../stores/connection-store";
import TaskHeader from "./TaskHeader";
import SkillTimeline from "./SkillTimeline";
import { Activity, Clock, Rocket, CheckCircle, XCircle } from "lucide-react";
import clsx from "clsx";
import { Eyebrow, Chip, Banner } from "../ui";

function formatTime(sec: number): string {
  const m = Math.floor(sec / 60);
  const s = Math.floor(sec % 60);
  return `${m}:${s.toString().padStart(2, "0")}`;
}

export default function TaskMonitorPanel() {
  const ts = useTaskStore((s) => s.taskState);
  const taskHistory = useTaskStore((s) => s.taskHistory);
  const connected = useConnectionStore((s) => s.connected);

  if (!ts) {
    return (
      <div className="flex flex-col h-full bg-paper">
        {/* Compact combined header: title + status chip on one row */}
        <div className="px-4 py-2 border-b border-hair flex items-center gap-2">
          <Activity className="w-4 h-4 text-terracotta" />
          <h2 className="text-[13px] font-medium text-ink">Task Monitor</h2>
          <span className="ml-auto">
            <Chip state={connected ? "done" : "failed"} showDot>
              {connected ? "Connected" : "Offline"}
            </Chip>
          </span>
        </div>

        {taskHistory.length > 0 ? (
          <div className="flex-1 overflow-auto px-4 py-2">
            <Eyebrow size="sm" tone="muted" className="block mb-1.5">
              Recent Tasks
            </Eyebrow>
            <div>
              {taskHistory.map((t) => {
                const chipState =
                  t.status === "SUCCESS" ? "done" : t.status === "FAILURE" ? "failed" : "idle";
                return (
                  <div
                    key={t.id}
                    className="flex items-center gap-2 text-[12.5px] px-2 py-1 border-l-2 border-transparent hover:border-terracotta hover:bg-cream transition-colors"
                  >
                    {t.status === "SUCCESS" ? (
                      <CheckCircle className="w-3.5 h-3.5 text-ok shrink-0" />
                    ) : t.status === "FAILURE" ? (
                      <XCircle className="w-3.5 h-3.5 text-err shrink-0" />
                    ) : (
                      <Activity className="w-3.5 h-3.5 text-muted shrink-0" />
                    )}
                    <span className="text-ink-soft truncate">{t.name || "Unnamed"}</span>
                    <span className="ml-auto shrink-0">
                      <Chip state={chipState}>{t.status}</Chip>
                    </span>
                    <span className="text-[10px] text-muted font-mono shrink-0 tracking-[0.06em]">
                      {t.id}
                    </span>
                  </div>
                );
              })}
            </div>
          </div>
        ) : (
          <div className="flex-1 flex flex-col items-center justify-center text-muted px-6 py-4 bg-cream-deep">
            <Rocket className="w-6 h-6 mb-2 opacity-30" />
            <p className="text-[13px] font-medium text-ink-soft">Idle — no tasks yet</p>
            <p className="text-[11.5px] text-center mt-0.5 text-muted">
              Run a tree from <span className="text-terracotta font-medium">Behavior Executor</span>
            </p>
          </div>
        )}
      </div>
    );
  }

  const progressPct = Math.round(ts.progress * 100);
  const progressFill =
    ts.status === "FAILURE"
      ? "bg-err"
      : ts.status === "SUCCESS"
        ? "bg-ok"
        : "bg-terracotta";

  return (
    <div className="flex flex-col h-full bg-paper">
      {/* Header — title + task chip + id all on one row */}
      <div className="px-4 py-2 border-b border-hair flex items-center gap-2 flex-wrap">
        <Activity className="w-4 h-4 text-terracotta shrink-0" />
        <h2 className="text-[13px] font-medium text-ink shrink-0">Task Monitor</h2>
        <span className="text-muted shrink-0">·</span>
        <TaskHeader
          taskId={ts.task_id}
          taskName={ts.task_name}
          status={ts.status}
        />
      </div>

      {/* Stats + progress on a single compact strip */}
      <div className="px-4 py-2 border-b border-hair-soft">
        <div className="flex items-center gap-3 mb-1.5">
          <div className="flex items-center gap-1.5 shrink-0">
            <Clock className="w-3 h-3 text-muted" />
            <span className="text-[14px] font-mono text-ink tracking-[0.04em]">
              {formatTime(ts.elapsed_sec)}
            </span>
          </div>
          <span className="text-muted shrink-0 text-[12px]">·</span>
          <div className="flex items-baseline gap-1.5 min-w-0 flex-1">
            <Eyebrow size="sm" tone="muted" className="shrink-0">Node</Eyebrow>
            <span className="text-[12px] font-mono text-running truncate tracking-[0.04em]">
              {ts.current_bt_node || "—"}
            </span>
          </div>
          <span className="text-[11px] font-mono text-ink-soft tracking-[0.06em] shrink-0">
            {progressPct}%
          </span>
        </div>
        <div className="h-1 bg-stone overflow-hidden">
          <div
            className={clsx("h-full transition-all duration-300", progressFill)}
            style={{ width: `${progressPct}%` }}
          />
        </div>
      </div>

      {ts.error_message && (
        <div className="px-4 py-2 border-b border-hair-soft">
          <Banner tone="err" title={`Error in ${ts.error_skill}`}>
            {ts.error_message}
          </Banner>
        </div>
      )}

      <div className="flex-1 overflow-auto px-4 py-2">
        <SkillTimeline
          completedSkills={ts.completed_skills}
          failedSkills={ts.failed_skills}
          currentSkill={ts.current_skill}
        />
      </div>
    </div>
  );
}
