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
        <div className="px-5 py-3 border-b border-hair">
          <div className="flex items-center gap-2">
            <Activity className="w-4 h-4 text-terracotta" />
            <h2 className="text-[14px] font-medium text-ink">Task Monitor</h2>
          </div>
        </div>

        <div className="px-5 py-3 border-b border-hair-soft">
          <div className="flex items-center gap-2 text-[13px]">
            <Chip state={connected ? "done" : "failed"} showDot>
              {connected ? "Connected" : "Offline"}
            </Chip>
            {connected && (
              <span className="text-muted">— Idle, no task running</span>
            )}
          </div>
        </div>

        {taskHistory.length > 0 ? (
          <div className="flex-1 overflow-auto px-5 py-3">
            <Eyebrow size="sm" tone="muted" className="block mb-2">
              Recent Tasks
            </Eyebrow>
            <div className="space-y-1">
              {taskHistory.map((t) => {
                const chipState =
                  t.status === "SUCCESS" ? "done" : t.status === "FAILURE" ? "failed" : "idle";
                return (
                  <div
                    key={t.id}
                    className="flex items-center gap-2 text-[13px] px-3 py-2 border-l-2 border-transparent hover:border-terracotta hover:bg-cream transition-colors"
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
          <div className="flex-1 flex flex-col items-center justify-center text-muted px-8 bg-cream-deep">
            <Rocket className="w-8 h-8 mb-3 opacity-30" />
            <p className="text-[14px] font-medium text-ink-soft">No tasks executed yet</p>
            <p className="text-[12px] text-center mt-1 text-muted">
              Open the <span className="text-terracotta font-semibold">Behavior Executor</span> panel
              to load a behavior tree and run it.
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
      <div className="px-5 py-3 border-b border-hair">
        <div className="flex items-center gap-2 mb-3">
          <Activity className="w-4 h-4 text-terracotta" />
          <h2 className="text-[14px] font-medium text-ink">Task Monitor</h2>
        </div>
        <TaskHeader
          taskId={ts.task_id}
          taskName={ts.task_name}
          status={ts.status}
        />
      </div>

      <div className="px-5 py-3 border-b border-hair-soft">
        <div className="flex items-center justify-between mb-1.5">
          <Eyebrow size="sm" tone="muted">Progress</Eyebrow>
          <span className="text-[12px] font-mono text-ink-soft tracking-[0.06em]">
            {progressPct}%
          </span>
        </div>
        <div className="h-1.5 bg-stone overflow-hidden">
          <div
            className={clsx("h-full transition-all duration-300", progressFill)}
            style={{ width: `${progressPct}%` }}
          />
        </div>
      </div>

      <div className="px-5 py-3 border-b border-hair-soft grid grid-cols-2 gap-4">
        <div>
          <div className="flex items-center gap-1.5 mb-1">
            <Clock className="w-3 h-3 text-muted" />
            <Eyebrow size="sm" tone="muted">Elapsed</Eyebrow>
          </div>
          <span className="text-[20px] font-mono text-ink tracking-[0.04em]">
            {formatTime(ts.elapsed_sec)}
          </span>
        </div>
        <div>
          <Eyebrow size="sm" tone="muted" className="block mb-1">Current Node</Eyebrow>
          <span className="text-[14px] font-mono text-running truncate block tracking-[0.04em]">
            {ts.current_bt_node || "—"}
          </span>
        </div>
      </div>

      {ts.error_message && (
        <div className="px-5 py-3 border-b border-hair-soft">
          <Banner tone="err" title={`Error in ${ts.error_skill}`}>
            {ts.error_message}
          </Banner>
        </div>
      )}

      <div className="flex-1 overflow-auto px-5 py-4">
        <SkillTimeline
          completedSkills={ts.completed_skills}
          failedSkills={ts.failed_skills}
          currentSkill={ts.current_skill}
        />
      </div>
    </div>
  );
}
