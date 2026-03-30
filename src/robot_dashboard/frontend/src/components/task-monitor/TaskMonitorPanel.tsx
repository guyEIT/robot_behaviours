import { useTaskStore } from "../../stores/task-store";
import { useConnectionStore } from "../../stores/connection-store";
import TaskHeader from "./TaskHeader";
import SkillTimeline from "./SkillTimeline";
import { Activity, Clock, AlertTriangle, Rocket, Wifi, WifiOff, CheckCircle, XCircle } from "lucide-react";
import clsx from "clsx";

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
      <div className="flex flex-col h-full">
        {/* Header */}
        <div className="px-4 py-3 border-b border-gray-800">
          <div className="flex items-center gap-2">
            <Activity className="w-4 h-4 text-blue-400" />
            <h2 className="text-sm font-semibold">Task Monitor</h2>
          </div>
        </div>

        {/* Status */}
        <div className="px-4 py-3 border-b border-gray-800">
          <div className="flex items-center gap-2 text-xs">
            {connected ? (
              <>
                <Wifi className="w-3.5 h-3.5 text-green-400" />
                <span className="text-green-400">Connected</span>
                <span className="text-gray-500">— Idle, no task running</span>
              </>
            ) : (
              <>
                <WifiOff className="w-3.5 h-3.5 text-red-400" />
                <span className="text-red-400">Disconnected</span>
              </>
            )}
          </div>
        </div>

        {/* Task history */}
        {taskHistory.length > 0 ? (
          <div className="flex-1 overflow-auto px-4 py-3">
            <div className="text-[10px] text-gray-500 font-medium uppercase tracking-wide mb-2">
              Recent Tasks
            </div>
            <div className="space-y-1.5">
              {taskHistory.map((t) => (
                <div key={t.id} className="flex items-center gap-2 text-xs p-2 rounded border border-gray-800 bg-gray-900/50">
                  {t.status === "SUCCESS" ? (
                    <CheckCircle className="w-3.5 h-3.5 text-green-500 shrink-0" />
                  ) : t.status === "FAILURE" ? (
                    <XCircle className="w-3.5 h-3.5 text-red-500 shrink-0" />
                  ) : (
                    <Activity className="w-3.5 h-3.5 text-gray-500 shrink-0" />
                  )}
                  <span className="text-gray-200 truncate">{t.name || "Unnamed"}</span>
                  <span className={clsx(
                    "ml-auto text-[10px] font-bold uppercase shrink-0",
                    t.status === "SUCCESS" ? "text-green-400" :
                    t.status === "FAILURE" ? "text-red-400" : "text-gray-500"
                  )}>
                    {t.status}
                  </span>
                  <span className="text-[10px] text-gray-600 font-mono shrink-0">{t.id}</span>
                </div>
              ))}
            </div>
          </div>
        ) : (
          <div className="flex-1 flex flex-col items-center justify-center text-gray-500 px-8">
            <Rocket className="w-8 h-8 mb-3 opacity-20" />
            <p className="text-sm font-medium text-gray-400">No tasks executed yet</p>
            <p className="text-xs text-center mt-1">
              Go to the <span className="text-blue-400 font-medium">Execute</span> tab to load a behavior tree and run it.
              Task progress will appear here in real time.
            </p>
          </div>
        )}
      </div>
    );
  }

  const progressPct = Math.round(ts.progress * 100);

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-4 py-3 border-b border-gray-800">
        <div className="flex items-center gap-2 mb-3">
          <Activity className="w-4 h-4 text-blue-400" />
          <h2 className="text-sm font-semibold">Task Monitor</h2>
        </div>
        <TaskHeader
          taskId={ts.task_id}
          taskName={ts.task_name}
          status={ts.status}
        />
      </div>

      {/* Progress */}
      <div className="px-4 py-3 border-b border-gray-800">
        <div className="flex items-center justify-between mb-1">
          <span className="text-xs text-gray-400">Progress</span>
          <span className="text-xs font-mono text-gray-300">
            {progressPct}%
          </span>
        </div>
        <div className="h-2 bg-gray-800 rounded-full overflow-hidden">
          <div
            className={clsx(
              "h-full rounded-full transition-all duration-300",
              ts.status === "FAILURE" ? "bg-red-500" :
              ts.status === "SUCCESS" ? "bg-green-500" : "bg-blue-500"
            )}
            style={{ width: `${progressPct}%` }}
          />
        </div>
      </div>

      {/* Stats */}
      <div className="px-4 py-3 border-b border-gray-800 grid grid-cols-2 gap-3">
        <div>
          <div className="flex items-center gap-1.5 text-xs text-gray-400 mb-0.5">
            <Clock className="w-3 h-3" />
            Elapsed
          </div>
          <span className="text-lg font-mono text-gray-100">
            {formatTime(ts.elapsed_sec)}
          </span>
        </div>
        <div>
          <div className="text-xs text-gray-400 mb-0.5">Current Node</div>
          <span className="text-sm font-mono text-blue-300 truncate block">
            {ts.current_bt_node || "-"}
          </span>
        </div>
      </div>

      {/* Error */}
      {ts.error_message && (
        <div className="px-4 py-2 border-b border-gray-800 bg-red-950/30">
          <div className="flex items-center gap-1.5 text-xs text-red-400 mb-0.5">
            <AlertTriangle className="w-3 h-3" />
            Error in {ts.error_skill}
          </div>
          <p className="text-xs text-red-300">{ts.error_message}</p>
        </div>
      )}

      {/* Timeline */}
      <div className="flex-1 overflow-auto px-4 py-3">
        <SkillTimeline
          completedSkills={ts.completed_skills}
          failedSkills={ts.failed_skills}
          currentSkill={ts.current_skill}
        />
      </div>
    </div>
  );
}
