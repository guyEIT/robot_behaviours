import { useCallback } from "react";
import { toast } from "sonner";
import { useTaskStore } from "../../stores/task-store";
import { useServiceCall } from "../../hooks/useServiceCall";
import { useActionClient } from "../../hooks/useActionClient";
import {
  OctagonX,
  RotateCcw,
  Home,
  Hand,
  AlertTriangle,
  CheckCircle,
  XCircle,
  Loader2,
  ShieldAlert,
} from "lucide-react";
import clsx from "clsx";

/**
 * Human intervention panel — always-visible controls for:
 *   - Emergency stop
 *   - Cancel current task
 *   - Quick recovery actions (open gripper, move home)
 *   - Task status summary with error details
 */
export default function InterventionPanel() {
  const taskState = useTaskStore((s) => s.taskState);
  const isRunning = taskState?.status === "RUNNING";
  const isFailed = taskState?.status === "FAILURE";
  const isIdle = !taskState || taskState.status === "IDLE" || taskState.status === "SUCCESS";

  // Cancel current task via action
  const { cancel } = useActionClient<any, any, any>(
    "/skill_server/execute_behavior_tree",
    "robot_skills_msgs/action/ExecuteBehaviorTree"
  );

  // Quick recovery: open gripper
  const { sendGoal: sendGripper, status: gripperStatus } = useActionClient<any, any, any>(
    "/skill_atoms/gripper_control",
    "robot_skills_msgs/action/GripperControl"
  );

  // Quick recovery: move home
  const { sendGoal: sendMoveHome, status: moveHomeStatus } = useActionClient<any, any, any>(
    "/skill_atoms/move_to_named_config",
    "robot_skills_msgs/action/MoveToNamedConfig"
  );

  // Robot enable/disable
  const { sendGoal: sendRobotEnable, status: enableStatus } = useActionClient<any, any, any>(
    "/skill_atoms/robot_enable",
    "robot_skills_msgs/action/RobotEnable"
  );

  const handleEStop = useCallback(() => {
    cancel();
    sendRobotEnable({ enable: false }).catch(() => {});
    toast.warning("Emergency stop triggered", { duration: 5000 });
  }, [cancel, sendRobotEnable]);

  const handleCancel = useCallback(() => {
    cancel();
    toast.info("Task cancel requested");
  }, [cancel]);

  const handleOpenGripper = useCallback(async () => {
    try {
      await sendGripper({ command: "open" });
      toast.success("Gripper opened");
    } catch (e: any) {
      toast.error("Failed to open gripper", { description: e.message });
    }
  }, [sendGripper]);

  const handleMoveHome = useCallback(async () => {
    try {
      await sendMoveHome({
        config_name: "home",
        velocity_scaling: 0.3,
        acceleration_scaling: 0.3,
      });
      toast.success("Moved to home");
    } catch (e: any) {
      toast.error("Failed to move home", { description: e.message });
    }
  }, [sendMoveHome]);

  const handleReEnable = useCallback(async () => {
    try {
      await sendRobotEnable({ enable: true });
      toast.success("Robot re-enabled");
    } catch (e: any) {
      toast.error("Failed to enable robot", { description: e.message });
    }
  }, [sendRobotEnable]);

  const anyRecoveryActive =
    gripperStatus === "active" || gripperStatus === "pending" ||
    moveHomeStatus === "active" || moveHomeStatus === "pending" ||
    enableStatus === "active" || enableStatus === "pending";

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-4 py-2 border-b border-gray-800 flex items-center gap-2">
        <ShieldAlert className="w-4 h-4 text-orange-400" />
        <h2 className="text-sm font-semibold">Intervention</h2>
      </div>

      <div className="flex-1 overflow-auto p-3 space-y-4">
        {/* ── Emergency Stop ─────────────────────────────────── */}
        <button
          onClick={handleEStop}
          className="w-full py-4 rounded-xl bg-red-700 hover:bg-red-600 active:bg-red-800 text-white font-bold text-lg flex items-center justify-center gap-3 transition-all shadow-lg shadow-red-900/30 border-2 border-red-500/50"
        >
          <OctagonX className="w-6 h-6" />
          EMERGENCY STOP
        </button>

        {/* ── Current Status ─────────────────────────────────── */}
        <div className={clsx(
          "p-3 rounded-lg border",
          isFailed ? "bg-red-950/30 border-red-800/50" :
          isRunning ? "bg-blue-950/30 border-blue-800/50" :
          "bg-gray-900/50 border-gray-800"
        )}>
          <div className="flex items-center gap-2 mb-2">
            {isRunning && <Loader2 className="w-4 h-4 text-blue-400 animate-spin" />}
            {isFailed && <XCircle className="w-4 h-4 text-red-400" />}
            {taskState?.status === "SUCCESS" && <CheckCircle className="w-4 h-4 text-green-400" />}
            {isIdle && !taskState?.status && <div className="w-4 h-4 rounded-full bg-gray-700" />}
            <span className={clsx(
              "text-xs font-bold uppercase",
              isFailed ? "text-red-400" :
              isRunning ? "text-blue-400" :
              taskState?.status === "SUCCESS" ? "text-green-400" :
              "text-gray-500"
            )}>
              {taskState?.status || "IDLE"}
            </span>
            {taskState?.task_name && (
              <span className="text-xs text-gray-400 truncate ml-auto">
                {taskState.task_name}
              </span>
            )}
          </div>

          {/* Progress */}
          {isRunning && taskState && (
            <div className="space-y-1">
              <div className="flex justify-between text-[10px] text-gray-400">
                <span>{taskState.current_skill || taskState.current_bt_node}</span>
                <span>{Math.round(taskState.progress * 100)}%</span>
              </div>
              <div className="h-1.5 bg-gray-800 rounded-full overflow-hidden">
                <div
                  className="h-full bg-blue-500 rounded-full transition-all duration-300"
                  style={{ width: `${taskState.progress * 100}%` }}
                />
              </div>
            </div>
          )}

          {/* Error details */}
          {isFailed && taskState && (
            <div className="mt-2 space-y-1">
              {taskState.error_skill && (
                <div className="flex items-center gap-1.5 text-xs text-red-400">
                  <AlertTriangle className="w-3 h-3 shrink-0" />
                  <span className="font-medium">Failed at: {taskState.error_skill}</span>
                </div>
              )}
              {taskState.error_message && (
                <p className="text-[11px] text-red-300/80 pl-[18px]">{taskState.error_message}</p>
              )}
              {taskState.failed_skills && taskState.failed_skills.length > 0 && (
                <div className="text-[10px] text-gray-500 pl-[18px]">
                  Failed nodes: {taskState.failed_skills.join(", ")}
                </div>
              )}
            </div>
          )}
        </div>

        {/* ── Task Control ───────────────────────────────────── */}
        {isRunning && (
          <div>
            <div className="text-[10px] text-gray-500 font-medium uppercase mb-2">
              Task Control
            </div>
            <button
              onClick={handleCancel}
              className="w-full py-2 rounded-lg bg-yellow-700/80 hover:bg-yellow-600/80 text-yellow-100 text-xs font-semibold flex items-center justify-center gap-2 transition-all"
            >
              <Hand className="w-3.5 h-3.5" />
              Cancel Task
            </button>
          </div>
        )}

        {/* ── Recovery Actions ────────────────────────────────── */}
        <div>
          <div className="text-[10px] text-gray-500 font-medium uppercase mb-2">
            Recovery Actions
          </div>
          <div className="grid grid-cols-2 gap-2">
            <RecoveryButton
              onClick={handleOpenGripper}
              disabled={anyRecoveryActive}
              loading={gripperStatus === "active" || gripperStatus === "pending"}
              label="Open Gripper"
              icon={<Hand className="w-3.5 h-3.5" />}
            />
            <RecoveryButton
              onClick={handleMoveHome}
              disabled={anyRecoveryActive}
              loading={moveHomeStatus === "active" || moveHomeStatus === "pending"}
              label="Move Home"
              icon={<Home className="w-3.5 h-3.5" />}
            />
            <RecoveryButton
              onClick={handleReEnable}
              disabled={anyRecoveryActive}
              loading={enableStatus === "active" || enableStatus === "pending"}
              label="Re-Enable"
              icon={<RotateCcw className="w-3.5 h-3.5" />}
              className="col-span-2"
            />
          </div>
        </div>
      </div>
    </div>
  );
}

function RecoveryButton({
  onClick,
  disabled,
  loading,
  label,
  icon,
  className,
}: {
  onClick: () => void;
  disabled: boolean;
  loading: boolean;
  label: string;
  icon: React.ReactNode;
  className?: string;
}) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={clsx(
        "py-2 px-3 rounded-lg text-xs font-medium flex items-center justify-center gap-1.5 transition-all",
        disabled
          ? "bg-gray-800 text-gray-600 cursor-not-allowed"
          : "bg-gray-700 hover:bg-gray-600 text-gray-200",
        className
      )}
    >
      {loading ? <Loader2 className="w-3.5 h-3.5 animate-spin" /> : icon}
      {label}
    </button>
  );
}
