import { useCallback } from "react";
import { toast } from "sonner";
import { useTaskStore } from "../../stores/task-store";
import { useActionClient } from "../../hooks/useActionClient";
import {
  OctagonX,
  RotateCcw,
  Home,
  Hand,
  AlertTriangle,
  Loader2,
  ShieldAlert,
} from "lucide-react";
import clsx from "clsx";
import { Button, Chip, Eyebrow, Banner } from "../ui";
import type { ChipState } from "../ui";

const STATUS_TO_CHIP: Record<string, ChipState> = {
  IDLE: "idle",
  RUNNING: "running",
  SUCCESS: "done",
  FAILURE: "failed",
  CANCELLED: "neutral",
};

export default function InterventionPanel() {
  const taskState = useTaskStore((s) => s.taskState);
  const isRunning = taskState?.status === "RUNNING";
  const isFailed = taskState?.status === "FAILURE";

  const { cancel } = useActionClient<any, any, any>(
    "/skill_server/execute_behavior_tree",
    "robot_skills_msgs/action/ExecuteBehaviorTree"
  );

  const { sendGoal: sendGripper, status: gripperStatus } = useActionClient<any, any, any>(
    "/skill_atoms/gripper_control",
    "robot_skills_msgs/action/GripperControl"
  );

  const { sendGoal: sendMoveHome, status: moveHomeStatus } = useActionClient<any, any, any>(
    "/skill_atoms/move_to_named_config",
    "robot_skills_msgs/action/MoveToNamedConfig"
  );

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
    <div className="flex flex-col h-full bg-paper">
      <div className="px-5 py-3 border-b border-hair flex items-center gap-2">
        <ShieldAlert className="w-4 h-4 text-terracotta" />
        <h2 className="text-[14px] font-medium text-ink">Intervention</h2>
      </div>

      <div className="flex-1 overflow-auto p-5 space-y-5">
        {/* Emergency Stop — semantic override: estop is always red-fill */}
        <button
          onClick={handleEStop}
          className="w-full py-4 bg-err hover:bg-err/90 active:opacity-80 text-paper font-mono font-semibold text-[16px] uppercase tracking-[0.16em] flex items-center justify-center gap-3 transition-colors border border-err"
        >
          <OctagonX className="w-6 h-6" />
          Emergency Stop
        </button>

        {/* Current status */}
        <div className="border border-hair p-4 bg-cream-deep">
          <div className="flex items-center gap-2 mb-2">
            <Eyebrow size="sm" tone="muted">Status</Eyebrow>
            <span className="ml-auto">
              <Chip state={STATUS_TO_CHIP[taskState?.status ?? "IDLE"] ?? "idle"} showDot>
                {taskState?.status ?? "IDLE"}
              </Chip>
            </span>
          </div>
          {taskState?.task_name && (
            <div className="text-[13px] text-ink-soft truncate mb-2">
              {taskState.task_name}
            </div>
          )}

          {isRunning && taskState && (
            <div className="space-y-1">
              <div className="flex justify-between text-[11px] text-muted font-mono tracking-[0.04em]">
                <span className="truncate">{taskState.current_skill || taskState.current_bt_node}</span>
                <span>{Math.round(taskState.progress * 100)}%</span>
              </div>
              <div className="h-1 bg-stone overflow-hidden">
                <div
                  className="h-full bg-terracotta transition-all duration-300"
                  style={{ width: `${taskState.progress * 100}%` }}
                />
              </div>
            </div>
          )}

          {isFailed && taskState?.error_message && (
            <Banner
              tone="err"
              title={taskState.error_skill ? `Failed at: ${taskState.error_skill}` : "Failure"}
              icon={<AlertTriangle className="w-4 h-4" />}
              className="mt-3"
            >
              <p>{taskState.error_message}</p>
              {taskState.failed_skills && taskState.failed_skills.length > 0 && (
                <p className="mt-1 text-[11px] font-mono tracking-[0.04em] opacity-80">
                  Failed nodes: {taskState.failed_skills.join(", ")}
                </p>
              )}
            </Banner>
          )}
        </div>

        {/* Task control */}
        {isRunning && (
          <div>
            <Eyebrow size="sm" className="block mb-2">Task Control</Eyebrow>
            <Button
              onClick={handleCancel}
              variant="secondary"
              size="sm"
              leftIcon={<Hand className="w-3.5 h-3.5" />}
              className="w-full"
            >
              Cancel Task
            </Button>
          </div>
        )}

        {/* Recovery actions */}
        <div>
          <Eyebrow size="sm" className="block mb-2">Recovery Actions</Eyebrow>
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
        "py-2 px-3 text-[12px] font-medium border rounded-sm flex items-center justify-center gap-1.5 transition-colors",
        disabled
          ? "border-hair bg-stone text-muted cursor-not-allowed"
          : "border-hair bg-paper text-ink-soft hover:bg-cream hover:border-terracotta",
        className,
      )}
    >
      {loading ? <Loader2 className="w-3.5 h-3.5 animate-spin" /> : icon}
      {label}
    </button>
  );
}
