import { Chip } from "../ui";
import type { ChipState } from "../ui";

interface Props {
  taskId: string;
  taskName: string;
  status: string;
}

const STATUS_TO_CHIP: Record<string, ChipState> = {
  IDLE: "idle",
  RUNNING: "running",
  SUCCESS: "done",
  FAILURE: "failed",
  CANCELLED: "neutral",
};

export default function TaskHeader({ taskId, taskName, status }: Props) {
  const chipState = STATUS_TO_CHIP[status] ?? "idle";
  return (
    <div className="flex items-center gap-3">
      <h3 className="text-[15px] font-medium text-ink truncate">
        {taskName || "No Task"}
      </h3>
      <Chip state={chipState} showDot>
        {status}
      </Chip>
      {taskId && (
        <span className="text-[10px] text-muted font-mono tracking-[0.06em]">{taskId}</span>
      )}
    </div>
  );
}
