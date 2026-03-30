import clsx from "clsx";

interface Props {
  taskId: string;
  taskName: string;
  status: string;
}

const STATUS_STYLES: Record<string, string> = {
  IDLE: "bg-gray-700 text-gray-300",
  RUNNING: "bg-blue-600 text-blue-100 animate-pulse",
  SUCCESS: "bg-green-700 text-green-100",
  FAILURE: "bg-red-700 text-red-100",
  CANCELLED: "bg-yellow-700 text-yellow-100",
};

export default function TaskHeader({ taskId, taskName, status }: Props) {
  return (
    <div className="flex items-center gap-3">
      <h3 className="text-sm font-semibold text-gray-100 truncate">
        {taskName || "No Task"}
      </h3>
      <span
        className={clsx(
          "px-2 py-0.5 rounded text-[10px] font-bold uppercase",
          STATUS_STYLES[status] ?? STATUS_STYLES.IDLE
        )}
      >
        {status}
      </span>
      {taskId && (
        <span className="text-[10px] text-gray-500 font-mono">{taskId}</span>
      )}
    </div>
  );
}
