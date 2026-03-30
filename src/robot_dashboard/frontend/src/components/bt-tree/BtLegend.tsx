import clsx from "clsx";

const ITEMS = [
  { label: "Idle", color: "bg-gray-500" },
  { label: "Running", color: "bg-blue-400" },
  { label: "Success", color: "bg-green-400" },
  { label: "Failure", color: "bg-red-400" },
] as const;

export default function BtLegend() {
  return (
    <div className="flex items-center gap-3 px-3 py-1.5 bg-gray-900/80 rounded-lg text-[10px] text-gray-300 backdrop-blur-sm">
      {ITEMS.map((item) => (
        <div key={item.label} className="flex items-center gap-1">
          <span className={clsx("w-2 h-2 rounded-full", item.color)} />
          {item.label}
        </div>
      ))}
    </div>
  );
}
