import clsx from "clsx";

const ITEMS = [
  { label: "Idle", color: "bg-muted" },
  { label: "Running", color: "bg-running" },
  { label: "Success", color: "bg-ok" },
  { label: "Failure", color: "bg-err" },
] as const;

export default function BtLegend() {
  return (
    <div className="flex items-center gap-3 px-3 py-1.5 bg-paper border border-hair font-mono text-[10px] uppercase tracking-[0.08em] text-ink-soft">
      {ITEMS.map((item) => (
        <div key={item.label} className="flex items-center gap-1.5">
          <span className={clsx("w-1.5 h-1.5 rounded-full", item.color)} />
          {item.label}
        </div>
      ))}
    </div>
  );
}
