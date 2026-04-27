import { useState } from "react";
import { ChevronRight, ChevronDown } from "lucide-react";
import clsx from "clsx";

interface Props {
  /** The dotted path / map key for this entry. */
  keyName: string;
  value: unknown;
  /** Indentation level for nested rendering. */
  depth: number;
  /** True when this row was just updated, drives a brief terracotta tint. */
  recentlyUpdated?: boolean;
}

/**
 * A single key/value row. Scalars render inline; lists and dicts collapse
 * to a triangle + count badge and expand on click. Type accents:
 *   string   → default ink
 *   number   → monospace, right-leaning
 *   boolean  → ✓ / ✗
 *   null     → muted "null"
 *   list     → "[N]" badge, expands to indexed rows
 *   dict     → "{N keys}" badge, expands to nested rows
 */
export function BbValueRow({ keyName, value, depth, recentlyUpdated }: Props) {
  const [open, setOpen] = useState(depth < 1);

  const indent = { paddingLeft: `${depth * 14 + 8}px` };
  const baseRow = clsx(
    "flex items-start gap-2 py-[3px] pr-2 text-[12px] border-b border-hair-soft/60 last:border-b-0",
    recentlyUpdated && "bg-terracotta-tint",
  );

  if (value === null || value === undefined) {
    return (
      <div className={baseRow} style={indent}>
        <span className="font-mono text-ink-soft truncate min-w-0">
          {shortKey(keyName)}
        </span>
        <span className="ml-auto font-mono text-muted">null</span>
      </div>
    );
  }
  if (typeof value === "boolean") {
    return (
      <div className={baseRow} style={indent}>
        <span className="font-mono text-ink-soft truncate min-w-0">
          {shortKey(keyName)}
        </span>
        <span
          className={clsx(
            "ml-auto font-mono",
            value ? "text-ok" : "text-err",
          )}
        >
          {value ? "✓ true" : "✗ false"}
        </span>
      </div>
    );
  }
  if (typeof value === "number") {
    return (
      <div className={baseRow} style={indent}>
        <span className="font-mono text-ink-soft truncate min-w-0">
          {shortKey(keyName)}
        </span>
        <span className="ml-auto font-mono text-ink">{String(value)}</span>
      </div>
    );
  }
  if (typeof value === "string") {
    const isRef = value.startsWith("{") && value.endsWith("}");
    return (
      <div className={baseRow} style={indent}>
        <span className="font-mono text-ink-soft truncate min-w-0">
          {shortKey(keyName)}
        </span>
        <span
          className={clsx(
            "ml-auto font-mono break-all min-w-0 text-right",
            isRef ? "text-terracotta" : "text-ink",
          )}
          title={value}
        >
          {value.length > 60 ? value.slice(0, 58) + "…" : value}
        </span>
      </div>
    );
  }
  if (Array.isArray(value)) {
    return (
      <>
        <button
          type="button"
          onClick={() => setOpen((o) => !o)}
          className={clsx(baseRow, "w-full text-left hover:bg-cream/50")}
          style={indent}
        >
          <span className="text-muted shrink-0">
            {open ? <ChevronDown className="w-3 h-3" /> : <ChevronRight className="w-3 h-3" />}
          </span>
          <span className="font-mono text-ink-soft truncate min-w-0">
            {shortKey(keyName)}
          </span>
          <span className="ml-auto font-mono text-[10px] px-1.5 py-[1px] border border-hair bg-cream text-ink-soft">
            [{value.length}]
          </span>
        </button>
        {open &&
          value.map((item, i) => (
            <BbValueRow
              key={i}
              keyName={`[${i}]`}
              value={item}
              depth={depth + 1}
            />
          ))}
      </>
    );
  }
  if (typeof value === "object") {
    const entries = Object.entries(value as Record<string, unknown>);
    return (
      <>
        <button
          type="button"
          onClick={() => setOpen((o) => !o)}
          className={clsx(baseRow, "w-full text-left hover:bg-cream/50")}
          style={indent}
        >
          <span className="text-muted shrink-0">
            {open ? <ChevronDown className="w-3 h-3" /> : <ChevronRight className="w-3 h-3" />}
          </span>
          <span className="font-mono text-ink-soft truncate min-w-0">
            {shortKey(keyName)}
          </span>
          <span className="ml-auto font-mono text-[10px] px-1.5 py-[1px] border border-hair bg-cream text-ink-soft">
            {`{${entries.length} keys}`}
          </span>
        </button>
        {open &&
          entries.map(([k, v]) => (
            <BbValueRow
              key={k}
              keyName={k}
              value={v}
              depth={depth + 1}
            />
          ))}
      </>
    );
  }
  return (
    <div className={baseRow} style={indent}>
      <span className="font-mono text-ink-soft">{shortKey(keyName)}</span>
      <span className="ml-auto font-mono text-muted">{String(value)}</span>
    </div>
  );
}

/**
 * Strip the leading "persistent." prefix from top-level keys so the eye
 * doesn't have to keep skipping over it. Nested keys (already short) and
 * list indices pass through.
 */
function shortKey(k: string): string {
  if (k.startsWith("persistent.")) return k.slice("persistent.".length);
  return k;
}
