import { useEffect, useRef, useState } from "react";
import { BbValueRow } from "./BbValueRow";

interface Props {
  persistent: Record<string, unknown>;
  filter: string;
}

/** Compute a cheap hash so we can detect "this row's value changed since last snapshot." */
function valueHash(v: unknown): string {
  try {
    return JSON.stringify(v);
  } catch {
    return String(v);
  }
}

/**
 * Render the persistent map as a list of named groups, each rendered as
 * a (possibly nested) BbValueRow. Tracks per-key value hashes across
 * snapshots and tints rows whose value just changed for ~1.5s.
 */
export function BbKeyTree({ persistent, filter }: Props) {
  const lastHashes = useRef<Map<string, string>>(new Map());
  const [recentlyUpdated, setRecentlyUpdated] = useState<Set<string>>(
    new Set(),
  );

  useEffect(() => {
    const next: string[] = [];
    const newHashes = new Map<string, string>();
    for (const [k, v] of Object.entries(persistent)) {
      const h = valueHash(v);
      newHashes.set(k, h);
      const prev = lastHashes.current.get(k);
      if (prev !== undefined && prev !== h) {
        next.push(k);
      }
    }
    lastHashes.current = newHashes;
    if (next.length > 0) {
      setRecentlyUpdated(new Set(next));
      const t = setTimeout(() => setRecentlyUpdated(new Set()), 1500);
      return () => clearTimeout(t);
    }
  }, [persistent]);

  const entries = Object.entries(persistent).filter(([k]) =>
    filter ? k.toLowerCase().includes(filter.toLowerCase()) : true,
  );

  if (entries.length === 0) {
    return (
      <div className="px-3 py-6 text-center text-[12px] text-muted">
        {filter
          ? "No keys match the filter."
          : "No persistent keys yet. Once a task starts populating persistent.* values, they will appear here."}
      </div>
    );
  }

  return (
    <div className="flex flex-col">
      {entries.map(([k, v]) => (
        <BbValueRow
          key={k}
          keyName={k}
          value={v}
          depth={0}
          recentlyUpdated={recentlyUpdated.has(k)}
        />
      ))}
    </div>
  );
}
