import clsx from "clsx";
import { HTMLAttributes, ReactNode } from "react";

export type ChipState = "idle" | "running" | "done" | "failed" | "neutral";

interface ChipProps extends HTMLAttributes<HTMLSpanElement> {
  state?: ChipState;
  showDot?: boolean;
  children: ReactNode;
}

const STATES: Record<ChipState, string> = {
  idle: "border-hair text-muted bg-paper",
  neutral: "border-hair text-ink-soft bg-paper",
  running: "border-running text-running bg-running-soft",
  done: "border-ok text-ok bg-ok-soft",
  failed: "border-err text-err bg-err-soft",
};

const DOT_BG: Record<ChipState, string> = {
  idle: "bg-muted",
  neutral: "bg-ink-soft",
  running: "bg-running animate-pulse",
  done: "bg-ok",
  failed: "bg-err",
};

export function Chip({ state = "idle", showDot = false, className, children, ...rest }: ChipProps) {
  return (
    <span
      className={clsx(
        "inline-flex items-center gap-1.5 px-3 py-[5px] rounded-pill border font-mono",
        "text-[11px] uppercase tracking-[0.08em] font-semibold",
        STATES[state],
        className,
      )}
      {...rest}
    >
      {showDot && <span className={clsx("w-1.5 h-1.5 rounded-full", DOT_BG[state])} />}
      {children}
    </span>
  );
}
