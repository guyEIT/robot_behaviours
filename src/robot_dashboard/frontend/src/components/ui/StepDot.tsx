import clsx from "clsx";
import { HTMLAttributes, ReactNode } from "react";

export type StepDotState = "idle" | "running" | "done" | "failed";

interface StepDotProps extends HTMLAttributes<HTMLSpanElement> {
  state?: StepDotState;
  size?: "sm" | "md";
  children?: ReactNode;
}

const STATES: Record<StepDotState, string> = {
  idle: "border-hair text-muted bg-paper",
  running: "border-running border-[1.5px] text-running bg-running-soft",
  done: "border-ok text-ok bg-ok-soft",
  failed: "border-err text-err bg-err-soft",
};

export function StepDot({
  state = "idle",
  size = "md",
  className,
  children,
  ...rest
}: StepDotProps) {
  return (
    <span
      className={clsx(
        "inline-flex items-center justify-center rounded-full border font-mono font-semibold",
        size === "sm" ? "w-5 h-5 text-[10px]" : "w-7 h-7 text-[11px]",
        STATES[state],
        state === "running" && "animate-pulse",
        className,
      )}
      {...rest}
    >
      {children}
    </span>
  );
}
