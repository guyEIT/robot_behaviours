import clsx from "clsx";
import { HTMLAttributes } from "react";

interface EyebrowProps extends HTMLAttributes<HTMLSpanElement> {
  size?: "sm" | "md";
  tone?: "terracotta" | "muted" | "ink";
}

export function Eyebrow({
  className,
  size = "md",
  tone = "terracotta",
  ...rest
}: EyebrowProps) {
  return (
    <span
      className={clsx(
        "font-mono font-semibold uppercase",
        size === "sm" ? "text-[10px] tracking-[0.1em]" : "text-[11px] tracking-[0.12em]",
        tone === "terracotta" && "text-terracotta",
        tone === "muted" && "text-muted",
        tone === "ink" && "text-ink",
        className,
      )}
      {...rest}
    />
  );
}
