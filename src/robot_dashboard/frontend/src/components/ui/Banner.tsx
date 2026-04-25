import clsx from "clsx";
import { HTMLAttributes, ReactNode } from "react";

export type BannerTone = "ok" | "err" | "info" | "neutral";

interface BannerProps extends HTMLAttributes<HTMLDivElement> {
  tone?: BannerTone;
  title?: string;
  icon?: ReactNode;
}

const TONES: Record<BannerTone, string> = {
  ok: "bg-ok-soft border-ok",
  err: "bg-err-soft border-err",
  info: "bg-running-soft border-running",
  neutral: "bg-cream-deep border-hair",
};

const TITLE_TONE: Record<BannerTone, string> = {
  ok: "text-ok",
  err: "text-err",
  info: "text-running",
  neutral: "text-ink",
};

export function Banner({
  tone = "neutral",
  title,
  icon,
  className,
  children,
  ...rest
}: BannerProps) {
  return (
    <div
      className={clsx(
        "flex gap-3.5 border rounded-sm px-[18px] py-3.5",
        TONES[tone],
        className,
      )}
      {...rest}
    >
      {icon && <div className={clsx("flex-shrink-0 mt-0.5", TITLE_TONE[tone])}>{icon}</div>}
      <div className="flex-1 min-w-0">
        {title && (
          <div className={clsx("text-[14px] font-semibold mb-0.5", TITLE_TONE[tone])}>{title}</div>
        )}
        {children && <div className="text-[13.5px] text-ink-soft">{children}</div>}
      </div>
    </div>
  );
}
