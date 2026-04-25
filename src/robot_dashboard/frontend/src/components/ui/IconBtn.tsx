import clsx from "clsx";
import { ButtonHTMLAttributes, forwardRef } from "react";

interface IconBtnProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  active?: boolean;
}

export const IconBtn = forwardRef<HTMLButtonElement, IconBtnProps>(function IconBtn(
  { className, active = false, disabled, ...rest },
  ref,
) {
  return (
    <button
      ref={ref}
      disabled={disabled}
      className={clsx(
        "inline-flex items-center justify-center w-7 h-7 rounded-sm border transition-colors duration-150",
        "focus:outline-none focus-visible:ring-1 focus-visible:ring-terracotta",
        active
          ? "bg-terracotta-tint border-terracotta text-terracotta"
          : "bg-transparent border-transparent text-ink-soft hover:bg-cream hover:border-hair",
        disabled && "opacity-45 cursor-not-allowed",
        className,
      )}
      {...rest}
    />
  );
});
