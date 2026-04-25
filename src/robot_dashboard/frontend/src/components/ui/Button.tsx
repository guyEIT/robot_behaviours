import clsx from "clsx";
import { ButtonHTMLAttributes, forwardRef, ReactNode } from "react";

export type ButtonVariant = "primary" | "secondary" | "ghost" | "link" | "danger";
export type ButtonSize = "sm" | "md";

interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: ButtonVariant;
  size?: ButtonSize;
  leftIcon?: ReactNode;
  rightIcon?: ReactNode;
}

const VARIANTS: Record<ButtonVariant, string> = {
  primary:
    "bg-terracotta text-paper border border-terracotta hover:bg-terracotta-hover hover:border-terracotta-hover",
  secondary:
    "bg-transparent text-ink border border-terracotta hover:bg-terracotta-tint",
  ghost:
    "bg-transparent text-ink-soft border border-hair hover:bg-cream",
  link:
    "bg-transparent text-terracotta border-0 border-b border-terracotta rounded-none px-0",
  danger:
    "bg-err text-paper border border-err hover:opacity-90",
};

const SIZES: Record<ButtonSize, string> = {
  sm: "px-3 py-1.5 text-[13px]",
  md: "px-[18px] py-[9px] text-label",
};

export const Button = forwardRef<HTMLButtonElement, ButtonProps>(function Button(
  { className, variant = "primary", size = "md", leftIcon, rightIcon, children, disabled, ...rest },
  ref,
) {
  return (
    <button
      ref={ref}
      disabled={disabled}
      className={clsx(
        "inline-flex items-center justify-center gap-2 font-medium rounded-sm transition-colors duration-150",
        "focus:outline-none focus-visible:ring-2 focus-visible:ring-terracotta focus-visible:ring-offset-1 focus-visible:ring-offset-paper",
        VARIANTS[variant],
        SIZES[size],
        disabled && "opacity-45 cursor-not-allowed pointer-events-none",
        className,
      )}
      {...rest}
    >
      {leftIcon}
      {children}
      {rightIcon}
    </button>
  );
});
