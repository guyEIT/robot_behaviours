import clsx from "clsx";
import { InputHTMLAttributes, forwardRef } from "react";

interface InputProps extends InputHTMLAttributes<HTMLInputElement> {
  invalid?: boolean;
}

export const Input = forwardRef<HTMLInputElement, InputProps>(function Input(
  { className, invalid = false, disabled, ...rest },
  ref,
) {
  return (
    <input
      ref={ref}
      disabled={disabled}
      className={clsx(
        "w-full bg-paper border rounded-DEFAULT px-3.5 py-3 text-body text-ink-soft",
        "placeholder:text-muted-2",
        "focus:outline-none focus:border-terracotta",
        invalid ? "border-err" : "border-hair",
        disabled && "bg-stone border-dashed border-hair text-muted cursor-not-allowed",
        className,
      )}
      {...rest}
    />
  );
});
