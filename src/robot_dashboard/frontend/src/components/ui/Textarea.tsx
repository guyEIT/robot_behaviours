import clsx from "clsx";
import { TextareaHTMLAttributes, forwardRef } from "react";

interface TextareaProps extends TextareaHTMLAttributes<HTMLTextAreaElement> {
  invalid?: boolean;
  mono?: boolean;
}

export const Textarea = forwardRef<HTMLTextAreaElement, TextareaProps>(function Textarea(
  { className, invalid = false, mono = false, disabled, ...rest },
  ref,
) {
  return (
    <textarea
      ref={ref}
      disabled={disabled}
      className={clsx(
        "w-full bg-paper border rounded-DEFAULT px-3.5 py-3 text-body text-ink-soft resize-none",
        "placeholder:text-muted-2",
        "focus:outline-none focus:border-terracotta",
        invalid ? "border-err" : "border-hair",
        disabled && "bg-stone border-dashed border-hair text-muted cursor-not-allowed",
        mono && "font-mono text-[12.5px] leading-[1.7]",
        className,
      )}
      {...rest}
    />
  );
});
