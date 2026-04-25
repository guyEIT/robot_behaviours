import clsx from "clsx";
import { HTMLAttributes, forwardRef } from "react";

interface CardProps extends HTMLAttributes<HTMLDivElement> {
  padded?: boolean;
  flush?: boolean;
}

export const Card = forwardRef<HTMLDivElement, CardProps>(function Card(
  { className, padded = false, flush = false, ...rest },
  ref,
) {
  return (
    <div
      ref={ref}
      className={clsx(
        "bg-paper border border-hair rounded-none",
        padded && "p-6",
        flush && "p-0",
        className,
      )}
      {...rest}
    />
  );
});
