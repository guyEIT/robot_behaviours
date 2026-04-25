import clsx from "clsx";
import { ReactNode } from "react";

export interface TabDef<T extends string> {
  id: T;
  label: ReactNode;
  badge?: ReactNode;
  disabled?: boolean;
}

interface TabBarProps<T extends string> {
  tabs: TabDef<T>[];
  active: T;
  onSelect: (id: T) => void;
  className?: string;
  size?: "sm" | "md";
}

export function TabBar<T extends string>({
  tabs,
  active,
  onSelect,
  className,
  size = "md",
}: TabBarProps<T>) {
  return (
    <div className={clsx("flex bg-cream-deep border-b border-hair", className)}>
      {tabs.map((t) => {
        const isActive = t.id === active;
        return (
          <button
            key={t.id}
            disabled={t.disabled}
            onClick={() => !t.disabled && onSelect(t.id)}
            className={clsx(
              "inline-flex items-center gap-2 font-medium font-mono uppercase tracking-[0.08em]",
              size === "sm" ? "px-3 py-2 text-[10px]" : "px-4 py-2.5 text-[11px]",
              "transition-colors duration-150 border-b-2",
              isActive
                ? "border-terracotta text-terracotta"
                : "border-transparent text-muted hover:text-ink-soft",
              t.disabled && "opacity-45 cursor-not-allowed",
            )}
          >
            {t.label}
            {t.badge}
          </button>
        );
      })}
    </div>
  );
}
