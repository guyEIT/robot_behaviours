import clsx from "clsx";

const CATEGORIES = ["motion", "perception", "manipulation", "utility", "compound"];

interface Props {
  selected: Set<string>;
  onChange: (selected: Set<string>) => void;
}

export default function CategoryFilter({ selected, onChange }: Props) {
  const toggle = (cat: string) => {
    const next = new Set(selected);
    if (next.has(cat)) {
      next.delete(cat);
    } else {
      next.add(cat);
    }
    onChange(next);
  };

  return (
    <div className="flex flex-wrap gap-1.5">
      {CATEGORIES.map((cat) => (
        <button
          key={cat}
          onClick={() => toggle(cat)}
          className={clsx(
            "inline-flex items-center px-3 py-1 border font-mono text-[10px] uppercase tracking-[0.08em] font-semibold transition-colors",
            selected.has(cat)
              ? "border-terracotta bg-terracotta text-paper"
              : "border-terracotta text-terracotta bg-paper hover:bg-terracotta-tint",
          )}
        >
          {cat}
        </button>
      ))}
    </div>
  );
}
