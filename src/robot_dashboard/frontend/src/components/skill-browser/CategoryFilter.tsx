import clsx from "clsx";

const CATEGORIES = ["motion", "perception", "manipulation", "utility", "compound"];

const CATEGORY_COLORS: Record<string, string> = {
  motion: "bg-blue-500/20 text-blue-300 border-blue-500/40",
  perception: "bg-purple-500/20 text-purple-300 border-purple-500/40",
  manipulation: "bg-orange-500/20 text-orange-300 border-orange-500/40",
  utility: "bg-gray-500/20 text-gray-300 border-gray-500/40",
  compound: "bg-green-500/20 text-green-300 border-green-500/40",
};

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
            "px-2 py-0.5 rounded text-[10px] font-medium uppercase border transition-all",
            selected.has(cat)
              ? CATEGORY_COLORS[cat]
              : "bg-gray-800 text-gray-500 border-gray-700 hover:border-gray-600"
          )}
        >
          {cat}
        </button>
      ))}
    </div>
  );
}

export { CATEGORY_COLORS };
