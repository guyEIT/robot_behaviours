import { useState } from "react";
import { Code, ChevronDown, ChevronRight } from "lucide-react";

interface Props {
  xml: string | null;
}

export default function BtXmlViewer({ xml }: Props) {
  const [open, setOpen] = useState(false);

  if (!xml) return null;

  return (
    <div className="border-t border-hair">
      <button
        onClick={() => setOpen(!open)}
        className="flex items-center gap-2 w-full px-4 py-2 text-[11px] font-mono uppercase tracking-[0.1em] font-semibold text-muted hover:text-terracotta hover:bg-cream transition-colors"
      >
        <Code className="w-3.5 h-3.5" />
        BT XML
        {open ? (
          <ChevronDown className="w-3.5 h-3.5 ml-auto" />
        ) : (
          <ChevronRight className="w-3.5 h-3.5 ml-auto" />
        )}
      </button>
      {open && (
        <pre className="sociius-code mx-4 mb-4 max-h-60 whitespace-pre-wrap">
          {xml}
        </pre>
      )}
    </div>
  );
}
