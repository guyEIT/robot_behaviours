import { useState } from "react";
import { Code, ChevronDown, ChevronRight } from "lucide-react";

interface Props {
  xml: string | null;
}

export default function BtXmlViewer({ xml }: Props) {
  const [open, setOpen] = useState(false);

  if (!xml) return null;

  return (
    <div className="border-t border-gray-800">
      <button
        onClick={() => setOpen(!open)}
        className="flex items-center gap-2 w-full px-3 py-2 text-xs text-gray-400 hover:text-gray-200 hover:bg-gray-800/50 transition-colors"
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
        <pre className="px-3 pb-3 text-[10px] leading-relaxed text-gray-400 overflow-auto max-h-60 whitespace-pre-wrap">
          {xml}
        </pre>
      )}
    </div>
  );
}
