import { useState } from "react";
import { toast } from "sonner";
import { X } from "lucide-react";
import { Button, Input, Eyebrow } from "../ui";
import { useServiceCall } from "../../hooks/useServiceCall";

interface Props {
  open: boolean;
  onClose: () => void;
}

interface AddPlateRequest {
  plate_name: string;
  barcode: string;
  target_cycles: number;
  cadence_min: number;
  imaging_protocol: string;
  liconic_slot: number;
}

interface AddPlateResponse {
  success: boolean;
  message: string;
  task_id: string;
}

const DEFAULT_PROTOCOL = "brightfield_3site_v1";

export default function AddPlateDialog({ open, onClose }: Props) {
  const [plateName, setPlateName] = useState("");
  const [barcode, setBarcode] = useState("");
  const [targetCycles, setTargetCycles] = useState(10);
  const [cadenceMin, setCadenceMin] = useState(60);
  const [protocol, setProtocol] = useState(DEFAULT_PROTOCOL);
  const [slot, setSlot] = useState(0);

  const { call, loading } = useServiceCall<AddPlateRequest, AddPlateResponse>(
    "/bb_operator/add_plate",
    "robot_skills_msgs/srv/AddPlate"
  );

  if (!open) return null;

  async function submit(e: React.FormEvent) {
    e.preventDefault();
    if (!plateName.trim()) {
      toast.error("Plate name is required");
      return;
    }
    try {
      const res = await call({
        plate_name: plateName.trim(),
        barcode: barcode.trim(),
        target_cycles: targetCycles,
        cadence_min: cadenceMin,
        imaging_protocol: protocol.trim() || DEFAULT_PROTOCOL,
        liconic_slot: slot,
      });
      if (res.success) {
        toast.success(`Queued plate ${plateName.trim()}`);
        // Reset for next add
        setPlateName("");
        setBarcode("");
        onClose();
      } else {
        toast.error("AddPlate refused", { description: res.message });
      }
    } catch {
      // useServiceCall already toasted the rosbridge error.
    }
  }

  return (
    <div
      className="fixed inset-0 z-40 flex items-center justify-center bg-black/30"
      onClick={onClose}
    >
      <form
        onSubmit={submit}
        onClick={(e) => e.stopPropagation()}
        className="bg-paper border border-hair rounded-md shadow-lg w-[420px] max-w-[92vw]"
      >
        <header className="flex items-center justify-between px-4 py-3 border-b border-hair">
          <h2 className="text-[14px] font-medium text-ink">Add plate to campaign</h2>
          <button
            type="button"
            onClick={onClose}
            className="text-muted hover:text-ink"
            aria-label="Close"
          >
            <X className="w-4 h-4" />
          </button>
        </header>
        <div className="p-4 space-y-3">
          <div>
            <Eyebrow size="sm">Plate name *</Eyebrow>
            <Input
              autoFocus
              value={plateName}
              onChange={(e) => setPlateName(e.target.value)}
              placeholder="e.g. P_2026_04_27_a"
            />
          </div>
          <div className="grid grid-cols-2 gap-3">
            <div>
              <Eyebrow size="sm">Barcode</Eyebrow>
              <Input
                value={barcode}
                onChange={(e) => setBarcode(e.target.value)}
                placeholder="optional"
              />
            </div>
            <div>
              <Eyebrow size="sm">Liconic slot</Eyebrow>
              <Input
                type="number"
                min={0}
                value={slot}
                onChange={(e) => setSlot(Number(e.target.value) || 0)}
              />
            </div>
          </div>
          <div className="grid grid-cols-2 gap-3">
            <div>
              <Eyebrow size="sm">Target cycles</Eyebrow>
              <Input
                type="number"
                min={1}
                value={targetCycles}
                onChange={(e) => setTargetCycles(Number(e.target.value) || 1)}
              />
            </div>
            <div>
              <Eyebrow size="sm">Cadence (min)</Eyebrow>
              <Input
                type="number"
                min={0}
                value={cadenceMin}
                onChange={(e) => setCadenceMin(Number(e.target.value) || 0)}
              />
            </div>
          </div>
          <div>
            <Eyebrow size="sm">Imaging protocol</Eyebrow>
            <Input
              value={protocol}
              onChange={(e) => setProtocol(e.target.value)}
              placeholder={DEFAULT_PROTOCOL}
            />
          </div>
        </div>
        <footer className="px-4 py-3 border-t border-hair flex justify-end gap-2">
          <Button type="button" variant="secondary" onClick={onClose}>
            Cancel
          </Button>
          <Button type="submit" disabled={loading || !plateName.trim()}>
            {loading ? "Queueing…" : "Add plate"}
          </Button>
        </footer>
      </form>
    </div>
  );
}
