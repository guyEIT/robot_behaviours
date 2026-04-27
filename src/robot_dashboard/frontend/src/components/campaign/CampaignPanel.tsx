import { useMemo, useState } from "react";
import { toast } from "sonner";
import { Plus, Pause, Play, Trash2, FlaskConical, OctagonX } from "lucide-react";
import clsx from "clsx";
import { Button, Chip, Eyebrow, Banner } from "../ui";
import type { ChipState } from "../ui";
import { useCampaignState } from "../../hooks/useCampaignState";
import { useServiceCall } from "../../hooks/useServiceCall";
import {
  readPlateQueue,
  readPlatesIndex,
  readOperatorDecisions,
  readPauseReason,
  type PlateRecord,
} from "./types";
import AddPlateDialog from "./AddPlateDialog";

const STATUS_CHIP: Record<string, ChipState> = {
  IDLE: "idle",
  RUNNING: "running",
  AWAITING_APPROVAL: "neutral",
  PAUSED: "neutral",
  SUCCESS: "done",
  FAILURE: "failed",
  HALTED: "neutral",
};

interface PauseRequest {
  paused: boolean;
  reason: string;
}
interface PauseResponse {
  success: boolean;
  message: string;
}

interface RetireRequest {
  plate_name: string;
  reason: string;
}

function formatDueAt(epoch: number | undefined, now: number): string {
  if (!epoch) return "—";
  const delta = epoch - now;
  if (delta <= 0) return "due now";
  if (delta < 60) return `${Math.round(delta)}s`;
  if (delta < 3600) return `${Math.round(delta / 60)}m`;
  if (delta < 86400) return `${(delta / 3600).toFixed(1)}h`;
  return `${(delta / 86400).toFixed(1)}d`;
}

function plateChipState(plate: PlateRecord): ChipState {
  if (plate.retiring) return "neutral";
  const target = plate.target_cycles ?? 0;
  if (target > 0 && (plate.cycle ?? 0) >= target) return "done";
  return "running";
}

function plateChipLabel(plate: PlateRecord): string {
  if (plate.retiring) return "retiring";
  const target = plate.target_cycles ?? 0;
  const cycle = plate.cycle ?? 0;
  if (target > 0 && cycle >= target) return "done";
  return "queued";
}

export default function CampaignPanel() {
  const snap = useCampaignState();
  const [addDialogOpen, setAddDialogOpen] = useState(false);
  const now = Date.now() / 1000;

  // Generic pause/resume — same service the Behavior Executor uses, so the
  // semantics are identical: ctx.paused is honoured by control-flow nodes
  // at step boundaries (mid-action goals complete naturally).
  const { call: callPause, loading: pauseLoading } = useServiceCall<
    PauseRequest,
    PauseResponse
  >("/skill_server/pause_execution", "robot_skills_msgs/srv/PauseCampaign");

  const { call: callRetire } = useServiceCall<RetireRequest, PauseResponse>(
    "/bb_operator/retire_plate",
    "robot_skills_msgs/srv/RetirePlate"
  );

  // Session-independent cancel via /skill_server/cancel_active_task —
  // works regardless of which client started the tree (essential for a
  // campaign that outlives the operator's browser session).
  const { call: callCancel } = useServiceCall<
    { reason: string },
    { success: boolean; message: string; task_id: string }
  >(
    "/skill_server/cancel_active_task",
    "robot_skills_msgs/srv/CancelActiveTask"
  );

  const queue = snap ? readPlateQueue(snap) : [];
  const platesIndex = snap ? readPlatesIndex(snap) : {};
  const decisions = snap ? readOperatorDecisions(snap) : {};
  const pauseReason = snap ? readPauseReason(snap) : "";

  // Merge: plates known to the campaign are everything in the index plus
  // anything in the queue not yet indexed (operator-added but not yet
  // observed in the per-name dict for whatever reason).
  const allPlates: PlateRecord[] = useMemo(() => {
    const byName: Record<string, PlateRecord> = { ...platesIndex };
    for (const p of queue) {
      if (!byName[p.name]) byName[p.name] = p;
    }
    return Object.values(byName).sort((a, b) =>
      a.name.localeCompare(b.name)
    );
  }, [platesIndex, queue]);

  const activeTask = snap?.task_id || "";
  const taskStatus = snap?.task_status || "IDLE";
  const isActive =
    taskStatus === "RUNNING" ||
    taskStatus === "AWAITING_APPROVAL" ||
    taskStatus === "PAUSED";
  // Source of truth for paused is the task_state status published by
  // BtExecutor, not persistent.paused — the new pause_execution service
  // is in-memory only, matching the Behavior Executor panel's behaviour.
  const paused = taskStatus === "PAUSED";

  async function togglePause() {
    if (!isActive) {
      toast.warning("No active campaign to pause");
      return;
    }
    try {
      const res = await callPause({
        paused: !paused,
        reason: paused
          ? "operator resume from campaign panel"
          : "operator pause from campaign panel",
      });
      if (res.success) {
        toast.success(paused ? "Campaign resumed" : "Pausing after current step");
      } else {
        toast.error("Pause/Resume refused", { description: res.message });
      }
    } catch {
      // toasted by useServiceCall
    }
  }

  async function cancelCampaign() {
    if (!isActive) {
      toast.warning("No active campaign to cancel");
      return;
    }
    if (
      !window.confirm(
        "Cancel the active campaign? The current step will halt where it can; any non-idempotent action mid-flight may need operator review on resume."
      )
    ) {
      return;
    }
    try {
      const res = await callCancel({ reason: "operator cancel from dashboard" });
      if (res.success) {
        toast.warning(`Campaign ${res.task_id} cancelling`);
      } else {
        toast.error("Cancel refused", { description: res.message });
      }
    } catch {
      // toasted by useServiceCall
    }
  }

  async function retire(plate: PlateRecord) {
    if (!isActive) {
      toast.warning("No active campaign");
      return;
    }
    if (
      !window.confirm(
        `Retire plate ${plate.name}? In-flight cycle will finish naturally; the plate will not be re-queued.`
      )
    ) {
      return;
    }
    try {
      await callRetire({ plate_name: plate.name, reason: "retired from dashboard" });
      toast.success(`Plate ${plate.name} marked retiring`);
    } catch {
      // toasted
    }
  }

  return (
    <div className="h-full flex flex-col bg-cream">
      <header className="flex items-center gap-3 px-4 py-2 border-b border-hair bg-paper shrink-0">
        <FlaskConical className="w-3.5 h-3.5 text-muted" />
        <Eyebrow size="sm">Campaign</Eyebrow>
        <Chip state={STATUS_CHIP[taskStatus] ?? "idle"} showDot>
          {taskStatus}
        </Chip>
        {paused && (
          <Chip state="failed" showDot>
            PAUSED
          </Chip>
        )}
        <span className="text-[11px] font-mono text-muted ml-2">
          {activeTask || "no active task"}
        </span>
        <div className="ml-auto flex items-center gap-2">
          <Button
            size="sm"
            variant="secondary"
            onClick={togglePause}
            disabled={!isActive || pauseLoading}
            title={
              paused
                ? "Resume execution"
                : "Pause after the current step completes — same mechanism as the Behavior Executor panel"
            }
          >
            {paused ? (
              <>
                <Play className="w-3 h-3 mr-1" />
                Resume
              </>
            ) : (
              <>
                <Pause className="w-3 h-3 mr-1" />
                Pause after step
              </>
            )}
          </Button>
          <Button
            size="sm"
            variant="danger"
            onClick={cancelCampaign}
            disabled={!isActive}
            title="Cancel the active campaign tree (halts at the next safe point)"
          >
            <OctagonX className="w-3 h-3 mr-1" />
            Cancel
          </Button>
          <Button
            size="sm"
            onClick={() => setAddDialogOpen(true)}
            disabled={!isActive}
          >
            <Plus className="w-3 h-3 mr-1" />
            Add plate
          </Button>
        </div>
      </header>

      <div className="flex-1 overflow-auto p-3 space-y-3">
        {!isActive && (
          <Banner tone="info">
            No campaign tree currently running. Submit a long-lived BT (e.g.
            <span className="font-mono"> plate_imaging_campaign.xml</span>)
            via the <strong>Execute</strong> panel — operator services
            unlock once it's active.
          </Banner>
        )}

        {paused && pauseReason && (
          <Banner tone="err">
            Paused: <span className="font-mono">{pauseReason}</span>
          </Banner>
        )}

        {Object.entries(decisions).filter(
          ([, d]) => !d.decided_at
        ).length > 0 && (
          <Banner tone="err">
            {Object.keys(decisions).length} operator decision(s) awaiting —
            see Service caller for /bb_operator/operator_decision.
          </Banner>
        )}

        <section>
          <div className="flex items-baseline justify-between mb-1.5">
            <Eyebrow>Plates ({allPlates.length})</Eyebrow>
            <span className="text-[11px] font-mono text-muted">
              queue head:{" "}
              {queue[0]?.name ?? "—"}
            </span>
          </div>
          <div className="border border-hair rounded-md overflow-hidden bg-paper">
            <table className="w-full text-[12px] font-mono">
              <thead className="bg-cream text-muted uppercase tracking-[0.06em] text-[10px]">
                <tr>
                  <th className="text-left px-3 py-1.5 font-medium">Plate</th>
                  <th className="text-left px-3 py-1.5 font-medium">Cycle</th>
                  <th className="text-left px-3 py-1.5 font-medium">
                    Cadence
                  </th>
                  <th className="text-left px-3 py-1.5 font-medium">
                    Next due
                  </th>
                  <th className="text-left px-3 py-1.5 font-medium">Status</th>
                  <th className="text-left px-3 py-1.5 font-medium">
                    Protocol
                  </th>
                  <th className="px-3 py-1.5"></th>
                </tr>
              </thead>
              <tbody>
                {allPlates.length === 0 && (
                  <tr>
                    <td
                      colSpan={7}
                      className="px-3 py-6 text-center text-muted"
                    >
                      no plates yet — click <strong>Add plate</strong> to
                      trickle one in
                    </td>
                  </tr>
                )}
                {allPlates.map((p) => (
                  <tr
                    key={p.name}
                    className={clsx(
                      "border-t border-hair",
                      p.retiring && "opacity-60"
                    )}
                  >
                    <td className="px-3 py-1.5 text-ink">{p.name}</td>
                    <td className="px-3 py-1.5">
                      <span className="text-ink">{p.cycle ?? 0}</span>
                      <span className="text-muted">
                        {" / "}
                        {p.target_cycles ?? "∞"}
                      </span>
                    </td>
                    <td className="px-3 py-1.5 text-muted">
                      {p.cadence_min ? `${p.cadence_min}m` : "—"}
                    </td>
                    <td className="px-3 py-1.5 text-muted">
                      {formatDueAt(p.next_due_at, now)}
                    </td>
                    <td className="px-3 py-1.5">
                      <Chip state={plateChipState(p)} showDot>
                        {plateChipLabel(p)}
                      </Chip>
                    </td>
                    <td className="px-3 py-1.5 text-muted truncate max-w-[160px]">
                      {p.imaging_protocol || "—"}
                    </td>
                    <td className="px-3 py-1.5 text-right">
                      {!p.retiring && (
                        <button
                          onClick={() => retire(p)}
                          className="text-muted hover:text-terracotta inline-flex items-center gap-1"
                          title="Retire plate"
                        >
                          <Trash2 className="w-3.5 h-3.5" />
                        </button>
                      )}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </section>
      </div>

      <AddPlateDialog
        open={addDialogOpen}
        onClose={() => setAddDialogOpen(false)}
      />
    </div>
  );
}
