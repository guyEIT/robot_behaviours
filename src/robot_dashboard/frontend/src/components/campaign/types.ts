// Shape of /skill_server/persistent_state JSON snapshots published by
// bb_operator. The `persistent` map keys are exactly the persistent.* keys
// from the active task's SQLite blackboard; values are arbitrary JSON.

export interface PlateRecord {
  name: string;
  barcode?: string;
  cycle?: number;
  target_cycles?: number;
  cadence_min?: number;
  imaging_protocol?: string;
  liconic_slot?: number;
  next_due_at?: number;       // unix epoch seconds
  added_at?: number;
  last_completed_at?: number;
  retiring?: boolean;
  retire_reason?: string;
}

export interface CampaignSnapshot {
  task_id: string;
  task_status: string;       // "RUNNING" | "AWAITING_APPROVAL" | "SUCCESS" | "FAILURE" | "HALTED" | "IDLE"
  paused: boolean;
  snapshot_at: number;
  persistent: Record<string, unknown>;
}

export interface OperatorDecisionEntry {
  choice: string;            // retry | skip-as-success | skip-as-failure | abort-tree
  reason?: string;
  decided_at?: number;
}

// Helpers to safely pull the typed slices out of the raw `persistent` map.

export function readPlateQueue(snap: CampaignSnapshot): PlateRecord[] {
  const v = snap.persistent["persistent.plate_queue"];
  return Array.isArray(v) ? (v as PlateRecord[]) : [];
}

export function readPlatesIndex(snap: CampaignSnapshot): Record<string, PlateRecord> {
  const v = snap.persistent["persistent.plates"];
  return v && typeof v === "object" && !Array.isArray(v)
    ? (v as Record<string, PlateRecord>)
    : {};
}

export function readOperatorDecisions(
  snap: CampaignSnapshot
): Record<string, OperatorDecisionEntry> {
  const v = snap.persistent["persistent.operator_decisions"];
  return v && typeof v === "object" && !Array.isArray(v)
    ? (v as Record<string, OperatorDecisionEntry>)
    : {};
}

export function readPauseReason(snap: CampaignSnapshot): string {
  const v = snap.persistent["persistent.pause_reason"];
  return typeof v === "string" ? v : "";
}
