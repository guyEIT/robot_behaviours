import { useBlackboardSnapshot } from "./useBlackboardSnapshot";
import type { CampaignSnapshot } from "../components/campaign/types";

/**
 * Thin alias around useBlackboardSnapshot — the campaign UI reads
 * specific persistent.* keys via the helpers in
 * components/campaign/types.ts, but the underlying snapshot is the
 * same shared latched topic. Centralizing the subscription here means
 * we don't double-decode when both panels are mounted.
 */
export function useCampaignState(): CampaignSnapshot | null {
  return useBlackboardSnapshot() as CampaignSnapshot | null;
}
