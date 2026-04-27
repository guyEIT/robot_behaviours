import type { BtLeafCategory } from "../types/bt";

/**
 * Pure mapping from BT XML tag name to a leaf sub-category for action /
 * condition nodes. Used to drive icon and tint in BtNode.
 *
 * The mapping is deliberately permissive: unknown tags get "default" so
 * unfamiliar provider atoms still render legibly with the Cog icon.
 */
const HUMAN = new Set([
  "HumanConfirm",
  "HumanInput",
  "HumanWarning",
  "HumanNotification",
  "HumanTask",
]);

const TIMING = new Set(["WaitUntil", "WaitForDuration"]);

const QUEUE = new Set(["PopFromQueue", "PushToQueue"]);

const POSE_TF = new Set([
  "SetPose",
  "ComputePreGraspPose",
  "TransformPose",
  "LookupTransform",
  "GetCurrentPose",
  "PublishStaticTF",
]);

const IO = new Set(["SetDigitalIO", "EmergencyStop"]);

const LEASE = new Set(["AcquireLease", "ReleaseLease"]);

const LOG = new Set(["LogEvent"]);

const BLACKBOARD = new Set(["BlackboardCondition", "AdvancePlate"]);

const CONDITIONS = new Set(["ScriptCondition", "Condition"]);

export function leafCategoryFor(btNodeType: string): BtLeafCategory {
  if (HUMAN.has(btNodeType)) return "human";
  if (TIMING.has(btNodeType)) return "timing";
  if (QUEUE.has(btNodeType)) return "queue";
  if (POSE_TF.has(btNodeType)) return "pose-tf";
  if (IO.has(btNodeType)) return "io";
  if (LEASE.has(btNodeType)) return "lease";
  if (LOG.has(btNodeType)) return "log";
  if (BLACKBOARD.has(btNodeType)) return "blackboard";
  if (CONDITIONS.has(btNodeType) || btNodeType.includes("Condition"))
    return "condition";
  // Most provider atoms (MoveTo*, LiconicFetch, ImagePlate, etc.) fall here.
  if (btNodeType.length > 0) return "skill";
  return "default";
}
