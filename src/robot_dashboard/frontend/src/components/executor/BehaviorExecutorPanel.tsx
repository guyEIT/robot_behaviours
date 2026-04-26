import { useState, useMemo, useCallback, useEffect, useRef } from "react";
import { useSkillStore } from "../../stores/skill-store";
import { useServiceCall } from "../../hooks/useServiceCall";
import { useActionClient } from "../../hooks/useActionClient";
import { useAvailableTrees } from "../../hooks/useAvailableTrees";
import type {
  ComposeTaskRequest,
  ComposeTaskResponse,
  TaskStep,
  SkillDescription,
} from "../../types/ros";
import {
  Play,
  Square,
  Plus,
  Trash2,
  ChevronDown,
  ChevronUp,
  FileCode,
  Loader2,
  AlertTriangle,
  Rocket,
  Search,
  PanelRightOpen,
  PanelRightClose,
  GripVertical,
  Pencil,
  Check,
} from "lucide-react";
import clsx from "clsx";
import {
  Button,
  Chip,
  Eyebrow,
  Input,
  Textarea,
  Banner,
  TabBar,
  IconBtn,
} from "../ui";
import type { ChipState } from "../ui";

// Predefined trees with embedded XML from robot_behaviors/trees/
const PRESET_TREES = [
  {
    name: "MoveToSeedReady",
    label: "Move to Seed Ready",
    description: "Open gripper, move to seed-ready position (recorded 2026-04-16)",
    xml: `<root BTCPP_format="4" main_tree_to_execute="MoveToSeedReady">
  <BehaviorTree ID="MoveToSeedReady">
    <Sequence>
      <GripperControl name="open_gripper" command="open"/>
      <MoveToJointConfig name="move_to_seed_ready"
                         joint_positions="0.9902, 0.2692, 1.0904, -0.2384, -1.2858, 0.2633"
                         velocity_scaling="0.2"/>
    </Sequence>
  </BehaviorTree>
</root>`,
  },
  {
    name: "MoveToHome",
    label: "Move to Home",
    description: "Move arm to home configuration (no gripper, safe for Meca500)",
    xml: `<root BTCPP_format="4" main_tree_to_execute="MoveToHome">
  <BehaviorTree ID="MoveToHome">
    <MoveToNamedConfig name="go_home" config_name="home" velocity_scaling="0.1" acceleration_scaling="0.1"/>
  </BehaviorTree>
</root>`,
  },
  {
    name: "MoveToHomeGripper",
    label: "Move to Home (with gripper)",
    description: "Open gripper first, then move arm to home configuration",
    xml: `<root BTCPP_format="4" main_tree_to_execute="MoveToHomeGripper">
  <BehaviorTree ID="MoveToHomeGripper">
    <Sequence name="move_to_home">
      <GripperControl name="open_gripper" command="open"/>
      <MoveToNamedConfig name="go_home" config_name="home" velocity_scaling="0.3" acceleration_scaling="0.3"/>
    </Sequence>
  </BehaviorTree>
</root>`,
  },
  {
    name: "SeedCollection",
    label: "Seed Collection",
    description: "Observe, detect seed, pick it up, retreat to home",
    xml: `<root BTCPP_format="4" main_tree_to_execute="SeedCollection">
  <BehaviorTree ID="SeedCollection">
    <Sequence name="seed_collection_main">
      <MoveToNamedConfig name="go_to_observe" config_name="observe" velocity_scaling="0.5" acceleration_scaling="0.4"/>
      <DetectObject name="find_seed" object_class="seed" confidence_threshold="0.75" max_detections="5" timeout_sec="8.0" require_pose="true" detected_objects="{detections}" best_object_pose="{seed_pose}"/>
      <ComputePreGraspPose name="compute_pregrasp" input_pose="{seed_pose}" z_offset_m="0.06" output_pose="{pregrasp_pose}"/>
      <GripperControl name="open_for_grasp" command="open"/>
      <MoveToCartesianPose name="move_to_pregrasp" target_pose="{pregrasp_pose}" velocity_scaling="0.4" acceleration_scaling="0.3"/>
      <MoveToCartesianPose name="descend_to_seed" target_pose="{seed_pose}" velocity_scaling="0.1" acceleration_scaling="0.1"/>
      <GripperControl name="close_for_grasp" command="close" force_limit="8.0" object_grasped="{object_grasped}" final_position="{grasp_final_position}"/>
      <ScriptCondition name="check_grasp_success" code="object_grasped == true"/>
      <MoveToCartesianPose name="retreat_after_grasp" target_pose="{pregrasp_pose}" velocity_scaling="0.2" acceleration_scaling="0.2"/>
      <SubTree ID="MoveToHome" name="go_home_with_seed"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="MoveToHome">
    <Sequence name="move_to_home">
      <MoveToNamedConfig name="go_home" config_name="home" velocity_scaling="0.3"/>
    </Sequence>
  </BehaviorTree>
</root>`,
  },
  {
    name: "PickAndPlace",
    label: "Pick and Place",
    description: "Generic pick-and-place with retry recovery, subtrees, and blackboard passing",
    xml: `<root BTCPP_format="4" main_tree_to_execute="PickAndPlace">
  <BehaviorTree ID="PickAndPlace">
    <Sequence name="pick_and_place_main">
      <SetPose name="set_place_target" x="0.3" y="-0.2" z="0.1" pose="{place_pose}"/>
      <RetryUntilSuccessful name="pick_with_retry" num_attempts="3">
        <SubTree ID="PickObject" name="pick_attempt" object_class="seed" detected_pose="{picked_pose}"/>
      </RetryUntilSuccessful>
      <SubTree ID="PlaceObject" name="place_object" place_pose="{place_pose}"/>
      <SubTree ID="MoveToHome" name="return_home"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="PickObject">
    <Sequence name="pick_sequence">
      <MoveToNamedConfig name="go_observe" config_name="observe" velocity_scaling="0.5"/>
      <DetectObject name="find_object" object_class="{object_class}" confidence_threshold="0.7" timeout_sec="6.0" require_pose="true" best_object_pose="{detected_pose}"/>
      <ComputePreGraspPose name="calc_pregrasp" input_pose="{detected_pose}" z_offset_m="0.06" output_pose="{pregrasp_pose}"/>
      <GripperControl name="open_gripper" command="open"/>
      <MoveToCartesianPose name="go_to_pregrasp" target_pose="{pregrasp_pose}" velocity_scaling="0.4"/>
      <MoveToCartesianPose name="descend" target_pose="{detected_pose}" velocity_scaling="0.08"/>
      <GripperControl name="grasp" command="close" force_limit="8.0" object_grasped="{object_grasped}"/>
      <ScriptCondition name="verify_grasp" code="object_grasped == true"/>
      <MoveToCartesianPose name="lift" target_pose="{pregrasp_pose}" velocity_scaling="0.2"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="PlaceObject">
    <Sequence name="place_sequence">
      <ComputePreGraspPose name="calc_pre_place" input_pose="{place_pose}" z_offset_m="0.08" output_pose="{pre_place_pose}"/>
      <MoveToCartesianPose name="move_to_pre_place" target_pose="{pre_place_pose}" velocity_scaling="0.4"/>
      <MoveToCartesianPose name="descend_to_place" target_pose="{place_pose}" velocity_scaling="0.1"/>
      <GripperControl name="release" command="open"/>
      <MoveToCartesianPose name="retreat_from_place" target_pose="{pre_place_pose}" velocity_scaling="0.3"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="MoveToHome">
    <Sequence name="go_home">
      <GripperControl name="open_gripper" command="open"/>
      <MoveToNamedConfig name="home" config_name="home" velocity_scaling="0.3"/>
    </Sequence>
  </BehaviorTree>
</root>`,
  },
  {
    name: "HumanInteractionDemo",
    label: "Human Interaction Demo",
    description: "Notification, warning, confirm, input, and task assignment — exercises all human interaction nodes",
    xml: `<root BTCPP_format="4" main_tree_to_execute="HumanInteractionDemo">
  <BehaviorTree ID="HumanInteractionDemo">
    <Sequence name="demo_main">
      <HumanNotification name="notify_start" title="Demo Starting" message="Human interaction demo is beginning. You will be asked to confirm, provide input, and complete a task."/>
      <WaitForDuration name="brief_pause" seconds="1.0"/>
      <HumanWarning name="warn_example" title="Workspace Active" message="Robot will be moving in the workspace. Stay clear of the work area." severity="warning"/>
      <WaitForDuration name="pause_after_warning" seconds="1.0"/>
      <HumanConfirm name="confirm_proceed" title="Ready to Proceed?" message="The robot will begin a pick operation. Is the workspace clear and safe?" timeout_sec="120" confirmed="{area_clear}"/>
      <HumanNotification name="notify_confirmed" title="Confirmed" message="Operator confirmed workspace is clear. Proceeding."/>
      <HumanInput name="select_target" title="Select Target Location" message="Where should the robot place the object?" input_type="choice" choices="Station A;Station B;Station C" timeout_sec="120" value="{target_station}"/>
      <HumanNotification name="notify_selection" title="Target Selected" message="Operator selected a target station. Proceeding to pick."/>
      <MoveToNamedConfig name="go_observe" config_name="observe" velocity_scaling="0.5"/>
      <MoveToNamedConfig name="go_home_after_pick" config_name="home" velocity_scaling="0.3"/>
      <HumanTask name="inspect_placement" title="Inspect Placement" message="Please visually inspect the placement area and confirm the object is correctly positioned." timeout_sec="300" completed="{inspection_ok}" notes="{inspection_notes}"/>
      <HumanNotification name="notify_complete" title="Demo Complete" message="All human interaction nodes exercised successfully."/>
    </Sequence>
  </BehaviorTree>
</root>`,
  },
] as const;

interface StepEntry {
  id: number;
  skill_name: string;
  parameters_json: string;
  retry_on_failure: boolean;
  max_retries: number;
  description: string;
}

const STATUS_TO_CHIP: Record<string, ChipState> = {
  idle: "idle",
  pending: "running",
  active: "running",
  succeeded: "done",
  failed: "failed",
  cancelled: "neutral",
};

type Mode = "preset" | "compose" | "raw";

export default function BehaviorExecutorPanel() {
  const serverTrees = useAvailableTrees();
  const presets = useMemo(() => {
    if (serverTrees.length > 0) {
      return serverTrees.map((t) => ({
        name: t.name,
        label: t.label,
        description: `From ${t.filename}`,
        xml: t.xml,
      }));
    }
    return PRESET_TREES;
  }, [serverTrees]);

  const [mode, setMode] = useState<Mode>("preset");
  const [selectedPreset, setSelectedPreset] = useState<string>("");
  const [presetSearch, setPresetSearch] = useState("");
  const [rawXml, setRawXml] = useState("");
  const [steps, setSteps] = useState<StepEntry[]>([]);
  const [composedXml, setComposedXml] = useState<string | null>(null);
  const [composeWarnings, setComposeWarnings] = useState<string[]>([]);
  const [taskName, setTaskName] = useState("my_task");
  const [executionLog, setExecutionLog] = useState<string[]>([]);
  const [showDetails, setShowDetails] = useState(false);
  // Tracks the most recent run's settled status (for the result banner) — separate from
  // the live actionStatus so the banner persists after the action client returns to "idle".
  const [lastResult, setLastResult] = useState<{
    state: "succeeded" | "failed" | "cancelled";
    name: string;
    elapsedSec?: number;
    message?: string;
  } | null>(null);

  const skills = useSkillStore((s) => s.skills);
  const logEndRef = useRef<HTMLDivElement>(null);

  const { call: composeTask, loading: composing } = useServiceCall<
    ComposeTaskRequest,
    ComposeTaskResponse
  >("/skill_server/compose_task", "robot_skills_msgs/srv/ComposeTask");

  const {
    status: actionStatus,
    feedback,
    sendGoal,
    cancel,
  } = useActionClient<any, any, any>(
    "/skill_server/execute_behavior_tree",
    "robot_skills_msgs/action/ExecuteBehaviorTree"
  );

  const isRunning = actionStatus === "active" || actionStatus === "pending";

  // Auto-scroll execution log
  useEffect(() => {
    logEndRef.current?.scrollIntoView({ behavior: "smooth", block: "end" });
  }, [executionLog]);

  // Filtered preset list
  const filteredPresets = useMemo(() => {
    if (!presetSearch) return presets;
    const q = presetSearch.toLowerCase();
    return presets.filter(
      (p) =>
        p.name.toLowerCase().includes(q) ||
        p.label.toLowerCase().includes(q) ||
        p.description.toLowerCase().includes(q),
    );
  }, [presets, presetSearch]);

  const selectedPresetObj = useMemo(
    () => presets.find((p) => p.name === selectedPreset),
    [presets, selectedPreset],
  );

  // Determine what XML would run, and a friendly label for the run bar
  const runnableXml = mode === "raw" ? rawXml : composedXml;
  const runnableLabel =
    mode === "preset"
      ? selectedPresetObj?.label ?? null
      : mode === "compose"
        ? steps.length > 0
          ? composedXml
            ? taskName
            : null
          : null
        : rawXml.trim()
          ? "Raw BT XML"
          : null;

  const addLog = (msg: string) => {
    setExecutionLog((prev) => [...prev.slice(-99), `[${new Date().toLocaleTimeString()}] ${msg}`]);
  };

  const handleLoadPreset = useCallback(
    (presetName: string) => {
      setSelectedPreset(presetName);
      const preset = presets.find((p) => p.name === presetName);
      if (preset) {
        setComposedXml(preset.xml);
        setRawXml(preset.xml);
        setTaskName(presetName);
        setComposeWarnings([]);
        setLastResult(null);
        addLog(`Loaded ${preset.label}`);
      }
    },
    [presets],
  );

  const addStep = () => {
    setSteps((prev) => [
      ...prev,
      {
        id: Date.now(),
        skill_name: skills[0]?.name || "",
        parameters_json: "{}",
        retry_on_failure: false,
        max_retries: 0,
        description: "",
      },
    ]);
  };

  const removeStep = (id: number) => {
    setSteps((prev) => prev.filter((s) => s.id !== id));
    setComposedXml(null); // invalidate compose
  };

  const updateStep = (id: number, field: keyof StepEntry, value: any) => {
    setSteps((prev) => prev.map((s) => (s.id === id ? { ...s, [field]: value } : s)));
    setComposedXml(null);
  };

  const moveStep = (id: number, dir: -1 | 1) => {
    setSteps((prev) => {
      const idx = prev.findIndex((s) => s.id === id);
      if (idx < 0) return prev;
      const newIdx = idx + dir;
      if (newIdx < 0 || newIdx >= prev.length) return prev;
      const arr = [...prev];
      [arr[idx], arr[newIdx]] = [arr[newIdx], arr[idx]];
      return arr;
    });
    setComposedXml(null);
  };

  const handleCompose = async () => {
    const taskSteps: TaskStep[] = steps.map((s) => ({
      skill_name: s.skill_name,
      parameters_json: s.parameters_json,
      input_blackboard_keys: [],
      output_blackboard_keys: [],
      retry_on_failure: s.retry_on_failure,
      max_retries: s.max_retries,
      condition_expression: "",
      description: s.description,
    }));

    try {
      const res = await composeTask({
        task_name: taskName,
        task_description: "",
        steps: taskSteps,
        sequential: true,
        add_precondition_checks: true,
      });
      if (res.success) {
        setComposedXml(res.bt_xml);
        setComposeWarnings(res.warnings || []);
        setRawXml(res.bt_xml);
        addLog(`Composed ${taskName}: ${res.bt_xml.length} chars, ${res.warnings.length} warnings`);
      } else {
        addLog(`Compose failed: ${res.message}`);
      }
    } catch (e: any) {
      addLog(`Compose error: ${e.message}`);
    }
  };

  const handleExecute = async () => {
    if (!runnableXml) {
      addLog("No BT XML to execute");
      return;
    }
    setLastResult(null);
    const runName = mode === "preset" ? selectedPreset || taskName : taskName;
    addLog(`▶ Running ${runName}…`);
    try {
      const result = await sendGoal({
        tree_xml: runnableXml,
        tree_name: runName,
        enable_groot_monitor: true,
        groot_zmq_port: 1666,
        // Currently a no-op on the Python tree_executor (fully async/event-driven,
        // no polling loop). Kept in the goal for contract parity with the action
        // schema. 5Hz is the expected ceiling if a future executor introduces
        // a TaskState publish rate cap.
        tick_rate_hz: 5.0,
      });
      const elapsedSec = result.total_execution_time_sec ?? 0;
      const final = String(result.final_status || "").toUpperCase();
      const state =
        final === "SUCCESS" ? "succeeded" : final === "CANCELLED" ? "cancelled" : "failed";
      setLastResult({
        state,
        name: runName,
        elapsedSec,
        message: result.message,
      });
      addLog(`✓ ${final} in ${elapsedSec.toFixed(1)}s`);
    } catch (e: any) {
      setLastResult({ state: "failed", name: runName, message: e.message });
      addLog(`✗ ${e.message}`);
    }
  };

  const currentNode = (feedback as any)?.current_node_name as string | undefined;
  const progress = (feedback as any)?.progress as number | undefined;

  return (
    <div className="flex flex-col h-full bg-paper relative">
      {/* Header — title + mode tabs (segmented) + details toggle */}
      <div className="px-4 py-2 border-b border-hair flex items-center gap-3 shrink-0">
        <div className="flex items-center gap-2 shrink-0">
          <Rocket className="w-4 h-4 text-terracotta" />
          <h2 className="text-[13px] font-medium text-ink">Behavior Executor</h2>
        </div>

        <div className="ml-auto flex items-center gap-1.5">
          <SegmentedControl
            options={[
              { id: "preset", label: "Preset" },
              { id: "compose", label: "Compose" },
              { id: "raw", label: "Raw" },
            ]}
            active={mode}
            onSelect={(id) => setMode(id as Mode)}
          />
          <IconBtn
            active={showDetails}
            onClick={() => setShowDetails((v) => !v)}
            title={showDetails ? "Hide XML & log" : "Show XML & log"}
          >
            {showDetails ? <PanelRightClose className="w-3.5 h-3.5" /> : <PanelRightOpen className="w-3.5 h-3.5" />}
          </IconBtn>
        </div>
      </div>

      {/* Live running banner — replaces silent header chip */}
      {isRunning && (
        <RunningBanner
          taskName={taskName}
          currentNode={currentNode}
          progress={progress}
          onCancel={cancel}
        />
      )}

      {/* Last-run result banner (shown when not running) */}
      {!isRunning && lastResult && (
        <ResultBanner result={lastResult} onDismiss={() => setLastResult(null)} />
      )}

      <div className="flex flex-1 overflow-hidden">
        {/* Main: mode-specific selection / composition */}
        <div className="flex-1 flex flex-col overflow-hidden">
          {mode === "preset" && (
            <PresetMode
              presets={filteredPresets}
              total={presets.length}
              search={presetSearch}
              onSearchChange={setPresetSearch}
              selected={selectedPreset}
              onSelect={handleLoadPreset}
            />
          )}

          {mode === "compose" && (
            <ComposeMode
              steps={steps}
              skills={skills}
              taskName={taskName}
              composedXml={composedXml}
              onTaskNameChange={(v) => {
                setTaskName(v);
                setComposedXml(null);
              }}
              onAddStep={addStep}
              onRemoveStep={removeStep}
              onUpdateStep={updateStep}
              onMoveStep={moveStep}
              onCompose={handleCompose}
              composing={composing}
              warnings={composeWarnings}
            />
          )}

          {mode === "raw" && <RawMode xml={rawXml} onChange={setRawXml} />}
        </div>

        {/* Right: details (XML preview + execution log) — collapsible */}
        {showDetails && (
          <DetailsPanel
            xml={composedXml || rawXml || ""}
            log={executionLog}
            logEndRef={logEndRef}
          />
        )}
      </div>

      {/* Sticky bottom Run bar — single primary action */}
      <RunBar
        isRunning={isRunning}
        canRun={Boolean(runnableXml) && !composing}
        runLabel={runnableLabel}
        mode={mode}
        composing={composing}
        composeReady={mode === "compose" && composedXml !== null}
        onRun={handleExecute}
        onCancel={cancel}
        onCompose={handleCompose}
      />
    </div>
  );
}

/* ─── Sub-components ──────────────────────────────────────────────── */

function SegmentedControl<T extends string>({
  options,
  active,
  onSelect,
}: {
  options: { id: T; label: string }[];
  active: T;
  onSelect: (id: T) => void;
}) {
  return (
    <div className="inline-flex border border-hair rounded-sm overflow-hidden">
      {options.map((opt, i) => {
        const isActive = active === opt.id;
        return (
          <button
            key={opt.id}
            onClick={() => onSelect(opt.id)}
            className={clsx(
              "px-3 py-1 font-mono text-[10px] font-semibold uppercase tracking-[0.08em] transition-colors",
              i > 0 && "border-l border-hair",
              isActive
                ? "bg-terracotta text-paper"
                : "bg-paper text-muted hover:text-ink-soft hover:bg-cream",
            )}
          >
            {opt.label}
          </button>
        );
      })}
    </div>
  );
}

function RunningBanner({
  taskName,
  currentNode,
  progress,
  onCancel,
}: {
  taskName: string;
  currentNode?: string;
  progress?: number;
  onCancel: () => void;
}) {
  const pct = typeof progress === "number" ? Math.round(progress * 100) : null;
  return (
    <div className="px-5 py-3 bg-running-soft border-b border-running flex items-center gap-4 shrink-0">
      <div className="flex items-center gap-2 shrink-0">
        <span className="w-2 h-2 rounded-full bg-running animate-pulse" />
        <Eyebrow size="sm" tone="ink" className="text-running">
          Running
        </Eyebrow>
      </div>
      <div className="flex-1 min-w-0">
        <div className="flex items-center gap-3">
          <span className="text-[13px] font-medium text-ink truncate">{taskName}</span>
          {currentNode && (
            <span className="text-[11px] font-mono text-running tracking-[0.04em] truncate">
              ▸ {currentNode}
            </span>
          )}
          {pct !== null && (
            <span className="ml-auto text-[11px] font-mono text-running tracking-[0.06em]">
              {pct}%
            </span>
          )}
        </div>
        {pct !== null && (
          <div className="h-[3px] bg-stone mt-1.5 overflow-hidden">
            <div className="h-full bg-running transition-all duration-300" style={{ width: `${pct}%` }} />
          </div>
        )}
      </div>
      <Button
        onClick={onCancel}
        variant="danger"
        size="sm"
        leftIcon={<Square className="w-3 h-3" />}
        className="shrink-0"
      >
        Cancel
      </Button>
    </div>
  );
}

function ResultBanner({
  result,
  onDismiss,
}: {
  result: { state: "succeeded" | "failed" | "cancelled"; name: string; elapsedSec?: number; message?: string };
  onDismiss: () => void;
}) {
  const tone = result.state === "succeeded" ? "ok" : result.state === "cancelled" ? "neutral" : "err";
  const icon = result.state === "succeeded" ? "✓" : result.state === "cancelled" ? "■" : "✗";
  const label =
    result.state === "succeeded" ? "Succeeded" : result.state === "cancelled" ? "Cancelled" : "Failed";
  return (
    <div
      className={clsx(
        "px-5 py-2.5 border-b flex items-center gap-3 shrink-0",
        tone === "ok" && "bg-ok-soft border-ok",
        tone === "err" && "bg-err-soft border-err",
        tone === "neutral" && "bg-cream-deep border-hair",
      )}
    >
      <span
        className={clsx(
          "font-mono font-bold text-[14px]",
          tone === "ok" && "text-ok",
          tone === "err" && "text-err",
          tone === "neutral" && "text-muted",
        )}
      >
        {icon}
      </span>
      <div className="flex-1 min-w-0">
        <div className="flex items-baseline gap-2">
          <Eyebrow
            size="sm"
            className={clsx(
              tone === "ok" && "text-ok",
              tone === "err" && "text-err",
              tone === "neutral" && "text-muted",
            )}
          >
            {label}
          </Eyebrow>
          <span className="text-[13px] text-ink truncate">{result.name}</span>
          {typeof result.elapsedSec === "number" && (
            <span className="text-[11px] font-mono text-muted tracking-[0.06em] ml-auto">
              {result.elapsedSec.toFixed(1)}s
            </span>
          )}
        </div>
        {result.message && tone === "err" && (
          <p className="text-[12px] text-err mt-0.5 truncate">{result.message}</p>
        )}
      </div>
      <IconBtn onClick={onDismiss} className="!w-6 !h-6">
        <Square className="w-2.5 h-2.5 rotate-45" />
      </IconBtn>
    </div>
  );
}

function RunBar({
  isRunning,
  canRun,
  runLabel,
  mode,
  composing,
  composeReady,
  onRun,
  onCancel,
  onCompose,
}: {
  isRunning: boolean;
  canRun: boolean;
  runLabel: string | null;
  mode: Mode;
  composing: boolean;
  composeReady: boolean;
  onRun: () => void;
  onCancel: () => void;
  onCompose: () => void;
}) {
  // In Compose mode, the workflow is two-step: build steps → compose XML → run.
  // Surface a separate Compose action when there's no XML yet.
  const showComposeButton = mode === "compose" && !composeReady && !composing;
  const placeholder =
    mode === "preset"
      ? "Select a preset to run"
      : mode === "compose"
        ? "Add steps and compose to run"
        : "Paste BT XML to run";
  return (
    <div className="border-t border-hair bg-cream-deep px-4 py-2 shrink-0">
      <div className="flex items-center gap-3">
        <div className="flex-1 min-w-0 flex items-baseline gap-2">
          <Eyebrow size="sm" tone="muted" className="shrink-0">
            {isRunning ? "Running" : runLabel ? "Ready" : "Idle"}
          </Eyebrow>
          {runLabel ? (
            <span className="text-[13px] font-medium text-ink truncate">{runLabel}</span>
          ) : (
            <span className="text-[12px] text-muted truncate">{placeholder}</span>
          )}
        </div>

        {showComposeButton && (
          <Button
            onClick={onCompose}
            variant="secondary"
            size="sm"
            leftIcon={<FileCode className="w-3.5 h-3.5" />}
          >
            Compose
          </Button>
        )}

        {isRunning ? (
          <Button
            onClick={onCancel}
            variant="danger"
            size="sm"
            leftIcon={<Square className="w-3.5 h-3.5" />}
            className="!px-4"
          >
            Cancel
          </Button>
        ) : (
          <Button
            onClick={onRun}
            disabled={!canRun}
            variant="primary"
            leftIcon={<Play className="w-3.5 h-3.5" fill="currentColor" />}
            className="!px-5 !py-1.5 text-[13px]"
          >
            Run
          </Button>
        )}
      </div>
    </div>
  );
}

function PresetMode({
  presets,
  total,
  search,
  onSearchChange,
  selected,
  onSelect,
}: {
  presets: readonly { name: string; label: string; description: string; xml: string }[];
  total: number;
  search: string;
  onSearchChange: (v: string) => void;
  selected: string;
  onSelect: (name: string) => void;
}) {
  return (
    <div className="flex flex-col h-full">
      <div className="px-4 py-2 border-b border-hair-soft shrink-0">
        <div className="relative">
          <Search className="absolute left-2.5 top-1/2 -translate-y-1/2 w-3.5 h-3.5 text-muted pointer-events-none" />
          <input
            type="text"
            value={search}
            onChange={(e) => onSearchChange(e.target.value)}
            placeholder={`Search ${total} preset${total === 1 ? "" : "s"}…`}
            className="w-full pl-8 pr-3 py-1 text-[12.5px] bg-paper border border-hair rounded-DEFAULT focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
          />
        </div>
      </div>

      <div className="flex-1 overflow-auto">
        {presets.length === 0 && (
          <div className="px-4 py-6 text-center text-muted text-[12px]">
            {total === 0 ? "No presets available" : "No presets match your search"}
          </div>
        )}
        {presets.map((p) => {
          const isSelected = selected === p.name;
          return (
            <button
              key={p.name}
              onClick={() => onSelect(p.name)}
              title={p.description}
              className={clsx(
                "w-full text-left px-4 py-1.5 border-b border-hair-soft border-l-2 transition-colors flex items-center gap-2.5",
                isSelected
                  ? "border-l-terracotta bg-terracotta-tint"
                  : "border-l-transparent hover:bg-cream",
              )}
            >
              <span
                className={clsx(
                  "shrink-0 w-3.5 h-3.5 rounded-full border flex items-center justify-center",
                  isSelected ? "border-terracotta bg-terracotta" : "border-hair bg-paper",
                )}
              >
                {isSelected && <Check className="w-2 h-2 text-paper" strokeWidth={3} />}
              </span>
              <div className="flex-1 min-w-0">
                <div className="flex items-baseline gap-2">
                  <span className="text-[13px] font-medium text-ink truncate">{p.label}</span>
                  <span className="font-mono text-[10px] text-muted truncate tracking-[0.04em]">
                    {p.name}
                  </span>
                </div>
                <p className="text-[11.5px] text-muted truncate">{p.description}</p>
              </div>
            </button>
          );
        })}
      </div>
    </div>
  );
}

function ComposeMode({
  steps,
  skills,
  taskName,
  composedXml,
  onTaskNameChange,
  onAddStep,
  onRemoveStep,
  onUpdateStep,
  onMoveStep,
  onCompose,
  composing,
  warnings,
}: {
  steps: StepEntry[];
  skills: SkillDescription[];
  taskName: string;
  composedXml: string | null;
  onTaskNameChange: (v: string) => void;
  onAddStep: () => void;
  onRemoveStep: (id: number) => void;
  onUpdateStep: (id: number, field: keyof StepEntry, value: any) => void;
  onMoveStep: (id: number, dir: -1 | 1) => void;
  onCompose: () => void;
  composing: boolean;
  warnings: string[];
}) {
  const [editingId, setEditingId] = useState<number | null>(null);

  return (
    <div className="flex flex-col h-full overflow-hidden">
      <div className="px-4 py-2 border-b border-hair-soft shrink-0">
        <div className="flex items-center gap-2">
          <Eyebrow size="sm" tone="muted" className="shrink-0">
            Task
          </Eyebrow>
          <Input
            type="text"
            value={taskName}
            onChange={(e) => onTaskNameChange(e.target.value)}
            placeholder="my_task"
            className="!py-1 !text-[12.5px] max-w-xs"
          />
          <Eyebrow size="sm" tone="muted" className="ml-auto shrink-0">
            {steps.length} step{steps.length === 1 ? "" : "s"}
          </Eyebrow>
        </div>
      </div>

      <div className="flex-1 overflow-auto p-4 space-y-1.5">
        {steps.length === 0 && (
          <div className="text-center text-muted text-[12px] py-4">
            No steps yet — add a skill below to start.
          </div>
        )}
        {steps.map((step, idx) => (
          <ComposeStepRow
            key={step.id}
            idx={idx}
            step={step}
            skills={skills}
            isEditing={editingId === step.id}
            isFirst={idx === 0}
            isLast={idx === steps.length - 1}
            onToggleEdit={() => setEditingId(editingId === step.id ? null : step.id)}
            onUpdate={(f, v) => onUpdateStep(step.id, f, v)}
            onMove={(d) => onMoveStep(step.id, d)}
            onRemove={() => onRemoveStep(step.id)}
          />
        ))}

        <button
          onClick={onAddStep}
          className="w-full flex items-center justify-center gap-1.5 py-1.5 border border-dashed border-hair text-[12px] text-muted hover:text-terracotta hover:border-terracotta hover:bg-cream transition-colors"
        >
          <Plus className="w-3 h-3" />
          Add Step
        </button>

        {warnings.length > 0 && (
          <Banner tone="info" title="Compose Warnings" icon={<AlertTriangle className="w-4 h-4" />}>
            <ul className="text-[12px] space-y-0.5">
              {warnings.map((w, i) => (
                <li key={i}>{w}</li>
              ))}
            </ul>
          </Banner>
        )}

        {steps.length > 0 && composedXml && !composing && (
          <div className="flex items-center gap-2 text-[12px] text-ok pt-1">
            <Check className="w-3.5 h-3.5" />
            BT XML composed — open Run to execute, or edit a step to recompose.
          </div>
        )}
      </div>
    </div>
  );
}

function ComposeStepRow({
  idx,
  step,
  skills,
  isEditing,
  isFirst,
  isLast,
  onToggleEdit,
  onUpdate,
  onMove,
  onRemove,
}: {
  idx: number;
  step: StepEntry;
  skills: SkillDescription[];
  isEditing: boolean;
  isFirst: boolean;
  isLast: boolean;
  onToggleEdit: () => void;
  onUpdate: (field: keyof StepEntry, value: any) => void;
  onMove: (dir: -1 | 1) => void;
  onRemove: () => void;
}) {
  const skill = skills.find((s) => s.name === step.skill_name);
  const skillLabel = skill?.display_name || step.skill_name || "(unset)";

  return (
    <div className={clsx("border bg-paper", isEditing ? "border-terracotta" : "border-hair")}>
      {/* Compact row */}
      <div className="flex items-center gap-2 px-3 py-2">
        <span className="font-mono text-[11px] text-muted w-5 tracking-[0.04em] shrink-0">
          {String(idx + 1).padStart(2, "0")}
        </span>
        <GripVertical className="w-3 h-3 text-muted shrink-0" />
        <div className="flex-1 min-w-0 flex items-baseline gap-2">
          <span className="text-[13px] font-medium text-ink truncate">{skillLabel}</span>
          {step.parameters_json && step.parameters_json !== "{}" && (
            <span className="font-mono text-[11px] text-muted truncate tracking-[0.02em]">
              {step.parameters_json}
            </span>
          )}
          {step.retry_on_failure && (
            <span className="font-mono text-[10px] text-terracotta tracking-[0.06em] shrink-0">
              ↻ {step.max_retries}
            </span>
          )}
        </div>
        <div className="flex items-center gap-0.5 shrink-0">
          <IconBtn
            onClick={() => onMove(-1)}
            disabled={isFirst}
            className="!w-6 !h-6"
            title="Move up"
          >
            <ChevronUp className="w-3 h-3" />
          </IconBtn>
          <IconBtn
            onClick={() => onMove(1)}
            disabled={isLast}
            className="!w-6 !h-6"
            title="Move down"
          >
            <ChevronDown className="w-3 h-3" />
          </IconBtn>
          <IconBtn onClick={onToggleEdit} active={isEditing} className="!w-6 !h-6" title="Edit">
            <Pencil className="w-3 h-3" />
          </IconBtn>
          <IconBtn
            onClick={onRemove}
            className="!w-6 !h-6 hover:!bg-err-soft hover:!text-err"
            title="Remove"
          >
            <Trash2 className="w-3 h-3" />
          </IconBtn>
        </div>
      </div>

      {/* Expanded editor */}
      {isEditing && (
        <div className="border-t border-hair-soft bg-cream-deep px-3 py-3 space-y-2">
          <div>
            <Eyebrow size="sm" tone="muted" className="block mb-1">
              Skill
            </Eyebrow>
            <select
              value={step.skill_name}
              onChange={(e) => onUpdate("skill_name", e.target.value)}
              className="w-full px-2 py-1.5 text-[12.5px] bg-paper border border-hair text-ink-soft rounded-DEFAULT focus:border-terracotta focus:outline-none"
            >
              <option value="">— select skill —</option>
              {skills.map((s) => (
                <option key={s.name} value={s.name}>
                  {s.display_name || s.name}
                </option>
              ))}
            </select>
            {skill?.description && (
              <p className="text-[11px] text-muted mt-1">{skill.description}</p>
            )}
          </div>

          <div>
            <Eyebrow size="sm" tone="muted" className="block mb-1">
              Parameters (JSON)
            </Eyebrow>
            <Textarea
              value={step.parameters_json}
              onChange={(e) => onUpdate("parameters_json", e.target.value)}
              placeholder='{"param": "value"}'
              rows={3}
              mono
            />
          </div>

          <div className="flex items-center gap-3">
            <label className="flex items-center gap-2 text-[12px] text-ink-soft">
              <input
                type="checkbox"
                checked={step.retry_on_failure}
                onChange={(e) => onUpdate("retry_on_failure", e.target.checked)}
                className="accent-terracotta"
              />
              Retry on failure
            </label>
            {step.retry_on_failure && (
              <label className="flex items-center gap-1.5 text-[12px] text-ink-soft">
                Max retries
                <input
                  type="number"
                  value={step.max_retries}
                  onChange={(e) => onUpdate("max_retries", parseInt(e.target.value) || 0)}
                  className="w-14 px-2 py-0.5 text-[11px] font-mono bg-paper border border-hair text-ink-soft rounded-DEFAULT focus:border-terracotta focus:outline-none"
                  min={0}
                  max={10}
                />
              </label>
            )}
          </div>
        </div>
      )}
    </div>
  );
}

function RawMode({ xml, onChange }: { xml: string; onChange: (xml: string) => void }) {
  return (
    <div className="flex flex-col h-full p-5">
      <Eyebrow size="sm" tone="muted" className="block mb-2">
        Raw BT XML
      </Eyebrow>
      <Textarea
        value={xml}
        onChange={(e) => onChange(e.target.value)}
        placeholder={`<root BTCPP_format="4" main_tree_to_execute="MyTree">\n  <BehaviorTree ID="MyTree">\n    …\n  </BehaviorTree>\n</root>`}
        mono
        className="flex-1 !h-full resize-none"
      />
    </div>
  );
}

function DetailsPanel({
  xml,
  log,
  logEndRef,
}: {
  xml: string;
  log: string[];
  logEndRef: React.RefObject<HTMLDivElement>;
}) {
  return (
    <div className="w-[400px] border-l border-hair flex flex-col shrink-0 bg-paper">
      <div className="flex-1 overflow-auto border-b border-hair-soft">
        <div className="px-4 pt-3 pb-2 flex items-center gap-2 sticky top-0 bg-paper border-b border-hair-soft">
          <FileCode className="w-3 h-3 text-muted" />
          <Eyebrow size="sm" tone="muted">
            BT XML
          </Eyebrow>
          {xml && (
            <span className="ml-auto font-mono text-[10px] text-muted tracking-[0.04em]">
              {xml.length} chars
            </span>
          )}
        </div>
        {xml ? (
          <pre className="sociius-code mx-4 my-3 text-[11px] whitespace-pre-wrap">{xml}</pre>
        ) : (
          <p className="px-4 py-6 text-[12px] text-muted italic">No XML yet</p>
        )}
      </div>

      <div className="h-44 overflow-auto px-4 py-3 bg-cream-deep shrink-0">
        <Eyebrow size="sm" tone="muted" className="block mb-1.5 sticky top-0 bg-cream-deep">
          Execution Log
        </Eyebrow>
        {log.map((line, i) => (
          <div
            key={i}
            className="text-[11px] font-mono text-ink-soft leading-relaxed tracking-[0.02em]"
          >
            {line}
          </div>
        ))}
        {log.length === 0 && (
          <div className="text-[11px] text-muted italic">No activity yet</div>
        )}
        <div ref={logEndRef} />
      </div>
    </div>
  );
}
