import { useState, useMemo, useCallback } from "react";
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
  CheckCircle,
  Rocket,
} from "lucide-react";
import clsx from "clsx";

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
    name: "FullDemo",
    label: "Full Demo",
    description: "Comprehensive demo: enable, scene setup, survey, pick, transport, place, cleanup — exercises all skills",
    xml: `<root BTCPP_format="4" main_tree_to_execute="FullDemo">
  <BehaviorTree ID="FullDemo">
    <Sequence name="full_demo_main">
      <SubTree ID="Startup" name="startup"/>
      <SubTree ID="SetupScene" name="setup_scene"/>
      <SubTree ID="Survey" name="survey"/>
      <RetryUntilSuccessful name="pick_with_retry" num_attempts="2">
        <SubTree ID="Pick" name="pick_object" detected_pose="{pick_pose}"/>
      </RetryUntilSuccessful>
      <SubTree ID="Transport" name="transport"/>
      <SubTree ID="Place" name="place_object"/>
      <SubTree ID="Cleanup" name="cleanup"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Startup">
    <Sequence name="startup_seq">
      <LogEvent name="log_start" event_name="demo_start" severity="info" message="Full demo starting"/>
      <RobotEnable name="enable_robot" enable="true"/>
      <SetVelocityOverride name="set_global_velocity" scaling="0.5" velocity_override="{velocity_override}"/>
      <RecordRosbag name="start_recording" output_path="/tmp/demo_recording" duration_sec="300.0"/>
      <MoveToNamedConfig name="go_home_initial" config_name="home" velocity_scaling="0.4"/>
      <LogEvent name="log_startup_done" event_name="startup_complete" severity="info" message="Robot enabled and homed"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="SetupScene">
    <Sequence name="setup_scene_seq">
      <SetPose name="set_table_pose" x="0.4" y="0.0" z="-0.01" frame_id="world" pose="{table_pose}"/>
      <UpdatePlanningScene name="add_table" object_id="table" operation="add" pose="{table_pose}" shape_type="box"/>
      <SetDigitalIO name="lights_on" pin_name="workspace_light" value="true"/>
      <LogEvent name="log_scene_ready" event_name="scene_setup" severity="info" message="Planning scene and I/O configured"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Survey">
    <Sequence name="survey_seq">
      <MoveToNamedConfig name="go_observe" config_name="observe" velocity_scaling="0.5"/>
      <GetCurrentPose name="read_eef_pose" frame_id="world" pose="{observe_eef_pose}"/>
      <CapturePointCloud name="capture_cloud" timeout_sec="5.0" apply_filters="true"/>
      <WaitForDuration name="settle" seconds="0.5"/>
      <LogEvent name="log_survey_done" event_name="survey_complete" severity="info" message="Point cloud captured from observation pose"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Pick">
    <Sequence name="pick_seq">
      <DetectObject name="find_object" object_class="seed" confidence_threshold="0.7" max_detections="5" timeout_sec="8.0" require_pose="true" best_object_pose="{detected_pose}"/>
      <TransformPose name="pose_to_world" input_pose="{detected_pose}" target_frame="world" output_pose="{pick_pose_world}"/>
      <ComputePreGraspPose name="calc_pregrasp" input_pose="{pick_pose_world}" z_offset_m="0.06" output_pose="{pregrasp_pose}"/>
      <GripperControl name="open_gripper" command="open"/>
      <MoveToCartesianPose name="go_pregrasp" target_pose="{pregrasp_pose}" velocity_scaling="0.4"/>
      <MoveCartesianLinear name="descend_linear" target_pose="{pick_pose_world}" velocity_scaling="0.05" step_size="0.002"/>
      <GripperControl name="grasp_object" command="close" force_limit="8.0" object_grasped="{object_grasped}" final_position="{grasp_position}"/>
      <CheckGraspSuccess name="verify_grasp" object_grasped="{object_grasped}" final_position="{grasp_position}" min_grasp_width="0.001"/>
      <MoveToCartesianPose name="lift_object" target_pose="{pregrasp_pose}" velocity_scaling="0.15"/>
      <LogEvent name="log_pick_done" event_name="pick_success" severity="info" message="Object grasped and lifted"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Transport">
    <Sequence name="transport_seq">
      <CheckCollision name="check_collision_before_transport"/>
      <LookupTransform name="read_base_to_hand" source_frame="world" target_frame="panda_hand" pose="{hand_pose_before_transport}"/>
      <MoveToNamedConfig name="go_ready" config_name="ready" velocity_scaling="0.3"/>
      <MoveToJointConfig name="go_place_zone" joint_positions="0.0;-0.5;0.0;-2.0;0.0;1.5;0.7" velocity_scaling="0.3"/>
      <LogEvent name="log_transport_done" event_name="transport_complete" severity="info" message="Object transported to place zone"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Place">
    <Sequence name="place_seq">
      <SetPose name="set_place_pose" x="0.4" y="0.2" z="0.05" frame_id="world" pose="{place_pose}"/>
      <ComputePreGraspPose name="calc_pre_place" input_pose="{place_pose}" z_offset_m="0.08" output_pose="{pre_place_pose}"/>
      <MoveToCartesianPose name="go_pre_place" target_pose="{pre_place_pose}" velocity_scaling="0.3"/>
      <MoveCartesianLinear name="descend_to_place" target_pose="{place_pose}" velocity_scaling="0.05" step_size="0.002"/>
      <GripperControl name="release_object" command="open"/>
      <WaitForDuration name="settle_after_release" seconds="0.3"/>
      <MoveToCartesianPose name="retreat_from_place" target_pose="{pre_place_pose}" velocity_scaling="0.3"/>
      <LogEvent name="log_place_done" event_name="place_success" severity="info" message="Object placed and retreated"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Cleanup">
    <Sequence name="cleanup_seq">
      <UpdatePlanningScene name="remove_table" object_id="table" operation="remove"/>
      <SetDigitalIO name="lights_off" pin_name="workspace_light" value="false"/>
      <GripperControl name="open_gripper_final" command="open"/>
      <MoveToNamedConfig name="go_home_final" config_name="home" velocity_scaling="0.3"/>
      <LogEvent name="log_demo_complete" event_name="demo_complete" severity="info" message="Full demo completed successfully"/>
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
      <LogEvent name="log_moving_observe" event_name="robot_move" message="Robot moving to observation position" severity="info" tags="human"/>
      <MoveToNamedConfig name="go_observe" config_name="observe" velocity_scaling="0.5"/>
      <LogEvent name="log_picking" event_name="robot_action" message="Robot performing pick operation" severity="info" tags="human"/>
      <MoveToNamedConfig name="go_home_after_pick" config_name="home" velocity_scaling="0.3"/>
      <LogEvent name="log_pick_done" event_name="robot_action" message="Robot returned home with object" severity="info" tags="human"/>
      <Fallback name="inspection_with_recovery">
        <HumanTask name="inspect_placement" title="Inspect Placement" message="Please visually inspect the placement area and confirm the object is correctly positioned. Press Done when verified or Failed if repositioning is needed." timeout_sec="300" completed="{inspection_ok}" notes="{inspection_notes}"/>
        <Sequence name="handle_inspection_failure">
          <HumanWarning name="warn_inspection_failed" title="Inspection Failed" message="Operator reported placement issue. Manual intervention may be needed." severity="error"/>
          <HumanConfirm name="confirm_continue_anyway" title="Continue Anyway?" message="The inspection failed. Do you want to continue the demo, or reject to abort?" timeout_sec="120" confirmed="{continue_after_fail}"/>
        </Sequence>
      </Fallback>
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

export default function BehaviorExecutorPanel() {
  const serverTrees = useAvailableTrees();
  // Use server-provided trees if available, fall back to hardcoded presets
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

  const [mode, setMode] = useState<"preset" | "compose" | "raw">("preset");
  const [selectedPreset, setSelectedPreset] = useState<string>("");
  const [rawXml, setRawXml] = useState("");
  const [steps, setSteps] = useState<StepEntry[]>([]);
  const [composedXml, setComposedXml] = useState<string | null>(null);
  const [composeWarnings, setComposeWarnings] = useState<string[]>([]);
  const [taskName, setTaskName] = useState("my_task");
  const [executionLog, setExecutionLog] = useState<string[]>([]);

  const skills = useSkillStore((s) => s.skills);

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

  // Load preset tree XML directly from embedded definitions
  const handleLoadPreset = useCallback(
    (presetName: string) => {
      setSelectedPreset(presetName);
      const preset = presets.find((p) => p.name === presetName);
      if (preset) {
        setComposedXml(preset.xml);
        setRawXml(preset.xml);
        setTaskName(presetName);
        setComposeWarnings([]);
        addLog(`Loaded preset: ${preset.label} (${preset.xml.length} chars)`);
      }
    },
    [presets]
  );

  const addLog = (msg: string) => {
    setExecutionLog((prev) => [...prev.slice(-49), `[${new Date().toLocaleTimeString()}] ${msg}`]);
  };

  // Add a step to the composer
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
  };

  const updateStep = (id: number, field: keyof StepEntry, value: any) => {
    setSteps((prev) =>
      prev.map((s) => (s.id === id ? { ...s, [field]: value } : s))
    );
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
  };

  // Compose from steps
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

  // Execute
  const handleExecute = async () => {
    const xml = mode === "raw" ? rawXml : composedXml;
    if (!xml) {
      addLog("No BT XML to execute");
      return;
    }

    addLog(`Executing '${taskName}'...`);
    try {
      const result = await sendGoal({
        tree_xml: xml,
        tree_name: taskName,
        enable_groot_monitor: true,
        groot_zmq_port: 1666,
        tick_rate_hz: 10.0,
      });
      addLog(
        `Completed: ${result.final_status} in ${result.total_execution_time_sec?.toFixed(1)}s`
      );
    } catch (e: any) {
      addLog(`Execution error: ${e.message}`);
    }
  };

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-4 py-2 border-b border-gray-800 flex items-center gap-3">
        <Rocket className="w-4 h-4 text-blue-400" />
        <h2 className="text-sm font-semibold">Behavior Executor</h2>

        {/* Mode tabs */}
        <div className="ml-4 flex gap-1">
          {(["preset", "compose", "raw"] as const).map((m) => (
            <button
              key={m}
              onClick={() => setMode(m)}
              className={clsx(
                "px-2 py-0.5 rounded text-[10px] font-medium uppercase",
                mode === m
                  ? "bg-blue-500/20 text-blue-400"
                  : "text-gray-500 hover:text-gray-300"
              )}
            >
              {m}
            </button>
          ))}
        </div>

        {/* Status */}
        <div className="ml-auto flex items-center gap-2">
          {isRunning && feedback && (
            <span className="text-[10px] text-blue-300 font-mono">
              {(feedback as any).current_node_name || "running"}
            </span>
          )}
          <StatusBadge status={actionStatus} />
        </div>
      </div>

      <div className="flex flex-1 overflow-hidden">
        {/* Left: configuration */}
        <div className="w-96 border-r border-gray-800 overflow-auto p-3 space-y-3 shrink-0">
          {mode === "preset" && (
            <PresetMode
              presets={presets}
              selected={selectedPreset}
              onSelect={handleLoadPreset}
              loading={composing}
            />
          )}

          {mode === "compose" && (
            <ComposeMode
              steps={steps}
              skills={skills}
              taskName={taskName}
              onTaskNameChange={setTaskName}
              onAddStep={addStep}
              onRemoveStep={removeStep}
              onUpdateStep={updateStep}
              onMoveStep={moveStep}
              onCompose={handleCompose}
              composing={composing}
            />
          )}

          {mode === "raw" && (
            <RawMode xml={rawXml} onChange={setRawXml} />
          )}

          {/* Warnings */}
          {composeWarnings.length > 0 && (
            <div className="p-2 rounded bg-yellow-950/30 border border-yellow-800/40">
              <div className="flex items-center gap-1 text-[10px] text-yellow-400 font-medium mb-1">
                <AlertTriangle className="w-3 h-3" />
                Warnings
              </div>
              {composeWarnings.map((w, i) => (
                <p key={i} className="text-[10px] text-yellow-300">
                  {w}
                </p>
              ))}
            </div>
          )}

          {/* Execute button */}
          <div className="flex gap-2">
            <button
              onClick={handleExecute}
              disabled={isRunning || (!composedXml && !rawXml)}
              className={clsx(
                "flex-1 flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-semibold transition-all",
                isRunning
                  ? "bg-gray-700 text-gray-400 cursor-not-allowed"
                  : "bg-blue-600 hover:bg-blue-500 text-white"
              )}
            >
              <Play className="w-3.5 h-3.5" />
              Execute
            </button>
            {isRunning && (
              <button
                onClick={cancel}
                className="px-3 py-2 rounded-lg text-xs font-semibold bg-red-700 hover:bg-red-600 text-white flex items-center gap-1"
              >
                <Square className="w-3 h-3" />
                Cancel
              </button>
            )}
          </div>
        </div>

        {/* Right: XML preview + execution log */}
        <div className="flex-1 flex flex-col overflow-hidden">
          {/* XML Preview */}
          <div className="flex-1 overflow-auto p-3 border-b border-gray-800">
            <div className="flex items-center gap-1.5 text-[10px] text-gray-500 font-medium uppercase mb-2">
              <FileCode className="w-3 h-3" />
              Generated BT XML
            </div>
            <pre className="text-[10px] font-mono text-gray-400 leading-relaxed whitespace-pre-wrap">
              {composedXml || rawXml || "No XML generated yet. Select a preset or compose a task."}
            </pre>
          </div>

          {/* Execution log */}
          <div className="h-40 overflow-auto p-3 bg-gray-950/50 shrink-0">
            <div className="text-[10px] text-gray-500 font-medium uppercase mb-1">
              Execution Log
            </div>
            {executionLog.map((line, i) => (
              <div key={i} className="text-[10px] font-mono text-gray-400">
                {line}
              </div>
            ))}
            {executionLog.length === 0 && (
              <div className="text-[10px] text-gray-600 italic">
                No activity yet
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}

function StatusBadge({ status }: { status: string }) {
  const styles: Record<string, string> = {
    idle: "text-gray-500",
    pending: "text-yellow-400 animate-pulse",
    active: "text-blue-400 animate-pulse",
    succeeded: "text-green-400",
    failed: "text-red-400",
    cancelled: "text-yellow-400",
  };
  if (status === "idle") return null;
  return (
    <span className={clsx("text-[10px] font-bold uppercase", styles[status])}>
      {status === "succeeded" && <CheckCircle className="w-3 h-3 inline mr-0.5" />}
      {status}
    </span>
  );
}

function PresetMode({
  presets,
  selected,
  onSelect,
  loading,
}: {
  presets: readonly { name: string; label: string; description: string }[];
  selected: string;
  onSelect: (name: string) => void;
  loading: boolean;
}) {
  return (
    <div className="space-y-2">
      <div className="text-[10px] text-gray-500 font-medium uppercase">
        Preset Behaviors
      </div>
      {presets.map((p) => (
        <button
          key={p.name}
          onClick={() => onSelect(p.name)}
          disabled={loading}
          className={clsx(
            "w-full text-left p-2 rounded-lg border transition-all",
            selected === p.name
              ? "border-blue-500 bg-blue-950/30"
              : "border-gray-800 hover:border-gray-600 bg-gray-900/50"
          )}
        >
          <div className="text-xs font-semibold text-gray-200">{p.label}</div>
          <div className="text-[10px] text-gray-400">{p.description}</div>
        </button>
      ))}
      {loading && (
        <div className="flex items-center gap-1 text-[10px] text-blue-400">
          <Loader2 className="w-3 h-3 animate-spin" />
          Composing...
        </div>
      )}
    </div>
  );
}

function ComposeMode({
  steps,
  skills,
  taskName,
  onTaskNameChange,
  onAddStep,
  onRemoveStep,
  onUpdateStep,
  onMoveStep,
  onCompose,
  composing,
}: {
  steps: StepEntry[];
  skills: SkillDescription[];
  taskName: string;
  onTaskNameChange: (v: string) => void;
  onAddStep: () => void;
  onRemoveStep: (id: number) => void;
  onUpdateStep: (id: number, field: keyof StepEntry, value: any) => void;
  onMoveStep: (id: number, dir: -1 | 1) => void;
  onCompose: () => void;
  composing: boolean;
}) {
  return (
    <div className="space-y-2">
      <div>
        <label className="text-[10px] text-gray-500 font-medium uppercase">
          Task Name
        </label>
        <input
          type="text"
          value={taskName}
          onChange={(e) => onTaskNameChange(e.target.value)}
          className="w-full mt-0.5 px-2 py-1 text-xs bg-gray-800 border border-gray-700 rounded focus:border-blue-500 focus:outline-none text-gray-200"
        />
      </div>

      <div className="text-[10px] text-gray-500 font-medium uppercase">
        Steps ({steps.length})
      </div>

      {steps.map((step, idx) => (
        <div
          key={step.id}
          className="p-2 rounded border border-gray-800 bg-gray-900/50 space-y-1.5"
        >
          <div className="flex items-center gap-1">
            <span className="text-[10px] text-gray-500 w-4">{idx + 1}.</span>
            <select
              value={step.skill_name}
              onChange={(e) => onUpdateStep(step.id, "skill_name", e.target.value)}
              className="flex-1 px-1 py-0.5 text-[10px] bg-gray-800 border border-gray-700 rounded text-gray-200"
            >
              {skills.map((s) => (
                <option key={s.name} value={s.name}>
                  {s.display_name || s.name}
                </option>
              ))}
            </select>
            <button onClick={() => onMoveStep(step.id, -1)} className="p-0.5 text-gray-500 hover:text-gray-300">
              <ChevronUp className="w-3 h-3" />
            </button>
            <button onClick={() => onMoveStep(step.id, 1)} className="p-0.5 text-gray-500 hover:text-gray-300">
              <ChevronDown className="w-3 h-3" />
            </button>
            <button onClick={() => onRemoveStep(step.id)} className="p-0.5 text-red-500 hover:text-red-300">
              <Trash2 className="w-3 h-3" />
            </button>
          </div>
          <input
            type="text"
            value={step.parameters_json}
            onChange={(e) => onUpdateStep(step.id, "parameters_json", e.target.value)}
            placeholder='{"param": "value"}'
            className="w-full px-1.5 py-0.5 text-[10px] font-mono bg-gray-800 border border-gray-700 rounded text-gray-300"
          />
          <div className="flex items-center gap-2">
            <label className="flex items-center gap-1 text-[9px] text-gray-500">
              <input
                type="checkbox"
                checked={step.retry_on_failure}
                onChange={(e) => onUpdateStep(step.id, "retry_on_failure", e.target.checked)}
                className="rounded-sm"
              />
              Retry
            </label>
            {step.retry_on_failure && (
              <input
                type="number"
                value={step.max_retries}
                onChange={(e) => onUpdateStep(step.id, "max_retries", parseInt(e.target.value) || 0)}
                className="w-12 px-1 py-0 text-[10px] bg-gray-800 border border-gray-700 rounded text-gray-300"
                min={0}
                max={10}
              />
            )}
          </div>
        </div>
      ))}

      <button
        onClick={onAddStep}
        className="w-full flex items-center justify-center gap-1 py-1.5 rounded border border-dashed border-gray-700 text-xs text-gray-500 hover:text-gray-300 hover:border-gray-500"
      >
        <Plus className="w-3 h-3" />
        Add Step
      </button>

      {steps.length > 0 && (
        <button
          onClick={onCompose}
          disabled={composing}
          className="w-full flex items-center justify-center gap-1 py-1.5 rounded bg-gray-700 hover:bg-gray-600 text-xs text-gray-200 font-medium"
        >
          {composing ? (
            <Loader2 className="w-3 h-3 animate-spin" />
          ) : (
            <FileCode className="w-3 h-3" />
          )}
          Compose BT XML
        </button>
      )}
    </div>
  );
}

function RawMode({
  xml,
  onChange,
}: {
  xml: string;
  onChange: (xml: string) => void;
}) {
  return (
    <div className="space-y-2">
      <div className="text-[10px] text-gray-500 font-medium uppercase">
        Raw BT XML
      </div>
      <textarea
        value={xml}
        onChange={(e) => onChange(e.target.value)}
        rows={20}
        placeholder="Paste BehaviorTree.CPP v4 XML here..."
        className="w-full px-2 py-1.5 text-[10px] font-mono bg-gray-800 border border-gray-700 rounded focus:border-blue-500 focus:outline-none text-gray-300 resize-y"
      />
    </div>
  );
}
