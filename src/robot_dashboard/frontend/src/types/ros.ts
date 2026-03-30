/** Matches robot_skills_msgs/msg/TaskState */
export interface TaskState {
  task_id: string;
  task_name: string;
  status: "IDLE" | "RUNNING" | "SUCCESS" | "FAILURE" | "CANCELLED";
  current_skill: string;
  current_bt_node: string;
  progress: number;
  completed_skills: string[];
  failed_skills: string[];
  started_at: RosTime;
  updated_at: RosTime;
  elapsed_sec: number;
  error_message: string;
  error_skill: string;
}

export interface RosTime {
  sec: number;
  nanosec: number;
}

/** Matches robot_skills_msgs/msg/SkillDescription */
export interface SkillDescription {
  name: string;
  display_name: string;
  description: string;
  version: string;
  category: string;
  tags: string[];
  preconditions: string[];
  postconditions: string[];
  effects: string[];
  constraints: string[];
  pddl_action: string;
  action_server_name: string;
  action_type: string;
  parameters_schema: string;
  is_compound: boolean;
  bt_xml: string;
  component_skills: string[];
  created_at: RosTime;
  updated_at: RosTime;
}

/** Matches rcl_interfaces/msg/Log */
export interface RosLog {
  stamp: RosTime;
  level: number;
  name: string;
  msg: string;
  file: string;
  function: string;
  line: number;
}

/** Log severity levels matching rcl_interfaces/msg/Log constants */
export const LOG_LEVELS = {
  DEBUG: 10,
  INFO: 20,
  WARN: 30,
  ERROR: 40,
  FATAL: 50,
} as const;

export type LogLevelName = keyof typeof LOG_LEVELS;

export function logLevelName(level: number): LogLevelName {
  if (level >= 50) return "FATAL";
  if (level >= 40) return "ERROR";
  if (level >= 30) return "WARN";
  if (level >= 20) return "INFO";
  return "DEBUG";
}

/** Matches diagnostic_msgs/msg/DiagnosticStatus */
export interface DiagnosticStatus {
  level: number; // 0=OK, 1=WARN, 2=ERROR, 3=STALE
  name: string;
  message: string;
  hardware_id: string;
  values: Array<{ key: string; value: string }>;
}

export const DIAG_LEVELS = {
  OK: 0,
  WARN: 1,
  ERROR: 2,
  STALE: 3,
} as const;

/** Matches diagnostic_msgs/msg/DiagnosticArray */
export interface DiagnosticArray {
  header: { stamp: RosTime; frame_id: string };
  status: DiagnosticStatus[];
}

/** Matches sensor_msgs/msg/JointState */
export interface JointState {
  header: { stamp: RosTime; frame_id: string };
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

/** Matches tf2_msgs/msg/TFMessage */
export interface TFMessage {
  transforms: TransformStamped[];
}

export interface TransformStamped {
  header: { stamp: RosTime; frame_id: string };
  child_frame_id: string;
  transform: {
    translation: { x: number; y: number; z: number };
    rotation: { x: number; y: number; z: number; w: number };
  };
}

/** ComposeTask service */
export interface TaskStep {
  skill_name: string;
  parameters_json: string;
  input_blackboard_keys: string[];
  output_blackboard_keys: string[];
  retry_on_failure: boolean;
  max_retries: number;
  condition_expression: string;
  description: string;
}

export interface ComposeTaskRequest {
  task_name: string;
  task_description: string;
  steps: TaskStep[];
  sequential: boolean;
  add_precondition_checks: boolean;
}

export interface ComposeTaskResponse {
  success: boolean;
  message: string;
  bt_xml: string;
  warnings: string[];
}

/** ExecuteBehaviorTree action goal */
export interface ExecuteBehaviorTreeGoal {
  tree_xml: string;
  tree_name: string;
  enable_groot_monitor: boolean;
  groot_zmq_port: number;
  tick_rate_hz: number;
}

/** GetSkillDescriptions service */
export interface GetSkillDescriptionsRequest {
  filter_categories: string[];
  filter_tags: string[];
  include_compounds: boolean;
  include_pddl: boolean;
}

export interface GetSkillDescriptionsResponse {
  skills: SkillDescription[];
  success: boolean;
  message: string;
}
