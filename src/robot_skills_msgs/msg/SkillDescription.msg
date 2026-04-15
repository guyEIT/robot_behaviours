# Full self-description of a robot skill for agent introspection and task planning

# Identity
string name                        # Unique identifier, e.g. "move_to_named_config"
string display_name                # Human-readable, e.g. "Move to Named Configuration"
string description                 # Natural language description for LLM agents
string version                     # Semantic version, e.g. "1.0.0"
string robot_id                    # Robot this skill belongs to, e.g. "meca500" or "franka"

# Classification
string category                    # "motion" | "perception" | "manipulation" | "utility" | "compound"
string[] tags                      # Free-form tags for filtering

# Planning metadata (for PDDL/HTN planners)
string[] preconditions             # What must be true before executing, e.g. ["robot_initialized", "gripper_open"]
string[] postconditions            # What is guaranteed true after success, e.g. ["robot_at_named_config"]
string[] effects                   # PDDL-style effect strings, e.g. ["(at ?robot ?config)"]
string[] constraints               # Safety/operational constraints, e.g. ["velocity_scaling <= 0.5 when humans_nearby"]
string pddl_action                 # Full PDDL action fragment (domain snippet)

# Interface
string action_server_name          # ROS2 action server topic, e.g. "/skill_atoms/move_to_named_config"
string action_type                 # Full action type string, e.g. "robot_skills_msgs/action/MoveToNamedConfig"
string parameters_schema           # JSON Schema string describing goal parameters

# Compound skill info
bool is_compound                   # True if this skill is composed from atoms
string bt_xml                      # BT XML definition (populated if is_compound=true)
string[] component_skills          # Names of constituent skills (if compound)

# Metadata
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
