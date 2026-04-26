# Published on /skill_server/task_state to allow monitoring of current execution

string task_id                     # Unique task execution ID
string task_name                   # Human-readable task name
string status                      # "IDLE" | "RUNNING" | "SUCCESS" | "FAILURE" | "CANCELLED"

# Current execution
string current_skill               # Name of currently executing skill
string current_bt_node             # Current BT node name
float64 progress                   # 0.0-1.0 overall task progress

# History
string[] completed_skills          # Skills completed in this task
string[] failed_skills             # Skills that failed (if any)

# Timing
builtin_interfaces/Time started_at
builtin_interfaces/Time updated_at
float64 elapsed_sec

# Error info (populated on FAILURE)
string error_message
string error_skill                 # Which skill caused the failure
