# A single step in a task composition request

string skill_name                  # Name of the skill to execute
string parameters_json             # JSON object with goal parameters, e.g. '{"config_name": "home"}'
string[] input_blackboard_keys     # BT blackboard keys to read as inputs
string[] output_blackboard_keys    # BT blackboard keys to write as outputs
bool retry_on_failure              # Whether to retry if this step fails
int32 max_retries                  # Max retry attempts (used if retry_on_failure=true)
string condition_expression        # Optional: only execute if this condition is met
string description                 # Human-readable step description
