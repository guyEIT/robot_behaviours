# Published by BT nodes to request human interaction.
# Subscribe on /skill_server/human_prompts

builtin_interfaces/Time stamp
string prompt_id                    # Unique ID for correlating prompt/response
string prompt_type                  # "notification" | "warning" | "confirm" | "input" | "task"
string title                        # Short title
string message                      # Detailed message body
string severity                     # "info" | "warning" | "error" | "critical"
string input_type                   # For input prompts: "text" | "number" | "choice"
string[] choices                    # For choice-type inputs: list of options
string default_value                # Default value for input prompts
float64 timeout_sec                 # 0 = no timeout; >0 = auto-fail after N seconds
string task_id                      # Associated task execution ID
string bt_node_name                 # Name of the BT node that issued this prompt
