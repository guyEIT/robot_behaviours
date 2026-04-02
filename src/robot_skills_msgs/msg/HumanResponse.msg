# Published by the dashboard when a human responds to a prompt.
# Subscribe on /skill_server/human_responses

builtin_interfaces/Time stamp
string prompt_id                    # Must match HumanPrompt.prompt_id
bool accepted                       # confirm/task: true=confirmed, false=rejected
string value                        # For input: the value entered by the operator
