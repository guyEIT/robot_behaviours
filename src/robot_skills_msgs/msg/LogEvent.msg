# Structured event log for skill execution traceability
builtin_interfaces/Time stamp
string event_name                   # e.g. "grasp_started", "object_detected", "task_complete"
string severity                     # "info", "warn", "error"
string message                      # Human-readable description
string task_id                      # Associated task ID (if any)
string skill_name                   # Skill that generated the event (if any)
string[] tags                       # Free-form tags for filtering
