# Latched on /skill_server/dryrun_status. Published when a SIM_THEN_REAL goal
# completes its sim phase and is parked in PHASE_AWAITING_APPROVAL. Subscribers
# (dashboard, MCP, autonomy) consume this to surface an approval prompt and
# then call /skill_server/approve_dry_run to release the gate.
#
# An empty task_id ("") means no goal is currently awaiting approval. A fresh
# DryRunStatus with a non-empty task_id supersedes any previous one.

string task_id                     # Task ID of the goal awaiting approval
string tree_name                   # Human-readable label (echoed from the goal)
string sim_status                  # "SUCCESS" | "FAILURE" | "HALTED" | "ERROR"
string message                     # Diagnostic from the sim phase
float64 sim_duration_sec           # Wall-clock time of the sim phase
string[] completed_skills          # Skills completed during sim
string[] failed_skills             # Skills that failed during sim (if any)
builtin_interfaces/Time published_at
