# Discrete event from the action server (state transition, tip pickup, error, etc.).
std_msgs/Header header

uint8 INFO=0
uint8 WARN=1
uint8 ERROR=2
uint8 severity

# Event kinds the node publishes. See MachineFSM and action_server.py.
string kind   # e.g. "fsm_transition", "tip_picked_up", "tip_dropped",
              # "core_gripper_picked_up", "core_gripper_returned",
              # "abort_motion", "error", "reset_error", "deck_loaded"
string message
string[] keys
string[] values   # parallel with keys; simple kv detail
