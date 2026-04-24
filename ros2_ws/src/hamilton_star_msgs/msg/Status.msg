# Full operational status of the Hamilton STAR action server node.
std_msgs/Header header

# Top-level FSM state (mirrors MachineFSM): "Disconnected", "Initializing",
# "Idle", "BusyShared", "BusyExclusive", "Aborting", "Error".
string op_state

bool connected
bool initialized
string error_reason  # empty unless op_state == "Error"

# Deck
bool deck_loaded
string deck_hash  # sha256 of the serialized deck, empty when no deck

# Channels
int32 num_channels
ChannelState[] channels
int32[] busy_channels

# CoRe II gripper
bool core_parked
int32[] core_channels  # the two channels that hold the gripper when not parked

# 96-head
bool core96_loaded

# What's currently running
string exclusive_op        # name of the in-flight exclusive goal, empty if none
int32 shared_goal_count    # number of in-flight non-exclusive goals

# iSWAP substate (all default false / 0 until setup completes).
# See machine_fsm.py for the transition rules.
bool iswap_initialized
bool iswap_y_path_clear            # true => C0 FY has run since last channel move
bool iswap_holding_plate           # true between handoff stages 3 and 5
bool iswap_drive_faulted           # true after an iSWAP-sourced firmware exception
bool iswap_at_safe_pose            # true when arm is retracted (post-park / post-init)
int32 last_handoff_stage_reached   # 0 = none/reset; 1-6 = stages of transfer()
