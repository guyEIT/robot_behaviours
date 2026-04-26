"""Python skill atoms for the Robot Skills Framework.

The C++ atoms in this package wrap MoveIt for arm motion. The Python
atoms here cover skills that map cleanly onto Python ROS APIs — currently
``RecordRosbag`` and ``StopRecording`` driving ``rosbag2_py.Recorder``
directly (no subprocess).

The two surfaces are independent: each Python atom is its own action
server node, advertised by the per-robot proxy alongside the C++ atom
components. They share nothing other than the `robot_arm_skills` package
name.
"""
