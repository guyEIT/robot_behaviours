"""In-process registry for active rosbag2 recorder subprocesses.

Both ``RecordRosbag`` (start) and ``StopRecording`` (stop) atoms run in
the same Python process — composed by the per-robot proxy launch — so
they share this module-level registry to look up active recorder PIDs
by ``bag_path``.

Why subprocess and not ``rosbag2_py.Recorder()`` in-process: the C++
``Recorder`` internally calls ``rclcpp::init()`` which fails the second
time it's invoked from inside an already-initialized rclpy interpreter
("context is already initialized"). Each ``ros2 bag record`` subprocess
gets its own ROS context, so back-to-back start/stop cycles work.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from subprocess import Popen


@dataclass
class _Entry:
    process: Popen


_lock = threading.RLock()
_recorders: dict[str, _Entry] = {}


def register(bag_path: str, process: Popen) -> None:
    with _lock:
        _recorders[bag_path] = _Entry(process=process)


def pop(bag_path: str) -> _Entry | None:
    with _lock:
        return _recorders.pop(bag_path, None)


def get(bag_path: str) -> _Entry | None:
    with _lock:
        return _recorders.get(bag_path)


def list_bags() -> list[str]:
    with _lock:
        return list(_recorders.keys())
