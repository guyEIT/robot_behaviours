"""In-process registry for active rosbag2 recorders.

Both ``RecordRosbag`` (start) and ``StopRecording`` (stop) atoms run in
the same Python process — composed by the per-robot proxy launch — so
they share this module-level registry to look up active recorders by
``bag_path``.

Threading: the recorder runs ``Recorder.record(...)`` on its own daemon
thread; the registry mutex protects the dict against concurrent
register/lookup/remove.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

from rosbag2_py import Recorder


@dataclass
class _Entry:
    recorder: Recorder
    thread: threading.Thread


_lock = threading.RLock()
_recorders: dict[str, _Entry] = {}


def register(bag_path: str, recorder: Recorder, thread: threading.Thread) -> None:
    with _lock:
        _recorders[bag_path] = _Entry(recorder=recorder, thread=thread)


def pop(bag_path: str) -> _Entry | None:
    with _lock:
        return _recorders.pop(bag_path, None)


def get(bag_path: str) -> _Entry | None:
    with _lock:
        return _recorders.get(bag_path)


def list_bags() -> list[str]:
    with _lock:
        return list(_recorders.keys())
