"""Tests for the bb_operator sidecar service handlers.

ROS dependencies stubbed via sys.modules. We construct BbOperator instances
with Node.__init__ skipped so the handlers can be exercised against a real
PersistentStore on disk.

Run via: colcon test --packages-select robot_skill_server
"""

from __future__ import annotations

import os
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock


def _stub_ros_modules() -> None:
    for mod_name in [
        "rclpy",
        "rclpy.callback_groups",
        "rclpy.node",
        "rclpy.qos",
        "std_msgs",
        "std_msgs.msg",
        "robot_skills_msgs",
        "robot_skills_msgs.msg",
        "robot_skills_msgs.srv",
    ]:
        if mod_name not in sys.modules:
            sys.modules[mod_name] = types.ModuleType(mod_name)

    sys.modules["rclpy.qos"].DurabilityPolicy = MagicMock()
    sys.modules["rclpy.qos"].QoSProfile = MagicMock
    sys.modules["std_msgs.msg"].String = MagicMock

    class _Node:
        def __init__(self, *a, **kw):
            pass

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            clk = MagicMock()
            clk.now.return_value.to_msg.return_value = MagicMock()
            return clk

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def create_service(self, *a, **kw):
            return MagicMock()

    sys.modules["rclpy.node"].Node = _Node
    sys.modules["rclpy.callback_groups"].ReentrantCallbackGroup = MagicMock

    # Message stubs
    class _LogEvent:
        def __init__(self):
            self.stamp = None
            self.event_name = ""
            self.severity = ""
            self.message = ""
            self.tags = []
            self.task_id = ""

    class _TaskState:
        def __init__(self):
            self.task_id = ""
            self.task_name = ""
            self.status = ""

    sys.modules["robot_skills_msgs.msg"].LogEvent = _LogEvent
    sys.modules["robot_skills_msgs.msg"].TaskState = _TaskState

    # Service stubs — only `Request` / `Response` shapes matter to the handlers.
    def _make_srv(req_fields, resp_fields):
        class _Srv:
            class Request:
                def __init__(self):
                    for k, v in req_fields.items():
                        setattr(self, k, v)
            class Response:
                def __init__(self):
                    for k, v in resp_fields.items():
                        setattr(self, k, v)
        return _Srv

    AddPlate = _make_srv(
        {
            "plate_name": "",
            "barcode": "",
            "target_cycles": 0,
            "cadence_min": 0,
            "imaging_protocol": "",
            "liconic_slot": 0,
        },
        {"success": False, "message": "", "task_id": ""},
    )
    RetirePlate = _make_srv(
        {"plate_name": "", "reason": ""},
        {"success": False, "message": ""},
    )
    PauseCampaign = _make_srv(
        {"paused": False, "reason": ""},
        {"success": False, "message": ""},
    )
    OperatorDecision = _make_srv(
        {"node_path": "", "choice": "", "reason": ""},
        {"accepted": False, "message": ""},
    )

    sys.modules["robot_skills_msgs.srv"].AddPlate = AddPlate
    sys.modules["robot_skills_msgs.srv"].RetirePlate = RetirePlate
    sys.modules["robot_skills_msgs.srv"].PauseCampaign = PauseCampaign
    sys.modules["robot_skills_msgs.srv"].OperatorDecision = OperatorDecision


_stub_ros_modules()

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from robot_skill_server.bb_operator_node import BbOperator  # noqa: E402
from robot_skill_server.persistent_blackboard import (  # noqa: E402
    PersistentStore,
    task_db_path,
)
from robot_skills_msgs.srv import (  # noqa: E402
    AddPlate,
    OperatorDecision,
    PauseCampaign,
    RetirePlate,
)


class _FakeBbOperator(BbOperator):
    """BbOperator with Node.__init__ bypassed; we wire just the attributes
    the handlers actually read."""

    def __init__(self, task_id: str):
        self._current_task_id = task_id
        self._current_task_status = "RUNNING"
        self._log_pub = MagicMock()
        # Stub the latched persistent-state publisher used by republish().
        self._persistent_state_pub = MagicMock()
        self._last_state_json = ""

    def get_clock(self):
        clk = MagicMock()
        clk.now.return_value.to_msg.return_value = MagicMock()
        return clk

    def get_logger(self):
        return MagicMock()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestNoActiveTaskRefuses(unittest.TestCase):
    """Every service must return success=False / accepted=False when no
    active task is registered."""

    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self._prev_xdg = os.environ.get("XDG_STATE_HOME")
        os.environ["XDG_STATE_HOME"] = self._tmpdir.name

    def tearDown(self):
        if self._prev_xdg is None:
            os.environ.pop("XDG_STATE_HOME", None)
        else:
            os.environ["XDG_STATE_HOME"] = self._prev_xdg
        self._tmpdir.cleanup()

    def test_add_plate_no_task(self):
        op = _FakeBbOperator(task_id=None)
        req = AddPlate.Request()
        req.plate_name = "P1"
        resp = op._on_add_plate(req, AddPlate.Response())
        self.assertFalse(resp.success)
        self.assertIn("active task", resp.message)

    def test_pause_no_task(self):
        op = _FakeBbOperator(task_id=None)
        req = PauseCampaign.Request()
        req.paused = True
        req.reason = "test"
        resp = op._on_pause(req, PauseCampaign.Response())
        self.assertFalse(resp.success)


class TestServiceHandlers(unittest.TestCase):
    """Service handlers mutate the persistent blackboard correctly."""

    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self._prev_xdg = os.environ.get("XDG_STATE_HOME")
        os.environ["XDG_STATE_HOME"] = self._tmpdir.name
        self.task_id = "ttestabc"
        # Pre-create the task DB so _open_active_store can find it.
        self.store = PersistentStore(task_db_path(self.task_id))
        self.op = _FakeBbOperator(task_id=self.task_id)

    def tearDown(self):
        self.store.close()
        if self._prev_xdg is None:
            os.environ.pop("XDG_STATE_HOME", None)
        else:
            os.environ["XDG_STATE_HOME"] = self._prev_xdg
        self._tmpdir.cleanup()

    def _reopen(self) -> PersistentStore:
        return PersistentStore(task_db_path(self.task_id))

    def test_add_plate_appends_to_queue_and_index(self):
        req = AddPlate.Request()
        req.plate_name = "P1"
        req.target_cycles = 5
        req.cadence_min = 30
        req.imaging_protocol = "brightfield_3site_v1"
        resp = self.op._on_add_plate(req, AddPlate.Response())
        self.assertTrue(resp.success)
        self.assertEqual(resp.task_id, self.task_id)

        s = self._reopen()
        try:
            queue = s.get_kv("persistent.plate_queue")
            self.assertEqual(len(queue), 1)
            self.assertEqual(queue[0]["name"], "P1")
            self.assertEqual(queue[0]["target_cycles"], 5)
            self.assertEqual(queue[0]["cadence_min"], 30)
            self.assertEqual(queue[0]["imaging_protocol"], "brightfield_3site_v1")
            self.assertEqual(queue[0]["cycle"], 0)
            self.assertFalse(queue[0]["retiring"])
            # Indexed by name too
            plates = s.get_kv("persistent.plates")
            self.assertIn("P1", plates)
        finally:
            s.close()

    def test_retire_plate_marks_index_and_queue(self):
        # Pre-seed a plate via AddPlate, then retire.
        req = AddPlate.Request()
        req.plate_name = "P1"
        self.op._on_add_plate(req, AddPlate.Response())

        retire = RetirePlate.Request()
        retire.plate_name = "P1"
        retire.reason = "operator stopped experiment"
        resp = self.op._on_retire_plate(retire, RetirePlate.Response())
        self.assertTrue(resp.success)

        s = self._reopen()
        try:
            plates = s.get_kv("persistent.plates")
            self.assertTrue(plates["P1"]["retiring"])
            self.assertEqual(plates["P1"]["retire_reason"],
                             "operator stopped experiment")
            queue = s.get_kv("persistent.plate_queue")
            self.assertTrue(queue[0]["retiring"])
        finally:
            s.close()

    def test_retire_unknown_plate_fails(self):
        req = RetirePlate.Request()
        req.plate_name = "Unknown"
        resp = self.op._on_retire_plate(req, RetirePlate.Response())
        self.assertFalse(resp.success)
        self.assertIn("unknown plate", resp.message)

    def test_pause_and_resume_toggle(self):
        # Pause
        req = PauseCampaign.Request()
        req.paused = True
        req.reason = "tip refill"
        resp = self.op._on_pause(req, PauseCampaign.Response())
        self.assertTrue(resp.success)
        s = self._reopen()
        try:
            self.assertTrue(s.get_kv("persistent.paused"))
            self.assertEqual(s.get_kv("persistent.pause_reason"), "tip refill")
        finally:
            s.close()

        # Resume
        req = PauseCampaign.Request()
        req.paused = False
        req.reason = "done refilling"
        self.op._on_pause(req, PauseCampaign.Response())
        s = self._reopen()
        try:
            self.assertFalse(s.get_kv("persistent.paused"))
        finally:
            s.close()

    def test_operator_decision_records_choice(self):
        req = OperatorDecision.Request()
        req.node_path = "/X/Y/Z"
        req.choice = "skip-as-success"
        req.reason = "iSWAP repositioned plate manually"
        resp = self.op._on_operator_decision(req, OperatorDecision.Response())
        self.assertTrue(resp.accepted)
        s = self._reopen()
        try:
            decisions = s.get_kv("persistent.operator_decisions")
            self.assertIn("/X/Y/Z", decisions)
            self.assertEqual(decisions["/X/Y/Z"]["choice"], "skip-as-success")
        finally:
            s.close()

    def test_operator_decision_rejects_invalid_choice(self):
        req = OperatorDecision.Request()
        req.node_path = "/X"
        req.choice = "bogus-choice"
        req.reason = ""
        resp = self.op._on_operator_decision(req, OperatorDecision.Response())
        self.assertFalse(resp.accepted)
        self.assertIn("must be one of", resp.message)


if __name__ == "__main__":
    unittest.main()
