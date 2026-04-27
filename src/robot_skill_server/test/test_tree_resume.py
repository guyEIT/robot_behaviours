"""Integration tests for tree-executor resume semantics.

Mirrors the test_skill_registry / test_task_composer pattern: stubs rclpy and
ROS msgs via sys.modules so we can import tree_executor without a live ROS
system. Then drives small synthetic trees through asyncio to validate:

  - Sequence advances are persisted between runs (no double execution).
  - RepeatNode preserves iteration counter across simulated crashes.
  - RosActionNode caches its terminal result so a crash AFTER success but
    BEFORE the parent advances doesn't re-submit.
  - PopFromQueue / PushToQueue round-trip via the persistent blackboard.
  - WaitUntil survives a restart (woke_at was in the past on resume).

Run via: colcon test --packages-select robot_skill_server
"""

from __future__ import annotations

import asyncio
import os
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Stubs — must precede tree_executor import.
# ---------------------------------------------------------------------------
def _stub_ros_modules() -> None:
    for mod_name in [
        "rclpy",
        "rclpy.action",
        "rclpy.node",
        "rclpy.task",
        "rclpy.time",
        "geometry_msgs",
        "geometry_msgs.msg",
        "robot_skills_msgs",
        "robot_skills_msgs.msg",
        "robot_skills_msgs.srv",
    ]:
        if mod_name not in sys.modules:
            sys.modules[mod_name] = types.ModuleType(mod_name)

    sys.modules["rclpy.action"].ActionClient = MagicMock
    sys.modules["rclpy.node"].Node = MagicMock

    class _PoseStamped:
        def __init__(self):
            self.header = MagicMock()
            self.pose = MagicMock()

    sys.modules["geometry_msgs.msg"].PoseStamped = _PoseStamped

    for name in (
        "HumanPrompt", "HumanResponse", "LogEvent", "TaskState",
    ):
        setattr(sys.modules["robot_skills_msgs.msg"], name, MagicMock)
    for name in ("AcquireLease", "ReleaseLease", "RenewLease"):
        setattr(sys.modules["robot_skills_msgs.srv"], name, MagicMock)


_stub_ros_modules()

# Make the package importable when running from the repo root or under colcon.
sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..")
    ),
)

from robot_skill_server import tree_executor as te  # noqa: E402
from robot_skill_server.persistent_blackboard import (  # noqa: E402
    PersistentStore,
)


# ---------------------------------------------------------------------------
# Test scaffolding
# ---------------------------------------------------------------------------

class CountingLeaf(te.TreeNode):
    """Sync-style leaf that records every tick. Optional crash/fail triggers
    let tests reproduce mid-tree-failure scenarios.

    Counts are CLASS-level so tests can reset them centrally.
    """

    counts: dict = {}
    raise_on_count: dict = {}
    fail_on_count: dict = {}

    @classmethod
    def reset(cls):
        cls.counts.clear()
        cls.raise_on_count.clear()
        cls.fail_on_count.clear()

    async def tick(self, bb, ctx):
        n = self.name
        CountingLeaf.counts[n] = CountingLeaf.counts.get(n, 0) + 1
        c = CountingLeaf.counts[n]
        if CountingLeaf.raise_on_count.get(n) == c:
            raise RuntimeError(f"{n} simulated crash on tick #{c}")
        if CountingLeaf.fail_on_count.get(n) == c:
            return te.NodeStatus.FAILURE
        return te.NodeStatus.SUCCESS


class FakeContext:
    """Minimal ExecutionContext stand-in — none of the ROS-publishing methods
    are exercised by the nodes under test (control flow, sync leaves, queue
    ops, WaitUntil with stub sleep). Anything else would need a real Node."""

    def __init__(self, store):
        self.persistent_store = store
        self.cancelled = False
        self.abort_requested = False
        self.halt_reason = ""
        self.paused = False
        self.pause_reason = ""
        self.ros_node = MagicMock()

    def is_halted(self):
        return self.cancelled or self.abort_requested

    def is_paused(self):
        return self.paused

    def on_skill_started(self, _name):
        pass

    def on_skill_completed(self, _name):
        pass

    def on_skill_failed(self, _name):
        pass

    def log(self, *_a, **_kw):
        pass

    def publish_log_event(self, *_a, **_kw):
        pass


def _make_tree(root: te.TreeNode) -> te.TreeNode:
    """Assign node paths and return the root for ticking."""
    root.node_path = "/__test__"
    te._assign_node_paths(root, root.node_path)
    return root


def _run(coro):
    return asyncio.get_event_loop().run_until_complete(coro)


def _new_store(tmp_path: Path, name: str = "task") -> PersistentStore:
    return PersistentStore(tmp_path / name / "state.db")


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSequenceResume(unittest.TestCase):
    def setUp(self):
        CountingLeaf.reset()
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def _build(self):
        seq = te.SequenceNode("seq", {})
        seq.children = [
            CountingLeaf("A", {}),
            CountingLeaf("B", {}),
            CountingLeaf("C", {}),
            CountingLeaf("D", {}),
        ]
        return _make_tree(seq)

    def test_sequence_resumes_at_crash_point(self):
        """Crash mid-sequence; resume picks up at the crashing child, no
        re-execution of earlier children."""
        # Crash on B's first tick.
        CountingLeaf.raise_on_count["B"] = 1

        store = _new_store(self.tmp)
        try:
            tree = self._build()
            bb = te.Blackboard(persistent_store=store)
            ctx = FakeContext(store)
            with self.assertRaises(RuntimeError):
                _run(tree.tick(bb, ctx))
            # A executed once and Sequence advanced past it.
            self.assertEqual(CountingLeaf.counts.get("A"), 1)
            self.assertEqual(CountingLeaf.counts.get("B"), 1)
            self.assertNotIn("C", CountingLeaf.counts)
        finally:
            store.close()

        # Simulate process restart: re-open store, rebuild tree, clear crash.
        CountingLeaf.raise_on_count.clear()
        store = _new_store(self.tmp)
        try:
            tree = self._build()
            bb = te.Blackboard(persistent_store=store)
            ctx = FakeContext(store)
            status = _run(tree.tick(bb, ctx))
            self.assertEqual(status, te.NodeStatus.SUCCESS)
        finally:
            store.close()

        # A did not re-execute. B re-ran and then C/D ran.
        self.assertEqual(CountingLeaf.counts.get("A"), 1)
        self.assertEqual(CountingLeaf.counts.get("B"), 2)
        self.assertEqual(CountingLeaf.counts.get("C"), 1)
        self.assertEqual(CountingLeaf.counts.get("D"), 1)


class TestRepeatResume(unittest.TestCase):
    def setUp(self):
        CountingLeaf.reset()
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def _build(self, num_cycles=3):
        rep = te.RepeatNode("rep", {"num_cycles": str(num_cycles)})
        rep.children = [CountingLeaf("X", {})]
        return _make_tree(rep)

    def test_repeat_preserves_iter_across_restart(self):
        """Crash on the 2nd iteration; resume should run only the remaining
        iterations, never replaying the ones already completed."""
        CountingLeaf.raise_on_count["X"] = 2

        store = _new_store(self.tmp)
        try:
            tree = self._build(num_cycles=3)
            bb = te.Blackboard(persistent_store=store)
            ctx = FakeContext(store)
            with self.assertRaises(RuntimeError):
                _run(tree.tick(bb, ctx))
            self.assertEqual(CountingLeaf.counts["X"], 2)
        finally:
            store.close()

        CountingLeaf.raise_on_count.clear()
        store = _new_store(self.tmp)
        try:
            tree = self._build(num_cycles=3)
            bb = te.Blackboard(persistent_store=store)
            ctx = FakeContext(store)
            status = _run(tree.tick(bb, ctx))
            self.assertEqual(status, te.NodeStatus.SUCCESS)
        finally:
            store.close()

        # First run: 1 success + 1 crash = count 2. Second run: 2 successes
        # to finish (iter 1 retried + iter 2 ran fresh) = count 4 total.
        self.assertEqual(CountingLeaf.counts["X"], 4)


class TestSequenceClearsChildOnAdvance(unittest.TestCase):
    """When a Sequence advances past child[i], child[i]'s persistent state must
    be wiped so a future re-invocation (e.g., inside a Repeat loop) re-executes
    the child instead of short-circuiting on stale cached state."""

    def setUp(self):
        CountingLeaf.reset()
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_repeat_of_sequence_re_executes_each_iteration(self):
        # Tree: Repeat(2) > Sequence(A, B)
        rep = te.RepeatNode("rep", {"num_cycles": "2"})
        seq = te.SequenceNode("seq", {})
        seq.children = [CountingLeaf("A", {}), CountingLeaf("B", {})]
        rep.children = [seq]
        tree = _make_tree(rep)

        store = _new_store(self.tmp)
        try:
            bb = te.Blackboard(persistent_store=store)
            ctx = FakeContext(store)
            status = _run(tree.tick(bb, ctx))
            self.assertEqual(status, te.NodeStatus.SUCCESS)
        finally:
            store.close()

        # Each child must have run twice — once per Repeat iteration.
        self.assertEqual(CountingLeaf.counts["A"], 2)
        self.assertEqual(CountingLeaf.counts["B"], 2)


class TestPersistentBlackboardThroughBlackboard(unittest.TestCase):
    """The Blackboard wrapper must mirror persistent.* keys to SQLite and
    refuse non-JSON values without corrupting in-memory state."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.store = PersistentStore(self.tmp / "state.db")

    def tearDown(self):
        self.store.close()
        self._tmp.cleanup()

    def test_persistent_key_mirrored(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.q", [1, 2, 3])
        # Reopen: a fresh blackboard hydrates from disk.
        bb2 = te.Blackboard(persistent_store=self.store)
        self.assertEqual(bb2.get("persistent.q"), [1, 2, 3])

    def test_transient_key_not_mirrored(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("ephemeral", {"x": 1})
        bb2 = te.Blackboard(persistent_store=self.store)
        self.assertIsNone(bb2.get("ephemeral"))

    def test_non_json_persistent_raises(self):
        bb = te.Blackboard(persistent_store=self.store)
        from robot_skill_server.persistent_blackboard import PersistenceError
        with self.assertRaises(PersistenceError):
            bb.set("persistent.bad", object())

    def test_read_through_picks_up_external_writes(self):
        """When the bb_operator sidecar (or any other writer with the same
        DB path) updates a persistent.* key, a Blackboard already in use
        by the running tree must see the update on its next read."""
        bb = te.Blackboard(persistent_store=self.store)
        # Read once → empty
        self.assertEqual(bb.get("persistent.plate_queue", []), [])
        # External writer (simulating bb_operator)
        with PersistentStore(self.tmp / "state.db") as external:
            external.set_kv(
                "persistent.plate_queue",
                [{"name": "P1", "cycle": 0}],
            )
        # Same Blackboard reads again → sees the new value (no eager cache)
        self.assertEqual(
            bb.get("persistent.plate_queue"),
            [{"name": "P1", "cycle": 0}],
        )


class TestDottedPathResolve(unittest.TestCase):
    """`resolve()` walks `{a.b.c}` into nested dict-valued blackboard keys."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.store = PersistentStore(self.tmp / "state.db")

    def tearDown(self):
        self.store.close()
        self._tmp.cleanup()

    def test_flat_key_unchanged(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("simple", 42)
        self.assertEqual(bb.resolve("{simple}"), 42)

    def test_dotted_into_persistent_dict(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.current_plate", {
            "name": "P1",
            "next_due_at": 1234567890.0,
            "metadata": {"barcode": "abc"},
        })
        self.assertEqual(
            bb.resolve("{persistent.current_plate.name}"), "P1"
        )
        self.assertEqual(
            bb.resolve("{persistent.current_plate.next_due_at}"),
            1234567890.0,
        )
        self.assertEqual(
            bb.resolve("{persistent.current_plate.metadata.barcode}"),
            "abc",
        )

    def test_missing_segment_returns_none(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.current_plate", {"name": "P1"})
        self.assertIsNone(
            bb.resolve("{persistent.current_plate.missing_field}")
        )
        self.assertIsNone(bb.resolve("{persistent.no_such_key.x}"))

    def test_literal_key_with_dots_wins(self):
        """If a literal flat key with dots in it exists, prefer it over
        treating the dots as path separators (back-compat)."""
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.flat.dotted.key", "literal")
        self.assertEqual(
            bb.resolve("{persistent.flat.dotted.key}"), "literal"
        )


class TestQueueOps(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.store = PersistentStore(self.tmp / "state.db")

    def tearDown(self):
        self.store.close()
        self._tmp.cleanup()

    def test_pop_failure_when_empty(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.q", [])
        node = te.PopFromQueueNode(
            "pop", {"key": "persistent.q", "output": "{persistent.head}"}
        )
        node.node_path = "/p"
        ctx = FakeContext(self.store)
        status = _run(node.tick(bb, ctx))
        self.assertEqual(status, te.NodeStatus.FAILURE)

    def test_pop_returns_head_and_persists_tail(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.q", [{"name": "P1"}, {"name": "P2"}])
        node = te.PopFromQueueNode(
            "pop", {"key": "persistent.q", "output": "{persistent.head}"}
        )
        node.node_path = "/p"
        ctx = FakeContext(self.store)
        status = _run(node.tick(bb, ctx))
        self.assertEqual(status, te.NodeStatus.SUCCESS)
        self.assertEqual(bb.get("persistent.head"), {"name": "P1"})
        # Persistent tail visible after restart.
        bb2 = te.Blackboard(persistent_store=self.store)
        self.assertEqual(bb2.get("persistent.q"), [{"name": "P2"}])

    def test_push_appends(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.q", [])
        node = te.PushToQueueNode("push", {"key": "persistent.q", "value": "P1"})
        node.node_path = "/q"
        ctx = FakeContext(self.store)
        _run(node.tick(bb, ctx))
        _run(te.PushToQueueNode(
            "push2", {"key": "persistent.q", "value": "P2"}
        ).tick(bb, ctx))
        self.assertEqual(bb.get("persistent.q"), ["P1", "P2"])


class TestWaitUntilResume(unittest.TestCase):
    """A WaitUntil whose deadline already passed at resume time must return
    SUCCESS immediately, never re-arm a fresh sleep."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_already_due_returns_immediately(self):
        # Pre-seed a WaitUntil node's checkpoint with a past wake_at.
        store = PersistentStore(self.tmp / "state.db")
        try:
            past = time.time() - 60
            store.save_node_state("/__t__/WaitUntilNode", "WaitUntilNode",
                                  {"wake_at": past})
            node = te.WaitUntilNode("w", {"timestamp": str(past)})
            node.node_path = "/__t__/WaitUntilNode"
            bb = te.Blackboard(persistent_store=store)
            ctx = FakeContext(store)
            t0 = time.time()
            status = _run(node.tick(bb, ctx))
            elapsed = time.time() - t0
            self.assertEqual(status, te.NodeStatus.SUCCESS)
            self.assertLess(elapsed, 0.5,
                            "WaitUntil with past deadline must return immediately")
            # State cleared after success.
            self.assertIsNone(store.load_node_state("/__t__/WaitUntilNode"))
        finally:
            store.close()


class TestBlackboardCondition(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.store = PersistentStore(self.tmp / "state.db")
        CountingLeaf.reset()

    def tearDown(self):
        self.store.close()
        self._tmp.cleanup()

    def _build_gate(self):
        gate = te.BlackboardConditionNode(
            "gate", {"key": "persistent.paused", "expected": "false"}
        )
        gate.children = [CountingLeaf("inner", {})]
        return _make_tree(gate)

    def test_gate_blocks_when_condition_fails(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.paused", True)
        ctx = FakeContext(self.store)
        tree = self._build_gate()
        status = _run(tree.tick(bb, ctx))
        self.assertEqual(status, te.NodeStatus.FAILURE)
        self.assertNotIn("inner", CountingLeaf.counts)

    def test_gate_admits_when_condition_holds(self):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.paused", False)
        ctx = FakeContext(self.store)
        tree = self._build_gate()
        status = _run(tree.tick(bb, ctx))
        self.assertEqual(status, te.NodeStatus.SUCCESS)
        self.assertEqual(CountingLeaf.counts.get("inner"), 1)

    def test_gate_admits_when_key_unset(self):
        """An operator-driven `persistent.paused` flag is unset until the
        first Pause service call. The gate must treat that as falsy so the
        campaign loop runs from the start, not refuse-until-touched."""
        bb = te.Blackboard(persistent_store=self.store)
        # `persistent.paused` was never written — Blackboard.get returns None.
        ctx = FakeContext(self.store)
        tree = self._build_gate()
        status = _run(tree.tick(bb, ctx))
        self.assertEqual(status, te.NodeStatus.SUCCESS)
        self.assertEqual(CountingLeaf.counts.get("inner"), 1)


class TestAdvancePlate(unittest.TestCase):
    """AdvancePlateNode: post-cycle bookkeeping.

    Increments cycle, computes next_due_at, re-queues unless retiring or
    target reached. Mirrors per-name index for dashboard.
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.store = PersistentStore(self.tmp / "state.db")

    def tearDown(self):
        self.store.close()
        self._tmp.cleanup()

    def _run(self, plate):
        bb = te.Blackboard(persistent_store=self.store)
        bb.set("persistent.current_plate", plate)
        bb.set("persistent.plate_queue", [])
        bb.set("persistent.plates", {plate["name"]: plate})
        node = te.AdvancePlateNode("adv", {})
        node.node_path = "/adv"
        ctx = FakeContext(self.store)
        # Stub publish_log_event used by the node.
        ctx.publish_log_event = lambda *a, **kw: None
        status = _run(node.tick(bb, ctx))
        return status, bb

    def test_normal_advance_requeues(self):
        plate = {
            "name": "P1",
            "cycle": 0,
            "cadence_min": 5,
            "target_cycles": 10,
            "retiring": False,
        }
        status, bb = self._run(plate)
        self.assertEqual(status, te.NodeStatus.SUCCESS)
        queue = bb.get("persistent.plate_queue")
        self.assertEqual(len(queue), 1)
        self.assertEqual(queue[0]["name"], "P1")
        self.assertEqual(queue[0]["cycle"], 1)
        self.assertGreater(queue[0]["next_due_at"], time.time())
        # Per-name index updated too.
        plates = bb.get("persistent.plates")
        self.assertEqual(plates["P1"]["cycle"], 1)

    def test_retiring_does_not_requeue(self):
        plate = {
            "name": "P1", "cycle": 4, "cadence_min": 5,
            "target_cycles": 10, "retiring": True,
        }
        status, bb = self._run(plate)
        self.assertEqual(status, te.NodeStatus.SUCCESS)
        self.assertEqual(bb.get("persistent.plate_queue"), [])
        # But the per-name index still gets the cycle bump.
        self.assertEqual(bb.get("persistent.plates")["P1"]["cycle"], 5)

    def test_target_reached_does_not_requeue(self):
        plate = {
            "name": "P1", "cycle": 9, "cadence_min": 5,
            "target_cycles": 10, "retiring": False,
        }
        status, bb = self._run(plate)
        self.assertEqual(status, te.NodeStatus.SUCCESS)
        self.assertEqual(bb.get("persistent.plate_queue"), [])
        self.assertEqual(bb.get("persistent.plates")["P1"]["cycle"], 10)

    def test_missing_current_plate_fails(self):
        bb = te.Blackboard(persistent_store=self.store)
        node = te.AdvancePlateNode("adv", {})
        node.node_path = "/adv"
        ctx = FakeContext(self.store)
        ctx.publish_log_event = lambda *a, **kw: None
        status = _run(node.tick(bb, ctx))
        self.assertEqual(status, te.NodeStatus.FAILURE)


class TestInflightReconciliation(unittest.TestCase):
    """RosActionNode resume reconciliation: an inflight row from a prior run
    should make idempotent actions auto-resubmit and non-idempotent actions
    refuse to re-execute (return FAILURE)."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.store = PersistentStore(self.tmp / "state.db")

    def tearDown(self):
        self.store.close()
        self._tmp.cleanup()

    def _make_node(self, idempotent: bool):
        # Bare RosActionNode — we only exercise the resume preamble before
        # any actual action client work.
        node = te.RosActionNode(
            name="X",
            attrs={},
            action_type=MagicMock(),
            server_name="/srv",
            input_map={},
            output_map={},
            idempotent=idempotent,
        )
        node.node_path = "/X"
        return node

    def test_non_idempotent_refuses_to_resubmit(self):
        # Pre-seed an inflight row from a "prior run".
        self.store.record_inflight("/X", "/srv", "T", "uid-old", False)
        node = self._make_node(idempotent=False)
        bb = te.Blackboard(persistent_store=self.store)
        ctx = FakeContext(self.store)
        # The RosActionNode tick path needs a logger via ros_node, plus the
        # short-circuit happens before any client lookup.
        logger = MagicMock()
        ctx.ros_node = MagicMock()
        ctx.ros_node.get_logger.return_value = logger
        status = _run(node.tick(bb, ctx))
        self.assertEqual(status, te.NodeStatus.FAILURE)
        # Inflight row preserved so operator can decide.
        self.assertIsNotNone(self.store.get_inflight("/X"))

    def test_idempotent_clears_inflight_and_continues(self):
        # Pre-seed an idempotent inflight row.
        self.store.record_inflight("/X", "/srv", "T", "uid-old", True)
        node = self._make_node(idempotent=True)
        bb = te.Blackboard(persistent_store=self.store)
        ctx = FakeContext(self.store)
        logger = MagicMock()
        ctx.ros_node = MagicMock()
        ctx.ros_node.get_logger.return_value = logger

        # We can't run the full tick (would need a real action client), but
        # the resume-clear happens before the client lookup. Force the
        # subsequent client lookup to fail fast so we observe the row was
        # cleared during the idempotent branch.
        client = MagicMock()
        client.wait_for_server.return_value = False  # surfaces as FAILURE
        node._client = client

        status = _run(node.tick(bb, ctx))
        # Server-unavailable path returns FAILURE — that's fine; what we care
        # about is that the OLD inflight row is gone.
        self.assertEqual(status, te.NodeStatus.FAILURE)
        self.assertIsNone(self.store.get_inflight("/X"))


class TestNodePaths(unittest.TestCase):
    def test_assign_node_paths_unique(self):
        seq = te.SequenceNode("outer", {})
        seq.children = [
            te.SequenceNode("inner1", {}),
            te.FallbackNode("inner2", {}),
        ]
        seq.children[0].children = [CountingLeaf("a", {}), CountingLeaf("b", {})]
        seq.children[1].children = [CountingLeaf("c", {})]
        _make_tree(seq)
        paths = []
        def walk(n):
            paths.append(n.node_path)
            for c in n.children:
                walk(c)
        walk(seq)
        self.assertEqual(len(paths), len(set(paths)),
                         f"duplicate node paths: {paths}")
        for p in paths:
            self.assertTrue(p.startswith("/__test__"))


if __name__ == "__main__":
    unittest.main()
