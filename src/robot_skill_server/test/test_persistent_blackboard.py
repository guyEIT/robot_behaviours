"""Unit tests for the persistent blackboard layer.

Pure stdlib — no ROS imports.
Run via: colcon test --packages-select robot_skill_server
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

# Make the package importable when running these tests outside colcon
sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..")
    ),
)

from robot_skill_server.persistent_blackboard import (  # noqa: E402
    PERSISTENT_PREFIX,
    PersistenceError,
    PersistentStore,
    default_state_root,
    hash_tree_xml,
    list_incomplete_tasks,
    reap_task,
    task_db_path,
)


class _TempState:
    """Context manager that points the module at a tmp dir via XDG_STATE_HOME."""

    def __enter__(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("XDG_STATE_HOME")
        os.environ["XDG_STATE_HOME"] = self._tmp.name
        return Path(self._tmp.name)

    def __exit__(self, *exc):
        if self._prev is None:
            os.environ.pop("XDG_STATE_HOME", None)
        else:
            os.environ["XDG_STATE_HOME"] = self._prev
        self._tmp.cleanup()


class TestStorePaths(unittest.TestCase):
    def test_default_state_root_uses_xdg(self):
        with _TempState() as root:
            self.assertTrue(
                str(default_state_root()).startswith(str(root))
            )

    def test_task_db_path(self):
        with _TempState() as root:
            p = task_db_path("t123abc")
            self.assertEqual(p.name, "state.db")
            self.assertEqual(p.parent.name, "t123abc")
            self.assertTrue(str(p).startswith(str(root)))


class TestStoreCRUD(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "task" / "state.db"
        self.store = PersistentStore(self.path)

    def tearDown(self):
        self.store.close()
        self._tmp.cleanup()

    def test_creates_file_and_schema(self):
        self.assertTrue(self.path.exists())
        self.assertEqual(self.store.get_meta("schema_version"), "1")

    def test_kv_roundtrip(self):
        self.store.set_kv("persistent.foo", {"a": 1, "b": [2, 3]})
        self.assertEqual(
            self.store.get_kv("persistent.foo"),
            {"a": 1, "b": [2, 3]},
        )

    def test_kv_default_when_missing(self):
        self.assertIsNone(self.store.get_kv("persistent.missing"))
        self.assertEqual(
            self.store.get_kv("persistent.missing", default=42), 42
        )

    def test_kv_rejects_non_persistent_prefix(self):
        with self.assertRaises(PersistenceError):
            self.store.set_kv("transient_key", 1)

    def test_kv_rejects_non_serializable(self):
        with self.assertRaises(PersistenceError):
            self.store.set_kv("persistent.bad", object())

    def test_kv_overwrite(self):
        self.store.set_kv("persistent.x", 1)
        self.store.set_kv("persistent.x", 2)
        self.assertEqual(self.store.get_kv("persistent.x"), 2)

    def test_kv_delete(self):
        self.store.set_kv("persistent.x", 1)
        self.store.delete_kv("persistent.x")
        self.assertIsNone(self.store.get_kv("persistent.x"))

    def test_all_kv_returns_full_map(self):
        self.store.set_kv("persistent.a", 1)
        self.store.set_kv("persistent.b", "hello")
        self.assertEqual(
            self.store.all_kv(),
            {"persistent.a": 1, "persistent.b": "hello"},
        )

    def test_node_state_roundtrip(self):
        self.store.save_node_state("/root/Sequence[0]", "Sequence", {"idx": 2})
        self.assertEqual(
            self.store.load_node_state("/root/Sequence[0]"),
            {"idx": 2},
        )

    def test_node_state_clear(self):
        self.store.save_node_state("/root/X", "Sequence", {"idx": 1})
        self.store.clear_node_state("/root/X")
        self.assertIsNone(self.store.load_node_state("/root/X"))

    def test_node_state_clear_subtree(self):
        self.store.save_node_state("/root/A", "Sequence", {"idx": 1})
        self.store.save_node_state("/root/A/B", "Sequence", {"idx": 0})
        self.store.save_node_state("/root/A/B/C", "Repeat", {"i": 5})
        self.store.save_node_state("/root/D", "Sequence", {"idx": 3})
        self.store.record_inflight(
            "/root/A/Action[0]", "/srv", "Type", "uuid1", False
        )

        self.store.clear_node_state_subtree("/root/A")

        self.assertIsNone(self.store.load_node_state("/root/A"))
        self.assertIsNone(self.store.load_node_state("/root/A/B"))
        self.assertIsNone(self.store.load_node_state("/root/A/B/C"))
        # Sibling untouched
        self.assertIsNotNone(self.store.load_node_state("/root/D"))
        # Inflight under the prefix also cleared
        self.assertEqual(self.store.list_inflight(), [])

    def test_inflight_roundtrip(self):
        self.store.record_inflight(
            "/root/Liconic[0]", "/skill_atoms/take_in",
            "robot_skills_msgs/action/TakeIn",
            "uuid-abc", False,
        )
        rows = self.store.list_inflight()
        self.assertEqual(len(rows), 1)
        r = rows[0]
        self.assertEqual(r["node_path"], "/root/Liconic[0]")
        self.assertEqual(r["server_name"], "/skill_atoms/take_in")
        self.assertEqual(r["goal_uuid"], "uuid-abc")
        self.assertFalse(r["idempotent"])

    def test_inflight_overwrite_same_node_path(self):
        # A node that submits, fails, retries should leave only one row.
        self.store.record_inflight("/root/X", "/a", "T", "u1", True)
        self.store.record_inflight("/root/X", "/a", "T", "u2", True)
        rows = self.store.list_inflight()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["goal_uuid"], "u2")

    def test_inflight_clear(self):
        self.store.record_inflight("/root/X", "/a", "T", "u1", True)
        self.store.clear_inflight("/root/X")
        self.assertEqual(self.store.list_inflight(), [])

    def test_meta_stamps(self):
        self.store.stamp_started("hash-a")
        self.assertEqual(self.store.get_meta("bt_xml_hash"), "hash-a")
        started = self.store.get_meta("started_at")
        self.assertIsNotNone(started)
        # Re-stamping doesn't move started_at
        self.store.stamp_started("hash-b")
        self.assertEqual(self.store.get_meta("started_at"), started)
        self.assertEqual(self.store.get_meta("bt_xml_hash"), "hash-b")


class TestPersistenceAcrossReopen(unittest.TestCase):
    def test_reopens_existing_db_intact(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x" / "state.db"
            with PersistentStore(path) as s1:
                s1.set_kv("persistent.q", [1, 2, 3])
                s1.save_node_state("/r/A", "Sequence", {"idx": 1})
                s1.record_inflight("/r/B", "/srv", "T", "uid", False)
                s1.stamp_started("hash-x")

            with PersistentStore(path) as s2:
                self.assertEqual(s2.get_kv("persistent.q"), [1, 2, 3])
                self.assertEqual(
                    s2.load_node_state("/r/A"), {"idx": 1}
                )
                self.assertEqual(len(s2.list_inflight()), 1)
                self.assertEqual(s2.get_meta("bt_xml_hash"), "hash-x")
                self.assertEqual(s2.get_meta("schema_version"), "1")


class TestListAndReap(unittest.TestCase):
    def test_list_and_reap(self):
        with _TempState() as _root:
            # Two task dirs; one with a db, one without.
            with PersistentStore(task_db_path("t-with-db")) as s:
                s.set_kv("persistent.x", 1)
            (default_state_root() / "t-empty").mkdir(parents=True)

            incomplete = [p.name for p in list_incomplete_tasks()]
            self.assertEqual(incomplete, ["t-with-db"])

            reap_task("t-with-db")
            self.assertEqual(list_incomplete_tasks(), [])


class TestHashTreeXml(unittest.TestCase):
    def test_deterministic(self):
        a = hash_tree_xml("<root>A</root>")
        b = hash_tree_xml("<root>A</root>")
        c = hash_tree_xml("<root>B</root>")
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)


if __name__ == "__main__":
    unittest.main()
