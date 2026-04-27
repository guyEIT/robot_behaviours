"""SQLite-backed persistence for long-lived behavior trees.

Convention: blackboard keys with prefix ``persistent.`` are mirrored to a
per-task SQLite file under ``~/.local/state/skill_server/tasks/{task_id}/``.
Everything else stays in-memory only.

The store also carries:
  - ``node_state``: per-node tick state (Sequence index, Repeat iteration, …)
  - ``action_inflight``: action goals submitted but not yet acknowledged as done
  - ``meta``: bt_xml_hash + timestamps so resume can validate invariants

Pure-stdlib, no ROS imports, no rclpy — so it tests with plain pytest.
"""

from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any, Iterable, Optional


PERSISTENT_PREFIX = "persistent."
SCHEMA_VERSION = 1


def default_state_root() -> Path:
    """Where per-task DBs live. Honours XDG_STATE_HOME."""
    base = os.environ.get("XDG_STATE_HOME", "")
    if base:
        return Path(base) / "skill_server" / "tasks"
    return Path.home() / ".local" / "state" / "skill_server" / "tasks"


def task_db_path(task_id: str, root: Optional[Path] = None) -> Path:
    return (root or default_state_root()) / task_id / "state.db"


def hash_tree_xml(xml: str) -> str:
    return hashlib.sha256(xml.encode("utf-8")).hexdigest()


class PersistenceError(Exception):
    """Raised on persistence-layer faults: bad schema, non-JSON values, etc."""


def _ensure_json_serializable(key: str, value: Any) -> str:
    try:
        return json.dumps(value, sort_keys=True, default=None)
    except (TypeError, ValueError) as exc:
        raise PersistenceError(
            f"persistent blackboard key {key!r} value is not JSON-serializable: "
            f"{type(value).__name__} ({exc})"
        ) from exc


class PersistentStore:
    """Per-task SQLite store. Use as a context manager or call ``close()``.

    Schema is created on first open. Concurrent reads are safe (WAL); the
    only writers are the BT executor and the bb_operator sidecar — they
    interleave via WAL without explicit locking.
    """

    def __init__(self, db_path: Path):
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(
            str(self.db_path),
            isolation_level=None,  # autocommit; we manage txns explicitly when needed
            check_same_thread=False,
        )
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.execute("PRAGMA synchronous=NORMAL;")
        self._conn.execute("PRAGMA foreign_keys=ON;")
        self._init_schema()

    def _init_schema(self) -> None:
        c = self._conn
        c.executescript(
            """
            CREATE TABLE IF NOT EXISTS blackboard (
                key TEXT PRIMARY KEY,
                value_json TEXT NOT NULL,
                updated_at REAL NOT NULL
            );
            CREATE TABLE IF NOT EXISTS node_state (
                node_path TEXT PRIMARY KEY,
                kind TEXT NOT NULL,
                state_json TEXT NOT NULL,
                updated_at REAL NOT NULL
            );
            CREATE TABLE IF NOT EXISTS action_inflight (
                node_path TEXT PRIMARY KEY,
                server_name TEXT NOT NULL,
                action_type TEXT NOT NULL,
                goal_uuid TEXT NOT NULL,
                idempotent INTEGER NOT NULL,
                submitted_at REAL NOT NULL
            );
            CREATE TABLE IF NOT EXISTS meta (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );
            """
        )
        if self.get_meta("schema_version") is None:
            self.set_meta("schema_version", str(SCHEMA_VERSION))

    # ── meta ────────────────────────────────────────────────────────────────

    def set_meta(self, key: str, value: str) -> None:
        self._conn.execute(
            "INSERT INTO meta(key, value) VALUES(?,?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
            (key, value),
        )

    def get_meta(self, key: str) -> Optional[str]:
        row = self._conn.execute(
            "SELECT value FROM meta WHERE key=?", (key,)
        ).fetchone()
        return row[0] if row else None

    def stamp_started(self, bt_xml_hash: str) -> None:
        self.set_meta("bt_xml_hash", bt_xml_hash)
        if self.get_meta("started_at") is None:
            self.set_meta("started_at", str(time.time()))
        self.set_meta("last_tick_at", str(time.time()))

    def stamp_tick(self) -> None:
        self.set_meta("last_tick_at", str(time.time()))

    # ── blackboard ──────────────────────────────────────────────────────────

    def set_kv(self, key: str, value: Any) -> None:
        if not key.startswith(PERSISTENT_PREFIX):
            raise PersistenceError(
                f"persistent store only accepts keys with prefix "
                f"{PERSISTENT_PREFIX!r}; got {key!r}"
            )
        encoded = _ensure_json_serializable(key, value)
        self._conn.execute(
            "INSERT INTO blackboard(key, value_json, updated_at) VALUES(?,?,?) "
            "ON CONFLICT(key) DO UPDATE SET value_json=excluded.value_json, "
            "updated_at=excluded.updated_at",
            (key, encoded, time.time()),
        )

    def get_kv(self, key: str, default: Any = None) -> Any:
        row = self._conn.execute(
            "SELECT value_json FROM blackboard WHERE key=?", (key,)
        ).fetchone()
        if row is None:
            return default
        return json.loads(row[0])

    def delete_kv(self, key: str) -> None:
        self._conn.execute("DELETE FROM blackboard WHERE key=?", (key,))

    def all_kv(self) -> dict[str, Any]:
        out = {}
        for key, value_json in self._conn.execute(
            "SELECT key, value_json FROM blackboard"
        ).fetchall():
            out[key] = json.loads(value_json)
        return out

    # ── node_state ──────────────────────────────────────────────────────────

    def save_node_state(self, node_path: str, kind: str, state: dict) -> None:
        encoded = _ensure_json_serializable(f"<node_state {node_path}>", state)
        self._conn.execute(
            "INSERT INTO node_state(node_path, kind, state_json, updated_at) "
            "VALUES(?,?,?,?) ON CONFLICT(node_path) DO UPDATE SET "
            "kind=excluded.kind, state_json=excluded.state_json, "
            "updated_at=excluded.updated_at",
            (node_path, kind, encoded, time.time()),
        )

    def load_node_state(self, node_path: str) -> Optional[dict]:
        row = self._conn.execute(
            "SELECT state_json FROM node_state WHERE node_path=?", (node_path,)
        ).fetchone()
        if row is None:
            return None
        return json.loads(row[0])

    def clear_node_state(self, node_path: str) -> None:
        self._conn.execute(
            "DELETE FROM node_state WHERE node_path=?", (node_path,)
        )

    def clear_node_state_subtree(self, node_path_prefix: str) -> None:
        """Clear this node and all descendants by node-path prefix.

        Used by parents when they advance past a child: the child's subtree
        starts fresh on its next invocation (e.g., next iteration of a Repeat).
        """
        like = node_path_prefix.rstrip("/") + "%"
        self._conn.execute(
            "DELETE FROM node_state WHERE node_path LIKE ?", (like,)
        )
        self._conn.execute(
            "DELETE FROM action_inflight WHERE node_path LIKE ?", (like,)
        )

    def all_node_state(self) -> list[tuple[str, str, dict]]:
        return [
            (path, kind, json.loads(state_json))
            for path, kind, state_json in self._conn.execute(
                "SELECT node_path, kind, state_json FROM node_state"
            ).fetchall()
        ]

    # ── action_inflight ─────────────────────────────────────────────────────

    def record_inflight(
        self,
        node_path: str,
        server_name: str,
        action_type: str,
        goal_uuid: str,
        idempotent: bool,
    ) -> None:
        self._conn.execute(
            "INSERT INTO action_inflight(node_path, server_name, action_type, "
            "goal_uuid, idempotent, submitted_at) VALUES(?,?,?,?,?,?) "
            "ON CONFLICT(node_path) DO UPDATE SET "
            "server_name=excluded.server_name, "
            "action_type=excluded.action_type, "
            "goal_uuid=excluded.goal_uuid, "
            "idempotent=excluded.idempotent, "
            "submitted_at=excluded.submitted_at",
            (
                node_path,
                server_name,
                action_type,
                goal_uuid,
                1 if idempotent else 0,
                time.time(),
            ),
        )

    def clear_inflight(self, node_path: str) -> None:
        self._conn.execute(
            "DELETE FROM action_inflight WHERE node_path=?", (node_path,)
        )

    def get_inflight(self, node_path: str) -> Optional[dict]:
        row = self._conn.execute(
            "SELECT node_path, server_name, action_type, goal_uuid, "
            "idempotent, submitted_at FROM action_inflight WHERE node_path=?",
            (node_path,),
        ).fetchone()
        if row is None:
            return None
        return {
            "node_path": row[0],
            "server_name": row[1],
            "action_type": row[2],
            "goal_uuid": row[3],
            "idempotent": bool(row[4]),
            "submitted_at": row[5],
        }

    def list_inflight(self) -> list[dict]:
        rows = self._conn.execute(
            "SELECT node_path, server_name, action_type, goal_uuid, "
            "idempotent, submitted_at FROM action_inflight"
        ).fetchall()
        return [
            {
                "node_path": r[0],
                "server_name": r[1],
                "action_type": r[2],
                "goal_uuid": r[3],
                "idempotent": bool(r[4]),
                "submitted_at": r[5],
            }
            for r in rows
        ]

    # ── lifecycle ───────────────────────────────────────────────────────────

    def close(self) -> None:
        try:
            self._conn.close()
        except Exception:
            pass

    def __enter__(self) -> "PersistentStore":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def list_incomplete_tasks(root: Optional[Path] = None) -> list[Path]:
    """Return per-task directories that have a ``state.db`` (i.e., were not
    cleanly reaped). Caller decides whether to resume each one."""
    base = root or default_state_root()
    if not base.exists():
        return []
    out = []
    for entry in sorted(base.iterdir()):
        if entry.is_dir() and (entry / "state.db").is_file():
            out.append(entry)
    return out


def reap_task(task_id: str, root: Optional[Path] = None) -> None:
    """Delete a task's persistent state. Called by BtExecutor on clean SUCCESS."""
    db = task_db_path(task_id, root)
    parent = db.parent
    for f in [db, db.parent / "state.db-wal", db.parent / "state.db-shm"]:
        try:
            f.unlink()
        except FileNotFoundError:
            pass
    try:
        parent.rmdir()
    except (FileNotFoundError, OSError):
        pass
