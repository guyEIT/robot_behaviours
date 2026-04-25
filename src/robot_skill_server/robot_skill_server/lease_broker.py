"""
LeaseBroker - Exclusive ownership leases for named resources.

A resource_id is a free-form string chosen by the caller; the common cases
are robots.yaml keys ("meca500") and action-server paths
("/meca500/skill_atoms/move_to_named_config"). The broker keeps a single
live lease per resource_id and rejects (or blocks, if wait=true) concurrent
acquires.

Services:
  /skill_server/acquire_lease
  /skill_server/renew_lease
  /skill_server/release_lease

Topics:
  /skill_server/leases         (latched JSON, snapshot of active leases)
  /skill_server/lease_events   (every acquire/renew/release/revoke)

Leases carry a TTL; holders must renew before expiry. A 1 Hz sweep
revokes expired leases and publishes an 'revoked' LeaseEvent so BtExecutor
can hard-cancel any tree that was relying on the lease.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from std_msgs.msg import String

from robot_skills_msgs.msg import LeaseEvent, LeaseState
from robot_skills_msgs.srv import AcquireLease, ReleaseLease, RenewLease


class _LeaseRecord:
    """In-memory lease record. Internal; not wire-visible as-is."""

    __slots__ = (
        "lease_id",
        "resource_id",
        "holder_id",
        "acquired_at_mono",
        "expires_at_mono",
        "ttl_sec",
    )

    def __init__(
        self,
        lease_id: str,
        resource_id: str,
        holder_id: str,
        ttl_sec: float,
    ):
        now = time.monotonic()
        self.lease_id = lease_id
        self.resource_id = resource_id
        self.holder_id = holder_id
        self.ttl_sec = ttl_sec
        self.acquired_at_mono = now
        self.expires_at_mono = now + ttl_sec

    def to_state_dict(self, clock_now_ros: TimeMsg) -> dict:
        """Render for the JSON /skill_server/leases snapshot.

        We track lifetimes in monotonic seconds (for robustness against
        wall-clock jumps) and approximate ROS-time fields from the current
        ROS clock plus monotonic offsets.
        """
        mono_now = time.monotonic()
        now_ns = (
            clock_now_ros.sec * 1_000_000_000 + clock_now_ros.nanosec
        )
        acquired_ns = now_ns - int(
            (mono_now - self.acquired_at_mono) * 1_000_000_000
        )
        expires_ns = now_ns + int(
            (self.expires_at_mono - mono_now) * 1_000_000_000
        )
        return {
            "lease_id": self.lease_id,
            "resource_id": self.resource_id,
            "holder_id": self.holder_id,
            "acquired_at": {
                "sec": acquired_ns // 1_000_000_000,
                "nanosec": acquired_ns % 1_000_000_000,
            },
            "expires_at": {
                "sec": expires_ns // 1_000_000_000,
                "nanosec": expires_ns % 1_000_000_000,
            },
            "ttl_sec": self.ttl_sec,
        }


def _ros_time_from_mono(
    clock_now_ros: TimeMsg, mono_target: float
) -> TimeMsg:
    mono_now = time.monotonic()
    now_ns = clock_now_ros.sec * 1_000_000_000 + clock_now_ros.nanosec
    target_ns = now_ns + int((mono_target - mono_now) * 1_000_000_000)
    t = TimeMsg()
    t.sec = target_ns // 1_000_000_000
    t.nanosec = target_ns % 1_000_000_000
    return t


class LeaseBroker(Node):
    def __init__(self):
        super().__init__("lease_broker")

        self._lock = threading.RLock()
        self._leases: Dict[str, _LeaseRecord] = {}           # lease_id -> record
        self._by_resource: Dict[str, str] = {}               # resource_id -> lease_id

        self._acquire_srv = self.create_service(
            AcquireLease,
            "/skill_server/acquire_lease",
            self._handle_acquire,
        )
        self._renew_srv = self.create_service(
            RenewLease,
            "/skill_server/renew_lease",
            self._handle_renew,
        )
        self._release_srv = self.create_service(
            ReleaseLease,
            "/skill_server/release_lease",
            self._handle_release,
        )

        from rclpy.qos import DurabilityPolicy, QoSProfile

        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._leases_pub = self.create_publisher(
            String, "/skill_server/leases", latched_qos
        )
        self._events_pub = self.create_publisher(
            LeaseEvent, "/skill_server/lease_events", 20
        )

        # Poll for expired leases once a second.
        self._expiry_timer = self.create_timer(1.0, self._expire_stale)

        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID("lease_broker")
        self._diag_updater.add("leases", self._produce_diagnostics)

        self._publish_snapshot()
        self.get_logger().info(
            "LeaseBroker started. "
            "Services: /skill_server/{acquire,renew,release}_lease"
        )

    # ── Service handlers ──────────────────────────────────────────────────────

    def _handle_acquire(
        self,
        request: AcquireLease.Request,
        response: AcquireLease.Response,
    ) -> AcquireLease.Response:
        if not request.resource_id:
            response.success = False
            response.reason = "resource_id is required"
            return response

        ttl = float(request.ttl_sec) if request.ttl_sec > 0 else 10.0
        deadline_mono = (
            time.monotonic() + float(request.wait_timeout_sec)
            if request.wait and request.wait_timeout_sec > 0
            else None
        )

        # Polling acquire loop (cheap; leases are typically held for seconds).
        while True:
            with self._lock:
                held_id = self._by_resource.get(request.resource_id)
                held = self._leases.get(held_id) if held_id else None
                if held is None or held.expires_at_mono <= time.monotonic():
                    # Free (or the current holder's TTL has lapsed — sweep
                    # hasn't run yet; reclaim inline so callers aren't blocked
                    # by the 1 Hz expiry cadence).
                    if held is not None:
                        self._revoke_locked(held, reason="ttl_expired")
                    record = _LeaseRecord(
                        lease_id=uuid.uuid4().hex,
                        resource_id=request.resource_id,
                        holder_id=request.holder_id or "anonymous",
                        ttl_sec=ttl,
                    )
                    self._leases[record.lease_id] = record
                    self._by_resource[record.resource_id] = record.lease_id
                    response.success = True
                    response.lease_id = record.lease_id
                    response.reason = ""
                    response.expires_at = _ros_time_from_mono(
                        self.get_clock().now().to_msg(),
                        record.expires_at_mono,
                    )
                    response.current_holder = record.holder_id
                    self._publish_event(
                        record, event="acquired", reason=""
                    )
                    self._publish_snapshot()
                    return response

                # Held by someone else.
                if not request.wait:
                    response.success = False
                    response.reason = (
                        f"resource '{request.resource_id}' is held by "
                        f"'{held.holder_id}'"
                    )
                    response.current_holder = held.holder_id
                    return response

                if deadline_mono is not None and time.monotonic() >= deadline_mono:
                    response.success = False
                    response.reason = (
                        f"wait_timeout_sec elapsed; resource still held by "
                        f"'{held.holder_id}'"
                    )
                    response.current_holder = held.holder_id
                    return response

            # Sleep outside the lock; 200 ms is fine — leases are coarse-grained.
            time.sleep(0.2)

    def _handle_renew(
        self,
        request: RenewLease.Request,
        response: RenewLease.Response,
    ) -> RenewLease.Response:
        with self._lock:
            record = self._leases.get(request.lease_id)
            if record is None:
                response.success = False
                response.reason = "unknown or already-revoked lease_id"
                return response
            new_ttl = (
                float(request.ttl_sec)
                if request.ttl_sec > 0
                else record.ttl_sec
            )
            record.ttl_sec = new_ttl
            record.expires_at_mono = time.monotonic() + new_ttl
            response.success = True
            response.expires_at = _ros_time_from_mono(
                self.get_clock().now().to_msg(), record.expires_at_mono
            )
            response.reason = ""
            self._publish_event(record, event="renewed", reason="")
        self._publish_snapshot()
        return response

    def _handle_release(
        self,
        request: ReleaseLease.Request,
        response: ReleaseLease.Response,
    ) -> ReleaseLease.Response:
        with self._lock:
            record = self._leases.pop(request.lease_id, None)
            if record is None:
                response.success = True
                response.message = "lease already released or expired"
                return response
            self._by_resource.pop(record.resource_id, None)
            self._publish_event(
                record, event="released", reason=request.reason or ""
            )
        self._publish_snapshot()
        response.success = True
        response.message = ""
        return response

    # ── Background sweep ──────────────────────────────────────────────────────

    def _expire_stale(self):
        now_mono = time.monotonic()
        revoked: list[_LeaseRecord] = []
        with self._lock:
            for lease_id in list(self._leases.keys()):
                record = self._leases[lease_id]
                if record.expires_at_mono <= now_mono:
                    self._revoke_locked(record, reason="ttl_expired")
                    revoked.append(record)
        if revoked:
            self._publish_snapshot()

    def _revoke_locked(self, record: _LeaseRecord, reason: str):
        """Remove a lease and emit a revoke event. Caller holds self._lock."""
        self._leases.pop(record.lease_id, None)
        existing = self._by_resource.get(record.resource_id)
        if existing == record.lease_id:
            self._by_resource.pop(record.resource_id, None)
        self._publish_event(record, event="revoked", reason=reason)
        self.get_logger().info(
            f"Revoked lease {record.lease_id} on '{record.resource_id}' "
            f"(holder={record.holder_id}, reason={reason})"
        )

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_event(self, record: _LeaseRecord, event: str, reason: str):
        msg = LeaseEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.lease_id = record.lease_id
        msg.resource_id = record.resource_id
        msg.holder_id = record.holder_id
        msg.event = event
        msg.reason = reason
        self._events_pub.publish(msg)

    def _publish_snapshot(self):
        with self._lock:
            now = self.get_clock().now().to_msg()
            payload = [r.to_state_dict(now) for r in self._leases.values()]
        msg = String()
        msg.data = json.dumps(payload)
        self._leases_pub.publish(msg)

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def _produce_diagnostics(self, stat):
        with self._lock:
            count = len(self._leases)
            held_resources = ", ".join(sorted(self._by_resource.keys())) or "(none)"
        stat.summary(DiagnosticStatus.OK, f"{count} active lease(s)")
        stat.add("active_count", str(count))
        stat.add("held_resources", held_resources)
        return stat
