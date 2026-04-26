"""Skill discovery via latched per-node `*/skills` advertisements.

Scans the ROS graph for topics matching `*/skills` of type
`robot_skills_msgs/msg/SkillManifest`, subscribes with the QoS profile
defined in `skill_advertiser.py`, and assembles a runtime registry of
skills keyed by `(robot_id, bt_tag)`.

Liveliness handling: TRANSIENT_LOCAL + LIVELINESS_AUTOMATIC means the
last manifest survives publisher restart and is delivered automatically
to new subscribers, while `on_liveliness_changed` fires when a publisher
crashes — entries from a dead publisher are evicted.

The runtime registry is now the only source of truth for the BT executor
(post phase 4). The `/skill_server/skill_registry` topic is the
aggregated view; `/skill_server/discovery_diff` retains a JSON snapshot
of the current registry for tooling that wants a flat dump.
"""

from __future__ import annotations

import importlib
import json
import re
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.qos_event import QoSLivelinessChangedInfo, SubscriptionEventCallbacks

from robot_skills_msgs.msg import (
    SkillAdvertisement,
    SkillDescription,
    SkillManifest,
)
from std_msgs.msg import String

from robot_skill_advertise import (
    SKILLS_TOPIC_SUFFIX,
    make_skills_qos,
)


SKILL_MANIFEST_TYPE = "robot_skills_msgs/msg/SkillManifest"
DEFAULT_SCAN_PERIOD_SEC = 5.0


@dataclass
class ActionEntry:
    """Resolved skill record consumed by the BT executor at parse time."""

    bt_tag: str
    robot_id: str
    server_name: str
    action_type_str: str            # e.g. "liconic_msgs/action/TakeIn"
    action_type: Optional[type]     # imported lazily; None if import_error set
    inputs: Dict[str, str] = field(default_factory=dict)   # xml_attr -> goal_field
    outputs: Dict[str, str] = field(default_factory=dict)  # result_field -> xml_attr
    defaults: Dict[str, str] = field(default_factory=dict)
    post_process: str = ""
    source_topic: str = ""           # publisher-side topic, used for liveliness eviction
    import_error: str = ""           # populated if action_type couldn't be imported


def _pascal_case(name: str) -> str:
    """Convert snake_case / kebab-case skill names to PascalCase BT tags."""
    return "".join(p.capitalize() for p in re.split(r"[_\-\s]+", name) if p)


def _import_action_type(type_str: str) -> Tuple[Optional[type], str]:
    """Import an action type from its ROS type string.

    Returns (cls, error). On success, error is empty. On failure, cls is
    None and error is a human-readable explanation that the BT parser can
    surface to the user (typically: missing provider msgs package).
    """
    try:
        pkg, kind, type_name = type_str.split("/")
    except ValueError:
        return None, f"malformed action type string {type_str!r}"
    if kind != "action":
        return None, f"expected /action/ middle segment, got {kind!r} in {type_str!r}"
    module_path = f"{pkg}.action"
    try:
        module = importlib.import_module(module_path)
    except ImportError as e:
        return None, f"could not import {module_path}: {e}"
    cls = getattr(module, type_name, None)
    if cls is None:
        return None, f"no {type_name!r} in {module_path}"
    return cls, ""


def _entry_from_advertisement(
    ad: SkillAdvertisement, source_topic: str
) -> ActionEntry:
    desc: SkillDescription = ad.description
    bt_tag = ad.bt_tag or _pascal_case(desc.name)
    action_cls, err = _import_action_type(desc.action_type)
    inputs: Dict[str, str] = {}
    if ad.input_xml_attrs:
        for attr in ad.input_xml_attrs:
            inputs[attr] = attr
    elif action_cls is not None:
        # Derive inputs from action Goal field names; xml_attr == goal_field.
        # rosidl-generated Goal types expose field names via __slots__.
        goal_cls = getattr(action_cls, "Goal", None)
        if goal_cls is not None:
            for slot in getattr(goal_cls, "__slots__", []):
                # Slots are prefixed with "_"; strip and use as xml_attr.
                attr = slot.lstrip("_")
                inputs[attr] = attr
    # Output mapping: result_field -> xml_attr. Default is 1:1 derived from
    # the action Result's slots (matches the original ACTION_REGISTRY shape
    # for the 13 atoms whose xml port name equals the result field name).
    # Explicit overrides in `output_renames` win — used when the names
    # diverge (e.g. DetectObject's `detections` -> `detected_objects`).
    outputs: Dict[str, str] = {}
    if action_cls is not None:
        result_cls = getattr(action_cls, "Result", None)
        if result_cls is not None:
            for slot in getattr(result_cls, "__slots__", []):
                field = slot.lstrip("_")
                if field in ("success", "message"):
                    continue  # surfaced separately by the executor
                outputs[field] = field
    for kv in ad.output_renames:
        outputs[kv.key] = kv.value
    defaults = {kv.key: kv.value for kv in ad.goal_defaults}
    return ActionEntry(
        bt_tag=bt_tag,
        robot_id=desc.robot_id,
        server_name=desc.action_server_name,
        action_type_str=desc.action_type,
        action_type=action_cls,
        inputs=inputs,
        outputs=outputs,
        defaults=defaults,
        post_process=ad.post_process_id,
        source_topic=source_topic,
        import_error=err,
    )


class SkillDiscovery(Node):
    """Latched-topic skill discovery node.

    Public API:
        get(robot_id, bt_tag) -> ActionEntry | None
        snapshot() -> Dict[Tuple[robot_id, bt_tag], ActionEntry]
        get_by_tag(bt_tag) -> List[ActionEntry]   # all robots advertising this tag
        wait_for_initial_scan(timeout_s) -> bool
    """

    def __init__(
        self,
        scan_period_sec: float = DEFAULT_SCAN_PERIOD_SEC,
        on_change: Optional[Callable[[], None]] = None,
    ):
        super().__init__("skill_discovery")

        self._lock = threading.RLock()
        self._registry: Dict[Tuple[str, str], ActionEntry] = {}
        self._subs: Dict[str, Any] = {}                      # topic -> Subscription
        self._topic_skills: Dict[str, List[Tuple[str, str]]] = {}  # topic -> keys
        self._on_change = on_change
        self._initial_scan_done = threading.Event()

        self._aggregate_pub = self.create_publisher(
            SkillManifest,
            "/skill_server/skill_registry",
            make_skills_qos(),
        )
        # Flat-JSON snapshot of the current registry, for tooling that doesn't
        # want to deserialise SkillManifest. TRANSIENT_LOCAL so late
        # `ros2 topic echo` calls get the latest snapshot immediately.
        self._diff_pub = self.create_publisher(
            String,
            "/skill_server/discovery_diff",
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._scan_timer = self.create_timer(scan_period_sec, self._scan_topics)

    # ── public lookup API ───────────────────────────────────────────────────

    def get(self, robot_id: Optional[str], bt_tag: str) -> Optional[ActionEntry]:
        with self._lock:
            if robot_id is not None and (robot_id, bt_tag) in self._registry:
                return self._registry[(robot_id, bt_tag)]
            # No robot_id given (or no exact match): collect all entries with
            # this bt_tag across robots. Caller decides what to do with
            # multiple matches.
            matches = [e for (rid, tag), e in self._registry.items() if tag == bt_tag]
            if len(matches) == 1:
                return matches[0]
            return None

    def get_by_tag(self, bt_tag: str) -> List[ActionEntry]:
        with self._lock:
            return [e for (_, tag), e in self._registry.items() if tag == bt_tag]

    def snapshot(self) -> Dict[Tuple[str, str], ActionEntry]:
        with self._lock:
            return dict(self._registry)

    def wait_for_initial_scan(self, timeout_s: float = 5.0) -> bool:
        return self._initial_scan_done.wait(timeout=timeout_s)

    # ── graph scanning ──────────────────────────────────────────────────────

    def _scan_topics(self) -> None:
        try:
            for name, types in self.get_topic_names_and_types():
                if SKILL_MANIFEST_TYPE not in types:
                    continue
                if not name.endswith(f"/{SKILLS_TOPIC_SUFFIX}"):
                    continue
                if name in self._subs:
                    continue
                self._subscribe(name)
        finally:
            self._initial_scan_done.set()

    def _subscribe(self, topic: str) -> None:
        callbacks = SubscriptionEventCallbacks(
            liveliness=lambda info, t=topic: self._on_liveliness_changed(t, info),
        )
        sub = self.create_subscription(
            SkillManifest,
            topic,
            lambda msg, t=topic: self._on_manifest(t, msg),
            make_skills_qos(),
            event_callbacks=callbacks,
        )
        self._subs[topic] = sub
        self.get_logger().info(f"SkillDiscovery: subscribed to {topic}")

    # ── manifest ingestion + liveliness ─────────────────────────────────────

    def _on_manifest(self, topic: str, msg: SkillManifest) -> None:
        new_keys: List[Tuple[str, str]] = []
        with self._lock:
            # Remove any prior entries from this topic — manifest is the
            # full list, not a delta.
            for key in self._topic_skills.get(topic, []):
                self._registry.pop(key, None)
            for ad in msg.skills:
                entry = _entry_from_advertisement(ad, source_topic=topic)
                key = (entry.robot_id, entry.bt_tag)
                self._registry[key] = entry
                new_keys.append(key)
            self._topic_skills[topic] = new_keys
        self._republish_aggregate()
        self._publish_diff()
        if self._on_change is not None:
            self._on_change()

    def _on_liveliness_changed(
        self, topic: str, info: QoSLivelinessChangedInfo
    ) -> None:
        if info.alive_count > 0:
            return
        # Publisher died: evict its skills.
        with self._lock:
            for key in self._topic_skills.pop(topic, []):
                self._registry.pop(key, None)
            sub = self._subs.pop(topic, None)
        if sub is not None:
            self.destroy_subscription(sub)
        self.get_logger().warning(
            f"SkillDiscovery: publisher of {topic} lost liveliness; entries evicted"
        )
        self._republish_aggregate()
        self._publish_diff()
        if self._on_change is not None:
            self._on_change()

    # ── outgoing aggregate + parity diff ────────────────────────────────────

    def _republish_aggregate(self) -> None:
        msg = SkillManifest()
        msg.published_at = self.get_clock().now().to_msg()
        msg.source_node = self.get_fully_qualified_name()
        with self._lock:
            for entry in self._registry.values():
                ad = SkillAdvertisement()
                ad.description.name = entry.bt_tag.lower()
                ad.description.robot_id = entry.robot_id
                ad.description.action_server_name = entry.server_name
                ad.description.action_type = entry.action_type_str
                ad.bt_tag = entry.bt_tag
                ad.post_process_id = entry.post_process
                ad.input_xml_attrs = list(entry.inputs.keys())
                msg.skills.append(ad)
        self._aggregate_pub.publish(msg)

    def _publish_diff(self) -> None:
        """Publish a flat-JSON snapshot of the current registry.

        Kept under the legacy `/skill_server/discovery_diff` topic name for
        backwards compat with tooling that grew up against the phase-1
        parity check.
        """
        with self._lock:
            entries = [
                {
                    "robot_id": robot_id,
                    "bt_tag": bt_tag,
                    "action_server": entry.server_name,
                    "action_type": entry.action_type_str,
                    "import_error": entry.import_error,
                }
                for (robot_id, bt_tag), entry in sorted(self._registry.items())
            ]
        self._diff_pub.publish(String(data=json.dumps({
            "count": len(entries),
            "entries": entries,
        })))
