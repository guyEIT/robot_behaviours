"""Helper for publishing a latched SkillManifest from any rclpy Node.

Used by Python action-server hosts (Liconic, Hamilton, future script server)
to expose their skill metadata on the standard `<node_fqn>/skills` topic.

The advertisement protocol — TRANSIENT_LOCAL durability + LIVELINESS_AUTOMATIC
— lets SkillDiscovery treat publisher death as authoritative for skill removal,
without a separate heartbeat. See `skill_discovery.py`.
"""

from __future__ import annotations

from typing import Iterable, List, Sequence

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from robot_skills_msgs.msg import SkillAdvertisement, SkillManifest


SKILLS_TOPIC_SUFFIX = "skills"
LIVELINESS_LEASE_SEC = 10.0


def make_skills_qos() -> QoSProfile:
    """QoS profile for `<node>/skills` advertisement topics.

    Must match between publisher and subscriber or DDS will refuse to connect
    them silently — discovery will appear to see nothing. Centralised here
    so both sides import the same definition.
    """
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        liveliness=QoSLivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=LIVELINESS_LEASE_SEC),
    )


class SkillAdvertiser:
    """Publishes a SkillManifest once on construction, on a latched topic
    relative to the host node (`<node_fqn>/skills`).

    Call `republish()` after registering or removing skills at runtime
    (e.g. the script server when MCP registers a new script).
    """

    def __init__(self, node: Node, skills: Sequence[SkillAdvertisement]):
        self._node = node
        self._skills: List[SkillAdvertisement] = list(skills)
        self._pub = node.create_publisher(
            SkillManifest,
            f"~/{SKILLS_TOPIC_SUFFIX}",
            make_skills_qos(),
        )
        self._publish()

    def set_skills(self, skills: Iterable[SkillAdvertisement]) -> None:
        self._skills = list(skills)
        self._publish()

    def add(self, skill: SkillAdvertisement) -> None:
        self._skills.append(skill)
        self._publish()

    def remove(self, bt_tag: str) -> None:
        self._skills = [s for s in self._skills if s.bt_tag != bt_tag]
        self._publish()

    def republish(self) -> None:
        self._publish()

    def _publish(self) -> None:
        msg = SkillManifest()
        msg.skills = list(self._skills)
        msg.published_at = self._node.get_clock().now().to_msg()
        msg.source_node = self._node.get_fully_qualified_name()
        self._pub.publish(msg)
