"""Unit tests for :mod:`hamilton_star_ros.handoff_registry`.

Pure-Python — no rclpy, no ROS message types. Registry CRUD semantics
only; the service-layer tests that drive ``/define_handoff`` +
``/list_handoffs`` + ``/delete_handoff`` live in ``test_ros2_handoff_services.py``.
"""

from __future__ import annotations

from hamilton_star_ros.handoff_registry import HandoffRecord, HandoffRegistry


def _record(name: str = "incubator_handoff", **overrides) -> HandoffRecord:
    defaults = dict(
        x=-216.1, y=170.6, z=123.6,
        plate_width=80.0,
        rotation="LEFT", wrist="STRAIGHT",
        grip_direction="FRONT",
        hotel_depth=500.0, hotel_clearance_height=8.0,
    )
    defaults.update(overrides)
    return HandoffRecord(name=name, **defaults)


def test_empty_registry() -> None:
    r = HandoffRegistry()
    assert len(r) == 0
    assert r.names() == []
    assert r.all() == []
    assert r.get("anything") is None
    assert "anything" not in r


def test_set_and_get() -> None:
    r = HandoffRegistry()
    rec = _record()
    assert r.set(rec) is True
    assert len(r) == 1
    assert r.get("incubator_handoff") == rec
    assert "incubator_handoff" in r
    assert r.names() == ["incubator_handoff"]


def test_set_refuses_overwrite_without_flag() -> None:
    r = HandoffRegistry()
    r.set(_record())
    rec2 = _record(x=-220.0)
    assert r.set(rec2, replace_existing=False) is False
    # original values preserved
    assert r.get("incubator_handoff").x == -216.1


def test_set_overwrites_with_flag() -> None:
    r = HandoffRegistry()
    r.set(_record())
    rec2 = _record(x=-220.0)
    assert r.set(rec2, replace_existing=True) is True
    assert r.get("incubator_handoff").x == -220.0


def test_delete_returns_false_for_missing() -> None:
    r = HandoffRegistry()
    assert r.delete("nope") is False


def test_delete_removes_record() -> None:
    r = HandoffRegistry()
    r.set(_record())
    assert r.delete("incubator_handoff") is True
    assert len(r) == 0
    assert r.delete("incubator_handoff") is False  # idempotent


def test_all_lists_every_record() -> None:
    r = HandoffRegistry()
    r.set(_record("a"))
    r.set(_record("b", x=50.0))
    r.set(_record("c", x=100.0))
    names = sorted(rec.name for rec in r.all())
    assert names == ["a", "b", "c"]


def test_seed_replaces_everything() -> None:
    r = HandoffRegistry()
    r.set(_record("old"))
    r.seed([_record("new1"), _record("new2")])
    assert set(r.names()) == {"new1", "new2"}
    assert "old" not in r


def test_from_dict_uses_defaults_for_new_fields() -> None:
    """Legacy handoff dicts (without hotel_depth etc.) get sensible
    defaults from the dataclass."""
    rec = HandoffRecord.from_dict(
        "legacy",
        {"x": 10.0, "y": 20.0, "z": 30.0, "plate_width": 80.0,
         "rotation": "LEFT", "wrist": "STRAIGHT"},
    )
    assert rec.grip_direction == "FRONT"
    assert rec.hotel_depth == 160.0  # dataclass default
    assert rec.hotel_clearance_height == 7.5


def test_from_dict_overrides_new_fields() -> None:
    rec = HandoffRecord.from_dict(
        "site",
        {
            "x": 10.0, "y": 20.0, "z": 30.0, "plate_width": 80.0,
            "rotation": "LEFT", "wrist": "STRAIGHT",
            "grip_direction": "LEFT",
            "hotel_depth": 600.0, "hotel_clearance_height": 10.0,
        },
    )
    assert rec.grip_direction == "LEFT"
    assert rec.hotel_depth == 600.0
    assert rec.hotel_clearance_height == 10.0


def test_record_is_immutable() -> None:
    """HandoffRecord is frozen — mutate via replace()/set(), not
    attribute assignment."""
    import dataclasses
    r = HandoffRegistry()
    rec = _record()
    r.set(rec)
    try:
        rec.x = 0.0  # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        pass
    else:
        raise AssertionError("HandoffRecord should be frozen")
    # but we can build a replacement
    updated = dataclasses.replace(rec, x=-200.0)
    r.set(updated)
    assert r.get(rec.name).x == -200.0
