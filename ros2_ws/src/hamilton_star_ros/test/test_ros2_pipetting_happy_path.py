"""End-to-end pipetting tests against a deck fixture with tip rack + plate.

These tests use ``sim_node_pipetting`` — a HamiltonStarActionServer boot up
with a ``deck_file`` pointing at a real tip-rack-carrier + plate-carrier
STAR deck JSON. Backend is the chatterbox sim, so each pipetting command
is simulated (no USB), but pylabrobot's internal tip-state / volume
trackers still update, so this exercises the full resolution +
substate-update path through the node.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    Aspirate, Dispense, DropTips, PickUpTips, Transfer,
)
from hamilton_star_msgs.srv import GetStatus, ListResources

TIP_RACK = "tip_rack_01"
PLATE = "plate_01"


def _client(helper, action_type, name, endpoint):
    return ActionClient(helper, action_type, f"/{name}/{endpoint}")


def _status(helper, name, ros_helpers):
    return ros_helpers.call_service(
        helper,
        helper.create_client(GetStatus, f"/{name}/get_status"),
        GetStatus.Request(),
    ).status


# -----------------------------------------------------------------------
# Fixture sanity: deck really has our named resources
# -----------------------------------------------------------------------

def test_pipetting_deck_lists_tip_rack_and_plate(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME
    client = helper.create_client(ListResources, f"/{name}/list_resources")
    resp = ros_helpers.call_service(helper, client, ListResources.Request())
    assert TIP_RACK in resp.names
    assert PLATE in resp.names
    assert "core_grippers" in resp.names


def test_pipetting_deck_status_reports_loaded(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME
    s = _status(helper, name, ros_helpers)
    assert s.deck_loaded is True
    assert s.deck_hash  # non-empty
    assert s.op_state == "Idle"


# -----------------------------------------------------------------------
# Single-channel full pipetting round trip
# -----------------------------------------------------------------------

def test_pick_up_tip_single_channel_happy_path(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    goal = PickUpTips.Goal(
        tip_rack=TIP_RACK, wells=["A1"], use_channels=[0],
    )
    result = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"), goal,
    )
    assert result.success is True, result.message

    s = _status(helper, name, ros_helpers)
    assert s.channels[0].state == s.channels[0].HAS_TIP
    # No other channel touched.
    for i in range(1, s.num_channels):
        assert s.channels[i].state == s.channels[i].FREE
    assert 0 in list(s.busy_channels)


def test_pick_up_aspirate_dispense_drop_round_trip(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # 1. pick up tip on channel 0 from A1
    pu = PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0])
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"), pu,
    )
    assert r.success is True, r.message
    assert list(r.channels_used) == [0]

    # 2. aspirate 100 uL from plate_01 well A1 (referenced hierarchically)
    asp = Aspirate.Goal(
        resources=[f"{PLATE}/A1"], volumes=[100.0], use_channels=[0],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"), asp,
    )
    assert r.success is True, r.message
    assert list(r.final_volumes) == [100.0]

    # 3. dispense into plate_01 well A2
    disp = Dispense.Goal(
        resources=[f"{PLATE}/A2"], volumes=[100.0], use_channels=[0],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Dispense, name, "dispense"), disp,
    )
    assert r.success is True, r.message

    # 4. drop tip back into A1
    drop = DropTips.Goal(
        target=TIP_RACK, wells=["A1"], use_channels=[0],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, DropTips, name, "drop_tips"), drop,
    )
    assert r.success is True, r.message

    # Final: channel 0 free, FSM Idle
    s = _status(helper, name, ros_helpers)
    assert s.channels[0].state == s.channels[0].FREE
    assert s.op_state == "Idle"
    assert s.shared_goal_count == 0


def test_aspirate_before_pickup_rejected_by_gate(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    goal = Aspirate.Goal(
        resources=[f"{PLATE}/A1"], volumes=[50.0], use_channels=[0],
    )
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, Aspirate, name, "aspirate"), goal,
    )
    assert accepted is False  # channel 0 has no tip


def test_drop_tips_to_trash_clears_channel(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["B1"], use_channels=[1]),
    )
    s = _status(helper, name, ros_helpers)
    assert s.channels[1].state == s.channels[1].HAS_TIP

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, DropTips, name, "drop_tips"),
        DropTips.Goal(
            target="trash",
            wells=["trash"],  # ignored when target=="trash"
            use_channels=[1],
            allow_nonzero_volume=True,
        ),
    )
    assert r.success is True, r.message

    s = _status(helper, name, ros_helpers)
    assert s.channels[1].state == s.channels[1].FREE


# -----------------------------------------------------------------------
# Multi-channel
# -----------------------------------------------------------------------

def test_pick_up_tips_on_multiple_channels(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    goal = PickUpTips.Goal(
        tip_rack=TIP_RACK,
        wells=["A1", "B1", "C1"],
        use_channels=[0, 1, 2],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"), goal,
    )
    assert r.success is True, r.message
    assert sorted(r.channels_used) == [0, 1, 2]

    s = _status(helper, name, ros_helpers)
    for ch in (0, 1, 2):
        assert s.channels[ch].state == s.channels[ch].HAS_TIP
    for ch in (3, 4, 5, 6, 7):
        assert s.channels[ch].state == s.channels[ch].FREE


def test_drop_tips_subset_of_loaded_channels(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=["A1", "B1"], use_channels=[0, 1],
        ),
    )

    # Drop only channel 0; channel 1 should remain loaded.
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, DropTips, name, "drop_tips"),
        DropTips.Goal(
            target=TIP_RACK, wells=["A1"], use_channels=[0],
            allow_nonzero_volume=True,
        ),
    )
    assert r.success is True, r.message

    s = _status(helper, name, ros_helpers)
    assert s.channels[0].state == s.channels[0].FREE
    assert s.channels[1].state == s.channels[1].HAS_TIP


def test_aspirate_dispense_on_multiple_channels(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=["A1", "B1"], use_channels=[0, 1],
        ),
    )

    asp = Aspirate.Goal(
        resources=[f"{PLATE}/A1", f"{PLATE}/B1"],
        volumes=[50.0, 75.0],
        use_channels=[0, 1],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"), asp,
    )
    assert r.success is True, r.message
    assert list(r.final_volumes) == [50.0, 75.0]

    disp = Dispense.Goal(
        resources=[f"{PLATE}/A12", f"{PLATE}/B12"],
        volumes=[50.0, 75.0],
        use_channels=[0, 1],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Dispense, name, "dispense"), disp,
    )
    assert r.success is True, r.message


# -----------------------------------------------------------------------
# Transfer
# -----------------------------------------------------------------------

def test_transfer_with_per_target_volumes(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # transfer uses channel 0 implicitly; it must have a tip.
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
    )

    # target_volumes wins when specified; source_volume is ignored.
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Transfer, name, "transfer"),
        Transfer.Goal(
            source=f"{PLATE}/A1",
            targets=[f"{PLATE}/A2", f"{PLATE}/A3"],
            target_volumes=[50.0, 50.0],
        ),
    )
    assert r.success is True, r.message


def test_transfer_with_source_volume_split_evenly(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # Transfer uses channel 0 by default in pylabrobot; FSM gate enforces
    # that channel 0 specifically has a tip.
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
    )

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Transfer, name, "transfer"),
        Transfer.Goal(
            source=f"{PLATE}/B1",
            targets=[f"{PLATE}/B2", f"{PLATE}/B3"],
            source_volume=100.0,  # evenly split across targets
        ),
    )
    assert r.success is True, r.message


def test_transfer_rejected_when_channel_zero_has_no_tip(
    sim_node_pipetting, ros_helpers,
):
    """Pick up tips on ch 1-7 but leave ch 0 free; Transfer must reject."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["B1"], use_channels=[1]),
    )

    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, Transfer, name, "transfer"),
        Transfer.Goal(
            source=f"{PLATE}/A1",
            targets=[f"{PLATE}/A2"],
            source_volume=50.0,
        ),
    )
    assert accepted is False


# -----------------------------------------------------------------------
# Error paths that surface from the underlying LiquidHandler
# -----------------------------------------------------------------------

def test_pick_up_tips_from_missing_rack_surfaces_error(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    goal = PickUpTips.Goal(
        tip_rack="__no_such_rack__", wells=["A1"], use_channels=[0],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"), goal,
    )
    assert r.success is False
    assert r.message  # populated with the exception text


# -----------------------------------------------------------------------
# Arbitrary well / channel coverage
# -----------------------------------------------------------------------
#
# Real workflows don't always start at A1 on channel 0. These tests exercise
# middle-of-rack wells, middle channels, non-contiguous channel selections,
# and cross-row / cross-column patterns to verify nothing in the resolver,
# FSM, or lock layer accidentally hardcodes the first row or first channel.

@pytest.mark.parametrize(("well", "channel"), [
    ("A1", 0),
    ("A5", 0),
    ("H1", 0),
    ("D7", 0),
    ("H12", 0),
    ("A1", 3),
    ("E6", 4),
    ("H12", 7),
])
def test_pick_up_tip_at_arbitrary_well_and_channel(
    sim_node_pipetting, ros_helpers, well, channel,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    goal = PickUpTips.Goal(
        tip_rack=TIP_RACK, wells=[well], use_channels=[channel],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"), goal,
    )
    assert r.success is True, r.message
    assert list(r.channels_used) == [channel]

    s = _status(helper, name, ros_helpers)
    assert s.channels[channel].state == s.channels[channel].HAS_TIP
    # No other channel affected.
    for i in range(s.num_channels):
        if i == channel:
            continue
        assert s.channels[i].state == s.channels[i].FREE


@pytest.mark.parametrize(("source_well", "target_well", "channel", "volume"), [
    ("A5", "A6", 0, 25.0),
    ("D4", "D8", 2, 80.0),
    ("H12", "A1", 5, 150.0),   # last → first well; channel 5
    ("B3", "G9", 6, 200.0),
])
def test_aspirate_dispense_at_arbitrary_wells_and_channel(
    sim_node_pipetting, ros_helpers,
    source_well, target_well, channel, volume,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # Pick up a tip on the same channel from the same column as the source,
    # just so tip rack geometry matches real usage.
    pickup_well = f"{source_well[0]}1"
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=[pickup_well], use_channels=[channel],
        ),
    )

    asp = Aspirate.Goal(
        resources=[f"{PLATE}/{source_well}"],
        volumes=[volume],
        use_channels=[channel],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"), asp,
    )
    assert r.success is True, r.message
    assert list(r.final_volumes) == [volume]

    disp = Dispense.Goal(
        resources=[f"{PLATE}/{target_well}"],
        volumes=[volume],
        use_channels=[channel],
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Dispense, name, "dispense"), disp,
    )
    assert r.success is True, r.message


def test_full_column_pickup_8_channels(sim_node_pipetting, ros_helpers):
    """All 8 channels picking up in parallel from a single column (A1..H1)."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    wells = ["A1", "B1", "C1", "D1", "E1", "F1", "G1", "H1"]
    channels = list(range(8))
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=wells, use_channels=channels,
        ),
    )
    assert r.success is True, r.message
    assert sorted(r.channels_used) == channels

    s = _status(helper, name, ros_helpers)
    for ch in range(8):
        assert s.channels[ch].state == s.channels[ch].HAS_TIP


def test_full_column_pickup_then_row_dispense(sim_node_pipetting, ros_helpers):
    """8-channel pickup from col 1, aspirate col 1, dispense into col 12."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    wells = ["A1", "B1", "C1", "D1", "E1", "F1", "G1", "H1"]
    channels = list(range(8))
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=wells, use_channels=channels),
    )

    plate_col_1 = [f"{PLATE}/{w}" for w in wells]
    plate_col_12 = [f"{PLATE}/{w[0]}12" for w in wells]

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"),
        Aspirate.Goal(
            resources=plate_col_1,
            volumes=[100.0] * 8,
            use_channels=channels,
        ),
    )
    assert r.success is True, r.message

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Dispense, name, "dispense"),
        Dispense.Goal(
            resources=plate_col_12,
            volumes=[100.0] * 8,
            use_channels=channels,
        ),
    )
    assert r.success is True, r.message


@pytest.mark.parametrize("channels", [
    [0, 2, 4, 6],       # every even channel
    [1, 3, 5, 7],       # every odd channel
    [0, 7],             # just the extremes
    [2, 5],             # two middles
    [3],                # single middle channel, no ch 0 / 7
])
def test_pickup_on_non_contiguous_channels(
    sim_node_pipetting, ros_helpers, channels,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # One well per channel, walking across column 1.
    rows = ["A", "B", "C", "D", "E", "F", "G", "H"]
    wells = [f"{rows[i]}2" for i in range(len(channels))]
    goal = PickUpTips.Goal(
        tip_rack=TIP_RACK, wells=wells, use_channels=channels,
    )
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"), goal,
    )
    assert r.success is True, r.message
    assert sorted(r.channels_used) == sorted(channels)

    s = _status(helper, name, ros_helpers)
    held = {i for i, ch in enumerate(s.channels) if ch.state == ch.HAS_TIP}
    assert held == set(channels)


def test_pickup_drop_pickup_different_well_same_channel(
    sim_node_pipetting, ros_helpers,
):
    """After drop, the channel is free to pick up from a different well."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # First pickup: H12 on ch 4.
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["H12"], use_channels=[4]),
    )
    # Drop to trash.
    ros_helpers.send_goal_blocking(
        helper, _client(helper, DropTips, name, "drop_tips"),
        DropTips.Goal(
            target="trash", wells=["trash"],
            use_channels=[4], allow_nonzero_volume=True,
        ),
    )
    s = _status(helper, name, ros_helpers)
    assert s.channels[4].state == s.channels[4].FREE

    # Second pickup on same channel from a different well.
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A7"], use_channels=[4]),
    )
    assert r.success is True, r.message
    s = _status(helper, name, ros_helpers)
    assert s.channels[4].state == s.channels[4].HAS_TIP


def test_per_channel_volumes_differ_across_aspirate(
    sim_node_pipetting, ros_helpers,
):
    """4 channels, 4 different volumes, 4 different wells — verifies the
    resources/volumes/use_channels arrays are aligned correctly downstream."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    channels = [1, 3, 5, 6]
    wells = ["B4", "D4", "F4", "G4"]
    volumes = [10.0, 75.0, 200.0, 500.0]

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK,
            wells=["B2", "D2", "F2", "G2"],
            use_channels=channels,
        ),
    )

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"),
        Aspirate.Goal(
            resources=[f"{PLATE}/{w}" for w in wells],
            volumes=volumes,
            use_channels=channels,
        ),
    )
    assert r.success is True, r.message
    assert list(r.final_volumes) == volumes


def test_drop_tips_at_non_origin_wells(sim_node_pipetting, ros_helpers):
    """Drop tips back into the rack at their source positions (non-A1)."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    channels = [2, 5]
    wells = ["C8", "F11"]

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=wells, use_channels=channels,
        ),
    )

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, DropTips, name, "drop_tips"),
        DropTips.Goal(
            target=TIP_RACK, wells=wells, use_channels=channels,
            allow_nonzero_volume=True,
        ),
    )
    assert r.success is True, r.message

    s = _status(helper, name, ros_helpers)
    for ch in channels:
        assert s.channels[ch].state == s.channels[ch].FREE


def test_pickup_drop_across_multiple_columns(sim_node_pipetting, ros_helpers):
    """Pick up from columns 1, 6, 12 simultaneously (if geometry allows).

    STAR channel-to-channel spacing is fixed, so you can't pick up from
    far-apart columns in one command — the backend rejects that. This test
    verifies that we can still do it as three serial pickups without
    geometry getting in the way.
    """
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    for col, ch in [("1", 0), ("6", 3), ("12", 7)]:
        r = ros_helpers.send_goal_blocking(
            helper, _client(helper, PickUpTips, name, "pick_up_tips"),
            PickUpTips.Goal(
                tip_rack=TIP_RACK, wells=[f"A{col}"], use_channels=[ch],
            ),
        )
        assert r.success is True, f"pickup at col={col} ch={ch} failed: {r.message}"

    s = _status(helper, name, ros_helpers)
    for ch in (0, 3, 7):
        assert s.channels[ch].state == s.channels[ch].HAS_TIP
    for ch in (1, 2, 4, 5, 6):
        assert s.channels[ch].state == s.channels[ch].FREE
