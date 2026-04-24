"""Async facade over pylabrobot's LiquidHandler for the Hamilton STAR.

Keeps resource resolution, CoRe II gripper pickup/return, and deck
serialization in one place so the node module is just plumbing.

Every coroutine here runs on the :class:`AsyncioBridge` loop.  None of
these methods acquire locks or touch the FSM directly — the caller
(``action_server.py``) owns FSM gating and lock acquisition.  On success,
the caller is responsible for telling the FSM about substate changes.
"""

from __future__ import annotations

import hashlib
import json
from typing import Any, Iterable, Optional

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.liquid_handling.backends.hamilton.STAR_chatterbox import (
    STARChatterboxBackend,
)
from pylabrobot.resources.hamilton import STARDeck
from pylabrobot.resources.deck import Deck

from . import iswap_handoff
from .iswap_handoff import HandoffCalibration, StageCallback


_SIM_BACKEND_NAMES = frozenset({"sim", "simulator", "chatterbox", "star_chatterbox"})
_REAL_BACKEND_NAMES = frozenset({"", "star", "hamilton_star"})


def _make_backend(kind: str, num_channels: int) -> STARBackend:
    k = (kind or "").lower()
    if k in _SIM_BACKEND_NAMES:
        return STARChatterboxBackend(num_channels=num_channels)
    if k in _REAL_BACKEND_NAMES:
        return STARBackend()
    raise ValueError(
        f"unknown backend kind: {kind!r} "
        f"(expected one of {sorted(_REAL_BACKEND_NAMES | _SIM_BACKEND_NAMES)})"
    )


def _hash_deck(deck: Deck) -> str:
    blob = json.dumps(deck.serialize(), sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def _rewire_trash96_if_needed(deck: Deck) -> None:
    """Work around a pylabrobot serialization quirk: HamiltonSTARDeck's
    ``serialize()`` always writes ``with_trash96=False`` with the note
    "data encoded as child", expecting deserialize to re-hydrate the
    ``_trash96`` pointer from the child tree. But there's no custom
    deserialize, so ``deck._trash96`` stays ``None`` after every JSON
    round-trip even when the ``trash_core96`` child is present. That
    trips ``lh.setup()`` with "Trash area for 96-well plates was not
    created." We patch up after load instead of tracking upstream."""
    if getattr(deck, "_trash96", "missing") is None:
        try:
            deck._trash96 = deck.get_resource("trash_core96")
        except Exception:
            pass


def _resolve(lh: LiquidHandler, name: str):
    """Look up a resource by hierarchical name (e.g. ``plate_01/A1``)."""
    if not name:
        raise ValueError("resource name is required")
    # Deck.get_resource recurses into children so hierarchical names work
    # at the top level. If that fails, fall back to parent + child split.
    try:
        return lh.deck.get_resource(name)
    except Exception:
        if "/" not in name:
            raise
        parent_name, _, child_name = name.rpartition("/")
        parent = lh.deck.get_resource(parent_name)
        if hasattr(parent, "get_item"):
            return parent.get_item(child_name)
        if hasattr(parent, "get_well"):
            return parent.get_well(child_name)
        raise


def _resolve_wells(lh: LiquidHandler, rack_name: str, well_names: Iterable[str]):
    rack = lh.deck.get_resource(rack_name)
    out = []
    for wn in well_names:
        if hasattr(rack, "get_item"):
            out.append(rack.get_item(wn))
        elif hasattr(rack, "get_well"):
            out.append(rack.get_well(wn))
        else:
            out.append(_resolve(lh, f"{rack_name}/{wn}"))
    return out


def _trash_resource(lh: LiquidHandler):
    for res in lh.deck.get_all_resources():
        if res.category == "trash":
            return res
    raise LookupError("no trash resource registered on deck")


class Machine:
    """Thin async wrapper around :class:`LiquidHandler` + :class:`STARBackend`."""

    def __init__(
        self,
        core_grippers: Optional[str] = "1000uL-5mL-on-waste",
        deck_file: Optional[str] = None,
        backend: str = "star",
        num_channels: int = 8,
    ) -> None:
        self.backend = _make_backend(backend, num_channels=num_channels)
        gripper_arg = None if core_grippers in (None, "none", "") else core_grippers
        deck = None
        if deck_file:
            with open(deck_file, "r") as f:
                data = json.load(f)
            # Tolerate placeholder/empty deck files (e.g. the seed
            # ``star_deck.json`` shipped by bringup) — fall back to a
            # default STARDeck so the server still boots. Callers push
            # a real deck via the LoadDeck service afterwards.
            if isinstance(data, dict) and "type" in data:
                deck = Deck.deserialize(data)
                _rewire_trash96_if_needed(deck)
        if deck is None:
            # with_trash96=True is required by pylabrobot's setup() —
            # the check fires during lh.setup() and aborts the server
            # with "Trash area for 96-well plates was not created."
            deck = STARDeck(core_grippers=gripper_arg, with_trash96=True)
        self.lh = LiquidHandler(backend=self.backend, deck=deck)

    @property
    def deck_hash(self) -> str:
        return _hash_deck(self.lh.deck)

    async def setup(self) -> None:
        await self.lh.setup()

    async def stop(self) -> None:
        await self.lh.stop()

    # ---- deck management ----
    async def load_deck_from_file(self, deck_file: str) -> str:
        with open(deck_file, "r") as f:
            new_deck = Deck.deserialize(json.load(f))
        _rewire_trash96_if_needed(new_deck)
        # LiquidHandler.deck is a plain attribute, but STARBackend caches
        # its own ``_deck`` reference (set once at setup time). Without
        # ``set_deck`` the backend's ``pickup.resource.get_location_wrt(
        # self.deck)`` check sees the STALE deck and fails with
        # "Resources '<name>' is not in the subtree of 'deck'" even
        # though the resource IS in lh.deck. Update both.
        self.lh.deck = new_deck
        self.backend.set_deck(new_deck)
        return self.deck_hash

    def serialize_deck_json(self) -> str:
        return json.dumps(self.lh.deck.serialize(), default=str)

    def list_resources(self) -> list[tuple[str, str]]:
        out: list[tuple[str, str]] = []
        for res in self.lh.deck.get_all_resources():
            out.append((res.name, getattr(res, "category", "") or ""))
        return out

    # ---- off-deck handoff injection ----
    def assign_handoff_holder(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        size_x: float = 127.76,
        size_y: float = 85.48,
        size_z: float = 15.0,
    ) -> None:
        """Attach (or replace) a ``ResourceHolder`` on the deck at an
        off-deck coordinate under ``name``. pylabrobot accepts negative
        X and computes grip coords correctly downstream. Makes the
        handoff addressable via the deck tree the same as any on-deck
        plate site."""
        from pylabrobot.resources import Coordinate, ResourceHolder

        # Drop any existing resource with this name so re-registration
        # is idempotent.
        self.unassign_handoff_holder(name)
        holder = ResourceHolder(
            name=name,
            size_x=size_x, size_y=size_y, size_z=size_z,
        )
        self.lh.deck.assign_child_resource(
            holder, location=Coordinate(float(x), float(y), float(z)),
        )

    def unassign_handoff_holder(self, name: str) -> bool:
        """Remove the handoff's ResourceHolder from the deck if
        present. Returns True iff a resource was removed."""
        try:
            existing = self.lh.deck.get_resource(name)
        except Exception:
            return False
        parent = getattr(existing, "parent", None)
        if parent is None:
            return False
        try:
            parent.unassign_child_resource(existing)
        except Exception:
            return False
        return True

    # ---- backend module init ----
    async def initialize_module(self, module: str) -> None:
        m = module.lower()
        if m in ("pip", "pipetting_channels"):
            await self.backend.initialize_pipetting_channels()
        elif m == "iswap":
            await self.backend.initialize_iswap()
        elif m == "autoload":
            await self.backend.initialize_autoload()
        elif m in ("core96", "core_96"):
            await self.backend.initialize_core_96_head()
        elif m == "hhc":
            await self.backend.initialize_hhc()
        else:
            raise ValueError(f"unknown module: {module}")

    # ---- iSWAP "safe to move" prep ----
    async def free_iswap_y_range(self) -> None:
        """Position the pipetting channels out of the iSWAP Y path
        (firmware C0 FY). Required before any forward iSWAP Y move to
        an on-deck plate site."""
        await iswap_handoff.free_iswap_y_range(self.backend)

    # ---- 1-8 channel pipetting ----
    async def pick_up_tips(
        self,
        tip_rack: str,
        wells: list[str],
        use_channels: Optional[list[int]] = None,
        offsets_z: Optional[list[float]] = None,
    ) -> None:
        spots = _resolve_wells(self.lh, tip_rack, wells)
        kwargs: dict[str, Any] = {}
        if use_channels:
            kwargs["use_channels"] = use_channels
        await self.lh.pick_up_tips(spots, **kwargs)

    async def drop_tips(
        self,
        target: str,
        wells: list[str],
        use_channels: Optional[list[int]] = None,
        allow_nonzero_volume: bool = False,
    ) -> None:
        if target.lower() == "trash":
            spots = [_trash_resource(self.lh)] * max(1, len(wells))
        else:
            spots = _resolve_wells(self.lh, target, wells)
        kwargs: dict[str, Any] = {"allow_nonzero_volume": allow_nonzero_volume}
        if use_channels:
            kwargs["use_channels"] = use_channels
        await self.lh.drop_tips(spots, **kwargs)

    async def aspirate(
        self,
        resources: list[str],
        volumes: list[float],
        use_channels: Optional[list[int]] = None,
        flow_rates: Optional[list[float]] = None,
        offsets_z: Optional[list[float]] = None,
        liquid_height: Optional[list[float]] = None,
    ) -> None:
        refs = [_resolve(self.lh, r) for r in resources]
        kwargs: dict[str, Any] = {}
        if use_channels:
            kwargs["use_channels"] = use_channels
        if flow_rates:
            kwargs["flow_rates"] = flow_rates
        if liquid_height:
            kwargs["liquid_height"] = liquid_height
        await self.lh.aspirate(refs, volumes, **kwargs)

    async def dispense(
        self,
        resources: list[str],
        volumes: list[float],
        use_channels: Optional[list[int]] = None,
        flow_rates: Optional[list[float]] = None,
        offsets_z: Optional[list[float]] = None,
        liquid_height: Optional[list[float]] = None,
        blow_out_air_volume: Optional[list[float]] = None,
    ) -> None:
        refs = [_resolve(self.lh, r) for r in resources]
        kwargs: dict[str, Any] = {}
        if use_channels:
            kwargs["use_channels"] = use_channels
        if flow_rates:
            kwargs["flow_rates"] = flow_rates
        if liquid_height:
            kwargs["liquid_height"] = liquid_height
        if blow_out_air_volume:
            kwargs["blow_out_air_volume"] = blow_out_air_volume
        await self.lh.dispense(refs, volumes, **kwargs)

    async def transfer(
        self,
        source: str,
        targets: list[str],
        source_volume: float,
        target_volumes: Optional[list[float]] = None,
        aspiration_flow_rate: Optional[float] = None,
        dispense_flow_rate: Optional[float] = None,
    ) -> None:
        src = _resolve(self.lh, source)
        tgts = [_resolve(self.lh, t) for t in targets]
        kwargs: dict[str, Any] = {}
        # pylabrobot rejects passing both source_vol and target_vols; prefer
        # target_volumes when supplied (per-target control), otherwise split
        # source_volume evenly across targets.
        if target_volumes:
            kwargs["target_vols"] = target_volumes
        else:
            kwargs["source_vol"] = source_volume
        if aspiration_flow_rate:
            kwargs["aspiration_flow_rate"] = aspiration_flow_rate
        if dispense_flow_rate:
            kwargs["dispense_flow_rates"] = dispense_flow_rate
        await self.lh.transfer(src, tgts, **kwargs)

    # ---- 96-head ----
    async def pick_up_tips96(self, tip_rack: str, offset: Optional[Any] = None) -> None:
        rack = self.lh.deck.get_resource(tip_rack)
        kwargs: dict[str, Any] = {}
        if offset is not None:
            kwargs["offset"] = offset
        await self.lh.pick_up_tips96(rack, **kwargs)

    async def drop_tips96(
        self,
        target: str,
        offset: Optional[Any] = None,
        allow_nonzero_volume: bool = False,
    ) -> None:
        if target.lower() == "trash":
            res = _trash_resource(self.lh)
        else:
            res = self.lh.deck.get_resource(target)
        kwargs: dict[str, Any] = {"allow_nonzero_volume": allow_nonzero_volume}
        if offset is not None:
            kwargs["offset"] = offset
        await self.lh.drop_tips96(res, **kwargs)

    async def aspirate96(
        self,
        resource: str,
        volume: float,
        offset: Optional[Any] = None,
        flow_rate: Optional[float] = None,
    ) -> None:
        res = _resolve(self.lh, resource)
        kwargs: dict[str, Any] = {}
        if offset is not None:
            kwargs["offset"] = offset
        if flow_rate:
            kwargs["flow_rate"] = flow_rate
        await self.lh.aspirate96(res, volume, **kwargs)

    async def dispense96(
        self,
        resource: str,
        volume: float,
        offset: Optional[Any] = None,
        flow_rate: Optional[float] = None,
        blow_out_air_volume: Optional[float] = None,
    ) -> None:
        res = _resolve(self.lh, resource)
        kwargs: dict[str, Any] = {}
        if offset is not None:
            kwargs["offset"] = offset
        if flow_rate:
            kwargs["flow_rate"] = flow_rate
        if blow_out_air_volume:
            kwargs["blow_out_air_volume"] = blow_out_air_volume
        await self.lh.dispense96(res, volume, **kwargs)

    # ---- moves ----
    async def move_resource(
        self,
        resource: str,
        to: str,
        transport: str = "auto",
        pickup_direction: str = "",
        drop_direction: str = "",
        pickup_offset: Optional[Any] = None,
        destination_offset: Optional[Any] = None,
        pickup_distance_from_top: Optional[float] = None,
        use_unsafe_hotel: bool = False,
        hotel_pickup: bool = False,
        hotel_drop: bool = False,
        hotel_depth: Optional[float] = None,
        hotel_clearance_height: Optional[float] = None,
        iswap_collision_control_level: Optional[int] = None,
        iswap_fold_up_sequence_at_the_end_of_process: Optional[bool] = None,
    ) -> None:
        from pylabrobot.liquid_handling.standard import GripDirection

        def _as_grip_dir(s: str) -> Optional[GripDirection]:
            # pylabrobot expects the GripDirection enum; our action
            # wire format uses the string name ("FRONT" / "BACK" /
            # "LEFT" / "RIGHT"). Case-insensitive; empty = caller
            # wants pylabrobot's default, so return None to skip.
            if not (s or "").strip():
                return None
            try:
                return GripDirection[s.upper()]
            except KeyError:
                raise ValueError(
                    f"invalid grip direction {s!r}; expected one of "
                    f"{[d.name for d in GripDirection]}"
                )

        src = _resolve(self.lh, resource)
        # `to` is always a deck-resource name. Registered handoffs are
        # auto-injected as off-deck ResourceHolders at __init__ /
        # define_handoff time, so they resolve here exactly like on-deck
        # resources.
        dst = _resolve(self.lh, to)
        # Diagnostic: surface the parent-chain + deck-identity state that
        # pylabrobot's is_in_subtree_of check consumes. A "not in subtree"
        # failure at the lh.move_resource call below almost always means
        # one of (a) a stale Resource instance whose parent got detached
        # by a deck swap, or (b) self.lh.deck != the deck that src's
        # parent chain terminates at.
        import sys
        src_chain: list[str] = []
        node = src
        while node is not None:
            src_chain.append(getattr(node, "name", "?"))
            node = getattr(node, "parent", None)
        dst_chain: list[str] = []
        node = dst
        while node is not None:
            dst_chain.append(getattr(node, "name", "?"))
            node = getattr(node, "parent", None)
        print(
            f"[move_resource] src={resource} chain={' > '.join(src_chain)} "
            f"dst={to} chain={' > '.join(dst_chain)} "
            f"lh.deck={self.lh.deck.name} "
            f"src_in_deck={src.is_in_subtree_of(self.lh.deck)} "
            f"dst_in_deck={dst.is_in_subtree_of(self.lh.deck)} "
            f"deck_children={[c.name for c in self.lh.deck.children]}",
            file=sys.stderr, flush=True,
        )
        pickup_dir = _as_grip_dir(pickup_direction)
        drop_dir = _as_grip_dir(drop_direction)

        # Decide hotel application per side. ``use_unsafe_hotel=True``
        # (legacy) flips BOTH sides on; new callers use the explicit
        # ``hotel_pickup`` / ``hotel_drop`` flags. For a typical
        # on-deck → off-deck transfer the source plate is on-deck
        # (normal ``iswap_get_plate`` / firmware ``PP``) and only the
        # drop is a hotel (``put_in_hotel`` / firmware ``PI``). The
        # reverse — picking up FROM an incubator dock — flips it.
        do_hotel_pickup = bool(hotel_pickup or use_unsafe_hotel)
        do_hotel_drop = bool(hotel_drop or use_unsafe_hotel)

        if not (do_hotel_pickup or do_hotel_drop):
            # Simple on-deck → on-deck path. One call, pylabrobot
            # handles orchestration + intermediate safe moves.
            kwargs: dict[str, Any] = {}
            if pickup_dir is not None:
                kwargs["pickup_direction"] = pickup_dir
            if drop_dir is not None:
                kwargs["drop_direction"] = drop_dir
            if pickup_offset is not None:
                kwargs["pickup_offset"] = pickup_offset
            if destination_offset is not None:
                kwargs["destination_offset"] = destination_offset
            if pickup_distance_from_top is not None:
                kwargs["pickup_distance_from_top"] = pickup_distance_from_top
            if iswap_collision_control_level is not None:
                kwargs["iswap_collision_control_level"] = int(
                    iswap_collision_control_level,
                )
            if iswap_fold_up_sequence_at_the_end_of_process is not None:
                kwargs["iswap_fold_up_sequence_at_the_end_of_process"] = bool(
                    iswap_fold_up_sequence_at_the_end_of_process,
                )
            await self.lh.move_resource(src, dst, **kwargs)
            return

        # Hotel-atomic path: orchestrate pick + drop manually so each
        # side gets ``use_unsafe_hotel`` set independently. pylabrobot's
        # own ``move_resource`` forwards the same kwargs to both sides
        # and can't do per-side hotel flags — calling ``PO``/``PI`` on
        # an on-deck endpoint raises firmware R027 PositionNotReachable.
        pickup_kwargs: dict[str, Any] = {}
        if pickup_dir is not None:
            pickup_kwargs["direction"] = pickup_dir
        if pickup_offset is not None:
            pickup_kwargs["offset"] = pickup_offset
        if pickup_distance_from_top is not None:
            pickup_kwargs["pickup_distance_from_top"] = pickup_distance_from_top
        if do_hotel_pickup:
            pickup_kwargs["use_unsafe_hotel"] = True
            if hotel_depth is not None:
                pickup_kwargs["hotel_depth"] = float(hotel_depth)
            if hotel_clearance_height is not None:
                pickup_kwargs["hotel_clearance_height"] = float(hotel_clearance_height)

        drop_kwargs: dict[str, Any] = {}
        if drop_dir is not None:
            drop_kwargs["direction"] = drop_dir
        if destination_offset is not None:
            drop_kwargs["offset"] = destination_offset
        if do_hotel_drop:
            drop_kwargs["use_unsafe_hotel"] = True
            if hotel_depth is not None:
                drop_kwargs["hotel_depth"] = float(hotel_depth)
            if hotel_clearance_height is not None:
                drop_kwargs["hotel_clearance_height"] = float(hotel_clearance_height)

        await self.lh.pick_up_resource(resource=src, **pickup_kwargs)
        await self.lh.drop_resource(destination=dst, **drop_kwargs)

    # ---- iSWAP handoff transfer ----
    async def handoff_transfer(
        self,
        calibration: HandoffCalibration,
        on_deck_resource: str,
        direction: str,
        pickup_distance_from_top: float = iswap_handoff.DEFAULT_PICKUP_DISTANCE_FROM_TOP,
        safe_y: float = iswap_handoff.SAFE_Y,
        traverse_z: float = iswap_handoff.TRAVERSE_Z,
        handoff_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
        on_stage: Optional[StageCallback] = None,
    ) -> None:
        """Manual-jog iSWAP transfer between an on-deck plate site and a
        calibrated off-deck handoff position. The caller supplies the
        ``calibration`` it read from ROS params at goal time."""
        pickup_xyz, release_xyz = iswap_handoff.resolve_endpoints(
            self.lh, calibration, on_deck_resource, direction,
            pickup_distance_from_top,
        )
        # handoff_offset applies to whichever endpoint is off-deck.
        if direction == "to_handoff":
            pickup_offset = (0.0, 0.0, 0.0)
            release_offset = handoff_offset
        else:
            pickup_offset = handoff_offset
            release_offset = (0.0, 0.0, 0.0)
        await iswap_handoff.transfer(
            self.backend,
            pickup_xyz=pickup_xyz,
            release_xyz=release_xyz,
            plate_width=calibration.plate_width,
            traverse_z=traverse_z,
            safe_y=safe_y,
            rotation=calibration.rotation,
            wrist=calibration.wrist,
            pickup_offset=pickup_offset,
            release_offset=release_offset,
            on_stage=on_stage,
        )

    # ---- CoRe II gripper ----
    async def pick_up_core_gripper(
        self,
        gripper_resource: str,
        front_channel: int,
    ) -> None:
        self.lh.deck.get_resource(gripper_resource)  # validate presence
        await self.backend.pick_up_core_gripper_tools(front_channel=front_channel)

    async def return_core_gripper(self, gripper_resource: str) -> None:
        self.lh.deck.get_resource(gripper_resource)
        await self.backend.return_core_gripper_tools()

    # ---- jog ----
    async def jog_channel(self, channel: int, axis: str, target: float) -> None:
        a = axis.lower()
        if a == "x":
            await self.lh.move_channel_x(channel, target)
        elif a == "y":
            await self.lh.move_channel_y(channel, target)
        elif a == "z":
            await self.lh.move_channel_z(channel, target)
        else:
            raise ValueError(f"unknown axis: {axis}")

    # ---- abort ----
    async def abort_motion(self) -> None:
        # STARBackend doesn't expose a single "abort" — the safe thing that
        # keeps the USB link alive is to request initialization status (a
        # no-op command that interrupts a running task on the firmware side
        # when it arrives between steps) and leave the driver to the FSM's
        # goal-cancellation to unwind in-flight coroutines.
        await self.backend.request_instrument_initialization_status()
