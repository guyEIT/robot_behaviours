"""Operational FSM + orthogonal substates for the Hamilton STAR action server.

Every ROS action/service gate consults :meth:`MachineFSM.can_accept` *before*
doing any hardware work; the lock layer is a secondary, finer-grained gate.
"""

from __future__ import annotations

import enum
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional

from transitions import Machine


class ActionKind(str, enum.Enum):
    # shared (non-exclusive)
    PICK_UP_TIPS = "pick_up_tips"
    DROP_TIPS = "drop_tips"
    ASPIRATE = "aspirate"
    DISPENSE = "dispense"
    TRANSFER = "transfer"
    # exclusive
    PICK_UP_TIPS_96 = "pick_up_tips_96"
    DROP_TIPS_96 = "drop_tips_96"
    ASPIRATE_96 = "aspirate_96"
    DISPENSE_96 = "dispense_96"
    MOVE_RESOURCE = "move_resource"
    PICK_UP_CORE_GRIPPER = "pick_up_core_gripper"
    RETURN_CORE_GRIPPER = "return_core_gripper"
    JOG_CHANNEL = "jog_channel"
    INIT_MODULE = "init_module"
    LOAD_DECK = "load_deck"
    HANDOFF_TRANSFER = "handoff_transfer"


SHARED_KINDS = frozenset({
    ActionKind.PICK_UP_TIPS,
    ActionKind.DROP_TIPS,
    ActionKind.ASPIRATE,
    ActionKind.DISPENSE,
    ActionKind.TRANSFER,
})

EXCLUSIVE_KINDS = frozenset({
    ActionKind.PICK_UP_TIPS_96,
    ActionKind.DROP_TIPS_96,
    ActionKind.ASPIRATE_96,
    ActionKind.DISPENSE_96,
    ActionKind.MOVE_RESOURCE,
    ActionKind.PICK_UP_CORE_GRIPPER,
    ActionKind.RETURN_CORE_GRIPPER,
    ActionKind.JOG_CHANNEL,
    ActionKind.INIT_MODULE,
    ActionKind.LOAD_DECK,
    ActionKind.HANDOFF_TRANSFER,
})

# Kinds that physically move the pipetting channels (or the CoRe 96 head)
# and therefore invalidate the C0-FY "free iSWAP Y range" guarantee. The
# handoff action re-runs FY when it sees ``iswap_y_path_clear`` go False.
KINDS_INVALIDATING_ISWAP_Y_PATH = frozenset(SHARED_KINDS | {
    ActionKind.PICK_UP_TIPS_96,
    ActionKind.DROP_TIPS_96,
    ActionKind.ASPIRATE_96,
    ActionKind.DISPENSE_96,
    ActionKind.PICK_UP_CORE_GRIPPER,
    ActionKind.RETURN_CORE_GRIPPER,
    ActionKind.JOG_CHANNEL,
    ActionKind.MOVE_RESOURCE,
})


class ChannelState(enum.Enum):
    FREE = 0
    HAS_TIP = 1
    HOLDS_GRIPPER = 2


@dataclass
class GateResult:
    ok: bool
    reason: str = ""


OK = GateResult(True)


def Reject(reason: str) -> GateResult:
    return GateResult(False, reason)


@dataclass
class FSMEvent:
    kind: str
    message: str = ""
    severity: int = 0  # 0=INFO, 1=WARN, 2=ERROR
    details: dict[str, str] = field(default_factory=dict)


STATES = [
    "Disconnected",
    "Initializing",
    "Idle",
    "BusyShared",
    "BusyExclusive",
    "Aborting",
    "Error",
]


class MachineFSM:
    """Thread-safe FSM wrapping ``transitions.Machine``.

    All public methods are safe to call from any thread — rclpy callback
    threads, the asyncio loop thread, and status-publisher timers share one
    instance via an internal ``RLock``.
    """

    def __init__(
        self,
        num_channels: int = 8,
        event_sink: Optional[Callable[[FSMEvent], None]] = None,
    ):
        self._lock = threading.RLock()
        self._event_sink = event_sink or (lambda _e: None)

        self.num_channels: int = num_channels
        self.channels: dict[int, ChannelState] = {
            i: ChannelState.FREE for i in range(num_channels)
        }
        self.core_parked: bool = True
        self.core_channels: Optional[tuple[int, int]] = None
        self.core96_loaded: bool = False
        self.deck_loaded: bool = False
        self.deck_hash: str = ""
        self.error_reason: str = ""
        # iSWAP "safe to move" substate — see iswap_handoff.py for the
        # motion rules these guard.
        self.iswap_initialized: bool = False
        self.iswap_y_path_clear: bool = False
        self.iswap_holding_plate: bool = False
        # iSWAP drive fault tracking. Set when any firmware exception
        # propagates out of an iSWAP-touching action; cleared only after
        # a successful soft-recovery round-trip (reset_error). Gates new
        # iSWAP-using goals so we don't re-trip the same fault.
        self.iswap_drive_faulted: bool = False
        # True when the iSWAP is known to be at a retracted / parked
        # pose (post-init or post-park). Cleared at the start of any
        # iSWAP motion. Handoff execute path auto-parks when False.
        self.iswap_at_safe_pose: bool = False
        # Diagnostic: last handoff stage entered (0 = none / reset).
        # Set in the on_stage callback; preserved across failure so
        # operators can tell whether a plate was actually gripped.
        self.last_handoff_stage_reached: int = 0

        self._shared_goal_count: int = 0
        self._exclusive_op: str = ""

        self._machine = Machine(
            model=self,
            states=STATES,
            initial="Disconnected",
            auto_transitions=False,
            ignore_invalid_triggers=False,
            after_state_change="_on_any_transition",
        )
        add = self._machine.add_transition
        add("begin_initializing", "Disconnected", "Initializing")
        add("initialization_ok", "Initializing", "Idle")
        add("initialization_failed", "Initializing", "Error")
        add("shared_acquired", "Idle", "BusyShared")
        add("last_shared_released", "BusyShared", "Idle")
        add("exclusive_acquired", "Idle", "BusyExclusive")
        add("exclusive_released", "BusyExclusive", "Idle")
        add("begin_aborting", ["Idle", "BusyShared", "BusyExclusive"], "Aborting")
        add("abort_ok", "Aborting", "Idle")
        add("abort_failed", "Aborting", "Error")
        add("fault", ["Idle", "BusyShared", "BusyExclusive", "Aborting", "Initializing"], "Error")
        add("error_reset", "Error", "Idle")

    def _on_any_transition(self) -> None:
        self._event_sink(FSMEvent(
            kind="fsm_transition",
            message=f"state -> {self.state}",
            details={"op_state": self.state, "error_reason": self.error_reason},
        ))

    def connect(self) -> None:
        with self._lock:
            self.begin_initializing()

    def finish_init(self, ok: bool, reason: str = "") -> None:
        with self._lock:
            if ok:
                self.initialization_ok()
            else:
                self.error_reason = reason or "initialization failed"
                self.initialization_failed()

    def mark_error(self, reason: str) -> None:
        # Legacy shim: pessimistically invalidate y_path and init because
        # whatever just faulted may have moved the arm / channels; but do
        # NOT mark the iSWAP drive faulted without evidence (callers that
        # know the fault came from an iSWAP op should call
        # mark_error_and_invalidate with mark_iswap_faulted=True).
        self.mark_error_and_invalidate(reason)

    def mark_error_and_invalidate(
        self,
        reason: str,
        *,
        invalidate_iswap_initialized: bool = True,
        invalidate_y_path: bool = True,
        mark_iswap_faulted: bool = False,
    ) -> None:
        """Transition to Error and clear the substate flags that an
        exception invalidates — all under a single lock so observers
        see a consistent snapshot. ``iswap_holding_plate`` is
        intentionally preserved (operator-acknowledged) and
        ``last_handoff_stage_reached`` is preserved for post-mortem.
        ``iswap_at_safe_pose`` is always cleared because a fault means
        the arm pose is unknown."""
        with self._lock:
            if self.state == "Error":
                self._event_sink(FSMEvent(
                    kind="error", severity=2, message=reason,
                ))
                return
            self.error_reason = reason
            details: dict[str, str] = {}
            if invalidate_iswap_initialized and self.iswap_initialized:
                self.iswap_initialized = False
                details["iswap_initialized"] = "False"
            if invalidate_y_path and self.iswap_y_path_clear:
                self.iswap_y_path_clear = False
                details["iswap_y_path_clear"] = "False"
            if mark_iswap_faulted and not self.iswap_drive_faulted:
                self.iswap_drive_faulted = True
                details["iswap_drive_faulted"] = "True"
            if self.iswap_at_safe_pose:
                self.iswap_at_safe_pose = False
                details["iswap_at_safe_pose"] = "False"
            self.fault()
            self._event_sink(FSMEvent(
                kind="error", severity=2, message=reason, details=details,
            ))

    def reset_error(self, acknowledgment: str) -> GateResult:
        with self._lock:
            if not acknowledgment.strip():
                return Reject("acknowledgment must be non-empty")
            if self.state != "Error":
                return Reject(f"not in Error state (currently {self.state})")
            prior = self.error_reason
            self.error_reason = ""
            self.error_reset()
            self._event_sink(FSMEvent(
                kind="reset_error",
                message=f"operator cleared error: {prior}",
                details={"acknowledgment": acknowledgment},
            ))
            return OK

    def begin_abort(self) -> GateResult:
        with self._lock:
            if self.state in ("Error", "Aborting", "Disconnected", "Initializing"):
                return Reject(f"cannot abort from {self.state}")
            self.begin_aborting()
            self._event_sink(FSMEvent(kind="abort_motion", severity=1, message="abort begun"))
            return OK

    def finish_abort(self, ok: bool, reason: str = "") -> None:
        with self._lock:
            if ok:
                self._shared_goal_count = 0
                self._exclusive_op = ""
                self.abort_ok()
            else:
                self.error_reason = reason or "abort failed"
                self.abort_failed()

    def can_accept(self, kind: ActionKind, params: dict[str, Any]) -> GateResult:
        with self._lock:
            if self.state == "Error":
                return Reject(f"machine in error state: {self.error_reason}")
            if self.state in ("Disconnected", "Initializing", "Aborting"):
                return Reject(f"machine not ready ({self.state})")

            if kind in SHARED_KINDS:
                if self.state == "BusyExclusive":
                    return Reject(f"exclusive op in progress ({self._exclusive_op})")
                return self._check_shared_params(kind, params)

            if kind in EXCLUSIVE_KINDS:
                if self.state != "Idle":
                    detail = f" ({self._exclusive_op})" if self._exclusive_op else ""
                    return Reject(
                        f"exclusive op requires Idle, currently {self.state}{detail}"
                    )
                return self._check_exclusive_params(kind, params)

            return Reject(f"unknown action kind: {kind}")

    def _check_shared_params(self, kind: ActionKind, params: dict[str, Any]) -> GateResult:
        channels: list[int] = list(params.get("use_channels") or [])
        for ch in channels:
            if ch < 0 or ch >= self.num_channels:
                return Reject(f"channel {ch} out of range (0..{self.num_channels - 1})")
            if self.channels[ch] == ChannelState.HOLDS_GRIPPER:
                return Reject(f"channel {ch} holds CoRe gripper")

        if kind == ActionKind.PICK_UP_TIPS:
            for ch in channels:
                if self.channels[ch] != ChannelState.FREE:
                    return Reject(
                        f"channel {ch} not free (state={self.channels[ch].name})"
                    )
        elif kind == ActionKind.DROP_TIPS:
            for ch in channels:
                if self.channels[ch] != ChannelState.HAS_TIP:
                    return Reject(f"channel {ch} has no tip to drop")
        elif kind in (ActionKind.ASPIRATE, ActionKind.DISPENSE, ActionKind.TRANSFER):
            for ch in channels:
                if self.channels[ch] != ChannelState.HAS_TIP:
                    return Reject(f"channel {ch} has no tip")
        return OK

    def _check_exclusive_params(self, kind: ActionKind, params: dict[str, Any]) -> GateResult:
        if kind == ActionKind.PICK_UP_CORE_GRIPPER:
            if not self.core_parked:
                return Reject("CoRe gripper already held")
            fc = params.get("front_channel")
            if fc is None:
                return Reject("front_channel required")
            if fc < 1 or fc >= self.num_channels:
                return Reject(
                    f"front_channel must be in [1, {self.num_channels - 1}] "
                    f"(back_channel = front_channel - 1)"
                )
            for ch in (fc - 1, fc):
                if self.channels[ch] != ChannelState.FREE:
                    return Reject(
                        f"channel {ch} not free (state={self.channels[ch].name})"
                    )
        elif kind == ActionKind.RETURN_CORE_GRIPPER:
            if self.core_parked:
                return Reject("CoRe gripper is already parked")
        elif kind == ActionKind.MOVE_RESOURCE:
            transport = (params.get("transport") or "auto").lower()
            if transport == "core" and self.core_parked:
                return Reject("MoveResource transport=core requires a held CoRe gripper")
            # When routing through the firmware hotel (either via
            # ``to_handoff`` or explicit ``use_unsafe_hotel``), the
            # iSWAP is mandatory — share the same health gates as
            # HandoffTransfer / HotelTransfer.
            uses_iswap_hotel = bool(
                params.get("to_handoff") or params.get("use_unsafe_hotel")
            )
            if uses_iswap_hotel:
                if self.iswap_drive_faulted:
                    return Reject(
                        "iSWAP drive is faulted — call reset_error "
                        "(or restart the action server if soft recovery fails)"
                    )
                if not self.iswap_initialized:
                    return Reject(
                        "iSWAP not initialized — call initialize_module iswap first"
                    )
                if self.iswap_holding_plate:
                    return Reject(
                        "iSWAP may still be holding a plate — "
                        "inspect the gripper, then call "
                        "~/acknowledge_plate_released"
                    )
        elif kind == ActionKind.PICK_UP_TIPS_96:
            if self.core96_loaded:
                return Reject("96-head already has tips")
        elif kind == ActionKind.DROP_TIPS_96:
            if not self.core96_loaded:
                return Reject("96-head has no tips to drop")
        elif kind in (ActionKind.ASPIRATE_96, ActionKind.DISPENSE_96):
            if not self.core96_loaded:
                return Reject("96-head has no tips")
        elif kind == ActionKind.HANDOFF_TRANSFER:
            if self.iswap_drive_faulted:
                return Reject(
                    "iSWAP drive is faulted — call reset_error "
                    "(or restart the action server if soft recovery fails)"
                )
            if not self.iswap_initialized:
                return Reject(
                    "iSWAP not initialized — call initialize_module iswap first"
                )
            if self.iswap_holding_plate:
                return Reject(
                    "iSWAP may still be holding a plate — "
                    "inspect the gripper, then call "
                    "~/acknowledge_plate_released (run iswap-release first "
                    "if the plate is actually stuck)"
                )
            direction = (params.get("direction") or "").lower()
            if direction not in ("to_handoff", "from_handoff"):
                return Reject(
                    f"direction must be 'to_handoff' or 'from_handoff', got {direction!r}"
                )
            if not (params.get("calibration_name") or "").strip():
                return Reject("calibration_name is required")
            if not (params.get("on_deck_resource") or "").strip():
                return Reject("on_deck_resource is required")
            # Note: iswap_y_path_clear is NOT a gate — the action server
            # re-runs C0 FY when it sees this false. Kept in snapshot()
            # so operators can observe when FY will run.
        return OK

    def on_shared_accepted(self, kind: ActionKind) -> None:
        with self._lock:
            self._shared_goal_count += 1
            if self.state == "Idle":
                self.shared_acquired()
            if kind in KINDS_INVALIDATING_ISWAP_Y_PATH and self.iswap_y_path_clear:
                self._invalidate_iswap_y_path_clear_locked(str(kind.value))

    def on_shared_finished(self, kind: ActionKind) -> None:
        with self._lock:
            self._shared_goal_count = max(0, self._shared_goal_count - 1)
            if self._shared_goal_count == 0 and self.state == "BusyShared":
                self.last_shared_released()

    def on_exclusive_accepted(self, kind: ActionKind) -> None:
        with self._lock:
            self._exclusive_op = kind.value
            self.exclusive_acquired()
            if kind in KINDS_INVALIDATING_ISWAP_Y_PATH and self.iswap_y_path_clear:
                self._invalidate_iswap_y_path_clear_locked(str(kind.value))

    def on_exclusive_finished(self, kind: ActionKind) -> None:
        with self._lock:
            self._exclusive_op = ""
            if self.state == "BusyExclusive":
                self.exclusive_released()

    def mark_tips_picked_up(self, channels: Iterable[int]) -> None:
        channels = list(channels)
        with self._lock:
            for ch in channels:
                self.channels[ch] = ChannelState.HAS_TIP
            self._event_sink(FSMEvent(
                kind="tip_picked_up", message=f"channels={channels}",
            ))

    def mark_tips_dropped(self, channels: Iterable[int]) -> None:
        channels = list(channels)
        with self._lock:
            for ch in channels:
                self.channels[ch] = ChannelState.FREE
            self._event_sink(FSMEvent(
                kind="tip_dropped", message=f"channels={channels}",
            ))

    def mark_core_gripper_picked_up(self, front_channel: int) -> None:
        back_channel = front_channel - 1
        with self._lock:
            self.core_parked = False
            self.core_channels = (back_channel, front_channel)
            self.channels[back_channel] = ChannelState.HOLDS_GRIPPER
            self.channels[front_channel] = ChannelState.HOLDS_GRIPPER
            self._event_sink(FSMEvent(
                kind="core_gripper_picked_up",
                message=f"channels=({back_channel},{front_channel})",
            ))

    def mark_core_gripper_returned(self) -> None:
        with self._lock:
            c1, c2 = self.core_channels or (None, None)
            self.core_parked = True
            self.core_channels = None
            if c1 is not None:
                self.channels[c1] = ChannelState.FREE
            if c2 is not None:
                self.channels[c2] = ChannelState.FREE
            self._event_sink(FSMEvent(
                kind="core_gripper_returned", message=f"channels=({c1},{c2})",
            ))

    def mark_core96_loaded(self, loaded: bool) -> None:
        with self._lock:
            self.core96_loaded = loaded
            self._event_sink(FSMEvent(
                kind="core96_tips_loaded" if loaded else "core96_tips_dropped",
                message=str(loaded),
            ))

    def mark_deck_loaded(self, deck_hash: str) -> None:
        with self._lock:
            self.deck_loaded = True
            self.deck_hash = deck_hash
            self._event_sink(FSMEvent(
                kind="deck_loaded", message=f"hash={deck_hash[:12]}",
            ))

    # ---- iSWAP "safe to move" substate ----
    def mark_iswap_initialized(self, initialized: bool) -> None:
        with self._lock:
            self.iswap_initialized = initialized
            if not initialized:
                # Re-init also re-homes, so the Y-path promise is gone too.
                self.iswap_y_path_clear = False
            self._event_sink(FSMEvent(
                kind="iswap_state",
                message=f"iswap_initialized={initialized}",
                details={"iswap_initialized": str(initialized)},
            ))

    def mark_iswap_y_path_clear(self, clear: bool) -> None:
        with self._lock:
            self.iswap_y_path_clear = clear
            self._event_sink(FSMEvent(
                kind="iswap_state",
                message=f"iswap_y_path_clear={clear}",
                details={"iswap_y_path_clear": str(clear)},
            ))

    def mark_iswap_holding_plate(self, holding: bool) -> None:
        with self._lock:
            self.iswap_holding_plate = holding
            self._event_sink(FSMEvent(
                kind="iswap_state",
                message=f"iswap_holding_plate={holding}",
                details={"iswap_holding_plate": str(holding)},
            ))

    def mark_iswap_drive_faulted(self, faulted: bool) -> None:
        with self._lock:
            self.iswap_drive_faulted = faulted
            self._event_sink(FSMEvent(
                kind="iswap_state",
                severity=2 if faulted else 0,
                message=f"iswap_drive_faulted={faulted}",
                details={"iswap_drive_faulted": str(faulted)},
            ))

    def mark_iswap_at_safe_pose(self, safe: bool) -> None:
        with self._lock:
            self.iswap_at_safe_pose = safe
            self._event_sink(FSMEvent(
                kind="iswap_state",
                message=f"iswap_at_safe_pose={safe}",
                details={"iswap_at_safe_pose": str(safe)},
            ))

    def mark_handoff_stage(self, stage: int) -> None:
        """Record the last handoff stage entered. Stage 0 = reset
        (on successful completion); 1-6 = the stages of transfer()."""
        with self._lock:
            self.last_handoff_stage_reached = int(stage)
            self._event_sink(FSMEvent(
                kind="handoff_stage",
                message=f"stage={stage}",
                details={"last_handoff_stage_reached": str(stage)},
            ))

    def _invalidate_iswap_y_path_clear_locked(self, reason: str) -> None:
        """Internal: clear the Y-path flag because some other action
        just moved the channels. Caller holds ``self._lock``."""
        self.iswap_y_path_clear = False
        self._event_sink(FSMEvent(
            kind="iswap_state",
            message=f"iswap_y_path_clear=False (invalidated by {reason})",
            details={"iswap_y_path_clear": "False", "reason": reason},
        ))

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "op_state": self.state,
                "error_reason": self.error_reason,
                "connected": self.state != "Disconnected",
                "initialized": self.state not in ("Disconnected", "Initializing"),
                "deck_loaded": self.deck_loaded,
                "deck_hash": self.deck_hash,
                "num_channels": self.num_channels,
                "channels": dict(self.channels),
                "busy_channels": [
                    ch for ch, st in self.channels.items() if st != ChannelState.FREE
                ],
                "core_parked": self.core_parked,
                "core_channels": list(self.core_channels) if self.core_channels else [],
                "core96_loaded": self.core96_loaded,
                "iswap_initialized": self.iswap_initialized,
                "iswap_y_path_clear": self.iswap_y_path_clear,
                "iswap_holding_plate": self.iswap_holding_plate,
                "iswap_drive_faulted": self.iswap_drive_faulted,
                "iswap_at_safe_pose": self.iswap_at_safe_pose,
                "last_handoff_stage_reached": self.last_handoff_stage_reached,
                "exclusive_op": self._exclusive_op,
                "shared_goal_count": self._shared_goal_count,
            }
