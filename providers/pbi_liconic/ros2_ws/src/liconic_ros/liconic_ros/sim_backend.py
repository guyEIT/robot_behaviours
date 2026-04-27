"""In-memory Liconic STX44 simulator backend.

Implements the subset of :class:`pylabrobot.storage.liconic.
ExperimentalLiconicBackend` that ``liconic_ros.machine.LiconicMachine`` calls.
No serial connection, no hardware dependency — all plate moves, climate
setpoints, shaker state, and error registers are tracked in Python attrs.

Why this exists: the real backend opens a PLC over RS-232 and can't run
headless. For BT integration testing (``test_liconic_smoke.xml``) and the
dashboard, we want TakeIn/Fetch/SetTemperature/etc. to return SUCCESS
immediately without hardware. The real backend is selected at runtime via
``ros2 launch liconic_ros liconic.launch.py simulation:=false`` (the
default); ``simulation:=true`` picks this sim.

Behaviour choices:
  - `check_transfer_sensor` reflects the in-memory tray state, so the
    "plate already on tray" / "no plate to lift" safety checks in
    ``LiconicMachine`` still exercise realistically.
  - Climate setpoints are echoed back immediately — real hardware ramps
    over minutes; the sim doesn't model that.
  - Error registers always report clear (``flag="0"``, ``code="0000"``),
    matching a healthy incubator. ``initialize()`` is a no-op that returns
    success; callers get the same contract as the real backend's reset.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from pylabrobot.resources import Plate, PlateCarrier
from pylabrobot.resources.corning import Cor_96_wellplate_360ul_Fb


logger = logging.getLogger(__name__)


class LiconicSimBackend:
    """Drop-in replacement for ExperimentalLiconicBackend — no hardware.

    By default the transfer tray starts occupied with a placeholder plate so
    a ``TakeIn`` action can succeed immediately (matches the usual smoke-test
    expectation: operator has placed a plate before calling the server). Set
    ``initial_tray_loaded=False`` to start empty.
    """

    def __init__(
        self,
        *,
        model: Any = None,
        port: str = "",
        initial_tray_loaded: bool = True,
        time_compression: float = 0.0,
    ) -> None:
        # Stored for parity with ExperimentalLiconicBackend's constructor.
        self._model = model
        self._port = port

        # time_compression is the dry-run lever for long-running operations.
        # 0.0 = no waits at all (sim returns immediately); >0.0 = future
        # incubate/shaker actions divide their nominal duration by this
        # factor before sleeping. Today's LiconicSimBackend has no native
        # wait points, but the helper :meth:`compressed_sleep` is wired so
        # that when long-duration actions are added (e.g. a future
        # WaitForStableClimate / Incubate), they call this and respect the
        # operator's compression preference.
        self._time_compression: float = float(time_compression)

        self._racks: List[PlateCarrier] = []
        # site_id (string, unique resource name) -> Plate on it
        self._site_contents: Dict[str, Plate] = {}
        self._transfer_tray: Optional[Plate] = (
            Cor_96_wellplate_360ul_Fb(name="liconic_sim_initial_tray_plate")
            if initial_tray_loaded
            else None
        )

        # Climate setpoints / readings. "_IC" model → full climate; "_NC" → no
        # climate control. ``LiconicMachine`` gates access via ``climate_supported``
        # before calling these, so behaviour here only matters for _IC models.
        self._target_temperature: float = 37.0
        self._temperature: float = 37.0
        self._target_humidity: float = 0.0
        self._humidity: float = 0.0

        self._shaking_hz: float = 0.0

    async def compressed_sleep(self, real_seconds: float) -> None:
        """Sleep ``real_seconds`` divided by the configured time_compression.

        ``time_compression == 0.0`` (default): no sleep at all — the dry-run
        skips the wait entirely. Otherwise the real duration collapses to
        ``real_seconds / time_compression``. Long-duration actions added in
        the future should call this rather than ``asyncio.sleep`` so the
        sim respects the launch-time compression knob.
        """
        import asyncio
        if self._time_compression <= 0.0:
            return
        await asyncio.sleep(real_seconds / self._time_compression)

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    async def setup(self) -> None:
        logger.info(
            "LiconicSimBackend: setup() model=%s port=%s (no hardware)",
            getattr(self._model, "value", self._model), self._port,
        )

    async def stop(self) -> None:
        logger.info("LiconicSimBackend: stop()")

    async def set_racks(self, racks: List[PlateCarrier]) -> None:
        self._racks = list(racks)
        logger.info(
            "LiconicSimBackend: set_racks(%d cassette(s), %d sites/cassette)",
            len(self._racks),
            len(self._racks[0].sites) if self._racks else 0,
        )

    async def initialize(self) -> None:
        # Real backend's initialize() re-inits the PLC and clears latched
        # alarms. Nothing to reset in-sim.
        logger.info("LiconicSimBackend: initialize() (noop — error state clear)")

    # ─── Transfer tray / plate moves ──────────────────────────────────────────

    async def check_transfer_sensor(self) -> bool:
        return self._transfer_tray is not None

    async def take_in_plate(self, plate: Plate, site: Any) -> None:
        """Transfer tray → carousel site. Tray must be occupied."""
        if self._transfer_tray is None:
            raise RuntimeError(
                "LiconicSim: take_in_plate called with empty transfer tray"
            )
        site_id = _site_id(site)
        if site_id in self._site_contents:
            raise RuntimeError(
                f"LiconicSim: site {site_id!r} already has a plate"
            )
        self._site_contents[site_id] = self._transfer_tray
        self._transfer_tray = None
        logger.info("LiconicSim: took plate in → %s", site_id)

    async def fetch_plate_to_loading_tray(self, plate: Plate) -> None:
        """Carousel site (derived from plate.parent) → transfer tray."""
        if self._transfer_tray is not None:
            raise RuntimeError(
                "LiconicSim: fetch_plate_to_loading_tray called with "
                "occupied transfer tray"
            )
        parent = getattr(plate, "parent", None)
        if parent is None:
            raise RuntimeError(
                "LiconicSim: plate has no parent site — cannot determine "
                "source location"
            )
        site_id = _site_id(parent)
        stored = self._site_contents.pop(site_id, None)
        # The real backend doesn't inspect the in-memory store; it obeys the
        # PLC. In sim we're cooperative: if the caller claims the plate is
        # there, we accept it (creating one on demand) — the registry is
        # the actual source of truth in ``liconic_ros``.
        self._transfer_tray = stored if stored is not None else plate
        logger.info("LiconicSim: fetched plate from %s → tray", site_id)

    # ─── Climate ──────────────────────────────────────────────────────────────

    async def set_temperature(self, temperature_c: float) -> None:
        self._target_temperature = float(temperature_c)
        # In the sim the actual tracks the target instantly.
        self._temperature = float(temperature_c)
        logger.info("LiconicSim: set_temperature(%.2f °C)", temperature_c)

    async def set_humidity(self, fraction: float) -> None:
        self._target_humidity = float(fraction)
        self._humidity = float(fraction)
        logger.info("LiconicSim: set_humidity(%.2f)", fraction)

    async def get_target_temperature(self) -> float:
        return self._target_temperature

    async def get_temperature(self) -> float:
        return self._temperature

    async def get_target_humidity(self) -> float:
        return self._target_humidity

    async def get_humidity(self) -> float:
        return self._humidity

    # ─── Shaker ───────────────────────────────────────────────────────────────

    async def start_shaking(self, frequency_hz: float) -> None:
        self._shaking_hz = float(frequency_hz)
        logger.info("LiconicSim: start_shaking(%.2f Hz)", frequency_hz)

    async def stop_shaking(self) -> None:
        self._shaking_hz = 0.0
        logger.info("LiconicSim: stop_shaking()")

    async def get_shaker_speed(self) -> float:
        return self._shaking_hz

    # ─── Low-level PLC command ────────────────────────────────────────────────

    async def _send_command(self, cmd: str) -> str:
        """Mimic ``RD 1814`` / ``RD DM200`` reads used by error introspection.

        Real backend forwards arbitrary PLC commands over serial. We only
        need to satisfy the two used by ``machine.read_error()`` /
        ``machine.reset_error()``. Everything else returns "" so a stray
        command doesn't crash the server.
        """
        token = cmd.strip().upper()
        if token == "RD 1814":
            return "0"      # alarm flag clear
        if token == "RD DM200":
            return "0000"   # no error code
        logger.debug("LiconicSim: unhandled _send_command(%r) → ''", cmd)
        return ""


def _site_id(site: Any) -> str:
    """Stable identifier for a PlateHolder / PlateCarrier child site."""
    name = getattr(site, "name", None)
    if name is not None:
        return str(name)
    return repr(site)
