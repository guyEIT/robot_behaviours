"""Thin async facade over pylabrobot's Liconic backend.

Owns the backend instance, an asyncio Lock serialising PLC access (the
Liconic accepts one command at a time), and the list of virtual racks
that pylabrobot uses to map (cassette, position) → DM0/DM5 PLC values.

The ROS node (``action_server.py``) consumes only the coroutines here;
it does not touch ExperimentalLiconicBackend directly.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Dict, List, Optional

from pylabrobot.resources import Plate, PlateCarrier, PlateHolder
from pylabrobot.resources.corning import Cor_96_wellplate_360ul_Fb
from pylabrobot.storage.liconic import ExperimentalLiconicBackend
from pylabrobot.storage.liconic import racks as liconic_racks
from pylabrobot.storage.liconic.constants import LiconicType

from .sim_backend import LiconicSimBackend


logger = logging.getLogger(__name__)


class LiconicMachineError(Exception):
    pass


class LiconicMachine:
    def __init__(
        self,
        port: str,
        model: str,
        rack_model: str,
        total_cassettes: int,
        simulation: bool = False,
        time_compression: float = 0.0,
    ) -> None:
        try:
            liconic_type = LiconicType(model)
        except ValueError as e:
            raise LiconicMachineError(f"unknown Liconic model {model!r}") from e

        factory = getattr(liconic_racks, rack_model, None)
        if factory is None:
            raise LiconicMachineError(
                f"unknown rack model {rack_model!r} — expected a "
                "pylabrobot.storage.liconic.racks factory name"
            )
        if total_cassettes < 1:
            raise LiconicMachineError(
                f"total_cassettes must be >= 1; got {total_cassettes}"
            )

        self._model = liconic_type
        self._port = port
        self._rack_model = rack_model
        self._total_cassettes = total_cassettes
        self._simulation = simulation
        self._racks: List[PlateCarrier] = [
            factory(name=f"liconic_cassette_{i + 1}") for i in range(total_cassettes)
        ]
        self._positions_per_cassette = len(self._racks[0].sites)

        # simulation=True swaps the PLC-backed backend for an in-memory one
        # (see liconic_ros.sim_backend.LiconicSimBackend). Same coroutine API,
        # so machine.py doesn't care which is live.
        if simulation:
            self.backend = LiconicSimBackend(
                model=liconic_type, port=port,
                time_compression=time_compression,
            )
        else:
            self.backend = ExperimentalLiconicBackend(
                model=liconic_type, port=port
            )
        self._lock = asyncio.Lock()
        self._connected = False

    # ---- introspection ----

    @property
    def model(self) -> str:
        return self._model.value

    @property
    def port(self) -> str:
        return self._port

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def climate_supported(self) -> bool:
        return not self._model.value.endswith("_NC")

    @property
    def total_cassettes(self) -> int:
        return self._total_cassettes

    @property
    def positions_per_cassette(self) -> int:
        return self._positions_per_cassette

    # ---- lifecycle ----

    async def setup(self) -> None:
        async with self._lock:
            await self.backend.setup()
            await self.backend.set_racks(self._racks)
            self._connected = True
            logger.info(
                "Liconic connected: %s @ %s (%d cassettes × %d sites, rack=%s)",
                self._model.value, self._port,
                self._total_cassettes, self._positions_per_cassette,
                self._rack_model,
            )

    async def stop(self) -> None:
        async with self._lock:
            if self._connected:
                try:
                    await self.backend.stop()
                except Exception as exc:  # noqa: BLE001
                    logger.warning("Liconic stop() raised: %s", exc)
            self._connected = False

    # ---- plate moves ----

    def _site(self, cassette: int, position: int) -> PlateHolder:
        if not 1 <= cassette <= self._total_cassettes:
            raise LiconicMachineError(
                f"cassette {cassette} out of range 1..{self._total_cassettes}"
            )
        if not 1 <= position <= self._positions_per_cassette:
            raise LiconicMachineError(
                f"position {position} out of range "
                f"1..{self._positions_per_cassette}"
            )
        return list(self._racks[cassette - 1].sites.values())[position - 1]

    async def take_in_plate(
        self,
        cassette: int,
        position: int,
        barcode: str = "",
    ) -> None:
        """Transfer tray → (cassette, position). Plate must be on tray."""
        site = self._site(cassette, position)
        plate = self._make_plate()
        async with self._lock:
            if not self._connected:
                raise LiconicMachineError("Liconic is not connected")
            # Last-line safety: transfer-tray sensor must report a plate
            # before the shovel lifts. Skips the PLC motion entirely on
            # mismatch instead of walking into a "no plate on shovel"
            # firmware error (00716) that leaves the carousel rotated.
            if not await self.backend.check_transfer_sensor():
                raise LiconicMachineError(
                    "transfer-tray sensor reports no plate — aborting "
                    "take-in before any motion. Place the plate on the tray "
                    "and retry."
                )
            await self.backend.take_in_plate(plate, site)

    async def fetch_plate(
        self,
        cassette: int,
        position: int,
    ) -> None:
        """(cassette, position) → transfer tray. Plate must be at source."""
        site = self._site(cassette, position)
        plate = self._make_plate()
        # fetch_plate_to_loading_tray derives the source site from
        # plate.parent, so we wire up the dummy plate first.
        if plate.parent is not None:
            plate.unassign()
        site.assign_child_resource(plate)
        try:
            async with self._lock:
                if not self._connected:
                    raise LiconicMachineError("Liconic is not connected")
                # Last-line safety: transfer tray must be empty before the
                # shovel delivers a plate to it. Fails fast instead of
                # colliding the incoming plate into one that's already
                # on the dock.
                if await self.backend.check_transfer_sensor():
                    raise LiconicMachineError(
                        "transfer-tray sensor reports a plate already "
                        "present — aborting fetch before any motion. "
                        "Clear the tray and retry."
                    )
                await self.backend.fetch_plate_to_loading_tray(plate)
        finally:
            # Always unassign — success or failure. Leaving the plate
            # on the site poisons the resource-name registry so the
            # next fetch can't assign a fresh ``liconic_transient_plate``
            # (_check_naming_conflicts rejects the duplicate name).
            try:
                plate.unassign()
            except Exception:  # noqa: BLE001
                pass

    # ---- climate ----

    async def set_temperature(self, temperature_c: float) -> None:
        if not self.climate_supported:
            raise LiconicMachineError(
                f"model {self._model.value} has no climate control"
            )
        async with self._lock:
            if not self._connected:
                raise LiconicMachineError("Liconic is not connected")
            await self.backend.set_temperature(temperature_c)

    async def set_humidity(self, fraction: float) -> None:
        if not self.climate_supported:
            raise LiconicMachineError(
                f"model {self._model.value} has no climate control"
            )
        if not 0.0 <= fraction <= 1.0:
            raise LiconicMachineError(
                f"humidity fraction must be in [0, 1]; got {fraction}"
            )
        async with self._lock:
            if not self._connected:
                raise LiconicMachineError("Liconic is not connected")
            await self.backend.set_humidity(fraction)

    # ---- shaker ----

    async def start_shaking(self, frequency_hz: float) -> None:
        if not 1.0 <= frequency_hz <= 50.0:
            raise LiconicMachineError(
                f"shaker frequency must be in [1.0, 50.0] Hz; got {frequency_hz}"
            )
        async with self._lock:
            if not self._connected:
                raise LiconicMachineError("Liconic is not connected")
            await self.backend.start_shaking(frequency_hz)

    async def stop_shaking(self) -> None:
        async with self._lock:
            if not self._connected:
                raise LiconicMachineError("Liconic is not connected")
            await self.backend.stop_shaking()

    async def get_shaker_frequency(self) -> float:
        async with self._lock:
            if not self._connected:
                return float("nan")
            try:
                return await self.backend.get_shaker_speed()
            except Exception as exc:  # noqa: BLE001
                logger.warning("get_shaker_speed raised: %s", exc)
                return float("nan")

    # ---- error state ----

    async def read_error(self) -> Dict[str, str]:
        """Read the PLC's latched-error flag (RD 1814) and code (RD DM200).

        Both fields are returned as strings — empty string if the device
        isn't connected or the read failed. A ``flag`` of "1" indicates
        the 30 s alarm is active and the device will refuse motion until
        the error is cleared (via ``reset_error``).
        """
        async with self._lock:
            if not self._connected:
                return {"flag": "", "code": ""}
            try:
                flag = await self.backend._send_command("RD 1814")
                code = await self.backend._send_command("RD DM200")
                return {"flag": str(flag), "code": str(code)}
            except Exception as exc:  # noqa: BLE001
                logger.warning("read_error raised: %s", exc)
                return {"flag": "", "code": ""}

    async def reset_error(self) -> Dict[str, str]:
        """Re-initialize the Liconic — clears most latched errors.

        Returns ``{"flag_before": ..., "code_before": ...}`` for
        diagnostic logging so callers can see what was cleared.
        """
        async with self._lock:
            if not self._connected:
                raise LiconicMachineError("Liconic is not connected")
            # Capture the error state BEFORE the reset — backend.initialize
            # would clear it, and callers want to know what they cleared.
            try:
                flag_before = str(await self.backend._send_command("RD 1814"))
                code_before = str(await self.backend._send_command("RD DM200"))
            except Exception as exc:  # noqa: BLE001
                logger.warning("pre-reset error read raised: %s", exc)
                flag_before = ""
                code_before = ""
            await self.backend.initialize()
            return {"flag_before": flag_before, "code_before": code_before}

    async def read_climate(self) -> Dict[str, float]:
        """Return target/actual temperature and humidity, or NaNs if unsupported."""
        nan = float("nan")
        if not self.climate_supported:
            return dict(target_t=nan, actual_t=nan, target_h=nan, actual_h=nan)
        async with self._lock:
            if not self._connected:
                return dict(target_t=nan, actual_t=nan, target_h=nan, actual_h=nan)
            target_t = await self.backend.get_target_temperature()
            actual_t = await self.backend.get_temperature()
            target_h = await self.backend.get_target_humidity()
            actual_h = await self.backend.get_humidity()
            return dict(
                target_t=target_t, actual_t=actual_t,
                target_h=target_h, actual_h=actual_h,
            )

    # ---- helpers ----

    @staticmethod
    def _make_plate() -> Plate:
        # Dummy SBS-footprint plate; Liconic backend only uses it for
        # parent-site lookup on fetch and optional barcode tagging.
        return Cor_96_wellplate_360ul_Fb(name="liconic_transient_plate")
