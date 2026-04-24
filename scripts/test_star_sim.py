"""Hamilton STAR simulator sanity-check (no hardware required).

Uses pylabrobot's ``STARChatterboxBackend`` to exercise the same API path
as ``test_star.py`` but without touching USB or triggering an end-stop
calibration. Useful for:

- Sanity-checking the pylabrobot install when the STAR is unplugged.
- Running the same workflow you'd run on hardware during development.
- CI-friendly verification of deck serialization and CoRe II gripper calls.
"""

import asyncio
import json

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends.hamilton.STAR_chatterbox import (
    STARChatterboxBackend,
)
from pylabrobot.resources.hamilton import STARDeck


async def main() -> None:
    backend = STARChatterboxBackend(num_channels=8)
    deck = STARDeck(core_grippers="1000uL-5mL-on-waste")
    lh = LiquidHandler(backend=backend, deck=deck)

    print("== setup ==")
    await lh.setup()

    print("\n== machine configuration ==")
    cfg = await backend.request_machine_configuration()
    print(f"num_pip_channels={cfg.num_pip_channels} pip_type_1000ul={cfg.pip_type_1000ul}")

    print("\n== instrument initialization status ==")
    initialized = await backend.request_instrument_initialization_status()
    print(f"initialized={initialized}")

    print("\n== deck resources ==")
    names = [r.name for r in deck.get_all_resources()]
    print(f"{len(names)} resources; sample: {names[:10]}")
    assert "core_grippers" in names, "expected CoRe II gripper parking on the deck"

    print("\n== CoRe II gripper pickup / return ==")
    # front_channel=7 -> picks up using channels (6, 7) on an 8-channel STAR;
    # pylabrobot derives back_channel = front_channel - 1 internally.
    await backend.pick_up_core_gripper_tools(front_channel=7)
    print(f"after pickup: core_parked={backend.core_parked}")
    await backend.return_core_gripper_tools()
    print(f"after return: core_parked={backend.core_parked}")

    print("\n== deck serialize round-trip ==")
    blob = json.dumps(deck.serialize(), default=str)
    print(f"serialized {len(blob)} bytes")

    print("\n== stop ==")
    await lh.stop()
    print("ok")


if __name__ == "__main__":
    asyncio.run(main())
