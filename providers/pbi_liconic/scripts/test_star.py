"""Connection sanity check for a Hamilton STAR via pylabrobot.

Runs setup() on the backend, prints a few identifying values, then stops.
Does NOT pick up tips or move liquid.
"""

import asyncio

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources.hamilton import STARDeck


async def main() -> None:
    backend = STARBackend()
    lh = LiquidHandler(backend=backend, deck=STARDeck())

    await lh.setup()
    try:
        cfg = await backend.request_machine_configuration()
        initialized = await backend.request_instrument_initialization_status()
        print("machine configuration:", cfg)
        print("instrument initialized:", initialized)
    finally:
        await lh.stop()


if __name__ == "__main__":
    asyncio.run(main())
