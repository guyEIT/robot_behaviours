"""Direct pylabrobot test for the CoRe II gripper (no ROS, real hardware).

Mirrors test_star.py but also exercises the CoRe II gripper: pick up
the tool pair from the parking slot, then return it. Does NOT move
any plates — stops at the cycle so you can verify the two channels
seize and release the tools correctly.

Run with:

    pixi run test-gripper

Assumes the 1000 uL / 5 mL gripper is parked on the waste (the default
STARDeck layout pylabrobot ships). Override with ``--front-channel N``
to use a different channel pair — the back channel is derived as
``front_channel - 1``.
"""

import argparse
import asyncio

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources.hamilton import STARDeck


async def main(front_channel: int) -> None:
    backend = STARBackend()
    deck = STARDeck(core_grippers="1000uL-5mL-on-waste")
    lh = LiquidHandler(backend=backend, deck=deck)

    await lh.setup()
    try:
        cfg = await backend.request_machine_configuration()
        print(
            f"machine: num_pip_channels={cfg.num_pip_channels} "
            f"pip_type_1000ul={cfg.pip_type_1000ul}"
        )
        print(f"before pickup: core_parked={backend.core_parked}")

        back_channel = front_channel - 1
        print(f"picking up gripper on channels [{back_channel}, {front_channel}]...")
        await backend.pick_up_core_gripper_tools(front_channel=front_channel)
        print(f"after pickup:  core_parked={backend.core_parked}")

        # Brief pause so an operator can visually verify the tools are seized
        # before they get returned.
        await asyncio.sleep(1.0)

        print("returning gripper...")
        await backend.return_core_gripper_tools()
        print(f"after return:  core_parked={backend.core_parked}")
    finally:
        # Keep the USB connection alive if the script is ever wrapped by a
        # longer-running driver; here we explicitly stop because this IS the
        # driver. A clean stop here still pays end-stop calibration on the
        # NEXT cold start, which is acceptable for a one-off sanity check.
        await lh.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--front-channel", type=int, default=7,
        help="front channel for gripper pickup (default: 7, uses channels 6+7)",
    )
    args = p.parse_args()
    asyncio.run(main(args.front_channel))
