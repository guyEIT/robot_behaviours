"""Clear a specific Liconic cassette/position registry entry without a restart.

Recovery for state drift in the cassette half of the registry. The tray
has a sensor; cassette slots don't. So if a plate leaves a cassette slot
outside the server's control (fetch motion failed partway, operator
physically retrieved a plate, prior workflow interrupted), the registry
can stay "plate here" while reality is "slot empty" — blocking the next
take-in.

Physical authority belongs to you: this service only updates the
registry, it doesn't move any motor or confirm physical state.

Run with:

    pixi run -e ros2 liconic-clear-slot                    # default cassette 1 position 1
    pixi run -e ros2 liconic-clear-slot -- --cassette 2 --position 5
"""

from __future__ import annotations

import argparse
import sys

import rclpy

from liconic_msgs.srv import ClearCassettePosition


SERVICE = "/liconic_action_server/clear_cassette_position"


def main(args: argparse.Namespace) -> int:
    rclpy.init()
    node = rclpy.create_node("liconic_clear_slot_client")
    try:
        cli = node.create_client(ClearCassettePosition, SERVICE)
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"service {SERVICE} not available — is liconic-server running?",
                  file=sys.stderr)
            return 1
        req = ClearCassettePosition.Request()
        req.cassette = args.cassette
        req.position = args.position
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            print(f"no response from {SERVICE}", file=sys.stderr)
            return 1
        if not resp.success:
            print(f"clear_cassette_position failed: {resp.message}", file=sys.stderr)
            return 1
        if resp.cleared_plate_name:
            print(f"ok — cleared {resp.cleared_plate_name!r} from "
                  f"cassette {args.cassette} position {args.position}")
        else:
            print(f"ok — cassette {args.cassette} position {args.position} "
                  "was already empty")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--cassette", type=int, default=1,
                   help="cassette number, 1-indexed (default: 1)")
    p.add_argument("--position", type=int, default=1,
                   help="position within cassette, 1-indexed (default: 1)")
    sys.exit(main(p.parse_args()))
