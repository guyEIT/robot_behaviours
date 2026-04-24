"""Tell the running Liconic server that the transfer tray is empty.

One-shot recovery utility for the registry-vs-physical-state mismatch
that arises when an external actor (the Hamilton iSWAP, an operator's
hand) removes a plate from the tray without going through the Liconic
server's ``fetch`` action. The Liconic PLC has no way to detect that
event, so the server's registry keeps the stale tray entry until it's
explicitly cleared.

Requires the Liconic action server to be running
(``pixi run -e ros2 liconic-server``).

Run with:

    pixi run -e ros2 liconic-clear-tray
"""

from __future__ import annotations

import sys

import rclpy

from liconic_msgs.srv import ClearLoadingTray


SERVICE = "/liconic_action_server/clear_loading_tray"


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("liconic_clear_tray_client")
    try:
        cli = node.create_client(ClearLoadingTray, SERVICE)
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"service {SERVICE} not available — is liconic-server running?",
                  file=sys.stderr)
            return 1
        fut = cli.call_async(ClearLoadingTray.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            print(f"no response from {SERVICE}", file=sys.stderr)
            return 1
        if not resp.success:
            print(f"clear_loading_tray failed: {resp.message}", file=sys.stderr)
            return 1
        if resp.cleared_plate_name:
            print(f"ok — cleared {resp.cleared_plate_name!r} from loading tray")
        else:
            print("ok — loading tray was already empty")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
