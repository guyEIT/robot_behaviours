"""Start or stop the Liconic incubator's internal shaker via the running
action server. No standalone USB access — requires
``pixi run -e ros2 liconic-server`` to be running.

Shaking persists until explicitly stopped or the unit is power-cycled.

    pixi run -e ros2 liconic-shake -- --frequency 10      # start at 10 Hz
    pixi run -e ros2 liconic-shake -- --stop              # stop

Note: pylabrobot's Liconic shaker wrappers are flagged UNTESTED for PLC
register scaling. Verify visually that the shaker actually runs at the
requested rate on first use.
"""

from __future__ import annotations

import argparse
import sys

import rclpy

from liconic_msgs.srv import StartShaking, StopShaking


START_SRV = "/liconic_action_server/start_shaking"
STOP_SRV = "/liconic_action_server/stop_shaking"


def _call(node, srv_cls, path, req):
    cli = node.create_client(srv_cls, path)
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f"service {path} not available — is liconic-server running?",
              file=sys.stderr)
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    return fut.result()


def main(args: argparse.Namespace) -> int:
    rclpy.init()
    node = rclpy.create_node("liconic_shake_client")
    try:
        if args.stop:
            resp = _call(node, StopShaking, STOP_SRV, StopShaking.Request())
        else:
            req = StartShaking.Request()
            req.frequency_hz = float(args.frequency)
            resp = _call(node, StartShaking, START_SRV, req)
        if resp is None:
            return 1
        if not resp.success:
            print(f"shake failed: {resp.message}", file=sys.stderr)
            return 1
        print(f"ok — {resp.message}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--frequency", type=float, default=10.0,
                   help="shaker frequency in Hz, 1.0–50.0 (default: 10.0)")
    p.add_argument("--stop", action="store_true",
                   help="stop shaking (ignores --frequency)")
    sys.exit(main(p.parse_args()))
