"""Reset the Liconic's latched error state via the running action server.

Clears most PLC errors and silences the 30 s alarm beep. Safe to run
when the device is idle; a plate partway through a transfer/shovel
motion at the moment of reset can land in an unexpected state, so only
call when the transfer tray and shovel are empty.

Prints the error code that was cleared (or "no latched error") so you
can cross-reference the HandlingError enum in
``pylabrobot.storage.liconic.constants``. Common codes:

    00001  GENERAL_HANDLING_ERROR
    00007  GATE_OPEN_ERROR
    00008  GATE_CLOSE_ERROR
    00009  GENERAL_LIFT_POSITIONING_ERROR
    00013  PLATE_TRANSFER_DETECTION_ERROR
    00014  LIFT_INITIALIZATION_ERROR
    00015  PLATE_ON_SHOVEL_DETECTION         (plate where none expected)
    00016  NO_PLATE_ON_SHOVEL_DETECTION      (no plate where one was expected)

Run with:

    pixi run -e ros2 liconic-reset-error
"""

from __future__ import annotations

import sys

import rclpy

from liconic_msgs.srv import ResetError


SERVICE = "/liconic_action_server/reset_error"


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("liconic_reset_error_client")
    try:
        cli = node.create_client(ResetError, SERVICE)
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"service {SERVICE} not available — is liconic-server running?",
                  file=sys.stderr)
            return 1
        fut = cli.call_async(ResetError.Request())
        # Allow up to 20 s: ST 1900 homes the carousel, then _wait_ready polls.
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        resp = fut.result()
        if resp is None:
            print(f"no response from {SERVICE}", file=sys.stderr)
            return 1
        if not resp.success:
            print(f"reset_error failed: {resp.message}", file=sys.stderr)
            return 1
        print(f"ok — {resp.message}")
        if resp.error_flag_before == "1":
            print(f"  error code cleared: {resp.error_code_before!r}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
