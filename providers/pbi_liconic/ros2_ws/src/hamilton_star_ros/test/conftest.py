"""Shared pytest fixtures for hamilton_star_ros tests.

Pure-Python tests (``test_machine_fsm``, ``test_machine_lock``,
``test_asyncio_bridge``) only need the ``sys.path`` patch at the bottom —
they don't touch any ROS fixtures defined here.

ROS 2 integration tests import rclpy / hamilton_star_msgs via
``pytest.importorskip``; in a non-ROS pixi environment this conftest
simply doesn't define the ROS fixtures, and tests that depend on them
skip cleanly.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Iterable

import pytest

# --- sys.path so pure-Python modules import without colcon build --------
_pkg_root = Path(__file__).resolve().parent.parent
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))


# --- ROS 2 integration fixtures (optional) ------------------------------
_rclpy = None
_hamilton_msgs = None
try:
    import rclpy as _rclpy  # noqa: F401
    import hamilton_star_msgs as _hamilton_msgs  # noqa: F401
except ImportError:  # pragma: no cover
    # Outside the ros2 pixi env; ROS-dependent fixtures below are never
    # collected because the tests that reference them use importorskip.
    pass


if _rclpy is not None and _hamilton_msgs is not None:
    import concurrent.futures.thread

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from hamilton_star_msgs.srv import GetStatus
    from hamilton_star_ros.action_server import HamiltonStarActionServer

    NODE_NAME = "hamilton_star_action_server"

    def call_service(
        _helper: Node, client: Any, request: Any, timeout: float = 3.0,
    ) -> Any:
        """Blocking service call that spins implicitly via MTE-backed executor."""
        assert client.wait_for_service(timeout_sec=timeout), (
            f"service {client.srv_name} never became available"
        )
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"service {client.srv_name} timed out")
            time.sleep(0.02)
        return future.result()

    def send_goal_blocking(
        _helper: Node, action_client: ActionClient, goal: Any, timeout: float = 8.0,
    ) -> Any:
        """Send an action goal, wait for the final result. Asserts goal was accepted."""
        assert action_client.wait_for_server(timeout_sec=timeout), (
            f"action {action_client._action_name} never became available"
        )
        send_future = action_client.send_goal_async(goal)
        deadline = time.monotonic() + timeout
        while not send_future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"send_goal on {action_client._action_name} timed out")
            time.sleep(0.02)
        gh = send_future.result()
        assert gh.accepted, f"goal on {action_client._action_name} was rejected"
        result_future = gh.get_result_async()
        while not result_future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"get_result on {action_client._action_name} timed out")
            time.sleep(0.02)
        return result_future.result().result

    def try_send_goal(
        _helper: Node, action_client: ActionClient, goal: Any, timeout: float = 3.0,
    ) -> bool:
        """Return True iff the server accepted the goal (doesn't wait for result)."""
        assert action_client.wait_for_server(timeout_sec=timeout)
        send_future = action_client.send_goal_async(goal)
        deadline = time.monotonic() + timeout
        while not send_future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("send_goal timed out")
            time.sleep(0.02)
        return send_future.result().accepted

    def wait_for(predicate: Callable[[], bool], timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    @pytest.fixture(scope="session")
    def rclpy_ctx():
        rclpy.init()
        yield
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # Python 3.12 + rclpy leaves a non-daemon ThreadPoolExecutor
        # worker registered in
        # ``concurrent.futures.thread._threads_queues`` after
        # ``rclpy.shutdown()``. ``threading._shutdown`` would then join
        # it (no timeout) on interpreter exit and hang pytest at the
        # "<N> passed" line. Clearing the queue at fixture teardown
        # (before the interpreter's own ``atexit`` chain runs) lets
        # Python exit cleanly. The thread is a harmless idle worker;
        # draining the queue drops the join reference without losing
        # any in-flight work (rclpy is already torn down above).
        concurrent.futures.thread._threads_queues.clear()

    _node_counter = {"n": 0}

    def _build_sim_node(
        rclpy_ctx: Any,
        extra_overrides: Iterable[Any] = (),
    ):
        _node_counter["n"] += 1
        unique_name = f"hamilton_star_action_server_t{_node_counter['n']}"

        overrides = [
            Parameter("backend", Parameter.Type.STRING, "simulator"),
            Parameter("core_grippers", Parameter.Type.STRING, "1000uL-5mL-on-waste"),
            Parameter("num_channels", Parameter.Type.INTEGER, 8),
            Parameter("disconnect_on_shutdown", Parameter.Type.BOOL, False),
            Parameter("on_conflict", Parameter.Type.STRING, "reject"),
            Parameter("deck_file", Parameter.Type.STRING, ""),
            *extra_overrides,
        ]
        node = HamiltonStarActionServer(
            node_name=unique_name, parameter_overrides=overrides,
        )

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        helper = rclpy.create_node(f"test_client_t{_node_counter['n']}")
        executor.add_node(helper)
        status_client = helper.create_client(GetStatus, f"/{unique_name}/get_status")

        initialized = wait_for(
            lambda: (
                status_client.wait_for_service(timeout_sec=0.1)
                and (
                    resp := call_service(
                        helper, status_client, GetStatus.Request(), timeout=1.0
                    )
                ) is not None
                and resp.status.initialized
            ),
            timeout=5.0,
        )
        assert initialized, "node never reported initialized=true"

        def _teardown():
            executor.remove_node(helper)
            helper.destroy_node()
            executor.remove_node(node)
            try:
                node.destroy_node()
            except Exception:
                pass
            executor.shutdown()
            spin_thread.join(timeout=2.0)

        return node, helper, executor, unique_name, _teardown

    @pytest.fixture
    def sim_node(rclpy_ctx):
        """HamiltonStarActionServer with backend=simulator and the default STARDeck."""
        node, helper, executor, unique_name, teardown = _build_sim_node(rclpy_ctx)
        # Stash for ros_helpers fixture to pick up.
        node._test_unique_name = unique_name
        yield node, helper, executor
        teardown()

    # ---- Pipetting-capable deck fixture ----------------------------------
    #
    # Writes a full STAR deck (tip rack carrier at rails 15 with a 1000 uL
    # tip rack, plate carrier at rails 21 with a 96-well plate) to a temp
    # JSON, then boots the node with deck_file=<that path>. Used by the
    # pipetting happy-path tests to exercise real pick_up / aspirate /
    # dispense / drop sequences against the chatterbox sim.

    TIP_RACK_NAME = "tip_rack_01"
    PLATE_NAME = "plate_01"

    def build_pipetting_deck_json() -> str:
        import json
        from pylabrobot.resources.hamilton import STARDeck
        from pylabrobot.resources.hamilton.plate_carriers import PLT_CAR_L5AC_A00
        from pylabrobot.resources.hamilton.tip_carriers import TIP_CAR_480_A00
        from pylabrobot.resources.hamilton.tip_racks import hamilton_96_tiprack_1000uL
        from pylabrobot.resources.nest.plates import NEST_96_wellplate_2200uL_Ub

        deck = STARDeck(core_grippers="1000uL-5mL-on-waste")
        tip_car = TIP_CAR_480_A00("tip_carrier_01")
        tip_car[0] = hamilton_96_tiprack_1000uL(TIP_RACK_NAME)
        deck.assign_child_resource(tip_car, rails=15)

        plate_car = PLT_CAR_L5AC_A00("plate_carrier_01")
        plate_car[0] = NEST_96_wellplate_2200uL_Ub(PLATE_NAME)
        deck.assign_child_resource(plate_car, rails=21)

        return json.dumps(deck.serialize(), default=str)

    @pytest.fixture
    def pipetting_deck_file(tmp_path):
        """Path to a temp JSON file holding a STARDeck with tip_rack_01 + plate_01."""
        path = tmp_path / "pipetting_deck.json"
        path.write_text(build_pipetting_deck_json())
        return path

    @pytest.fixture
    def sim_node_pipetting(rclpy_ctx, pipetting_deck_file):
        """HamiltonStarActionServer with a pre-loaded tip rack + plate on the deck."""
        overrides = [
            Parameter(
                "deck_file", Parameter.Type.STRING, str(pipetting_deck_file),
            ),
        ]
        node, helper, executor, unique_name, teardown = _build_sim_node(
            rclpy_ctx, extra_overrides=overrides,
        )
        node._test_unique_name = unique_name
        yield node, helper, executor
        teardown()

    @pytest.fixture
    def ros_helpers(request):
        """Test-friendly helper namespace bound to whichever sim_node* ran first.

        Tests that request ``ros_helpers`` must also request one of
        ``sim_node`` or ``sim_node_pipetting`` — NODE_NAME is looked up on the
        node so both fixtures are interchangeable from a test's perspective.
        """
        node = None
        for fname in ("sim_node", "sim_node_pipetting"):
            if fname in request.fixturenames:
                node = request.getfixturevalue(fname)[0]
                break
        unique_name = (
            getattr(node, "_test_unique_name", NODE_NAME) if node is not None else NODE_NAME
        )

        class _H:
            NODE_NAME = unique_name
            call_service = staticmethod(call_service)
            send_goal_blocking = staticmethod(send_goal_blocking)
            try_send_goal = staticmethod(try_send_goal)
            wait_for = staticmethod(wait_for)
        return _H
