"""Unit tests for :mod:`hamilton_star_ros.asyncio_bridge`."""

from __future__ import annotations

import asyncio
import time

import pytest

from hamilton_star_ros.asyncio_bridge import AsyncioBridge


class _Cancel:
    def __init__(self) -> None:
        self._flag = False

    def set(self) -> None:
        self._flag = True

    @property
    def is_cancel_requested(self) -> bool:  # mimics rclpy GoalHandle property
        return self._flag


def test_start_stop_is_idempotent() -> None:
    bridge = AsyncioBridge()
    bridge.start()
    try:
        assert bridge.loop.is_running()
    finally:
        bridge.stop()
    # second stop is a no-op
    bridge.stop()


def test_submit_runs_coroutine() -> None:
    bridge = AsyncioBridge()
    bridge.start()
    try:
        async def coro() -> int:
            await asyncio.sleep(0.01)
            return 42

        fut = bridge.submit(coro())
        assert fut.result(timeout=2.0) == 42
    finally:
        bridge.stop()


def test_run_returns_value_without_cancel_source() -> None:
    bridge = AsyncioBridge()
    bridge.start()
    try:
        async def coro() -> str:
            return "done"

        assert bridge.run(coro) == "done"
    finally:
        bridge.stop()


def test_run_propagates_exception() -> None:
    bridge = AsyncioBridge()
    bridge.start()
    try:
        async def coro() -> None:
            raise RuntimeError("boom")

        with pytest.raises(RuntimeError, match="boom"):
            bridge.run(coro)
    finally:
        bridge.stop()


def test_run_cancels_via_cancel_source() -> None:
    bridge = AsyncioBridge()
    bridge.start()
    try:
        cancel = _Cancel()

        async def coro() -> None:
            await asyncio.sleep(5.0)

        def trigger_cancel() -> None:
            time.sleep(0.05)
            cancel.set()

        import threading
        threading.Thread(target=trigger_cancel, daemon=True).start()

        with pytest.raises((asyncio.CancelledError, Exception)):
            bridge.run(
                coro,
                cancel_check=lambda: cancel.is_cancel_requested,
                poll_interval=0.02,
            )
    finally:
        bridge.stop()
