"""Dedicated asyncio loop thread with cancel-aware coroutine dispatch.

The ROS 2 action callbacks run on rclpy executor threads. pylabrobot's
LiquidHandler is an asyncio-native API. This bridge owns a single
dedicated event loop running on one background thread, and every action
callback submits its coroutine via :meth:`AsyncioBridge.run`.

If ``goal_handle.is_cancel_requested`` becomes True while a coroutine is
in-flight, the concurrent future is cancelled, which propagates as a
``asyncio.CancelledError`` into the coroutine — pylabrobot surfaces that
as a motion abort on the backend.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import threading
from concurrent.futures import Future
from typing import Any, Awaitable, Callable, Optional


class AsyncioBridge:
    """Runs an asyncio event loop on a background thread.

    Not a context manager — the ROS node owns lifecycle. Call ``start()``
    during node ``__init__`` and ``stop()`` from ``destroy_node``.
    """

    def __init__(self, name: str = "hamilton-star-asyncio") -> None:
        self._name = name
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._ready = threading.Event()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("AsyncioBridge already started")
        self._thread = threading.Thread(
            target=self._run_loop, name=self._name, daemon=True,
        )
        self._thread.start()
        self._ready.wait()

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._ready.set()
        try:
            self._loop.run_forever()
        finally:
            self._loop.close()

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        if self._loop is None:
            raise RuntimeError("AsyncioBridge not started")
        return self._loop

    def submit(self, coro: Awaitable[Any]) -> Future[Any]:
        """Schedule ``coro`` on the loop; return a concurrent ``Future``."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def run(
        self,
        coro_factory: Callable[[], Awaitable[Any]],
        cancel_check: Optional[Callable[[], bool]] = None,
        poll_interval: float = 0.1,
    ) -> Any:
        """Submit ``coro_factory()`` and block until it returns.

        If ``cancel_check`` is given, invokes it every ``poll_interval``
        seconds; when it returns True, cancels the concurrent Future,
        which triggers ``CancelledError`` inside the coroutine. Accepts
        any zero-arg callable so callers can pass either a bound method
        (``obj.is_cancel_requested_method``) or a lambda wrapping a
        property (``lambda: goal_handle.is_cancel_requested``).
        """
        fut = self.submit(coro_factory())
        try:
            while True:
                try:
                    return fut.result(timeout=poll_interval)
                except TimeoutError:
                    if cancel_check is not None and cancel_check():
                        fut.cancel()
                        try:
                            return fut.result(timeout=2.0)
                        except concurrent.futures.CancelledError as exc:
                            # concurrent.futures.CancelledError is a plain
                            # Exception in 3.11; normalise to asyncio's variant
                            # so callers only need one except clause.
                            raise asyncio.CancelledError() from exc
        except concurrent.futures.CancelledError as exc:
            raise asyncio.CancelledError() from exc
        except asyncio.CancelledError:
            raise

    def stop(self, timeout: float = 2.0) -> None:
        if self._loop is None or self._thread is None:
            return
        loop = self._loop
        loop.call_soon_threadsafe(loop.stop)
        self._thread.join(timeout=timeout)
        self._loop = None
        self._thread = None
        self._ready.clear()
