"""Dedicated asyncio loop thread for the Liconic action server.

Identical pattern to hamilton_star_ros.asyncio_bridge: one background
thread runs an asyncio event loop; ROS action/service callbacks submit
coroutines to it via :meth:`AsyncioBridge.run`, which blocks the
calling thread until the coroutine resolves.

Vendored rather than imported from hamilton_star_ros to keep the two
packages independent at runtime.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import threading
from concurrent.futures import Future
from typing import Any, Awaitable, Callable, Optional


class AsyncioBridge:
    """Runs an asyncio event loop on a background thread."""

    def __init__(self, name: str = "liconic-asyncio") -> None:
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
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def run(
        self,
        coro_factory: Callable[[], Awaitable[Any]],
        cancel_check: Optional[Callable[[], bool]] = None,
        poll_interval: float = 0.1,
    ) -> Any:
        """Submit ``coro_factory()`` and block until it returns.

        If ``cancel_check`` returns True while in-flight, cancels the
        concurrent Future which raises ``asyncio.CancelledError`` inside
        the coroutine.
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
