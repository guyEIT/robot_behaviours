"""Writer-preference RW lock + per-channel + per-resource asyncio locks.

Tip-state and CoRe-gripper-state tracking live on :class:`MachineFSM`; this
module is purely about serializing coroutine access to the shared
``LiquidHandler``.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import AsyncIterator, Iterable


class RWLock:
    """Async readers/writer lock with writer preference.

    A single exclusive holder blocks all shared holders; while any exclusive
    waiters are pending, new shared acquisitions are also blocked, so
    exclusive ops cannot be starved by a steady drip of shared ops.
    """

    def __init__(self) -> None:
        self._cond = asyncio.Condition()
        self._readers = 0
        self._writer_active = False
        self._writers_waiting = 0

    @asynccontextmanager
    async def shared(self) -> AsyncIterator[None]:
        async with self._cond:
            while self._writer_active or self._writers_waiting > 0:
                await self._cond.wait()
            self._readers += 1
        try:
            yield
        finally:
            async with self._cond:
                self._readers -= 1
                if self._readers == 0:
                    self._cond.notify_all()

    @asynccontextmanager
    async def exclusive(self) -> AsyncIterator[None]:
        async with self._cond:
            self._writers_waiting += 1
            try:
                while self._writer_active or self._readers > 0:
                    await self._cond.wait()
                self._writer_active = True
            finally:
                self._writers_waiting -= 1
        try:
            yield
        finally:
            async with self._cond:
                self._writer_active = False
                self._cond.notify_all()

    async def try_shared(self) -> bool:
        """Non-blocking shared acquire. True on success (caller must release)."""
        async with self._cond:
            if self._writer_active or self._writers_waiting > 0:
                return False
            self._readers += 1
            return True

    async def release_shared(self) -> None:
        async with self._cond:
            self._readers -= 1
            if self._readers == 0:
                self._cond.notify_all()

    async def try_exclusive(self) -> bool:
        async with self._cond:
            if self._writer_active or self._readers > 0:
                return False
            self._writer_active = True
            return True

    async def release_exclusive(self) -> None:
        async with self._cond:
            self._writer_active = False
            self._cond.notify_all()


class MachineLock:
    """RW-lock + per-channel + per-resource locks, with fixed acquisition order.

    Resource-lock keys and channel indices are both sorted before acquisition,
    and channels always precede resources, so transfer-style ops cannot
    deadlock against plain aspirate/dispense.
    """

    def __init__(self, num_channels: int = 8) -> None:
        self._rw = RWLock()
        self._channel_locks: dict[int, asyncio.Lock] = {
            i: asyncio.Lock() for i in range(num_channels)
        }
        self._resource_locks: dict[str, asyncio.Lock] = {}
        self._resource_locks_guard = asyncio.Lock()

    async def _get_resource_lock(self, name: str) -> asyncio.Lock:
        async with self._resource_locks_guard:
            lock = self._resource_locks.get(name)
            if lock is None:
                lock = asyncio.Lock()
                self._resource_locks[name] = lock
            return lock

    @asynccontextmanager
    async def shared(
        self,
        channels: Iterable[int],
        resources: Iterable[str] = (),
    ) -> AsyncIterator[None]:
        """Hold the RW lock shared + per-channel + per-resource locks."""
        channel_list = sorted(set(channels))
        resource_list = sorted(set(resources))
        async with self._rw.shared():
            acquired_channels: list[int] = []
            acquired_resources: list[asyncio.Lock] = []
            try:
                for ch in channel_list:
                    await self._channel_locks[ch].acquire()
                    acquired_channels.append(ch)
                for name in resource_list:
                    lock = await self._get_resource_lock(name)
                    await lock.acquire()
                    acquired_resources.append(lock)
                yield
            finally:
                for lock in reversed(acquired_resources):
                    lock.release()
                for ch in reversed(acquired_channels):
                    self._channel_locks[ch].release()

    @asynccontextmanager
    async def exclusive(
        self,
        resources: Iterable[str] = (),
    ) -> AsyncIterator[None]:
        """Hold the RW lock exclusive + any named resources.

        Channel locks are implicitly held by the exclusive write lock — no
        shared operation can start while an exclusive op runs.
        """
        resource_list = sorted(set(resources))
        async with self._rw.exclusive():
            acquired_resources: list[asyncio.Lock] = []
            try:
                for name in resource_list:
                    lock = await self._get_resource_lock(name)
                    await lock.acquire()
                    acquired_resources.append(lock)
                yield
            finally:
                for lock in reversed(acquired_resources):
                    lock.release()
