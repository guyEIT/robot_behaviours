"""Unit tests for :mod:`hamilton_star_ros.machine_lock`."""

from __future__ import annotations

import asyncio

import pytest

from hamilton_star_ros.machine_lock import MachineLock, RWLock


pytestmark = pytest.mark.asyncio


async def test_shared_allows_concurrent_readers() -> None:
    rw = RWLock()
    entered = []
    release = asyncio.Event()

    async def reader(idx: int) -> None:
        async with rw.shared():
            entered.append(idx)
            await release.wait()

    tasks = [asyncio.create_task(reader(i)) for i in range(3)]
    await asyncio.sleep(0.05)
    assert len(entered) == 3
    release.set()
    await asyncio.gather(*tasks)


async def test_exclusive_blocks_shared() -> None:
    rw = RWLock()
    order: list[str] = []

    async def writer() -> None:
        async with rw.exclusive():
            order.append("w_start")
            await asyncio.sleep(0.05)
            order.append("w_end")

    async def reader() -> None:
        async with rw.shared():
            order.append("r")

    w_task = asyncio.create_task(writer())
    await asyncio.sleep(0.01)
    r_task = asyncio.create_task(reader())
    await asyncio.gather(w_task, r_task)
    assert order.index("w_end") < order.index("r")


async def test_writer_preference_blocks_new_readers() -> None:
    rw = RWLock()
    order: list[str] = []
    r1_enter = asyncio.Event()
    r1_release = asyncio.Event()

    async def r1() -> None:
        async with rw.shared():
            r1_enter.set()
            order.append("r1_enter")
            await r1_release.wait()
            order.append("r1_exit")

    async def w() -> None:
        async with rw.exclusive():
            order.append("w")

    async def r2() -> None:
        async with rw.shared():
            order.append("r2")

    t1 = asyncio.create_task(r1())
    await r1_enter.wait()
    tw = asyncio.create_task(w())
    await asyncio.sleep(0.01)
    t2 = asyncio.create_task(r2())
    await asyncio.sleep(0.01)
    r1_release.set()
    await asyncio.gather(t1, tw, t2)
    assert order.index("w") < order.index("r2"), (
        f"writer must run before the waiting reader; got {order}"
    )


async def test_machine_lock_shared_disjoint_channels_are_concurrent() -> None:
    ml = MachineLock(num_channels=8)
    starts: list[int] = []
    release = asyncio.Event()

    async def op(ch: int) -> None:
        async with ml.shared(channels=[ch], resources=[f"plate_{ch}"]):
            starts.append(ch)
            await release.wait()

    t0 = asyncio.create_task(op(0))
    t1 = asyncio.create_task(op(1))
    await asyncio.sleep(0.05)
    assert set(starts) == {0, 1}
    release.set()
    await asyncio.gather(t0, t1)


async def test_machine_lock_same_channel_serializes() -> None:
    ml = MachineLock(num_channels=8)
    order: list[str] = []

    async def op(tag: str) -> None:
        async with ml.shared(channels=[0]):
            order.append(f"{tag}_start")
            await asyncio.sleep(0.02)
            order.append(f"{tag}_end")

    await asyncio.gather(op("a"), op("b"))
    # whichever ran first must fully complete before the other starts
    assert order[0].endswith("_start")
    assert order[1].endswith("_end")
    assert order[2].endswith("_start")
    assert order[3].endswith("_end")


async def test_machine_lock_exclusive_blocks_shared() -> None:
    ml = MachineLock(num_channels=8)
    order: list[str] = []

    async def shared_op() -> None:
        async with ml.shared(channels=[5]):
            order.append("shared")

    async def exclusive_op() -> None:
        async with ml.exclusive(resources=["plate_1"]):
            order.append("exclusive_start")
            await asyncio.sleep(0.05)
            order.append("exclusive_end")

    tex = asyncio.create_task(exclusive_op())
    await asyncio.sleep(0.01)
    tsh = asyncio.create_task(shared_op())
    await asyncio.gather(tex, tsh)
    assert order == ["exclusive_start", "exclusive_end", "shared"]
