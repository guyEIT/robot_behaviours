# Tree executor: async with reactive halt

**Decision (2026-04-25):** keep the async/await tree executor. Don't refactor
to BT.CPP-style tick-based execution. Add reactive halt primitives on top of
async to get the responsiveness benefits a tick loop would have provided.

## Why async over tick-based

The TODO points the project toward concurrent trees, async tool calls (MCP),
and agentic skills. Async fits all three; tick-based works against them:

| Future requirement                | async                                                    | tick-based                                |
| --------------------------------- | -------------------------------------------------------- | ----------------------------------------- |
| Parallel sub-trees                | `asyncio.gather` (already in `ParallelNode`)             | manual per-tick state reduction           |
| Multiple trees concurrently       | `asyncio.gather(tree_a, tree_b)`                         | needs an inter-tree scheduler             |
| MCP / LLM / async tool nodes      | `await mcp.call_tool(...)` inside `tick`                 | a phase machine per node, every time      |
| Reactive halt                     | `await_or_halt(future, ctx)` (this doc)                  | tick-period-bounded                       |
| Cancellation latency              | sub-100 ms (50 ms poll inside `await_or_halt`)           | bounded by tick period                    |
| Mid-action condition re-eval      | `ReactiveGuard` wrapper races condition vs. work         | natural                                   |

The case where tick-based wins — many small reactive behaviors needing
lock-step BT.CPP semantics — isn't this codebase. Lab orchestration of
long-running skills with eventually-arriving async tools is async territory.

## What landed (2026-04-25)

1. **Reactive halt primitives** in [tree_executor.py](../src/robot_skill_server/robot_skill_server/tree_executor.py):
   - `HaltedError` — raised mid-await when `ctx.is_halted()` becomes true.
   - `await_or_halt(future, ctx, poll_interval=0.05)` — wraps an `rclpy.Future`
     await with a 50 ms poll against the halt flag.
   - `sleep_or_halt(ctx, seconds, poll_interval=0.05)` — same idea for sleeps.
   - `ExecutionContext.is_halted()` covers both `cancelled` (operator Cancel)
     and `abort_requested` (lease revoke / hard stop).
2. **Halt-aware nodes**: `RosActionNode`, `WaitForDurationNode`, and
   `HumanBlockingNode` now use these helpers. Operator Cancel during a 60 s
   wait, a long Meca move, or a pending human prompt returns within
   ~50 ms instead of waiting for the underlying future.
3. **5 Hz heartbeat publisher** in [bt_executor.py](../src/robot_skill_server/robot_skill_server/bt_executor.py)
   — keeps the dashboard's TaskState updates regular even when no transition
   is happening (e.g. during a long Meca move). Independent of execution model.
4. **Action contract docstring fixed** —
   [ExecuteBehaviorTree.action](../src/robot_skills_msgs/action/ExecuteBehaviorTree.action)
   `tick_rate_hz` is now documented as the heartbeat rate, not a tree-tick
   frequency. Honoured by the heartbeat timer; clamped to [0.5, 20] Hz.

## Patterns for new node types

When adding skills that talk to MCP, an LLM, or other async tools:

```python
class MyAsyncToolNode(TreeNode):
    async def tick(self, bb, ctx):
        ctx.on_skill_started(self.name)
        try:
            # The mcp/openai/anthropic clients are already async — just await.
            result = await self._call_tool(...)
        except Exception as e:
            ctx.on_skill_failed(self.name)
            ...
            return NodeStatus.FAILURE
        ctx.on_skill_completed(self.name)
        return NodeStatus.SUCCESS
```

If the tool exposes a `Future`-like, race it against halt:

```python
fut = client.do_long_thing()
try:
    result = await await_or_halt(fut, ctx)
except HaltedError:
    fut.cancel(); return NodeStatus.FAILURE
```

No phase machines, no per-tick state.

## Open follow-ups (not blocking)

- **`ReactiveGuard` wrapper** for the rare cases where a condition needs to
  preempt an action mid-flight based on something other than ctx.is_halted().
  Implement when an actual tree needs it.
- **`ParallelNode` halt cascade** — when one parallel branch fails fast and
  triggers halt, sibling branches' inflight goals get cancelled via the
  existing `ctx.inflight_goals` walker. Verify with a parallel-fail test.

## History

The original tick-based migration plan (a ~1100-line refactor of every node
class into phase machines, plus a fixed-rate tick loop in the executor)
was drafted and parked in favour of the lighter async + reactive-halt
approach above. See git history of this file for the full draft.
