# plr — pylabrobot + ROS 2 for a Hamilton STAR

Scripts and a ROS 2 action server for driving a Hamilton STAR (8-channel)
through [pylabrobot](https://github.com/PyLabRobot/pylabrobot), including
off-deck plate handoffs to external instruments (incubator, cytomat, etc.)
via the STAR's firmware "hotel" commands.

Everything runs inside [`pixi`](https://pixi.sh) environments so the
underlying conda + ROS 2 Humble (RoboStack) stack stays reproducible.

## Quick start

```bash
pixi install -e ros2             # builds the three ROS 2 packages as
                                 # conda packages via pixi-build-ros

# One shell — run the action server against real hardware
pixi run -e ros2 action-server

# Another shell — drive it
pixi run -e ros2 ros2-move-to-incubator -- --yes
```

To run without hardware (STAR chatterbox simulator):

```bash
pixi run -e ros2 action-server-sim
```

Editing a source file? Just rerun `pixi run -e ros2 <task>` — pixi-build
sees the change and rebuilds the affected package automatically. No
`colcon build`, no `source install/setup.bash`.

## Repo layout

```
scripts/       Direct-hardware scripts + ROS 2 client scripts + the
               teach pendant. Each one is runnable via a pixi task.
ros2_ws/       The ROS 2 workspace. Five packages:
  hamilton_star_msgs/    STAR action/msg/srv definitions
  hamilton_star_ros/     STAR action server implementation
  hamilton_star_bringup/ STAR launch files + default deck + handoff config
  liconic_msgs/          STX incubator action/msg/srv definitions
  liconic_ros/           STX action server + launch file + plate registry
```

## Two worlds

The repo has two sets of tools depending on how much infrastructure you
want in the way:

- **Direct scripts** (`scripts/`, default pixi env) — open their own USB
  connection to the STAR, do one thing, disconnect. Useful for bring-up,
  recovery, and teaching calibrations. Examples: `pixi run test-iswap`,
  `pixi run iswap-teach`, `pixi run iswap-release`.
- **ROS 2 action server** (`ros2_ws/`, `ros2` pixi env) — persistent
  process, one USB connection, async actions + services. This is what
  bigger workflows should target. Launched via
  `pixi run -e ros2 action-server`; client scripts in `scripts/` with a
  `test_ros2_*.py` prefix hit it via ROS 2 actions.

## ROS 2 architecture

```
hamilton_star_msgs/     action + msg + srv definitions
hamilton_star_ros/      the node implementation
  action_server.py      rclpy plumbing (one action per goal type)
  machine.py            async facade over pylabrobot.LiquidHandler
  machine_fsm.py        operational state + substate gating
  machine_lock.py       per-channel / per-resource RW lock
  iswap_handoff.py      6-stage manual-jog recipe for off-deck transfers
  handoff_registry.py   runtime CRUD for off-deck handoff sites
  asyncio_bridge.py     bridges rclpy's callback threads to one asyncio loop
hamilton_star_bringup/  launch files + deck/handoff config
```

### Actions & services

The server exposes roughly two flavours:

- **Pipetting / move**: `pick_up_tips`, `drop_tips`, `aspirate`, `dispense`,
  `transfer`, `aspirate96` etc., `move_resource`, `pick_up_core_gripper`,
  `return_core_gripper`, `jog_channel`.
- **iSWAP-specific**: `handoff_transfer` (6-stage manual-jog, diagnostic
  per-stage feedback — useful when the firmware atomic paths misbehave).

Plus services for lifecycle and configuration: `reset_error`,
`abort_motion`, `initialize_module`, `load_deck`, `serialize_deck`,
`list_resources`, `get_status`, `acknowledge_plate_released`,
`define_handoff`, `list_handoffs`, `delete_handoff`.

### Off-deck plate handoffs: the one-line story

```python
await move_resource(resource="six_well_01", to="incubator_handoff")
```

The server has a **handoff registry** (`handoff_registry.py`) of off-deck
sites — each entry holds the XYZ coordinate, the iSWAP grip parameters,
and the firmware hotel approach parameters (`hotel_depth`,
`hotel_clearance_height`). When you register a handoff via
`~/define_handoff`, the server also injects a matching `ResourceHolder`
onto the deck tree at that coordinate, so `MoveResource.to = <name>`
Just Works — you don't have to think about "is this on-deck or off-deck".
The server auto-detects that `to` names a registered handoff and routes
through pylabrobot's firmware-atomic `use_unsafe_hotel=True` path.

For tuning or diagnostic flows where you want step-by-step visibility,
`handoff_transfer` (6-stage manual jog) is still available.

### State machine

`machine_fsm.py` is the single authority over `op_state` (Disconnected /
Initializing / Idle / BusyShared / BusyExclusive / Aborting / Error) and
a bundle of orthogonal substate flags: per-channel tip/gripper state,
iSWAP init / y-path / holding-plate / drive-faulted / at-safe-pose, and
the last reached handoff stage (for post-mortem). Key recovery semantics:

- Any firmware exception during an iSWAP-driven action sets
  `iswap_drive_faulted = True`. Future goals that need the iSWAP are
  rejected at the FSM gate until cleared.
- `~/reset_error` does **soft** firmware recovery: re-init the iSWAP,
  verify it responds with a position query, *then* clear the FSM. If the
  firmware is still faulted, the service returns `success=False` with a
  hint to restart the server or power-cycle the STAR.
- `iswap_holding_plate` survives exceptions on purpose — the operator
  must physically inspect the gripper (optionally run `iswap-release`)
  and acknowledge via `~/acknowledge_plate_released`.

The [FSM self-healing plan](.claude/plans/what-can-we-do-prancy-river.md)
in the repo walks through the design.

## Pixi tasks

### Default env (direct, no ROS)

| Task | What it does |
|------|--------------|
| `pixi run test-star` | Minimal hardware ping |
| `pixi run test-sim` | Same against the chatterbox sim |
| `pixi run test-gripper` | CoRe II gripper pickup/return |
| `pixi run test-iswap` | iSWAP sanity test |
| `pixi run iswap-teach` | Launch the FastAPI teach pendant (port 8080) |
| `pixi run test-iswap-grab` | Pick up a 6-well plate (direct) |
| `pixi run iswap-release` | Recovery: release a stuck plate and park |
| `pixi run iswap-to-incubator` | Direct manual-jog transfer to the incubator |
| `pixi run iswap-from-incubator` | Same, reverse direction |

### `ros2` env (action server + clients)

| Task | What it does |
|------|--------------|
| `pixi install -e ros2` | Build & install the three ROS 2 packages as conda packages (pixi-build-ros). Re-runs automatically when source changes. |
| `pixi run -e ros2 action-server` | Launch the action server vs real hardware |
| `pixi run -e ros2 action-server-sim` | Same, against the chatterbox simulator |
| `pixi run -e ros2 test-ros2-sim` | `pytest` the hamilton_star_ros tests |
| `pixi run -e ros2 ros2-iswap-to-incubator` | HandoffTransfer (6-stage) client |
| `pixi run -e ros2 ros2-iswap-from-incubator` | HandoffTransfer, reverse |
| `pixi run -e ros2 ros2-move-to-incubator` | **Canonical** MoveResource-to-registered-handoff |
| `pixi run -e ros2 liconic-server` | Launch the Liconic STX action server (own process, own /dev/liconic_stx44, plate-registry JSON at `~/.local/state/liconic/plate_registry.json`) |
| `pixi run -e ros2 liconic-roundtrip` | Client: Liconic take-in + fetch (needs only the Liconic server running) |
| `pixi run -e ros2 liconic-workcell-roundtrip` | Client: full Hamilton → Liconic → Hamilton round-trip (needs BOTH action servers running) |

Pass extra args to any ROS 2 client with `--`:

```bash
pixi run -e ros2 ros2-move-to-incubator -- --yes --rails 15
```

## Handoff registry by example

```bash
# Look at what's already registered (seeded from handoffs.yaml on boot)
ros2 service call /hamilton_star_action_server/list_handoffs \
    hamilton_star_msgs/srv/ListHandoffs '{}'

# Register a new off-deck site
ros2 service call /hamilton_star_action_server/define_handoff \
    hamilton_star_msgs/srv/DefineHandoff '{
      handoff: {
        name: "cytomat",
        x: -216.0, y: 350.0, z: 120.0,
        plate_width: 80.0,
        rotation: "LEFT", wrist: "STRAIGHT", grip_direction: "FRONT",
        hotel_depth: 500.0,
        hotel_clearance_height: 8.0,
      },
      replace_existing: true
    }'

# Now move a plate there via the generic MoveResource action
ros2 action send_goal /hamilton_star_action_server/move_resource \
    hamilton_star_msgs/action/MoveResource \
    '{ resource: "six_well_01", to: "cytomat", transport: "iswap" }'
```

## Development

```bash
pixi run -e ros2 test-ros2-sim    # full sim-backed pytest
rm -rf .pixi/build && pixi install -e ros2   # force a clean rebuild
```

Pure-Python unit tests (FSM, locks, bridge, registry) don't need rclpy
and run in `~1 second`; the rclpy-backed tests spin up an in-process
action server with the chatterbox simulator.

### ROS 2 build

The three packages under `ros2_ws/src/` are built by
[pixi-build-ros](https://pixi.prefix.dev/latest/build/backends/pixi-build-ros/)
(preview feature in pixi). Each package has its own `pixi.toml`
declaring the backend, the ROS distro (`humble`), and any sibling
path-deps; the root `pixi.toml` references them via conda-style path
entries under `[feature.ros2.dependencies]`. Pixi resolves, builds,
and installs the packages into the `ros2` env as real conda packages,
so `import hamilton_star_msgs.action`, `ros2 launch hamilton_star_bringup
…`, and the pytest suite all work without a `source install/setup.bash`
step.

The legacy `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/` dirs
from the old colcon setup can be deleted — they're not used.

## Hardware setup

Plug in the STAR's USB cable; install the udev rule once:

```bash
sudo ./install-udev.sh
```

See [README-hamilton-udev.md](README-hamilton-udev.md) for details.
