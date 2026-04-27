# Robot Skills Framework

## Development Environment

This is a ROS2 Jazzy workspace. All builds, tests, and ROS commands must run inside the Docker container.

### Starting the containers

**Real robot** (Meca500 with MoveIt2 running on host):
```bash
docker compose up -d
```

This starts the dev container which connects to the host's MoveIt2 instance via host networking. The skill server auto-configures for the Meca500 (planning group `meca500_arm`, controller `/joint_trajectory_controller`).

**Simulation** (Panda mock hardware):
```bash
docker compose --profile sim up -d
# or: ROBOT_HARDWARE_MODE=sim docker compose --profile sim up -d
```

This starts the sim container (MoveIt2 + ros2_control with Panda), waits for `move_group` to be healthy, then starts the dev container with sim-mode parameters.

### Running commands inside the container

Use `docker exec` to run commands:

```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash 2>/dev/null; <COMMAND>"
```

### Common commands

Build all packages:
```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install"
```

Build a single package:
```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install --packages-select <PACKAGE>"
```

Run tests:
```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash && cd /home/ws && colcon test && colcon test-result --verbose"
```

### Launch the skill server

```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash && ros2 launch robot_skill_server skill_server.launch.py"
```

### Lite sim mode (no MoveIt2, single container)

For dashboard and orchestration development without the full MoveIt2 stack:

```bash
pixi run lite-up        # start
pixi run lite-down      # stop
pixi run lite-restart   # restart
pixi run lite-logs      # follow logs
pixi run lite-shell     # shell into container
```

Dashboard: http://localhost:8081 | rosbridge: ws://localhost:9090

After frontend changes (`vite build`), update the container symlinks:
```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install --packages-select robot_dashboard"
```

After C++ changes in the container, rebuild and restart:
```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && source /opt/bt_ros2_ws/install/setup.bash && cd /home/ws && colcon build --packages-select <PACKAGE>"
pixi run lite-restart
```

### Live updating of skills and behavior trees

The workspace is designed so that Python, XML, and frontend changes take effect without rebuilding Docker images.

**How it works:**

1. **Volume mounts** — Docker Compose mounts `./src:/home/ws/src:rw` into the container. Edits on the host are immediately visible inside the container.
2. **Symlink install** — `colcon build --symlink-install` creates symlinks from `install/` back to `src/` for Python packages and XML share files. This means Python code and behavior tree XMLs are served directly from the mounted source.
3. **Behavior tree auto-discovery** — `BtExecutor` (`bt_executor.py`) prefers the source directory `/home/ws/src/robot_behaviors/trees` over the installed share. A 2-second polling timer (`_check_trees_changed`) detects new, modified, or deleted `.xml` files and re-publishes the tree list to the latched `/skill_server/available_trees` topic.
4. **Frontend live subscription** — The dashboard subscribes to `/skill_server/available_trees` via rosbridge (WebSocket on port 9090). New trees appear in the Preset selector within ~2 seconds of the file being saved.

**What requires what:**

| Change type | Action needed |
|---|---|
| **New/edited behavior tree XML** | Save the `.xml` file in `src/robot_behaviors/trees/` — auto-discovered in ~2s |
| **Python skill server code** | Save the file — symlink-install means it's already live. Restart the node if it caches state at startup |
| **Frontend (React/Vite)** | Run `vite build` then `colcon build --symlink-install --packages-select robot_dashboard` inside the container (or use `pixi run dashboard-dev` for Vite hot-reload during development) |
| **C++ skill atoms or BT nodes** | Rebuild the package inside the container (`colcon build --packages-select <PKG>`) and restart the relevant nodes |
| **New C++ package or Dockerfile change** | Rebuild the Docker image |

**Uploading skills from the dashboard:**

The frontend provides three execution modes in the Behavior Executor panel:

- **Preset** — Select from auto-discovered tree XML files (from `src/robot_behaviors/trees/`)
- **Compose** — Build a task from registered skill steps. Calls `/skill_server/compose_task` to generate BT XML, then executes via `/skill_server/execute_behavior_tree`
- **Raw** — Paste arbitrary BT.CPP v4 XML and execute it directly

Compound skills can be registered at runtime via `/skill_server/register_compound_skill` and are persisted to `~/.ros2_robot_skills/compound_skills/{name}.yaml` so they survive container restarts. Atom skills self-register with the SkillRegistry when their action servers start.

**Key ROS2 topics and services:**

| Endpoint | Type | Purpose |
|---|---|---|
| `/skill_server/available_trees` | Topic (latched) | JSON list of discovered tree files and their XML content |
| `/skill_server/active_bt_xml` | Topic (latched) | XML of the currently-executing behavior tree |
| `/skill_server/task_state` | Topic | Execution progress (current node, completed/failed skills) |
| `/skill_server/execute_behavior_tree` | Action | Execute arbitrary BT XML at runtime |
| `/skill_server/compose_task` | Service | Generate BT XML from an ordered list of TaskSteps |
| `/skill_server/register_compound_skill` | Service | Register and persist a new compound skill |
| `/skill_server/get_skill_descriptions` | Service | Query the skill registry (supports category/tag filters) |

### Packages

Framework (at `src/`):
- `robot_skills_msgs` — ROS2 interfaces (actions/msgs/srvs)
- `robot_arm_skills` — C++ (or Python) primitive skill action servers (generic MoveIt2 / controller_manager wrappers; not robot-specific)
- `robot_mock_skill_atoms` — C++ mock skill atoms for lite sim mode
- `robot_bt_nodes` — C++ BehaviorTree.CPP v4 leaf node plugins
- `robot_skill_server` — Python orchestrator (SkillRegistry, TaskComposer, BtExecutor) + C++ bt_runner
- `robot_behaviors` — XML behavior tree definitions
- `robot_dashboard` — React web dashboard (Vite + Three.js + rosbridge)
- `robot_sim_config` — **Panda** URDF / SRDF / MoveIt config used by the lite-native stack and the `docker compose --profile sim` path. *Not* the Meca sim — Meca500's sim comes from `providers/meca500/src/meca500_moveit/` via `pixi run meca500-sim-up`. Two deliberate sim paths:
  - **Panda generic sim** (`robot_sim_config`) — 7-DoF arm used by mocks and dashboard dev; no robot-specific kinematics needed.
  - **Meca500 sim** (`providers/meca500/`) — 6-DoF MoveIt fake_hardware; what you want for exercising real Meca BTs without the physical arm.

  Could eventually become `providers/panda_sim/` for consistency with the providers pattern, but leaving as-is to avoid churn in lite-native / docker-sim callers.

Providers (at `providers/`) — each is a git subtree of an upstream repo OR a fresh in-tree provider that contributes skills to the SkillRegistry:
- `providers/meca500/` — Mecademic Meca500 arm (upstream: `guyEIT/meca500_ros2` @ main). ROS 2 Jazzy. Four packages: `meca500_description`, `meca500_hardware`, `meca500_moveit`, `meca500_bringup`.
- `providers/pbi_liconic/` — PBI Liconic STX44 incubator + Hamilton STAR liquid handler (upstream: `guyEIT/pbi_liconic` @ master). Originally ROS 2 Humble; **migrated to Jazzy locally** so it can share the root pixi workspace's solve (the per-package `distro = "jazzy"` changes are a local divergence from upstream — push back via `git subtree push` once validated on real hardware). Five packages under `ros2_ws/src/`: `liconic_msgs`, `liconic_ros`, `hamilton_star_msgs`, `hamilton_star_ros`, `hamilton_star_bringup`.
- `providers/imaging_station/` — generic plate imaging-station provider, **fresh in-tree (not a subtree)**. Single ament-python package at `providers/imaging_station/src/imaging_station/`. Owns the `robot_skills_msgs/action/ImagePlate` interface (declared `idempotent=true` in its manifest) and ships a sim backend that writes a placeholder PNG + JSON metadata per requested site under `~/.local/state/imaging_station/{plate_name}/{epoch_ms}_{counter}/`. Real driver backends (BMG / Tecan Spark / BioTek Cytation / microscope) plug into the same action — the directory layout and launch-file already make room for the swap.

The provider pattern: any upstream workspace OR fresh in-tree component that provides skills (arm, instrument, imager, software-only vision/planner) goes under `providers/<name>/`. Generic framework atoms stay in `src/robot_arm_skills/` — a skill atom belongs in a provider's own package only when its implementation depends on a vendor SDK or a robot-specific behavior that can't be parameterized via ROS params.

### Per-PC pixi environments

Each PC installs only what it needs via a scoped pixi environment:

| Environment      | Target PC              | `pixi run …` tasks                                       |
|------------------|------------------------|----------------------------------------------------------|
| `lite-native`    | dev box                | `lite-native-up`                                         |
| `real-native`    | single-box real robot  | `real-native-up`                                         |
| `orchestrator`   | control PC             | `orchestrator-up`                                        |
| `meca500-host`   | Meca500 robot PC       | `meca500-moveit-run` + `meca500-atoms-run` (two shells); `meca500-sim-up` for fake_hardware |
| `liconic-host`   | Liconic/Hamilton PC    | `liconic-up`, `liconic-sim-up`, `hamilton-up`, `hamilton-sim-up` |

`~/channel` still hosts `ros-jazzy-robot-skills-msgs` (built once via `pixi run update-msgs`) — every env depends on it by name.

**System requirement note:** root pixi.toml declares `[system-requirements] libc = { family = "glibc", version = "2.34" }` so Linux envs can resolve packages like `ros-jazzy-realsense2-camera` (pulled in by `meca500_bringup`) that gate on `__glibc >=2.34`. Without this, pixi assumes older glibc and the meca500-host env refuses to solve.

### Running sims end-to-end

#### Whole-lab one-shot (for clicking through BTs in the UI)

```bash
pixi run lab-sim-up
# open http://localhost:8081 — pick test_meca500_sim / test_hamilton_sim
# / test_liconic_smoke from the preset selector and hit Run
```

Brings up *every* provider sim + skill atoms + skill_server + web dashboard in a single `ros2 launch` invocation. Validated 2026-04-25: hamilton 0.54s, liconic 0.54s, meca500 8.80s — all SUCCESS through the unified launch.

The `lab-sim` pixi env is a superset of `meca500-host` + `liconic-host` + `orchestrator` (everything on one box). Use it for dev / smoke tests; production deploys still split across per-host envs.

#### Per-provider one-shot (single sim only)

If you only want one sim running, the per-provider launches are still wired up — useful when you're hammering on one device and don't want the other sims spamming logs.

| Sim       | One-shot command            | What it launches                                             | Tree to submit             |
|-----------|-----------------------------|--------------------------------------------------------------|----------------------------|
| Meca500   | `pixi run meca500-sim-test` | `moveit.launch.py simulation:=true` + `skill_atoms_remote.launch.py robot_name:=meca500` (4s delay so MoveIt is up first) + `skill_server_node` | `test_meca500_sim.xml`     |
| Hamilton  | `pixi run hamilton-sim-test`| `hamilton_star_bringup/action_server.launch.py backend:=simulator` + `skill_server_node` | `test_hamilton_sim.xml`    |
| Liconic   | `pixi run liconic-sim-test` | `liconic_ros/liconic.launch.py simulation:=true` + `skill_server_node`                   | `test_liconic_smoke.xml`   |

The composite launches live in `src/robot_skill_server/launch/sim_<provider>.launch.py` and use `IncludeLaunchDescription` to pull in the upstream provider launch files — no duplication.

Submitting a tree (any of the three sims):
```bash
python3 -c "import yaml; xml=open('src/robot_behaviors/trees/test_<sim>.xml').read(); \
  print(yaml.safe_dump({'tree_xml': xml, 'tree_name': '<sim>'}, default_style='|'))" > /tmp/g.yaml
pixi run -e liconic-host bash -c 'ros2 action send_goal \
  /skill_server/execute_behavior_tree robot_skills_msgs/action/ExecuteBehaviorTree \
  "$(cat /tmp/g.yaml)"'
```

Validated 2026-04-25: hamilton 0.53s, liconic 0.55s, meca500 8.67s — all SUCCESS via one-shot launch.

If you'd rather run pieces individually (debugging, mixing real + sim, splitting across PCs), the granular tasks are still there: `meca500-sim-up` / `meca500-up` (atoms only) / `liconic-sim-up` / `hamilton-sim-up` / and a bare `skill_server_node` invocation. See `pixi info` for the full list.

The Liconic sim starts with the transfer tray pre-loaded so TakeIn succeeds immediately — real hardware would require an operator to physically place a plate first. Source: [providers/pbi_liconic/ros2_ws/src/liconic_ros/liconic_ros/sim_backend.py](providers/pbi_liconic/ros2_ws/src/liconic_ros/liconic_ros/sim_backend.py). Selected at launch time via `simulation:=true`.

### Test behavior trees — validation status

Three provider smoke tests live in `src/robot_behaviors/trees/`:

- **`test_hamilton_sim.xml`** — validated SUCCESS end-to-end 2026-04-24. PickUpCoreGripper (channels 6+7) → 0.5s settle → ReturnCoreGripper. Runs against STARChatterboxBackend (`pixi run hamilton-sim-up`) via the skill_server BtExecutor (`ros2 action send_goal /skill_server/execute_behavior_tree`). Total runtime 0.54s. Original MoveResource-based tree was rewritten because the default `star_deck.json` has no plate carrier resources — core gripper pick/return is the cheapest round-trip that exercises a real action server.

- **`test_meca500_sim.xml`** — validated SUCCESS end-to-end 2026-04-24 (9.85s). Check-system-ready → home → joint config A → joint config B → home. Runs against `mock_components/GenericSystem` (MoveIt fake_hardware) with real MoveIt motion planning + joint_trajectory_controller execution. Required two fixes in `providers/meca500/`: (1) URDF xacro now switches the `<plugin>` on `$(arg simulation)` (mock vs. real Meca500Hardware); (2) `moveit.launch.py` calls `.perform(context)` on `LaunchConfiguration` mappings before passing them to `moveit_configs_utils.robot_description()` (the util doesn't resolve Substitutions on its own). Both changes are local divergences from upstream `guyEIT/meca500_ros2` — push back via `git subtree push` once real-hardware regression confirms parity.

- **`test_liconic_smoke.xml`** — validated SUCCESS end-to-end 2026-04-24 (1.09s) against `LiconicSimBackend`. TakeIn of a placeholder plate into cassette 1 position 1, then Fetch it back by plate_name. The sim implements the subset of `pylabrobot.storage.liconic.ExperimentalLiconicBackend` that `liconic_ros.machine.LiconicMachine` calls — plate placement, climate setpoints, shaker state, error registers — all in memory. Selected via `simulation:=true` launch arg. For real STX44 hardware: `pixi run liconic-up` (same tree, same actions).

The BT executor's `ACTION_REGISTRY` is conditional: `LiconicTakeIn`, `LiconicFetch`, `HamiltonMoveResource`, `HamiltonHandoffTransfer`, `HamiltonPickUpCoreGripper`, `HamiltonReturnCoreGripper` register only when `liconic_msgs` / `hamilton_star_msgs` are importable. The `orchestrator` pixi feature pulls them in as path-deps so the server can build goals for those actions. Default `server_name` values match the upstream launch-file node names (`/hamilton_star_action_server/...`, `/liconic_action_server/...`); override per-tree via `server_name="..."` XML attribute.

### Bugs uncovered and fixed during validation

- **`WaitForDurationNode` used `asyncio.sleep`** — raised `RuntimeError: no running event loop` because rclpy's action-server `execute_callback` doesn't run inside an asyncio loop. Replaced with `rclpy.create_timer` + `rclpy.Future` so the await yields to the rclpy executor.
- **Many `pixi.toml` files didn't glob their Python / config sources** — the default `pixi-build-ros` input set covers only `package.xml` + `CMakeLists.txt`, so edits to `*.py`, `*.xacro`, `*.yaml`, `launch/*` silently reused the cached build and left stale `.pyc` files in the env. Fixed in: `src/robot_skill_server/pixi.toml` (globs `robot_skill_server/**/*.py`), `providers/meca500/src/meca500_moveit/pixi.toml` (globs `config/**/*` + `launch/**/*`), `providers/meca500/src/meca500_bringup/pixi.toml` (globs `launch/` + `config/` + `rviz/` + `scripts/`), `providers/meca500/src/meca500_description/pixi.toml` (globs `urdf/` + `meshes/`), `providers/pbi_liconic/ros2_ws/src/liconic_ros/pixi.toml`, and `providers/pbi_liconic/ros2_ws/src/hamilton_star_ros/pixi.toml`. This class of bug is easy to miss because `pixi install` prints "installed successfully" either way.
- **`meca500_hardware` always tried to TCP-connect to the real Meca** — the URDF xacro hardcoded `<plugin>meca500_hardware/Meca500Hardware</plugin>`, ignoring `simulation:=true`. Fixed by wrapping the hardware block in `<xacro:if value="$(arg simulation)">` (mock_components) / `<xacro:unless>` (real plugin), and declaring `simulation` as a xacro:arg in both `meca500.urdf.xacro` and `meca500.ros2_control.xacro`.
- **`moveit_configs_utils.robot_description(mappings=...)` doesn't resolve LaunchConfigurations** — it calls `str()` on the Substitution objects and ends up passing `<LaunchConfiguration at 0x...>` to xacro, which silently ignores them. Fixed by calling `.perform(context)` on each mapping value in `meca500_bringup/launch/moveit.launch.py`.
- **`meca500-host` was missing many runtime ROS deps** — `meca500_bringup`'s launch file pulls in `robot_state_publisher`, `moveit_ros_move_group`, `controller_manager`, `joint_trajectory_controller`, `xacro`, etc. via `Node(package=...)`. These are `exec_depend` in package.xml but robostack's dependency mapping doesn't always surface them; listed explicitly in the `meca500-host` feature. Plus `filelock` (pypi) which `controller_manager`'s spawner imports but robostack doesn't declare.
- **`check_system_ready` only recognized `arm_controller` / `gripper_controller` system labels** — but `robots.yaml` had `joint_trajectory_controller` in `check_systems`. The atom's switch statement fell to the unknown-system branch, returned false, and the BT's `CheckSystemReady` node failed. Fixed `robots.yaml` to use the canonical `arm_controller` label (the atom then looks up the actual action path via the `arm_controller_action` param).
- **Provider server-name assumption** — initial `ACTION_REGISTRY` defaults used `/hamilton/...` and `/liconic/...` namespaces; actual upstream launch files name the nodes `hamilton_star_action_server` and `liconic_action_server`, so actions live at `/<node_name>/<action>`. Fixed.

### Working with provider subtrees

The two upstream repos (`guyEIT/meca500_ros2`, `guyEIT/pbi_liconic`) are pulled in via `git subtree`, not submodules. Their history is squash-merged into this repo under `providers/<name>/`.

Update to latest upstream:
```bash
git fetch meca500 && git subtree pull --prefix=providers/meca500      meca500 main   --squash
git fetch liconic && git subtree pull --prefix=providers/pbi_liconic  liconic master --squash
```

Push local changes back upstream (when a fix belongs there, not here):
```bash
git subtree push --prefix=providers/meca500     meca500 <branch>
git subtree push --prefix=providers/pbi_liconic liconic <branch>
```

Remotes are already configured (`meca500` → `guyEIT/meca500_ros2.git`, `liconic` → `guyEIT/pbi_liconic.git`).

### Long-lived BT support (added 2026-04-27)

Framework persistence so a single tree can run for weeks while the operator
trickles plates in and out of the imaging campaign:

- **Persistent blackboard layer** ([persistent_blackboard.py](src/robot_skill_server/robot_skill_server/persistent_blackboard.py)) — SQLite WAL store at `~/.local/state/skill_server/tasks/{task_id}/state.db`. Keys with prefix `persistent.` mirror automatically; type-checked at write (raises `PersistenceError` for non-JSON values).
- **Per-node tick checkpoints** — `_Checkpoint` mixin on `Sequence`, `Fallback`, `RetryUntilSuccessful`, plus the new `Repeat`, `KeepRunningUntilFailure`, `WhileDoElse`. Each persists the index/iteration/attempt that has *successfully completed*; on resume, the parent re-ticks the next child.
- **Action-inflight reconciliation** — `RosActionNode` records `(node_path, server_name, goal_uuid, idempotent)` before submit; on resume, idempotent → auto-resubmit, non-idempotent → log error with the DB path and FAIL the tree (operator clears the row or issues an `OperatorDecision` to proceed). Idempotency is declared per-skill in `SkillDescription.idempotent` (default false).
- **New leaf nodes** — `WaitUntil timestamp="..."` (deadline-preserving across restart), `BlackboardCondition`, `PopFromQueue` / `PushToQueue`, `AdvancePlate` (post-cycle bookkeeping).
- **bb_operator sidecar** ([bb_operator_node.py](src/robot_skill_server/robot_skill_server/bb_operator_node.py)) — operator services that mutate the active task's persistent blackboard. Endpoints: `/bb_operator/{add_plate,retire_plate,pause_campaign,operator_decision}`. Tracks the active task via `/skill_server/task_state` and writes through SQLite WAL — both processes interleave safely.
- **Campaign tree** at [src/robot_behaviors/trees/campaign/plate_imaging_campaign.xml](src/robot_behaviors/trees/campaign/plate_imaging_campaign.xml) — `KeepRunningUntilFailure → BlackboardCondition(paused == false) → Sequence(Pop → WaitUntil → SubTree → AdvancePlate)`. Inner `PlateImageCycle` wraps the iSWAP transfers + `ImagePlate` + Liconic round-trip in `RetryUntilSuccessful num_attempts=2`.
- **Imaging station calibration** — `imaging_handoff` site added to [providers/pbi_liconic/ros2_ws/src/hamilton_star_bringup/config/handoffs.yaml](providers/pbi_liconic/ros2_ws/src/hamilton_star_bringup/config/handoffs.yaml). Placeholder coordinates — needs a real `recalibrate_handoff.xml` run when the imager is physically positioned.
- **Dashboard mirror topic** — `bb_operator` publishes `/skill_server/persistent_state` (latched `std_msgs/String` JSON snapshot of the active task's `persistent.*` blackboard + derived `task_id` / `task_status` / `paused`). Republished after every successful service handler and on a 1 Hz timer when a task is active. The dashboard's **Campaign** panel ([src/robot_dashboard/frontend/src/components/campaign/](src/robot_dashboard/frontend/src/components/campaign/)) subscribes to it; AddPlate / Pause / Cancel / Retire buttons call back through `/bb_operator/*` and `/skill_server/cancel_active_task`.
- **Framework-level execution control on `BtExecutor`**:
  - `/skill_server/pause_execution` (`PauseCampaign.srv` shape — generic, not campaign-specific) sets `ctx.paused` honoured at step boundaries.
  - `/skill_server/cancel_active_task` (`CancelActiveTask.srv`) — session-independent hard cancel. Walks `_current_ctx`, sets `cancelled`, and calls `_cancel_inflight`. Reachable by any client (the action-cancel handshake needs the original goal id, which a restarted dashboard doesn't have).
- **Bug fixes during integration**: (1) `Blackboard` for `persistent.*` keys now reads through to SQLite on every access (no in-memory cache) — operator writes via `bb_operator` are immediately visible to the running tree. (2) `Blackboard.resolve()` walks dotted paths into dict-valued blackboard keys (`{persistent.current_plate.next_due_at}` works). (3) `BlackboardCondition` treats unset keys (None) as falsy via Python truthiness — so `expected="false"` gates on `persistent.paused` admit the loop *before* the first Pause call has touched the flag. (4) `_publish_available_trees` / `_check_trees_changed` skip files that vanish between `glob()` and `stat()` (race on symlink-installed trees during a concurrent build). (5) `colcon symlink-install` of the dashboard's `dist/` doesn't pick up new bundle hashes after a `npm run build` — operator workaround: `rm -rf .pixi/colcon/install/robot_dashboard .pixi/colcon/build/robot_dashboard` then re-run colcon build.

Tests cover all of this: `test_persistent_blackboard.py` (20), `test_tree_resume.py` (25), `test_bb_operator.py` (8), and the imaging provider's `test_sim_backend.py` (13). End-to-end validation in chromium via Playwright walks empty-state → submit campaign → AddPlate → Pause → Resume → Cancel, taking screenshots of each transition. Run via `cd /tmp/dash-test && node test_campaign_panel.mjs` (after `pixi run -e local-dev` has installed the playwright chromium under `~/.cache/ms-playwright/`).

### Known follow-ups (flagged by the providers integration)

- **`server_name` XML override** — `RosActionNode` in `tree_executor.py` reads `server_name` from XML attrs, so per-tree routing to `/meca500/skill_atoms/...` works. The three test BTs (`test_meca500_sim.xml`, `test_hamilton_sim.xml`, `test_liconic_smoke.xml`) rely on this. A follow-up tidy: add a `robot_namespace` BT input to subtrees so the override only has to be written once per branch.
- **Liconic / Hamilton are not yet in `robots.yaml`** — they have self-contained action servers that don't use `SkillBase`, so `skill_atoms_remote.launch.py` (which assumes MoveIt-based arm atoms) isn't the right integration point. A separate change needs to teach SkillRegistry to discover Liconic/Hamilton skills directly, or add a thin wrapper that republishes them under `/skill_atoms/...`.
- **Meca500 planning group / MoveIt namespace** — SRDF declares `meca500_arm` (matches `robots.yaml`), but `providers/meca500/src/meca500_bringup/launch/moveit.launch.py` also sets a launch param `planning_group_name: "meca500"`. Verify during first real-robot bringup whether the arm atoms on the orchestrator can reach move_group at `/move_action` or whether MoveIt is under the `/meca500` namespace.
- **pbi_liconic Jazzy migration needs upstream PR** — the `distro = "humble"` → `"jazzy"` and `robostack-humble` → `robostack-jazzy` edits in `providers/pbi_liconic/**/pixi.toml` are local-only. Once real-hardware regression is done, push via `git subtree push --prefix=providers/pbi_liconic liconic <branch>` and open a PR upstream.
- **Docker image** — `.devcontainer/Dockerfile` still apt-installs its ROS deps and doesn't know about `providers/`. Either rebuild the image to include provider packages, or switch Docker to use pixi inside the container.

### Documentation

- `docs/adding-skills.md` — How to add new skills (atoms, mocks, BT nodes, compound skills)
- `.claude/commands/new-skill-atom.md` — Claude command: create a new skill atom
- `.claude/commands/new-compound-skill.md` — Claude command: create a compound skill
- `.claude/commands/new-behavior-tree.md` — Claude command: create a behavior tree
- `.claude/commands/debug-skill.md` — Claude command: diagnose skill/BT failures
