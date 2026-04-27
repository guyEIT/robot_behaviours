# Robot Skills Framework — Architecture

A ROS 2 Jazzy workspace that exposes every robot/instrument skill — hardware-bound or software-only — as a self-advertising ROS action. The orchestrator discovers skills at runtime by subscribing to latched `*/skills` topics; nothing is hardcoded. Behavior trees are authored as XML, composed from skill steps, and executed by a Python tree executor.

## Core principles

1. **A ROS action is the only skill API.** C++/MoveIt arm skills, Python/pylabrobot instrument skills, and agent-authored scripts all advertise the same way and are dispatched the same way.
2. **Discovery, not registration.** Every node hosting skills publishes a latched `<node>/skills` manifest with `TRANSIENT_LOCAL` durability and `LIVELINESS_AUTOMATIC` QoS. Restart and late-join are handled by DDS, not heartbeats.
3. **Topology = trust.** Code runs on the host that owns its execution context. The orchestrator never executes agent-authored code; it dispatches to ROS endpoints. An LLM-authored script lives on the agent's machine — DDS sees it as just another action server.
4. **One process per provider host.** Each robot/instrument PC runs a single skill-server-proxy node hosting all of that host's actions, not a fleet of one-skill-per-node executables.
5. **Each top-level directory maps to one deployment role.** A package's location tells you where it runs.

## Runtime graph

```
  EXTERNAL CLIENTS — neither lives on the orchestrator.

  ┌─────────────────────────────────────┐    ┌────────────────────────────────────────────────────┐
  │ Browser (human)                     │    │ AGENT HOST (LLM laptop / sandbox VM)               │
  │   loads SPA from :8081              │    │  ┌──────────────────────────────────────────────┐  │
  │   live ROS via ws://9090            │    │  │ Agent / LLM (Claude Code, custom MCP client) │  │
  │   (rosbridge)                       │    │  │   spawns bt_executor_mcp as stdio subprocess │  │
  │                                     │    │  └──────────────────┬───────────────────────────┘  │
  │   Surfaces: BT executor, task       │    │                     │ stdio                         │
  │   monitor, skill browser, BT tree   │    │                     ▼                               │
  │   viewer, log/plot/diagnostics      │    │  ┌──────────────────────────────────────────────┐  │
  └────────────┬────────────────────────┘    │  │ bt_executor_mcp   (agent/robot_skill_mcp)    │  │
               │ rosbridge JSON                │  │   FastMCP stdio + mcp_ros_bridge             │  │
               │ over WebSocket                │  │   rclpy node — joins lab DDS                 │  │
               │ ws://orchestrator:9090        │  │   tools: list_skills / list_trees /          │  │
               │                               │  │          compose_task / execute_tree /       │  │
               │                               │  │        ★ register_script                     │  │
               │                               │  └────────────────────┬─────────────────────────┘  │
               │                               │                       │ owns                       │
               │                               │                       ▼                            │
               │                               │  ┌──────────────────────────────────────────────┐  │
               │                               │  │ script_action_server                         │  │
               │                               │  │   (agent/robot_script_server)                │  │
               │                               │  │   per-MCP-session lifetime                   │  │
               │                               │  │   advertises /script_action_server_<id>/skills│ │
               │                               │  │   hosts /scripts_<id>/<name> (RunScript)     │  │
               │                               │  └──────────────────────────────────────────────┘  │
               │                               └─────────────────────┬──────────────────────────────┘
               │                                                     │ */skills (latched) +
               │                                                     │ action goal calls
               │                                                     │ over ROS 2 DDS
┌──────────────┼─────────────────────────────────────────────────────┼─────────────────────────────┐
│ ORCHESTRATOR │                                                     ▼                             │
│   PC         ▼                                                                                   │
│  ┌────────────────────────────────────┐                                                          │
│  │ rosbridge_suite + dashboard SPA    │                                                          │
│  │   :8081 static + :9090 WebSocket   │                                                          │
│  │   pure transport, no business logic│                                                          │
│  └─────────────────┬──────────────────┘                                                          │
│                    │ ROS 2 service / action / topic                                              │
│                    ▼                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────────────────────────────┐ │
│  │ skill_server_node                          (src/robot_skill_server)                         │ │
│  │   • SkillDiscovery — subscribes to every */skills topic; aggregates                         │ │
│  │   • TreeExecutor    — async BT runtime; resolves (robot_id, bt_tag) → action server        │ │
│  │   • BtExecutor      — hosts /skill_server/execute_behavior_tree                             │ │
│  │   • TaskComposer    — generates BT XML from TaskStep lists                                  │ │
│  │   • LeaseBroker     — exclusive resource leases (TTL, renewal)                              │ │
│  │   • SkillRegistry   — vetted shared compound skills (persistent on disk)                    │ │
│  │   publishes:  /skill_server/skill_registry  /available_trees  /active_bt_xml                │ │
│  │              /task_state  /log_events  /lease_events                                        │ │
│  └─────────────────────────────────────────────────────────────────────────────────────────────┘ │
└────────────────────┬───────────────────────────┬───────────────────────────────┬─────────────────┘
                     │                           │                               │
                     │ */skills (latched, TRANSIENT_LOCAL, LIVELINESS_AUTOMATIC) │
                     │ + action goal calls                                       │
                     │                           │                               │
   ┌─────────────────┴─────┐  ┌─────────────────┴────────────┐  ┌────────────────┴───────────┐
   │ MECA500 PC            │  │ LICONIC PC                   │  │ DEV BOX (lite-native)      │
   │  meca500_skill_server │  │  liconic_action_server       │  │  panda mock skill atoms    │
   │   ┌─────────────────┐ │  │  ┌─────────────────────────┐ │  │  (mock backend, fake_hw)   │
   │   │MoveGroupInterface│ │  │  │pylabrobot Liconic STX44│ │  │                            │
   │   │ ├─ MoveToNamed* │ │  │  │  ├─ TakeIn / Fetch     │ │  │  loads:                    │
   │   │ ├─ MoveToJoint* │ │  │  └─────────────────────────┘ │  │   robot_sim_config         │
   │   │ ├─ MoveCartesian│ │  │  hamilton_star_action_server │  │   (Panda URDF/SRDF)        │
   │   │ ├─ Gripper /    │ │  │  ┌─────────────────────────┐ │  │                            │
   │   │ │   SetDIO /    │ │  │  │STAR backend            │ │  │  publishes:                │
   │   │ │   RobotEnable │ │  │  │  ├─ MoveResource       │ │  │   <node>/skills            │
   │   │ ├─ CheckSystem* │ │  │  │  ├─ HandoffTransfer    │ │  │                            │
   │   │ ├─ CheckCollis* │ │  │  │  └─ PickUp/ReturnCore* │ │  │  pkg dirs:                 │
   │   │ └─ UpdateScene* │ │  │  └─────────────────────────┘ │  │   providers/panda_sim/src/ │
   │   └─────────────────┘ │  │                              │  │                            │
   │  publishes:           │  │  publishes:                  │  │                            │
   │   /meca500_skill_     │  │   /liconic_action_server/    │  │                            │
   │     server/skills     │  │     skills                   │  │                            │
   │  + MoveIt2 stack      │  │   /hamilton_star_action_     │  │                            │
   │   (move_group,        │  │     server/skills            │  │                            │
   │    ros2_control,      │  │                              │  │                            │
   │    joint_traj_ctrl)   │  │  pkg dirs:                   │  │                            │
   │                       │  │   providers/pbi_liconic/     │  │                            │
   │  pkg dirs:            │  │     ros2_ws/src/{liconic_ros,│  │                            │
   │   providers/meca500/  │  │     hamilton_star_ros, …}    │  │                            │
   │     src/meca500_*     │  │                              │  │                            │
   └───────────────────────┘  └──────────────────────────────┘  └────────────────────────────┘
```

**Reading the diagram.** Two external client paths — browser and agent — neither runs on the orchestrator. Discovery flows up (every proxy publishes `<node>/skills`); action calls flow back down (the orchestrator's `TreeExecutor` resolves entries from the runtime registry, then dispatches the goal directly to the right host). Single-box deployments (`lab-sim`, `real-native`, `lite-native`) collapse all hosts onto one PC; the DDS graph is identical, the network just stops being involved.

## Skill discovery and advertisement

The architectural backbone. Every node hosting one or more skill action servers publishes one latched topic `<node_fqn>/skills`.

- **Type.** [robot_skills_msgs/msg/SkillManifest](lib/robot_skills_msgs/msg/SkillManifest.msg) wraps a list of [SkillAdvertisement](lib/robot_skills_msgs/msg/SkillAdvertisement.msg) entries, each pairing a full [SkillDescription](lib/robot_skills_msgs/msg/SkillDescription.msg) with BT-specific bindings (`bt_tag`, `goal_defaults`, `output_renames`, `post_process_id`, `input_xml_attrs`).
- **QoS.** `TRANSIENT_LOCAL` durability, `RELIABLE`, `KEEP_LAST(1)`, `LIVELINESS_AUTOMATIC` with `liveliness_lease_duration = 10s`. Late joiners receive the latched manifest; dead publishers are evicted by the discovery node's `on_liveliness_changed` callback.
- **Helpers.** [lib/robot_skill_advertise/](lib/robot_skill_advertise/) provides Python and C++ helpers so providers don't take a transitive dependency on the orchestrator. Used from Liconic, Hamilton, the Meca500 proxy, and the agent script server.
- **Discovery.** [skill_discovery.py](src/robot_skill_server/robot_skill_server/skill_discovery.py) runs on the orchestrator: 5 s timer scans `get_topic_names_and_types()` for `*/skills`, subscribes with the matching QoS, lazily imports each entry's action type, and aggregates the merged view onto `/skill_server/skill_registry`.
- **Multi-robot keying.** Runtime registry is `dict[(robot_id, bt_tag), ActionEntry]`. BT XML grows by one optional attribute on each action node: `robot_id="meca500"`. Resolution walks the parent subtree for an inherited `robot_id`, then falls back to `bt_tag` alone (parse error if ambiguous). Legacy `server_name="..."` remains a supported override.

## Behavior tree execution

[tree_executor.py](src/robot_skill_server/robot_skill_server/tree_executor.py) is a pure-Python BT.CPP-v4-compatible parser/executor. It supports:

- **Control flow** — Sequence, Fallback, Parallel, RetryUntilSuccessful, ForceSuccess/Failure, Decorators, SubTree.
- **Action nodes** — every `*/skills` advertisement registers as an XML tag. The executor reflects the action's Goal type to derive ports, applies `goal_defaults`, and runs `post_process_id` on the result before writing to the blackboard.
- **Utility nodes** — `WaitForDuration`, `Log`, `SetBlackboard`, plus human-interaction nodes (`HumanPrompt`, `WaitForHumanResponse`).
- **Cancellation.** Terminal states are `SUCCESS` / `FAILURE` / `HALTED` (cancel publishes `HALTED`, not `CANCELLED`).

[bt_executor.py](src/robot_skill_server/robot_skill_server/bt_executor.py) hosts the [/skill_server/execute_behavior_tree](lib/robot_skills_msgs/action/ExecuteBehaviorTree.action) action, polls [src/robot_behaviors/trees/](src/robot_behaviors/trees/) every 2 s for changes, and publishes the latched tree list to `/skill_server/available_trees`.

[task_composer.py](src/robot_skill_server/robot_skill_server/task_composer.py) generates BT XML from an ordered list of `TaskStep`s via [/skill_server/compose_task](lib/robot_skills_msgs/srv/ComposeTask.srv). The dashboard's *Compose* mode and the MCP `compose_task` tool both call it.

## Compound and scripted skills — two surfaces, two trust levels

| Surface | Host | Lifetime | Use case |
|---|---|---|---|
| [/skill_server/register_compound_skill](lib/robot_skills_msgs/srv/RegisterCompoundSkill.srv) | Orchestrator | Persistent (`~/.ros2_robot_skills/compound_skills/`) | Vetted, reused, lab-wide skills |
| `register_script` MCP tool ([RegisterScript.srv](lib/robot_skills_msgs/srv/RegisterScript.srv)) | Agent's machine | MCP session | Ad-hoc, session-scoped, agent-authored |

The agent path uses [RunScript.action](lib/robot_skills_msgs/action/RunScript.action) — a generic action with `KeyValue[] inputs / outputs`. The script server hosts one action server per registered script at `/scripts_<session>/<name>`, and dispatches on `kind ∈ {bt_xml, action_sequence, python_callable}`. `python_callable` is gated behind `--allow-python-scripts` and only ever runs on the agent's host.

The orchestrator's `SkillDiscovery` picks up agent-host scripts the same way it picks up hardware proxies; from the orchestrator's point of view an LLM-authored skill is indistinguishable from a Liconic action. From a security point of view the orchestrator never executes agent code.

## Resource leases

[lease_broker.py](src/robot_skill_server/robot_skill_server/lease_broker.py) provides exclusive named resource leases with TTL and renewal — used to gate concurrent access to shared hardware (e.g. the deck on the Hamilton, the Meca500 arm). Services: `acquire_lease`, `renew_lease`, `release_lease`. Latched snapshot on `/skill_server/leases`; lifecycle stream on `/skill_server/lease_events`.

## Directory layout — one bucket per deployment role

| Top-level dir | Role | Runs on |
|---|---|---|
| [lib/](lib/) | Shared libraries — host-agnostic, no runtime | Consumed by every host |
| [src/](src/) | Orchestrator processes + assets | Orchestrator PC |
| [agent/](agent/) | MCP-host processes + agent-local services | Agent's machine (anywhere on DDS) |
| [providers/](providers/) | Per-robot/instrument code | The robot/instrument PC |

### lib/ — shared

- [robot_skills_msgs](lib/robot_skills_msgs/) — actions, services, messages (incl. `SkillManifest`, `RunScript`, `KeyValue`).
- [robot_skill_advertise](lib/robot_skill_advertise/) — Python + C++ advertise/discovery helpers.
- [robot_arm_skills](lib/robot_arm_skills/) — MoveIt2 arm skill library (C++ headers + 12 atom implementations under [src/](lib/robot_arm_skills/src/)) and Python rosbag skills ([rosbag_skills_node.py](lib/robot_arm_skills/robot_arm_skills/rosbag_skills_node.py) — `RecordRosbag`, `StopRecording`). Compiled into both the Meca500 and Panda proxies.

### src/ — orchestrator

- [robot_skill_server](src/robot_skill_server/) — Python orchestrator (`skill_server_node`, `skill_discovery`, `tree_executor`, `bt_executor`, `task_composer`, `lease_broker`, `skill_registry`).
- [robot_dashboard](src/robot_dashboard/) — React 18 + Vite + TypeScript + Three.js SPA. Panels: bt-tree, executor, task-monitor, skill-browser, joint-viewer, tf-viewer, plotter, logs, diagnostics, intervention, human-prompts, service-caller, topics. Talks to ROS via rosbridge over `ws://9090`.
- [robot_behaviors](src/robot_behaviors/) — XML behavior tree definitions under [trees/](src/robot_behaviors/trees/).

### agent/ — MCP host

- [robot_skill_mcp](agent/robot_skill_mcp/) — FastMCP stdio server + `mcp_ros_bridge` rclpy node. Tools: `list_skills`, `list_trees`, `compose_task`, `execute_tree`, `register_script`, `list_scripts`, `delete_script`.
- [robot_script_server](agent/robot_script_server/) — per-MCP-session scripted action server. Co-launched with `bt_executor_mcp`.

### providers/

- [meca500/src/](providers/meca500/src/) — Mecademic Meca500 6-DoF arm. Five packages: `meca500_description` / `_hardware` / `_moveit` / `_bringup` (subtree of `guyEIT/meca500_ros2`), plus the local [meca500_skill_server](providers/meca500/src/meca500_skill_server/) per-robot proxy that wraps MoveIt once and hosts all 12 arm atoms in-process.
- [panda_sim/src/](providers/panda_sim/src/) — generic 7-DoF Panda sim. [robot_sim_config](providers/panda_sim/src/robot_sim_config/) (URDF/SRDF/MoveIt config) + [robot_mock_skill_atoms](providers/panda_sim/src/robot_mock_skill_atoms/) (mock backend, fake_hardware). Used by the dashboard dev path and `lab-sim`.
- [pbi_liconic/ros2_ws/src/](providers/pbi_liconic/ros2_ws/src/) — Liconic STX44 incubator + Hamilton STAR liquid handler (subtree of `guyEIT/pbi_liconic`). Each runs a single Python action-server proxy.

## Deployment topologies

| Mode | What runs where |
|---|---|
| **Distributed prod** | Orchestrator PC: `skill_server_node` + dashboard + rosbridge. Meca500 PC: `meca500_skill_server` + MoveIt. Liconic PC: `liconic_action_server` + `hamilton_star_action_server`. Agent host: anywhere on DDS. |
| **`lab-sim`** | Every provider sim + skill atoms + skill_server + dashboard, single `ros2 launch` on the dev box. Validated end-to-end (hamilton 0.54s, liconic 0.54s, meca500 8.80s). |
| **`real-native`** | Single-box real-robot deployment on the orchestrator PC. |
| **`lite-native`** | Dashboard + orchestration dev without MoveIt; uses `robot_mock_skill_atoms`. |
| **Per-provider sim** | `meca500-sim-test` / `hamilton-sim-test` / `liconic-sim-test` — one provider in isolation. |

Per-PC pixi environments (`lite-native`, `real-native`, `orchestrator`, `meca500-host`, `liconic-host`, `lab-sim`) install only what each PC needs; all share `ros-jazzy-robot-skills-msgs` from `~/channel`.

## Live-update behavior

Edits on the host are immediately visible inside the dev container via volume mount; `colcon build --symlink-install` symlinks Python and XML share files back to `src/`. Behavior tree XMLs are auto-discovered within ~2 s; the dashboard's tree list updates over the latched topic without a refresh. Python skill server code is live on save (restart the node only if it caches state at startup). C++ atoms require `colcon build --packages-select <pkg>` + node restart. Frontend changes need `vite build` + `colcon build --symlink-install --packages-select robot_dashboard` (or `pixi run dashboard-dev` for hot reload).
