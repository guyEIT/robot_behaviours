# FR3 provider — upstream provenance

Cherry-picked from upstream Franka Robotics repos. We deliberately did **not**
`git subtree add` the whole repos: `franka_ros2` ships ~10 packages and we
only need 6, and pulling the whole tree would carry Gazebo / mobile-base /
multi-arm code we don't run here.

Trade-off: `git subtree pull` is unavailable. To re-sync with upstream, clone
the source repos at the SHAs (or newer) listed below and re-copy the listed
paths. Diff against the local divergences flagged in **Local edits** before
overwriting.

## Sources

| Source                                                | SHA                                          | Paths copied                                                                                              | Local destination                                              |
|-------------------------------------------------------|----------------------------------------------|-----------------------------------------------------------------------------------------------------------|----------------------------------------------------------------|
| `github.com/frankarobotics/franka_description` `main` | `72baf5bf4e88eaec27f0eb61be1b20a001abf2ab`   | trimmed: only `robots/{fr3,common}/`, `end_effectors/{franka_hand,common}/`, `meshes/robots/fr3/`, `meshes/robot_ee/franka_hand_*/`, `launch/`, `rviz/`, `scripts/`. Other robot models (fer, fp3, fr3v2*, tmrv0_2, mobile_fr3_duo*, fr3_duo) and accessories meshes were dropped to keep the package under 30 MB. | `providers/fr3/src/franka_description/`                        |
| `github.com/frankarobotics/franka_ros2` `jazzy`       | `79e55bfaf1702b0c73d469bb45f24162859e6580`   | `franka_msgs/`, `franka_hardware/`, `franka_semantic_components/`, `franka_robot_state_broadcaster/`, `franka_gripper/`, `franka_fr3_moveit_config/` (whole packages, no edits)                                | `providers/fr3/src/{franka_msgs,franka_hardware,franka_semantic_components,franka_robot_state_broadcaster,franka_gripper,franka_fr3_moveit_config}/` |
| `libfranka`                                           | conda-forge `libfranka 0.21.1` (`hf20a59c_1`) | NONE — pulled in as a binary conda dep (`libfranka >=0.21.1` in `feature.fr3-host` / `feature.lab-sim`).  | (env-prefix only, never on disk)                               |

## Local edits

Edits we made on top of the cherry-picked sources:

- **Per-package `pixi.toml`** added to each cherry-picked package so they
  build via `pixi-build-ros` with the right input globs (`urdf/**`,
  `meshes/**`, `config/**`, `launch/**`, etc.). These are pure additions —
  they don't conflict with `git subtree pull` since upstream doesn't ship
  pixi metadata.
- **No xacro / URDF / hardware-plugin edits.** Upstream's
  `robots/common/franka_arm.ros2_control.xacro` already swaps between
  `mock_components/GenericSystem` and `franka_hardware/FrankaHardwareInterface`
  via the `use_fake_hardware` xacro arg — no equivalent of the Meca500
  `<xacro:if simulation>` patch is needed.

## Packages we add on top (not from upstream)

These are local-only, won't ever come from upstream:

- `providers/fr3/src/fr3_bringup/` — wrapper launch translating
  `simulation:=true/false` to `use_fake_hardware:=true/false`, plus a
  sim-specific `fr3_ros_controllers_sim.yaml` that uses position command
  interfaces (mock_components doesn't simulate effort dynamics).
- `providers/fr3/src/fr3_skill_server/` — proxy that composes 12
  `robot_arm_skills` atoms under `/fr3` plus the FrankaGripperSkill.
- `providers/fr3/src/franka_gripper_skill/` — bridges
  `robot_skills_msgs/FrankaGripperControl` onto `franka_gripper`'s
  `Move`/`Grasp`/`Homing` actions; STOP cancels the in-flight upstream goal.

## Refresh procedure

```bash
# 1. Clone upstream at desired SHAs into a scratch dir.
git clone https://github.com/frankarobotics/franka_description /tmp/fr3_upstream/desc
git clone --branch jazzy https://github.com/frankarobotics/franka_ros2 /tmp/fr3_upstream/ros2

# 2. Diff against local. Anything outside of pixi.toml, fr3_bringup/,
#    fr3_skill_server/, franka_gripper_skill/ should match upstream.
diff -ruN /tmp/fr3_upstream/desc providers/fr3/src/franka_description \
  --exclude=.git --exclude=pixi.toml | less
# (likewise for franka_ros2 packages)

# 3. Re-copy paths listed in the table above; preserve our pixi.toml files.
# 4. Update the SHAs in this file and in the commit message.
```

If upstream changes `franka_arm.ros2_control.xacro` (the sim/real switch),
re-validate `pixi run fr3-sim-test` and `pixi run fr3-moveit-run` after the
refresh — those are the load-bearing assumptions.
