# meca500_ros2
ROS 2 Controllers for the Mecademic Meca500 robot.

# Requirements
* [pixi](https://pixi.sh)
* ROS 2 Jazzy (provisioned automatically by pixi via RoboStack — no system install required)

# Installation
```bash
git clone git@github.com:guyEIT/meca500_ros2.git
cd meca500_ros2
pixi install
```

`pixi install` resolves every `package.xml` dependency against RoboStack and builds the four ROS packages via [pixi-build-ros](https://prefix-dev.github.io/pixi-build-backends/backends/pixi-build-ros/).

# Run Controller

```bash
pixi run launch-moveit
```

Other tasks (`pixi task list` for the full set):

* `pixi run launch-calibration` — startup calibration launch
* `pixi run moveit-setup` — MoveIt setup assistant
* `pixi run rviz` — rviz2
* `pixi run -e tools optimize-mecatable --help` — mesh optimizer (isolated env, no ROS deps)
