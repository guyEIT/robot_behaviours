#!/usr/bin/env bash
# Bootstrap shared packages (robot_skills_msgs + robot_skill_advertise) into
# ~/channel.
#
# Run this ONCE per machine after cloning, and any time you edit .msg/.srv/.action
# files in lib/robot_skills_msgs/ or change the SkillAdvertiser helper in
# lib/robot_skill_advertise/. It bypasses `pixi run` so the lite-native /
# real-native envs don't have to be solvable yet (they need msgs and the
# advertise lib to exist in ~/channel, which is what this script creates).
#
# After bootstrap, `pixi run lite-native-up` / `pixi run real-native-up` work.
set -euo pipefail

cd "$(dirname "$0")/.."

CHANNEL="$HOME/channel"
mkdir -p "$CHANNEL/linux-64" "$CHANNEL/noarch"

# Create empty repodata so pixi can read the channel before msgs is built.
pixi exec rattler-index fs "$CHANNEL" --force

# Build msgs directly (no `pixi run` wrapper, so no env resolution).
env -u PIXI_PROJECT_MANIFEST pixi build \
  --target-platform linux-64 \
  --path lib/robot_skills_msgs \
  --output-dir "$CHANNEL/linux-64" \
  --build-dir "$PWD/.pixi/build/robot_skills_msgs" \
  --no-install

# Re-index so pixi can resolve ros-jazzy-robot-skills-msgs before we build
# the advertise package (which depends on it).
pixi exec rattler-index fs "$CHANNEL" --force

# robot_skill_advertise is consumed by every provider proxy + the
# orchestrator's skill_discovery + skill_base.hpp. Building it into the
# channel mirrors the msgs pattern so providers don't need deeply-nested
# path-deps in their per-pkg pixi.toml.
env -u PIXI_PROJECT_MANIFEST pixi build \
  --target-platform linux-64 \
  --path lib/robot_skill_advertise \
  --output-dir "$CHANNEL/linux-64" \
  --build-dir "$PWD/.pixi/build/robot_skill_advertise" \
  --no-install

# Re-index again so pixi can resolve ros-jazzy-robot-skill-advertise.
pixi exec rattler-index fs "$CHANNEL" --force

echo
echo "✔ robot_skills_msgs and robot_skill_advertise are now available at $CHANNEL"
echo "  Run \`pixi run lite-native-up\` or \`pixi run real-native-up\` to use it."
