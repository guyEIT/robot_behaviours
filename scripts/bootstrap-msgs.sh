#!/usr/bin/env bash
# Bootstrap robot_skills_msgs into ~/channel.
#
# Run this ONCE per machine after cloning, and any time you edit .msg/.srv/.action
# files in src/robot_skills_msgs/. It bypasses `pixi run` so the lite-native /
# real-native envs don't have to be solvable yet (they need msgs to exist in
# ~/channel, which is what this script creates).
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
  --path src/robot_skills_msgs \
  --output-dir "$CHANNEL/linux-64" \
  --build-dir "$PWD/.pixi/build/robot_skills_msgs" \
  --no-install

# Re-index so pixi can resolve ros-jazzy-robot-skills-msgs.
pixi exec rattler-index fs "$CHANNEL" --force

echo
echo "✔ robot_skills_msgs is now available at $CHANNEL"
echo "  Run \`pixi run lite-native-up\` or \`pixi run real-native-up\` to use it."
