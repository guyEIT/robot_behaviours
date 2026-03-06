#!/usr/bin/env bash
# start_sim.sh — One-command simulation startup for the Robot Skills Framework
#
# Starts the MoveIt2 simulation container and waits until move_group is ready.
# Run this BEFORE starting the dev container skill_server.
#
# Usage:
#   ./start_sim.sh           # Start sim with RViz2
#   ./start_sim.sh --no-rviz # Start sim without RViz2 (headless)
#   ./start_sim.sh --rebuild # Force rebuild of sim image
#   ./start_sim.sh --stop    # Stop the sim container
#   ./start_sim.sh --shell   # Open a shell in the running sim container

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/.devcontainer/docker-compose.sim.yml"
CONTAINER_NAME="ros2_robot_sim"

# Parse arguments
USE_RVIZ=true
REBUILD=false
ACTION="start"

for arg in "$@"; do
  case $arg in
    --no-rviz) USE_RVIZ=false ;;
    --rebuild) REBUILD=true ;;
    --stop) ACTION="stop" ;;
    --shell) ACTION="shell" ;;
    --help|-h)
      echo "Usage: $0 [--no-rviz] [--rebuild] [--stop] [--shell]"
      exit 0
      ;;
  esac
done

# ── macOS display setup ───────────────────────────────────────────────────────
setup_display_macos() {
  if [[ "$(uname)" == "Darwin" ]]; then
    # Check if XQuartz is running
    if ! pgrep -x "Xquartz" > /dev/null 2>&1; then
      echo "⚠️  XQuartz not running. Starting XQuartz for RViz2 display..."
      open -a XQuartz || true
      sleep 2
    fi
    # Allow local connections
    xhost +localhost > /dev/null 2>&1 || true
    # Set DISPLAY if not set
    if [[ -z "${DISPLAY:-}" ]]; then
      export DISPLAY="host.docker.internal:0"
      echo "ℹ️  Set DISPLAY=${DISPLAY}"
    fi
  fi
}

# ── Actions ───────────────────────────────────────────────────────────────────

case $ACTION in
  stop)
    echo "Stopping sim container..."
    docker compose -f "${COMPOSE_FILE}" down
    echo "✓ Sim container stopped"
    exit 0
    ;;

  shell)
    echo "Opening shell in sim container..."
    docker exec -it "${CONTAINER_NAME}" bash
    exit 0
    ;;

  start)
    echo ""
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║     Robot Skills Framework - Starting Simulation Server      ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo ""

    # Setup display for macOS
    [[ "${USE_RVIZ}" == "true" ]] && setup_display_macos

    # Rebuild if requested
    if [[ "${REBUILD}" == "true" ]]; then
      echo "▶ Rebuilding sim image (--rebuild requested)..."
      docker compose -f "${COMPOSE_FILE}" build --no-cache sim
    fi

    # Inject use_rviz into the container's launch command
    if [[ "${USE_RVIZ}" == "false" ]]; then
      echo "ℹ️  Starting WITHOUT RViz2 (headless mode)"
      # Override the command to pass use_rviz:=false
      export SIM_LAUNCH_ARGS="use_rviz:=false"
    else
      echo "ℹ️  Starting WITH RViz2 (requires XQuartz on macOS)"
      export SIM_LAUNCH_ARGS="use_rviz:=true"
    fi

    # Start in detached mode
    echo "▶ Starting sim container..."
    docker compose -f "${COMPOSE_FILE}" up -d --build sim

    echo ""
    echo "▶ Waiting for simulation to be ready..."
    echo "  (This takes ~30-60s for MoveIt2 to initialize)"
    echo ""

    # Wait for move_group to appear
    MAX_WAIT=120
    WAITED=0
    while [[ $WAITED -lt $MAX_WAIT ]]; do
      if docker exec "${CONTAINER_NAME}" bash -c \
        "source /opt/ros/jazzy/setup.bash && \
         source /home/ws/install/setup.bash 2>/dev/null || true && \
         ros2 node list 2>/dev/null | grep -q move_group" 2>/dev/null; then
        echo ""
        echo "╔══════════════════════════════════════════════════════════════╗"
        echo "║  ✓ Simulation READY!                                         ║"
        echo "╠══════════════════════════════════════════════════════════════╣"
        echo "║                                                              ║"
        echo "║  Now start the skill server in the dev container:           ║"
        echo "║    ros2 launch robot_skill_server skill_server.launch.py    ║"
        echo "║                                                              ║"
        echo "║  Or verify the sim:                                          ║"
        echo "║    ros2 node list                                            ║"
        echo "║    ros2 action list                                          ║"
        echo "║    ros2 service call /skill_server/get_skill_descriptions \\ ║"
        echo "║      robot_skills_msgs/srv/GetSkillDescriptions '{}'        ║"
        echo "║                                                              ║"
        echo "║  View sim logs:                                              ║"
        echo "║    docker logs -f ${CONTAINER_NAME}                         ║"
        echo "║                                                              ║"
        echo "║  Open shell in sim:                                          ║"
        echo "║    ./start_sim.sh --shell                                    ║"
        echo "╚══════════════════════════════════════════════════════════════╝"
        exit 0
      fi

      printf "."
      sleep 3
      WAITED=$((WAITED + 3))
    done

    echo ""
    echo "⚠️  Timed out waiting for move_group. Check logs:"
    echo "   docker logs ${CONTAINER_NAME}"
    exit 1
    ;;
esac
