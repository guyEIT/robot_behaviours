FROM osrf/ros:jazzy-desktop-full

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install core ROS2 packages and robot tooling
# Upgrade first: the base image's FastCDR may be older than the message
# packages pulled from the ROS2 repo, causing ABI mismatches at runtime.
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    # Behavior Trees
    ros-jazzy-behaviortree-cpp \
    # MoveIt2
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-servo \
    # ROS2 Control
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-gripper-controllers \
    # Intel RealSense
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    # Perception
    ros-jazzy-pcl-ros \
    ros-jazzy-vision-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    # Simulation
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    # Build tools
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-rosdep \
    python3-pip \
    python3-pydantic \
    # Diagnostics & Monitoring
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-aggregator \
    # rosbridge (WebSocket bridge for web dashboard)
    ros-jazzy-rosbridge-suite \
    # Utilities
    git wget curl build-essential ccache \
    vim nano tmux tree \
    && rm -rf /var/lib/apt/lists/*


# Install Python packages for the orchestration layer
RUN pip3 install --no-cache-dir --break-system-packages \
    jsonschema \
    pyyaml \
    lxml \
    git+https://github.com/dheera/rosboard.git

# Groot2 - BehaviorTree.CPP visualization IDE (v1.9.0)
# AppImage needs FUSE or --appimage-extract-and-run; on Wayland/X11 may need --no-sandbox
RUN wget -q https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/Groot2-v1.9.0-x86_64.AppImage \
    -O /usr/local/bin/groot2 \
    && chmod +x /usr/local/bin/groot2 \
    || echo "WARNING: Groot2 download failed - install manually from https://www.behaviortree.dev/groot/"

# FUSE is needed for AppImage execution (or use --appimage-extract-and-run)
RUN apt-get update && apt-get install -y --no-install-recommends \
    fuse libfuse2t64 \
    && rm -rf /var/lib/apt/lists/*

# ── Patch rosbridge QoS: default to RELIABLE so /tf works ────────────────────
RUN find /opt/ros/jazzy -path '*/rosbridge_library/internal/subscribers.py' \
      -exec sed -i 's/reliability=ReliabilityPolicy.BEST_EFFORT/reliability=ReliabilityPolicy.RELIABLE/' {} \; \
    && find /opt/ros/jazzy -path '*/rosbridge_library/capabilities/send_action_goal.py' \
      -exec sed -i '/client_handler.run()/a\\n        import time; time.sleep(0.1)' {} \;

# Setup ccache for faster rebuilds
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_DIR=/home/ros/.ccache

# Create non-root user with sudo access
RUN useradd -m -s /bin/bash ros \
    && apt-get update && apt-get install -y sudo \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# Workspace setup
RUN mkdir -p /home/ws/src && chown -R ros:ros /home/ws

# Shell setup for ros user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc \
    && echo "source /home/ws/install/setup.bash 2>/dev/null || true" >> /home/ros/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ros/.bashrc \
    && echo "alias cb='cd /home/ws && colcon build --symlink-install'" >> /home/ros/.bashrc \
    && echo "alias cbt='cd /home/ws && colcon build --symlink-install --packages-select'" >> /home/ros/.bashrc \
    && echo "alias ct='cd /home/ws && colcon test'" >> /home/ros/.bashrc

# ── Install Node.js 22 for dashboard build ───────────────────────────────────
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update --rosdistro jazzy

# ═══════════════════════════════════════════════════════════════════════════════
# Build layers — npm dependency install is the slow step (~30s) and only
# changes when package.json does. Everything else rebuilds on source changes
# but is fast (~30s total for vite + colcon).
# ═══════════════════════════════════════════════════════════════════════════════

# ── Cached layer: npm dependencies (changes when package.json changes) ───────
COPY src/robot_dashboard/frontend/package.json src/robot_dashboard/frontend/package-lock.json \
     /home/ws/src/robot_dashboard/frontend/
RUN cd /home/ws/src/robot_dashboard/frontend && npm ci --ignore-scripts

# ── Source + builds (invalidated on any src/ change) ─────────────────────────
COPY --chown=ros:ros src/ /home/ws/src/

RUN cd /home/ws/src/robot_dashboard/frontend && npx vite build

WORKDIR /home/ws
USER ros

RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install \
      --packages-ignore robot_bt_nodes \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

CMD ["bash", "-c", \
    "source /opt/ros/jazzy/setup.bash && \
     source /home/ws/install/setup.bash && \
     ros2 launch robot_skill_server skill_server.launch.py"]
