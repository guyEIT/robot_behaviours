FROM osrf/ros:jazzy-desktop-full

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install core ROS2 packages and robot tooling
RUN apt-get update && apt-get install -y \
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

# behaviortree_ros2 is not released for Jazzy — build from source
RUN mkdir -p /opt/bt_ros2_ws/src \
    && git clone --branch 0.3.0 --depth 1 https://github.com/BehaviorTree/BehaviorTree.ROS2.git \
       /opt/bt_ros2_ws/src/behaviortree_ros2 \
    && bash -c "source /opt/ros/jazzy/setup.bash \
       && cd /opt/bt_ros2_ws \
       && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && echo "source /opt/bt_ros2_ws/install/setup.bash" >> /etc/bash.bashrc \
    && rm -rf /opt/bt_ros2_ws/build /opt/bt_ros2_ws/log

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
    && echo "source /opt/bt_ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/ros/.bashrc \
    && echo "source /home/ws/install/setup.bash 2>/dev/null || true" >> /home/ros/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ros/.bashrc \
    && echo "alias cb='cd /home/ws && colcon build --symlink-install'" >> /home/ros/.bashrc \
    && echo "alias cbt='cd /home/ws && colcon build --symlink-install --packages-select'" >> /home/ros/.bashrc \
    && echo "alias ct='cd /home/ws && colcon test'" >> /home/ros/.bashrc

# Initialize rosdep
RUN rosdep update --rosdistro jazzy

# ── Copy source and build at image build time ─────────────────────────────────
COPY --chown=ros:ros src/ /home/ws/src/

# ── Install Node.js 22 for dashboard build ───────────────────────────────────
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# ── Build web dashboard frontend ─────────────────────────────────────────────
RUN cd /home/ws/src/robot_dashboard/frontend \
    && npm ci --ignore-scripts \
    && npx vite build

WORKDIR /home/ws
USER ros

RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    source /opt/bt_ros2_ws/install/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

CMD ["bash", "-c", \
    "source /opt/ros/jazzy/setup.bash && \
     source /opt/bt_ros2_ws/install/setup.bash && \
     source /home/ws/install/setup.bash && \
     ros2 launch robot_skill_server skill_server.launch.py"]
