# All-in-one image: ROS 2 Jazzy + Nav2 + Robot Raconteur bridge.
# Build only via scripts/build.sh (passes the digest-pinned BASE_IMAGE).
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Official Ubuntu + ROS apt repos only (already configured and key-verified in the base image).
# Explicit Nav2 packages instead of the navigation2 metapackage, which pulls in rviz/GL.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-amcl ros-jazzy-nav2-behavior-tree ros-jazzy-nav2-behaviors \
      ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-controller \
      ros-jazzy-nav2-core ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-dwb-controller \
      ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-msgs ros-jazzy-nav2-navfn-planner \
      ros-jazzy-nav2-planner ros-jazzy-nav2-smoother ros-jazzy-nav2-velocity-smoother \
      ros-jazzy-nav2-waypoint-follower ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-util \
      ros-jazzy-nav2-rotation-shim-controller \
      ros-jazzy-tf2-ros-py \
      python3-venv python3-numpy python3-yaml python3-lxml \
      iproute2 \
 && rm -rf /var/lib/apt/lists/*

# Unprivileged runtime user. Noble ships 'ubuntu' at uid 1000; replace it so
# --userns=keep-id maps the host developer (uid 1000) onto 'nav'.
RUN (userdel -r ubuntu 2>/dev/null || true) \
 && useradd --uid 1000 --create-home --shell /bin/bash nav \
 && mkdir -p /home/nav/.ros && chown nav:nav /home/nav/.ros

# Python deps: venv over system site-packages (rclpy, numpy), hash-pinned, no deps resolution.
COPY requirements/container.txt /tmp/requirements.txt
RUN python3 -m venv --system-site-packages /opt/rr_nav2/venv \
 && /opt/rr_nav2/venv/bin/pip install --no-cache-dir --disable-pip-version-check \
      --require-hashes --no-deps -r /tmp/requirements.txt \
 && rm /tmp/requirements.txt

COPY --chown=root:root robdef/ /opt/rr_nav2/robdef/
COPY --chown=root:root config/ /opt/rr_nav2/config/
COPY --chown=root:root launch/ /opt/rr_nav2/launch/
COPY --chown=root:root src/ /opt/rr_nav2/src/
COPY --chown=root:root --chmod=0755 scripts/container/ /opt/rr_nav2/bin/

# ROS isolation: discovery never leaves the container, fixed domain.
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_LOG_DIR=/home/nav/.ros/log \
    PYTHONPATH=/opt/rr_nav2/src \
    PYTHONDONTWRITEBYTECODE=1 \
    RR_NAV2_PORT=62354

USER nav
WORKDIR /home/nav
ENTRYPOINT ["/opt/rr_nav2/bin/entrypoint.sh"]
CMD ["bridge"]
