#!/usr/bin/env bash
# Container entrypoint. Sources ROS, then:
#   bridge        -> run the RR bridge (it launches/activates Nav2 on start()) (default)
#   <anything>    -> exec it inside the ROS environment
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PATH="/opt/rr_nav2/venv/bin:$PATH"
if [[ "${1:-bridge}" == "bridge" ]]; then
    shift || true
    exec python3 -m rr_nav2_bridge "$@"
fi
exec "$@"
