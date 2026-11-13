# shellcheck shell=bash
# Single source of truth for names, labels, limits and hardened podman flags.
# Sourced by the other scripts; do not run directly.

PROJECT_LABEL="project=rr_nav2_bridge"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Base image: fully qualified + pinned digest (resolved 2026-11-13).
BASE_IMAGE="docker.io/library/ros:jazzy-ros-base@sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698"

IMAGE="localhost/rr-nav2-bridge:dev"
RUN_NAME="rr-nav2-run"
DEV_NAME="rr-nav2-dev"
TEST_NAME="rr-nav2-test"

RR_PORT="${RR_NAV2_PORT:-62354}"
# Loopback only in dev. Binding elsewhere requires TLS + auth (see docs).
RR_BIND_ADDR="${RR_NAV2_BIND_ADDR:-127.0.0.1}"

# User-approved ceiling for the whole project: 58 GB RAM, 10 CPUs.
# If two containers run at once, pass a smaller share via these variables.
LIMIT_MEMORY="${RR_NAV2_MEMORY:-58g}"
LIMIT_CPUS="${RR_NAV2_CPUS:-10}"
LIMIT_PIDS="${RR_NAV2_PIDS:-512}"

# Optional local overrides (git-ignored).
if [[ -f "$PROJECT_ROOT/scripts/local.env" ]]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/scripts/local.env"
fi

if [[ "$RR_BIND_ADDR" != "127.0.0.1" && "${RR_NAV2_ALLOW_NONLOCAL:-}" != "yes-with-tls" ]]; then
    echo "Refusing to bind RR port to $RR_BIND_ADDR without RR_NAV2_ALLOW_NONLOCAL=yes-with-tls" >&2
    exit 1
fi

# Flags applied to every container run.
HARDENED_RUN_FLAGS=(
    --label "$PROJECT_LABEL"
    --init
    --cap-drop=ALL
    --security-opt=no-new-privileges
    --read-only
    --tmpfs /tmp:rw,nosuid,nodev,size=1g
    --tmpfs /home/nav/.ros:rw,nosuid,nodev,size=512m,mode=0700,U
    --tmpfs /run:rw,nosuid,nodev,size=64m
    --tmpfs /home/nav/.local:rw,nosuid,nodev,size=64m,mode=0700,U
    --shm-size=512m
    "--memory=$LIMIT_MEMORY"
    "--memory-swap=$LIMIT_MEMORY"
    "--cpus=$LIMIT_CPUS"
    "--pids-limit=$LIMIT_PIDS"
)

# Network flags for containers that serve RR: only the RR port, loopback by default.
RR_NET_FLAGS=(
    --network=bridge
    -p "$RR_BIND_ADDR:$RR_PORT:$RR_PORT"
    -e "RR_NAV2_PORT=$RR_PORT"
    -e "RR_NAV2_ROS_LOG_LEVEL=${RR_NAV2_ROS_LOG_LEVEL:-warn}"
)

# Read-only mounts of the project's runtime dirs (overlay the baked-in copies).
RO_SOURCE_MOUNTS=(
    -v "$PROJECT_ROOT/src:/opt/rr_nav2/src:ro,Z"
    -v "$PROJECT_ROOT/robdef:/opt/rr_nav2/robdef:ro,Z"
    -v "$PROJECT_ROOT/config:/opt/rr_nav2/config:ro,Z"
    -v "$PROJECT_ROOT/launch:/opt/rr_nav2/launch:ro,Z"
    -v "$PROJECT_ROOT/scripts/container:/opt/rr_nav2/bin:ro,Z"
)

# podman build has no --pids-limit; wrap it in a systemd user scope so the
# build gets the same memory / CPU / task ceiling.
build_scope() {
    local cpu_pct=$(( LIMIT_CPUS * 100 ))
    if command -v systemd-run >/dev/null 2>&1; then
        systemd-run --user --scope -q \
            -p "TasksMax=$LIMIT_PIDS" -p "MemoryMax=${LIMIT_MEMORY^^}" \
            -p MemorySwapMax=0 -p "CPUQuota=${cpu_pct}%" "$@"
    else
        echo "warning: systemd-run unavailable; build limited by podman flags only" >&2
        "$@"
    fi
}
