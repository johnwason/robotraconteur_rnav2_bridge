#!/usr/bin/env bash
# Build the all-in-one image with resource limits.
set -euo pipefail
source "$(dirname "$0")/config.sh"

build_scope podman build \
    --label "$PROJECT_LABEL" \
    --build-arg "BASE_IMAGE=$BASE_IMAGE" \
    --security-opt=no-new-privileges \
    "--memory=$LIMIT_MEMORY" "--memory-swap=$LIMIT_MEMORY" \
    --cpu-period=100000 "--cpu-quota=$(( LIMIT_CPUS * 100000 ))" \
    -t "$IMAGE" \
    -f "$PROJECT_ROOT/Containerfile" \
    "$PROJECT_ROOT" "$@"
