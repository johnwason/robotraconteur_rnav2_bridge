#!/usr/bin/env bash
# Dev container: same image, src/ bind-mounted read-write, keep-id user mapping.
# Default command runs the bridge from the mounted source; pass a command to override.
set -euo pipefail
source "$(dirname "$0")/config.sh"

TTY=()
[[ -t 0 ]] && TTY=(-it)

exec podman run --rm "${TTY[@]}" --name "$DEV_NAME" \
    "${HARDENED_RUN_FLAGS[@]}" \
    --userns=keep-id \
    -v "$PROJECT_ROOT/src:/opt/rr_nav2/src:rw,Z" \
    -v "$PROJECT_ROOT/robdef:/opt/rr_nav2/robdef:ro,Z" \
    -v "$PROJECT_ROOT/config:/opt/rr_nav2/config:ro,Z" \
    "$IMAGE" "$@"
