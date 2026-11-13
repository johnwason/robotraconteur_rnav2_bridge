#!/usr/bin/env bash
# Run a one-off command in a throwaway hardened container with NO network and the
# project's runtime dirs mounted read-only (checks, validation, interface surveys).
set -euo pipefail
source "$(dirname "$0")/config.sh"
exec podman run --rm --name "rr-nav2-oneshot-$$" \
    "${HARDENED_RUN_FLAGS[@]}" --network=none \
    "${RO_SOURCE_MOUNTS[@]}" \
    "$IMAGE" "$@"
