#!/usr/bin/env bash
# Dev container: same image, src/ bind-mounted read-write, keep-id user mapping.
# Default command runs the bridge from the mounted source; pass a command to override.
set -euo pipefail
source "$(dirname "$0")/config.sh"

TTY=()
# Same as RO_SOURCE_MOUNTS, but src/ is read-write.
DEV_MOUNTS=("${RO_SOURCE_MOUNTS[@]/src:ro,Z/src:rw,Z}")
[[ -t 0 ]] && TTY=(-it)
[[ "${RR_NAV2_DETACH:-}" == "1" ]] && TTY=(-d)

exec podman run --rm "${TTY[@]}" --name "$DEV_NAME" \
    "${HARDENED_RUN_FLAGS[@]}" "${RR_NET_FLAGS[@]}" \
    --userns=keep-id \
    "${DEV_MOUNTS[@]}" \
    "$IMAGE" "$@"
