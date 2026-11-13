#!/usr/bin/env bash
# Run the bridge (source baked into the image, read-only rootfs).
# Extra args are appended to the container command.
set -euo pipefail
source "$(dirname "$0")/config.sh"

NAME="${RR_NAV2_NAME:-$RUN_NAME}"
DETACH=()
[[ "${RR_NAV2_DETACH:-}" == "1" ]] && DETACH=(-d)

exec podman run --rm "${DETACH[@]}" --name "$NAME" \
    "${HARDENED_RUN_FLAGS[@]}" \
    "$IMAGE" "$@"
