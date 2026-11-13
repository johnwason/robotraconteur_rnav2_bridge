#!/usr/bin/env bash
# Run a command inside a running project container (default: rr-nav2-run).
set -euo pipefail
source "$(dirname "$0")/config.sh"
NAME="${RR_NAV2_NAME:-$RUN_NAME}"
# Only allow exec into our own labeled containers.
if ! podman ps --filter "label=$PROJECT_LABEL" --filter "name=^${NAME}\$" --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "No running project container named $NAME" >&2; exit 1
fi
exec podman exec "$NAME" "$@"
