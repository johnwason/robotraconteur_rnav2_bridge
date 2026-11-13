#!/usr/bin/env bash
# Verify a running bridge container: inside-container sockets + host-side port mapping.
set -euo pipefail
source "$(dirname "$0")/config.sh"
NAME="${RR_NAV2_NAME:-$RUN_NAME}"
"$(dirname "$0")/exec.sh" /opt/rr_nav2/bin/entrypoint.sh /opt/rr_nav2/bin/verify_isolation.sh
echo "== host port mappings for $NAME"
mappings="$(podman port "$NAME")"
echo "$mappings" | sed 's/^/  /'
if echo "$mappings" | grep -v "^${RR_PORT}/tcp -> ${RR_BIND_ADDR}:${RR_PORT}\$" | grep -q .; then
    echo "FAIL: unexpected port mapping"; exit 1
fi
echo "HOST MAPPING OK"
