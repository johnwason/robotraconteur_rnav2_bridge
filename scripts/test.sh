#!/usr/bin/env bash
# Run the test suites against the hardened run container (baked image, read-only rootfs,
# all limits). Usage: scripts/test.sh [smoke|hardening|all] [extra args for the suite]
# Requires: scripts/build.sh and scripts/setup_venv.sh done.
set -euo pipefail
source "$(dirname "$0")/config.sh"
SUITE="${1:-all}"; shift || true
PY="$PROJECT_ROOT/.venv/bin/python"
export RR_NAV2_NAME="$TEST_NAME"
LOG_DIR="$PROJECT_ROOT/logs"; mkdir -p "$LOG_DIR"

cleanup() {
    podman logs "$TEST_NAME" > "$LOG_DIR/bridge_test.log" 2>&1 || true
    podman stop -t 10 "$TEST_NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT
podman rm -f "$TEST_NAME" >/dev/null 2>&1 || true
busy="$(podman ps --filter "label=$PROJECT_LABEL" --format '{{.Names}}')"
if [[ -n "$busy" ]]; then
    echo "Stop the running project container(s) first (they hold port $RR_PORT): $busy" >&2
    trap - EXIT; exit 2
fi
RR_NAV2_DETACH=1 "$(dirname "$0")/run.sh" >/dev/null

for _ in $(seq 60); do
    (exec 3<>"/dev/tcp/127.0.0.1/$RR_PORT") 2>/dev/null && break
    sleep 0.5
done

rc=0
if [[ "$SUITE" == smoke || "$SUITE" == all ]]; then
    "$PY" -I "$PROJECT_ROOT/tests/smoke_test.py" "$@" || rc=1
fi
if [[ "$SUITE" == hardening || "$SUITE" == all ]]; then
    "$PY" -I "$PROJECT_ROOT/tests/hardening_test.py" "$@" || rc=1
    "$(dirname "$0")/verify_isolation.sh" || rc=1
fi
echo "bridge log: $LOG_DIR/bridge_test.log"
exit $rc
