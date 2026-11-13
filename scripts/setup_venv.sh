#!/usr/bin/env bash
# Create the project-local host venv (.venv/, git-ignored) for RR clients/tests.
# Never installs anything outside the project directory.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENV="$ROOT/.venv"
if [[ ! -x "$VENV/bin/python" ]]; then
    python3 -m venv "$VENV"
fi
"$VENV/bin/python" -m pip install --quiet --disable-pip-version-check \
    --require-hashes --prefer-binary -r "$ROOT/requirements/host.txt"
"$VENV/bin/python" -I -c "from RobotRaconteur.Client import RRN; print(\"RobotRaconteur\", RRN.RobotRaconteurVersion)"
