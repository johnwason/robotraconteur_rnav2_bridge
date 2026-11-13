#!/usr/bin/env bash
# Remove ONLY containers/images labeled project=rr_nav2_bridge.
# Without --yes it just lists what would be removed.
set -euo pipefail
source "$(dirname "$0")/config.sh"

echo "Containers with label $PROJECT_LABEL:"
podman ps -a --filter "label=$PROJECT_LABEL" --format '  {{.ID}} {{.Names}} {{.Status}}'
echo "Images with label $PROJECT_LABEL:"
podman images --filter "label=$PROJECT_LABEL" --format '  {{.ID}} {{.Repository}}:{{.Tag}} {{.Size}}'

if [[ "${1:-}" != "--yes" ]]; then
    echo "(dry run; pass --yes to remove the above)"
    exit 0
fi
podman ps -a -q --filter "label=$PROJECT_LABEL" | xargs -r podman rm -f
podman images -q --filter "label=$PROJECT_LABEL" | sort -u | xargs -r podman rmi -f
# Dangling build layers of our image only.
podman image prune -f --filter "label=$PROJECT_LABEL"
