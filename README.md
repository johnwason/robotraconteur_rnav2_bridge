# rr_nav2_bridge

A Robot Raconteur (RR) service that exposes ROS 2 **Nav2** (Jazzy) to RR clients.
Ships as a single, hardened, rootless Podman image; ROS 2 is completely hidden
inside the container and only the RR TCP port is published.

Status: prototype (branch `dev/prototype`). See [HANDOFF.md](HANDOFF.md) for the
design and [DEVELOPMENT_PLAN.md](DEVELOPMENT_PLAN.md) for the work plan.

## Scope

Algorithm only. Nav2 never talks to a real robot: the client streams odometry,
laser scans and a map in over RR and receives plans and `cmd_vel` out.

## Local-machine safety rules (summary)

- Write only inside this project directory (and the agent scratchpad). No `sudo`.
- Host Python packages only in the project-local `.venv/` (hash-pinned).
- Containers: rootless, `--cap-drop=ALL`, `--security-opt=no-new-privileges`,
  default seccomp, `--read-only` rootfs + tmpfs, never `--privileged` /
  `--network=host` / `--pid=host` / `--ipc=host` / device passthrough.
- Limits on every build and run: `--memory=58g --memory-swap=58g --cpus=10
  --pids-limit=512` (total project budget; split it between concurrent containers).
- Dev publishes the RR port on loopback only (`127.0.0.1:62354`).
- Everything is named `rr-nav2-*` and labeled `project=rr_nav2_bridge`; cleanup
  filters by that label. Never a bare `podman system prune`.
- All podman flags live in [`scripts/`](scripts/); no ad-hoc podman commands.
- Base image pinned by digest; pip pinned with `--require-hashes`.
