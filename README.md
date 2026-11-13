# rr_nav2_bridge

A Robot Raconteur (RR) service that gives RR clients ROS 2 **Nav2** (Jazzy) path planning
and path following. It ships as one hardened, rootless Podman image. ROS 2 stays hidden
inside the container: only the RR TCP port is published.

**Scope: algorithm only.** Nav2 never talks to a robot. The client streams odometry,
laser scans (optionally point clouds) and a map in over RR, and reads plans and
`cmd_vel` out. The client decides whether to forward `cmd_vel` to its motors.

> **⚠️ AI-generated experiment: untested on real hardware. Do not use it to drive a real robot.**
>
> - The code, configuration, tests and docs were written by an AI coding agent
>   (Claude Code, Claude Opus 5.5) in one session in November 2026, working from a
>   human-written plan ([HANDOFF.md](HANDOFF.md), [DEVELOPMENT_PLAN.md](DEVELOPMENT_PLAN.md)).
>   No human has reviewed the code line by line.
> - "Tested" here means only the automated tests in `tests/`. They run against a
>   simulated robot ([`tests/fake_robot.py`](tests/fake_robot.py)): a simple unicycle
>   model with a perfect ray-cast laser on one small test map. Nothing has been tried
>   with real robots, real sensor noise, real odometry drift or real network conditions.
> - TLS has not been tested (it needs an RR-issued node certificate). The Quadlet unit
>   was validated only with Podman's dry-run check, not installed or run.
> - The bridge is not a safety-rated component. Its watchdog and the Nav2 collision
>   monitor are no substitute for a robot's own emergency stop and low-level limits.
> - Known deviations from the plan and open issues are listed in
>   [HANDOFF.md §9](HANDOFF.md#9-implementation-notes-deviations-and-findings-2026-11-13).

Status: experimental prototype; the automated smoke and hardening tests pass in simulation.

## Quick start

```bash
scripts/setup_venv.sh          # project-local host venv (hash-pinned robotraconteur)
scripts/build.sh               # build the image (digest-pinned Jazzy base)
scripts/test.sh                # run all tests against the hardened container
RR_NAV2_DETACH=1 scripts/run.sh
.venv/bin/python -I tests/fake_robot.py   # simulated robot drives to (8, 6)
podman stop rr-nav2-run
```

Clients connect to `rr+tcp://127.0.0.1:62354?service=nav2`. Start with
`documentation("")`: RR does not transmit robdef comments, so the service serves them.

## Documentation

| Document | Contents |
|---|---|
| [docs/usage.md](docs/usage.md) | Connecting, a minimal client, LLM notes, safety model, auth/TLS, operations |
| [robdef/experimental.nav2_bridge.robdef](robdef/experimental.nav2_bridge.robdef) | The interface, with every member documented |
| [docs/survey.md](docs/survey.md) | Standard RR types vs Jazzy `nav2_msgs`, and the mapping |
| [HANDOFF.md](HANDOFF.md) | Design decisions and architecture |
| [DEVELOPMENT_PLAN.md](DEVELOPMENT_PLAN.md) | Work plan and local-machine safety rules |

## Layout

```
robdef/       experimental.nav2_bridge.robdef (custom service; data uses com.robotraconteur.*)
src/          rr_nav2_bridge Python package (rclpy + RR, one process)
launch/       Nav2 launch file (started by the bridge on start())
config/       Nav2 default params, MPPI/DWB controller blocks, Fast DDS loopback profile
scripts/      all podman commands (single source of hardened flags: scripts/config.sh)
tests/        fake robot client, smoke tests, hardening tests, params unit test
deploy/       generated Quadlet unit
requirements/ hash-pinned pip requirements (host venv, container)
```

## Local-machine safety rules (summary)

- Write only inside this project directory (and the agent scratchpad). No `sudo`.
- Host Python packages only in the project-local `.venv/` (hash-pinned).
- Containers: rootless, `--cap-drop=ALL`, `--security-opt=no-new-privileges`,
  default seccomp, `--read-only` rootfs + tmpfs, unprivileged `nav` user; never
  `--privileged` / `--network=host` / `--pid=host` / `--ipc=host` / device passthrough.
- Limits on every build and run: `--memory=58g --memory-swap=58g --cpus=10
  --pids-limit=512` (total project budget; split it between concurrent containers).
- The RR port is published on loopback only (`127.0.0.1:62354`). Binding elsewhere
  requires `RR_NAV2_TLS=1` and `RR_NAV2_AUTH=1`.
- Everything is named `rr-nav2-*` and labeled `project=rr_nav2_bridge`; cleanup
  filters by that label. Never a bare `podman system prune`.
- All podman flags live in [`scripts/`](scripts/); no ad-hoc podman commands.
- Base image pinned by digest; pip pinned with `--require-hashes`.
