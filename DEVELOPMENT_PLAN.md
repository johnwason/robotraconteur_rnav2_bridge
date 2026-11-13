# Development Plan: RR ↔ Nav2 Bridge (with local-machine safety)

Companion to [HANDOFF.md](HANDOFF.md), which records the design decisions and architecture. This file covers **how** the work gets done safely on the development machine.

- **Target:** ROS 2 Jazzy (LTS) · Python prototype · rootless Podman · small commits
- **Status:** not started. No code yet; the directory is not yet a git repo.

---

## 1. Host machine (re-checked 2026-11-13 after moving to a new machine)

| Item | Result | What it means |
|---|---|---|
| Machine | VMware VM, Ubuntu 24.04.5, kernel 7.0 | Replaces the earlier 5-core / 8 GB machine |
| Podman | 4.9.3, **rootless**, cgroup v2 | Containers run as the user, not root; cpu/memory/pids limits are delegated, so they work rootless |
| SELinux / AppArmor | no SELinux; AppArmor **on in the kernel, but rootless Podman applies no profile** | Containers still get no MAC confinement, so container hardening matters more (seccomp is on) |
| CPU / RAM | **12 vCPUs** (Ryzen 9 9950X3D host), **62 GB total, ~53 GB available** | Memory is no longer tight; hard limits stay to contain a runaway Nav2 or build |
| Disk | ~133 GB free; store at `~/.local/share/containers` | Plenty of space; still prune only what we create |
| Python | 3.12.3 | Matches Jazzy's Ubuntu 24.04, so the host client venv lines up |

## 2. Safety rules (apply to every step)

### Filesystem
- Write only inside `/home/wasonj/scratch/rr_rosnav_bridge/` and the agent's session scratchpad. Never `/etc`, `/usr`, other home directories, or system Python.
- **No `sudo`, ever.** If something seems to need root, stop and ask.
- Bind-mount only project subdirectories, mostly **read-only**: source is read-only in the run container and read-write only in the dev container. Never mount `$HOME`, `/`, the Podman socket, `/dev`, or credential directories (`~/.ssh`, `~/.gnupg`, …).
- Host Python packages go only in a project-local `.venv/` (git-ignored). No `pip install --user` and no global pip.

### Containers (rootless Podman, hardened by default)
- `--userns=keep-id` for the dev container, so files created in the mounted source belong to the user, not a subuid.
- In the image, processes run as an unprivileged `nav` user (`USER` in the Containerfile).
- Always `--cap-drop=ALL` and `--security-opt=no-new-privileges`; keep the default seccomp profile.
- **Never** `--privileged`, `--network=host`, `--pid=host`, `--ipc=host`, or device passthrough.
- `--read-only` root filesystem, with `--tmpfs` for `/tmp`, ROS logs, and `~/.ros`.
- Resource limits on every run **and** build: `--memory=58g --memory-swap=58g --cpus=10 --pids-limit=512`. This is the user-approved ceiling (58 GB RAM, 10 cores), and it leaves ~4 GB and 2 vCPUs for the desktop while containing a runaway Nav2 or build. The ceiling covers everything the project runs at once, so if two containers run at the same time (e.g. dev + test), split the budget between them.
- **Dev: publish the RR port on loopback only** (`-p 127.0.0.1:<port>:<port>`), so nothing on the LAN can reach an unauthenticated bridge. Binding to all interfaces is an explicit, documented production option that requires TLS + authentication.
- Name and label everything (`rr-nav2-*`, `label=project=rr_nav2_bridge`) so cleanup never affects other Podman containers. Prune by label filter only. **Never run a bare `podman system prune`.**
- All hardened flags live in `scripts/` as the single source of truth. No ad-hoc podman commands.

### Network and supply chain
- Base image by fully qualified name + pinned digest: `docker.io/library/ros:jazzy-ros-base@sha256:…`, with the digest recorded in a commit.
- apt packages only from the official Ubuntu and ROS repos (ROS key verified, as in the official image).
- pip packages pinned to exact versions with `--require-hashes`, in both the image and the host venv.
- Downloads go into an empty scratch directory and are treated as untrusted. Nothing is executed from a download directory.
- No telemetry, no pushes to git remotes, and no image pushes to registries unless the user asks.

### Git
- `git init`; small commits on a working branch `dev/prototype`; leave `main` alone until the user approves a merge.
- Commit messages end with the attribution trailer.
- `.gitignore`: `.venv/`, logs, `__pycache__`, generated certs, local overrides.
- No secrets in the repo. Dev TLS certs are generated into a git-ignored directory.
- No history rewriting or force operations.

### Process
- Before any destructive or hard-to-undo operation (removing images or volumes, changes outside the project), show what will be affected and ask first.
- Long builds and runs go in the background with logs in the scratchpad, so a hung process can be stopped cleanly.
- If tests leave containers behind, a labeled cleanup script removes only those.

## 3. Phases and commits (one line ≈ one commit)

### Phase 0: Safe scaffolding
1. `git init`, `dev/prototype` branch, `.gitignore`, README stub with these safety rules
2. `scripts/` stubs with the hardened `podman build` / `run` / `clean` flags centralized
3. Host venv setup script (project-local, hash-pinned `robotraconteur`)

### Phase 1: Interface design
4. Survey notes: standard `com.robotraconteur.*` types vs Jazzy `nav2_msgs`
5. Robdef skeleton: root object, input members, `##` docstrings
6. Robdef: outputs, generators, sub-objects, configuration; full `##` LLM-oriented docs
7. Robdef validation: parse it with the RR library inside the container

### Phase 2: Image
8. Containerfile: digest-pinned Jazzy base, Nav2, hash-pinned robotraconteur, non-root `nav` user
9. Launch file + default Nav2 params (autostart off, `enable_stamped_cmd_vel: true`, no map_server)
10. Isolation config: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, fixed `ROS_DOMAIN_ID`, read-only rootfs + tmpfs layout
11. Verification script: inside the running container, confirm DDS stays on localhost and only the RR port is listening (`ss -lntu`)

### Phase 3: Bridge (Python)
12. Scaffolding: rclpy executor thread, RR node, loopback-only port in dev
13. Odometry wire → `/odom` + TF
14. Map input → `/map`; laser scan pipe → `/scan`
15. External-pose mode → TF `map→odom`; AMCL mode
16. Lifecycle `start` / `pause` / `reset` with precondition checks
17. `compute_path_to_pose` / `compute_path_through_poses`
18. Output wires: `cmd_vel` (stamped), plans, robot pose, status
19. `navigate_to_pose` generator (feedback, result, cancel)
20. Remaining generators (through-poses, waypoints, follow_path, behaviors)
21. Costmap sub-objects (memory) + `clear_costmaps`
22. Configuration object → ROS parameters
23. **Watchdog:** stale inputs abort goals, zero `cmd_vel`, and set a reason in status. This protects whatever the client is driving.
24. **Input validation:** reject NaN/Inf, oversized maps and scans, absurd velocity limits. This protects the container from bad clients and caps memory use.

### Phase 4: Testing (no ROS on the host)
25. Fake-robot RR client in the host venv: unicycle model, ray-cast laser scans, test map
26. Smoke tests: planning only, full navigation loop, cancel, watchdog
27. Hardening tests: the container runs with all limits; the bridge survives malformed input; nothing is reachable except `127.0.0.1:<port>`

### Phase 5: Production options and docs
28. Optional TLS + RR user authentication. Certs come from a mounted, git-ignored directory; the docs state that non-loopback binding requires this.
29. Optional Quadlet unit with the same hardening flags, for running on a robot or server
30. Docs: client URL usage (`rr+tcp://host:port?service=nav2`), LLM usage notes, safety model, cleanup

## 4. Dev workflow at a glance

- **Dev container:** the same image with the source bind-mounted read-write and `--userns=keep-id`, so changes don't need a rebuild.
- **Run container:** source baked in, read-only rootfs, all limits.
- **Host side:** only the project `.venv` with `robotraconteur`. No ROS installed on the host; the fake-robot client proves the isolation end to end.
- **Cleanup:** `scripts/clean` removes only containers and images labeled `project=rr_nav2_bridge`.

## 5. Open questions

1. **RR port:** default to `62354` (configurable) unless another is preferred.
2. ~~Resource limits~~: **resolved.** Up to 58 GB RAM and 10 cores.
3. **Default controller:** MPPI (better, heavier on CPU) or DWB (lighter)? With 12 vCPUs, MPPI is now affordable; suggested: MPPI by default, DWB available through configuration. Default robot assumed diff-drive.
4. **TLS default:** on or off?
5. **Sensors in the prototype:** is a 2D laser scan enough, or are point clouds needed from the start?
