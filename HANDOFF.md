# Handoff: Robot Raconteur frontend for ROS 2 Nav2

**Status (2026-11-13):** all 30 plan steps implemented on branch `dev/prototype`; smoke (12) and hardening (6) tests pass against the hardened container. `main` is untouched pending the user's approval to merge. See §9 for what changed relative to this plan.

---

## 1. Goal

Build a Robot Raconteur (RR) service that gives RR clients access to ROS 2 Nav2 capabilities, following the established pattern of an RR frontend placed over a set of ROS nodes. Ship it as a single all-in-one Podman image in which ROS 2 is completely hidden from the user.

## 2. Decisions (settled — do not reopen without the user)

| Topic | Decision |
|---|---|
| ROS distro | **ROS 2 Jazzy** (LTS, EOL May 2029). Kilted was considered and rejected (non-LTS, EOL ~Dec 2026). |
| Scope | **Algorithm only.** Nav2 never talks to a real robot. The client streams sensors/odometry/map in over RR and receives plans and `cmd_vel` out; the client decides how (or whether) to command the robot. |
| Types | Standard `com.robotraconteur.*` types for **data**; a **custom robdef** for the Nav2 bridge service itself. |
| Docs in robdef | Every member gets `##` docstring comments written for an **LLM client** (units, frames, preconditions, failure behavior, which call fits which intent, expected call order). |
| Packaging | **All-in-one image** (Nav2 + bridge). ROS 2 fully isolated inside; **only the RR TCP port is forwarded.** |
| Language | **Python** for the initial prototype (rclpy + `robotraconteur` pip package, one process). |
| Workflow | **Small commits** to track progress. **Podman** (not Docker) for development. When the user says "just plan", produce no code. |
| Resources | The project may use **up to 58 GB RAM and 10 CPU cores** in total (approved by the user 2026-11-13). |

## 3. Architecture

```
 RR client (robot owner)                    Podman container (only RR port published)
 ──────────────────────                     ─────────────────────────────────────────
 odometry  ───────────────►  bridge ──► /odom, TF odom→base_link
 laser scans / clouds ────►  bridge ──► /scan, /points (costmap obstacle layers)
 map ─────────────────────►  bridge ──► /map (transient-local; replaces map_server)
 pose (external loc.) ────►  bridge ──► TF map→odom        (or AMCL, optional)
 goals (generators) ──────►  bridge ──► Nav2 action servers
 cmd_vel  ◄───────────────  bridge ◄── final /cmd_vel (after smoother/collision monitor)
 plans, costmaps, status ◄─  bridge ◄── /plan, /local_plan, costmap topics
```

### ROS → RR mapping
- **Actions → RR generators.** `Next()` returns feedback; the generator ends with a result or raises an exception on failure; `Abort()`/`Close()` cancel the ROS goal.
- **Continuous state → `wire`** (readonly outputs, writeonly inputs).
- **Per-message sensor data → `pipe`** (writeonly).
- **Maps/costmaps → `memory`** for cell data, plus metadata properties and a `map_changed` event.
- **Services → `function`.**
- **Organization:** a root object with `objref` sub-objects.

### Robdef outline (working name `experimental.nav2_bridge`)
Root object `Nav2Navigator`:
- **Inputs:** `wire odometry [writeonly]`, `pipe laser_scan_in [writeonly]`, `pipe point_cloud_in [writeonly]`, `wire external_pose [writeonly]`
- **Outputs (readonly wires):** `cmd_vel` (stamped), `global_plan`, `local_plan`, `robot_pose`, `nav_status`
- **Generators:** `navigate_to_pose`, `navigate_through_poses`, `follow_waypoints`, `follow_path`, `spin`, `backup`, `drive_on_heading`, `wait`
- **Functions:** `compute_path_to_pose`, `compute_path_through_poses` (planning only, never produce motion), `cancel_all`, `clear_costmaps`, `set_initial_pose`, `start`, `pause`, `reset`
- **Sub-objects:** `map` (`set_map`, readonly current map, `map_changed` event), `global_costmap` / `local_costmap` (metadata + `memory`), `configuration` (footprint/radius, velocity/accel limits, frame names, localization mode; applied as ROS params internally, never exposing ROS names)
- **Service docstring:** expected call order is configure → set map → stream odometry + scans → `start` → navigate.

Candidate standard types (**verify against the standard robdef repo first** — it is not confirmed which exist for occupancy grids, paths, or odometry): `com.robotraconteur.geometry`, `.laserscan`, `.pointcloud`, `.sensordata`, `.action.ActionStatusCode`.

### Bridge internals
- rclpy runs a `MultiThreadedExecutor` (reentrant callback groups) on a background thread. RR handlers wait on rclpy futures **with timeouts** and never spin rclpy themselves.
- Incoming data is stamped with container time on arrival by default, so client/container clock skew doesn't break TF.
- Nav2 runs with **autostart off**; `start()` checks preconditions, then activates the stack through the lifecycle manager.
- The bridge publishes `/map` itself (no map_server). AMCL is optional and selected through `configuration`. SLAM is out of scope.
- **Watchdog:** stale odometry or sensor input → abort active goals, zero `cmd_vel`, set a reason in `nav_status`. `cmd_vel` is timestamped so clients can detect stale output.
- **Input validation:** reject NaN/Inf, oversized maps and scans, absurd limits.
- Jazzy specifics: set `enable_stamped_cmd_vel: true` in the Nav2 params (the Jazzy default is unstamped `Twist`). `nav2_route` doesn't exist in Jazzy, so there is no route server.

## 4. Host environment (re-checked 2026-11-13 after moving to a new machine)

| Item | Value | Implication |
|---|---|---|
| Machine | VMware VM, Ubuntu 24.04.5, kernel 7.0 | Replaces the earlier 5-core / 8 GB machine |
| Podman | 4.9.3, rootless, cgroup v2 (cpu/memory/pids delegated) | Good baseline; resource limits work rootless |
| SELinux / AppArmor | no SELinux; AppArmor on in the kernel, but rootless Podman applies no profile | Container hardening is still the main confinement; seccomp is on |
| CPU / RAM | 12 vCPUs (Ryzen 9 9950X3D host), 62 GB total, ~53 GB available | Plenty of headroom; keep hard limits anyway to contain runaways |
| Disk | ~133 GB free; store at `~/.local/share/containers` | Fine; prune only labeled objects |
| Host Python | 3.12.3 | Matches Ubuntu 24.04 / Jazzy |
| git | 2.43.0 | — |

## 5. Local-machine safety rules (mandatory)

**Filesystem**
- Write only inside this project directory and the agent's session scratchpad. Never `/etc`, `/usr`, other home directories, or system Python.
- **No `sudo`.** If something seems to need root, stop and ask.
- Bind-mount only project subdirectories: read-only in run containers, read-write source mount only in the dev container. Never mount `$HOME`, `/`, the Podman socket, `/dev`, or credential directories (`~/.ssh`, `~/.gnupg`, …).
- Host Python packages go only in a project-local `.venv/` (git-ignored). No `pip install --user` and no global pip.

**Containers**
- Dev container: `--userns=keep-id`. In the image, run as an unprivileged `nav` user.
- Always use `--cap-drop=ALL --security-opt=no-new-privileges`, with the default seccomp profile.
- Never use `--privileged`, `--network=host`, `--pid=host`, `--ipc=host`, or device passthrough.
- `--read-only` rootfs + `--tmpfs` for `/tmp`, ROS logs, and `~/.ros`.
- Limits on every run and build: `--memory=58g --memory-swap=58g --cpus=10 --pids-limit=512`, which is the user-approved ceiling. The ceiling covers everything the project runs at once, so if two containers run at the same time (e.g. dev + test), split the budget between them.
- **Dev: publish the RR port on loopback only** (`-p 127.0.0.1:<port>:<port>`). Binding to all interfaces is a documented production option that requires TLS + authentication.
- Name and label everything (`rr-nav2-*`, `label=project=rr_nav2_bridge`). Cleanup filters by label. **Never run a bare `podman system prune`.**
- Keep all hardened flags in `scripts/` as the single source of truth. No ad-hoc podman commands.

**Network / supply chain**
- Base image by fully qualified name + pinned digest: `docker.io/library/ros:jazzy-ros-base@sha256:…`, with the digest recorded in a commit.
- apt packages only from the official Ubuntu and ROS repos. pip pinned to exact versions with `--require-hashes` (image and host venv).
- Downloads go into an empty scratch directory and are treated as untrusted; never execute from there.
- No pushes to git remotes or image registries unless the user asks.

**Git / process**
- `git init`; work on branch `dev/prototype`; leave `main` alone until the user approves a merge. Small commits with the attribution trailer. No history rewriting or force operations.
- `.gitignore`: `.venv/`, logs, `__pycache__`, generated certs, local overrides. No secrets in the repo.
- Before anything destructive or hard to undo (removing images or volumes, changes outside the project): show what will be affected and ask first.
- Run long builds and runs in the background with logs in the scratchpad.

## 6. Work plan (one line ≈ one commit)

**Phase 0 – Safe scaffolding**
1. `git init`, `dev/prototype` branch, `.gitignore`, README stub with the safety rules
2. `scripts/` with hardened `podman build` / `run` / `clean` flags centralized
3. Host venv setup script (project-local, hash-pinned `robotraconteur`)

**Phase 1 – Interface design**
4. Survey notes: standard RR types vs Jazzy `nav2_msgs`
5. Robdef skeleton: root object, inputs, `##` docs
6. Robdef: outputs, generators, sub-objects, configuration, full LLM-oriented docs
7. Robdef validation: parse with the RR library inside the container

**Phase 2 – Image**
8. Containerfile: digest-pinned Jazzy base, Nav2, hash-pinned robotraconteur, non-root `nav` user
9. Launch file + default Nav2 params (autostart off, `enable_stamped_cmd_vel: true`, no map_server)
10. Isolation config: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, fixed `ROS_DOMAIN_ID`, read-only rootfs + tmpfs layout
11. Verification script: DDS stays on localhost; only the RR port listens (`ss -lntu`)

**Phase 3 – Bridge (Python)**
12. Scaffolding: rclpy executor thread, RR node, loopback-only port in dev
13. Odometry wire → `/odom` + TF
14. Map → `/map`; laser scan pipe → `/scan`
15. External-pose mode → TF `map→odom`; AMCL mode
16. Lifecycle `start` / `pause` / `reset` with precondition checks
17. `compute_path_to_pose` / `compute_path_through_poses`
18. Output wires: `cmd_vel` (stamped), plans, robot pose, status
19. `navigate_to_pose` generator (feedback, result, cancel)
20. Remaining generators (through-poses, waypoints, follow_path, behaviors)
21. Costmap sub-objects (memory) + `clear_costmaps`
22. Configuration object → ROS parameters
23. Watchdog / stale-input handling
24. Input validation and size caps

**Phase 4 – Testing (no ROS on the host)**
25. Fake-robot RR client in the host venv: unicycle model, ray-cast scans, test map
26. Smoke tests: planning only, full navigation loop, cancel, watchdog
27. Hardening tests: limits enforced, survives malformed input, only `127.0.0.1:<port>` reachable

**Phase 5 – Production options and docs**
28. Optional TLS + RR user authentication (certs from a mounted, git-ignored directory)
29. Optional Quadlet unit with the same hardening flags
30. Docs: client URL usage (`rr+tcp://host:port?service=nav2`), LLM usage notes, safety model, cleanup

## 7. Open questions for the user

1. **RR port:** default to `62354` (configurable) unless the user prefers another.
2. ~~Resource limits~~: **resolved.** Up to 58 GB RAM and 10 cores (see §2).
3. **Default controller:** MPPI (better, heavier on CPU) or DWB (lighter)? With 12 vCPUs, MPPI is now affordable; suggested: MPPI by default, DWB available through configuration. Default robot assumed diff-drive.
4. **TLS default:** on or off?
5. **Sensors in the prototype:** is a 2D laser scan enough, or are point clouds needed from the start?

## 8. Known caveats

- RR UDP multicast discovery won't cross the published-port boundary. Clients connect by URL; document this prominently.
- Fast DDS shared memory isn't an issue because everything ROS runs inside one container. Keep it that way.
- If the project later moves to Lyrical (LTS, May 2026), expect only base-image and Nav2 API changes; the RR interface should remain stable.

## 9. Implementation notes (deviations and findings, 2026-11-13)

- **`nav2_route` exists in Jazzy 1.3.13** (§3 said it does not). Still out of scope.
- **Docstrings are not transmitted by RR 1.2.9.** Service definitions reach clients
  re-serialized without `##` comments. The root object adds `documentation(name)`, which
  serves the original robdef text or a single entry. Likely worth raising upstream.
- **RR reserves `set_`/`get_` member prefixes.** Renamed: `set_map` → `map.load_map`,
  `set_initial_pose` → `initialize_localization`, plus `read_map`, `cost_at`, `describe`.
- **Executor:** `SingleThreadedExecutor`, not `MultiThreadedExecutor`. The multithreaded
  executor used about 109% of a core (GIL-bound) just for input streaming, which slowed
  Nav2's control loop and caused dropped DDS responses. Single-threaded: 11% streaming,
  20% navigating; the full navigation test went from 130 s to 32 s.
- **Nav2 runs as a subprocess launched on `start()`** and is torn down on `reset()`, so
  controller type and localization mode can change. Lifecycle autostart stays off.
  `start()` sends `STARTUP` only after an `is_active` round-trip, because a request sent
  right after `wait_for_service()` can be lost before DDS matching completes.
- **TF timing:** `odom→base_link` is re-published at 50 Hz, future-dated by 50 ms;
  `map→odom` (external mode) by 100 ms. Sensor data is stamped on arrival. Without this,
  the controller hit "extrapolation into the future" errors.
- **AMCL:** `initialize_localization()` must be called *before* `start()` in amcl mode
  (the global costmap cannot activate until AMCL publishes `map→odom`).
- **Velocity smoother** allows reverse up to the forward limit, so `backup()` works when
  `max_reverse_velocity` is 0 (the controller keeps its own reverse limit).
- **Map size cap is 10 M cells** (not 16 M), to stay under RR's default message size.
- **Fast DDS:** loopback + SHM profile. Fast DDS still opens one ephemeral *send* socket
  per interface; `verify_isolation.sh` allows those but fails on any non-loopback DDS
  receive port. An SHM-only profile made Fast DDS fall back to builtin UDP on 0.0.0.0.
- **RR auth** uses the MD5 password file (`PasswordFileUserAuthenticator`); a users-file
  line without a privileges column fails with a raw `vector::_M_range_check` error.
  **RR TLS** needs an RR-issued node certificate, so it is implemented with a fail-fast
  check but a TLS session was not tested.
- **Client reconnects:** right after a session to the same service disconnects, a new
  session sometimes lost its transport while attaching members ("Transport connection
  to remote host not found"). The fake robot retries the whole connect sequence.

### Open questions resolved by default (user may revisit)

| Question | Default used |
|---|---|
| RR port | 62354 (`RR_NAV2_PORT`) |
| Default controller | MPPI; DWB through `configuration.controller` (both tested) |
| TLS default | off (loopback only); non-loopback binding requires TLS + auth |
| Sensors | 2D laser required; point cloud input implemented and tested (cloud-only obstacle avoided) |
