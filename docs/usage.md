# Using the Nav2 bridge

The bridge is a Robot Raconteur (RR) service that runs a complete ROS 2 Jazzy Nav2 stack
inside a container. Clients never see ROS: they stream robot data in and get plans and
velocity commands out over RR.

- **Scope: algorithm only.** The bridge never moves a robot. Your client reads `cmd_vel`
  and decides whether to send it to the motors.
- **Interface:** [`robdef/experimental.nav2_bridge.robdef`](../robdef/experimental.nav2_bridge.robdef).
  Every member is documented there (units, frames, errors, call order).

## Connecting

```
rr+tcp://<host>:62354?service=nav2
```

- RR UDP discovery does not cross the container's published port, so clients **must
  connect by URL**. Discovery-based lookups (`FindServiceByType`) will not find it.
- In the default dev setup the port is bound to `127.0.0.1` only; connect from the same
  machine. See [Exposing the port](#exposing-the-port-beyond-localhost).
- `start()` can take up to ~60 s in the worst case. Set the client request timeout to at
  least 60 s (Python: `RRN.RequestTimeout = 60`).

### Documentation over the wire

Robot Raconteur 1.2.9 does **not** transmit `##` comments from a robdef to clients (the
definition is re-serialized without them). The service therefore offers:

```python
c.documentation("")                      # the whole robdef with all comments
c.documentation("navigate_to_pose")      # one member
c.documentation("map.load_map")          # member of a sub-object (objref name)
c.documentation("NavStatus")             # a type
```

LLM-driven clients should call `documentation("")` first.

## Minimal Python client

```python
import numpy as np
import RobotRaconteur as RR
from RobotRaconteur.Client import RRN

RRN.RequestTimeout = 60
c = RRN.ConnectService("rr+tcp://127.0.0.1:62354?service=nav2")
Pose2D = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose2D", c)

def pose(x, y, yaw):
    p = np.zeros((1,), dtype=Pose2D)
    p[0]["position"]["x"], p[0]["position"]["y"], p[0]["orientation"] = x, y, yaw
    return p

# 1. configure (only while inactive)
cfg = c.get_configuration()
cfg.robot_radius = 0.25
cfg.max_linear_velocity = 0.4

# 2. map
grid = RRN.NewStructure("experimental.nav2_bridge.OccupancyGrid", c)
grid.resolution, grid.width, grid.height = 0.05, 200, 160
grid.origin = pose(0, 0, 0)
grid.data = my_int8_cells.reshape(-1)          # row-major, row 0 at the origin
c.get_map().load_map(grid)

# 3. stream inputs (in your own loops): odometry >= 10 Hz, laser >= 5 Hz,
#    external_pose >= 5 Hz (default localization_mode external)
odom = c.odometry.Connect();  scans = c.laser_scan_in.Connect(-1)
ext = c.external_pose.Connect();  cmd = c.cmd_vel.Connect()
# odom.OutValue = <Odometry2D>; scans.SendPacket(<LaserScan>); ext.OutValue = pose(...)

# 4. activate, 5. navigate
c.start()
gen = c.navigate_to_pose(pose(8.0, 6.0, 0.0))
try:
    while True:
        fb = gen.Next()                        # NavigationFeedback, ~1 Hz or faster
        # meanwhile, in the control loop: v = cmd.InValue
        #   apply v.velocity only if v.active and (now - v.stamp) < 0.5
except RR.StopIterationException:
    print("arrived")
```

On the client side, RR Python exposes sub-objects as `get_<name>()`:
`c.get_map()`, `c.get_global_costmap()`, `c.get_local_costmap()`, `c.get_configuration()`.

[`tests/fake_robot.py`](../tests/fake_robot.py) is a complete, runnable example client
(simulated robot, ray-cast laser, streaming loops).

## Notes for LLM clients

- Call `documentation("")` once and keep it. It contains the call order and the intent
  table ("plan only" vs "go there" vs "stop now").
- Planning (`compute_path_*`) never moves the robot. Navigation generators do (through
  `cmd_vel`), but only if your client forwards `cmd_vel` to the motors.
- Errors are explicit: `InvalidArgument` (bad value; the message says the allowed
  range), `InvalidOperation` (wrong state or missing input; the message lists what is
  missing), `OperationFailed` (Nav2 could not do it; the message has the Nav2 error
  name, e.g. `GOAL_OUTSIDE_MAP (204)`), `OperationAborted` (cancelled, preempted, or
  watchdog).
- Read `nav_status` when something fails; `message` explains the latest event.
- `cancel_all()` is always safe and stops motion.

## Safety model

| Layer | What it guarantees |
|---|---|
| Algorithm-only scope | The service cannot actuate anything; it only publishes `cmd_vel` on an RR wire. |
| `cmd_vel` stamping | Every command carries a service timestamp, a sequence number and an `active` flag. Clients must stop the robot when the stamp stops advancing (> ~0.5 s). |
| Watchdog | Stale odometry (> `odometry_timeout`), laser/point cloud (> `sensor_timeout`) or external pose (> 1 s) aborts every task, zeroes `cmd_vel` and refuses new tasks until inputs recover. |
| Collision monitor | Nav2's collision monitor slows/stops the final command based on the latest laser scan. |
| Input validation | NaN/Inf and out-of-range values are rejected; maps are capped at 10 M cells, scans at 10,000 ranges, point clouds at 200,000 points, waypoint lists at 100. |
| Container | Rootless Podman, unprivileged `nav` user, all capabilities dropped, `no-new-privileges`, read-only rootfs, tmpfs for writable dirs, limits on memory, CPU and PIDs. |
| Network | Only the RR TCP port is published (loopback by default). ROS 2 DDS traffic stays inside the container (discovery range LOCALHOST, Fast DDS on loopback + shared memory). `scripts/verify_isolation.sh` checks this on a running container. |

The bridge is not a safety-rated component. Use the robot's own emergency stop and
low-level limits.

## Authentication and TLS (optional)

Secrets live in `certs/` (git-ignored; override with `RR_NAV2_SECRETS_DIR`) and are
mounted read-only. Both features are off by default.

**Authentication** (`RR_NAV2_AUTH=1`): every connection must log in as a user from
`certs/users.txt` (RR `PasswordFileUserAuthenticator`; object locking is disabled).

```bash
scripts/add_user.sh alice                 # prompts for the password
RR_NAV2_AUTH=1 scripts/run.sh
```

```python
c = RRN.ConnectService(url, "alice", {"password": RR.VarValue("<password>", "string")})
```

The RR password file stores MD5 hashes. Keep `certs/` private (the script sets mode
600/700) and use TLS whenever the port is reachable from a network.

**TLS** (`RR_NAV2_TLS=1`): the bridge loads the node certificate for `RR_NAV2_NODE_ID` and
requires TLS on every connection. RR TLS uses node certificates issued for a specific
NodeID; put the certificate files in `certs/tls/`, which is mounted as the RR
certificate store (`~/.local/share/RobotRaconteur/certificates`) in the container. If
no certificate can be loaded, the bridge exits with an error instead of running.
Clients connect with `rrs+tcp://<host>:62354?service=nav2`.

Verified in this repository: authentication end to end (no credentials or a wrong
password are refused, the right password connects), and the fail-fast check for a
missing certificate. A TLS session was **not** tested, because that needs an RR-issued
node certificate.

## Exposing the port beyond localhost

The scripts refuse to bind anywhere but `127.0.0.1` unless both TLS and authentication
are on:

```bash
RR_NAV2_BIND_ADDR=0.0.0.0 RR_NAV2_TLS=1 RR_NAV2_AUTH=1 \
RR_NAV2_NODE_ID=<uuid> scripts/run.sh
```

An unauthenticated bridge on a LAN would let anyone command your navigation stack.

## Operating the container

| Task | Command |
|---|---|
| Build the image | `scripts/build.sh` |
| Host client venv | `scripts/setup_venv.sh` |
| Run (baked source) | `scripts/run.sh` (`RR_NAV2_DETACH=1` to detach) |
| Dev (live `src/`) | `scripts/dev.sh` |
| Tests | `scripts/test.sh [smoke\|hardening\|all]` |
| Isolation check | `RR_NAV2_NAME=rr-nav2-run scripts/verify_isolation.sh` |
| One-off command in the image (no network) | `scripts/oneshot.sh <cmd>` |
| Add an RR user | `scripts/add_user.sh <name>` |
| Quadlet unit | `scripts/quadlet.sh` (writes `deploy/`, installs nothing) |
| Cleanup | `scripts/clean.sh` (dry run), `--dangling` (old builds only), `--yes` (everything) |

Cleanup only touches containers and images labeled `project=rr_nav2_bridge`.

Environment knobs (all optional): `RR_NAV2_PORT` (62354), `RR_NAV2_MEMORY` (58g),
`RR_NAV2_CPUS` (10), `RR_NAV2_PIDS` (512), `RR_NAV2_ROS_LOG_LEVEL` (warn). Nav2's log is
`/home/nav/.ros/log/nav2_launch.log` inside the container (tmpfs).

## Known limitations

- 2D only: Pose2D everywhere; point clouds are flattened into the 2D costmaps.
- One robot per container.
- No SLAM: the map must come from the client. AMCL is available as a localizer.
- `nav2_route` ships with this Jazzy release but is not exposed.
- Docstrings are not transmitted by RR; use `documentation()`.
