"""Bridge state shared by all RR objects: configuration, map, input validation,
lifecycle state. RR adapters in service.py translate RR values to calls on BridgeCore."""
from __future__ import annotations

import logging
import math
import os
import threading
import time

import numpy as np

from .nav2_process import Nav2Process
from .params import NavConfig, write_params

log = logging.getLogger("rr_nav2_bridge.core")

# Input size caps (protect the container from bad clients).
MAX_SCAN_RANGES = 10000
MAX_CLOUD_POINTS = 200000
MAX_MAP_CELLS = 10_000_000  # keeps load_map() under the RR message size limit
MAX_COORD = 1.0e6          # |x|, |y| in meters for any pose
MAX_SPEED_IN = 100.0       # odometry velocity sanity bound


class InputError(ValueError):
    """Client sent invalid data (maps to InvalidArgumentException)."""


class StateError(Exception):
    """Call not allowed in the current state (maps to InvalidOperationException)."""


def _finite(*v) -> bool:
    return all(math.isfinite(float(x)) for x in v)


def check_pose(x, y, yaw, what="pose"):
    if not _finite(x, y, yaw) or abs(x) > MAX_COORD or abs(y) > MAX_COORD:
        raise InputError(f"{what} must be finite with |x|,|y| <= {MAX_COORD:g} m")


class MapState:
    def __init__(self):
        self.resolution = 0.0
        self.width = 0
        self.height = 0
        self.origin = (0.0, 0.0, 0.0)
        self.data = np.zeros((0,), dtype=np.int8)
        self.stamp = 0.0
        self.version = 0


class BridgeCore:
    def __init__(self, ros):
        self.ros = ros
        self.lock = threading.RLock()
        self.config = NavConfig()
        self.state = "inactive"   # inactive | activating | active | paused | error
        self.message = "inactive: configure, load a map, stream inputs, then call start()"
        self.map = MapState()
        self.initial_pose = None  # amcl mode
        self._warn_times: dict[str, float] = {}
        self.map_listeners = []   # callables(version)

    # ------------------------------------------------------------- diagnostics
    def warn(self, key: str, msg: str):
        """Record an input problem in nav_status.message (rate-limited logging)."""
        now = time.monotonic()
        with self.lock:
            self.message = msg
        if now - self._warn_times.get(key, 0.0) > 5.0:
            self._warn_times[key] = now
            log.warning(msg)

    # ------------------------------------------------------------- inputs
    def on_odometry(self, x, y, yaw, vx, vy, wz):
        if not _finite(x, y, yaw, vx, vy, wz):
            return self.warn("odom", "odometry rejected: NaN/Inf value")
        if abs(x) > MAX_COORD or abs(y) > MAX_COORD:
            return self.warn("odom", "odometry rejected: position out of range")
        if max(abs(vx), abs(vy), abs(wz)) > MAX_SPEED_IN:
            return self.warn("odom", "odometry rejected: velocity out of range")
        self.ros.publish_odometry(x, y, yaw, vx, vy, wz)

    def on_scan(self, info: dict, ranges, intensities):
        ranges = np.asarray(ranges, dtype=np.float64).reshape(-1)
        n = ranges.size
        if n == 0 or n > MAX_SCAN_RANGES:
            return self.warn("scan", f"laser scan rejected: {n} ranges (allowed 1..{MAX_SCAN_RANGES})")
        a0, a1, inc = info["angle_min"], info["angle_max"], info["angle_increment"]
        rmin, rmax = info["range_min"], info["range_max"]
        if not _finite(a0, a1, inc, rmin, rmax) or inc == 0.0 or rmax <= rmin or rmax > 1000:
            return self.warn("scan", "laser scan rejected: invalid angle/range metadata")
        expected = int(round((a1 - a0) / inc)) + 1
        if abs(expected - n) > 1:
            return self.warn("scan", f"laser scan rejected: {n} ranges but angle_min/max/"
                                     f"increment imply {expected}")
        if np.isnan(ranges).any():
            return self.warn("scan", "laser scan rejected: NaN range (use +Inf for no return)")
        intensities = np.asarray(intensities, dtype=np.float64).reshape(-1)
        if intensities.size not in (0, n) or not np.isfinite(intensities).all():
            intensities = np.zeros((0,))
        self.ros.publish_scan(a0, a1, inc, info.get("time_increment", 0.0),
                              info.get("scan_time", 0.0), rmin, rmax, ranges, intensities)

    def on_points(self, xyz: np.ndarray):
        if xyz.shape[0] > MAX_CLOUD_POINTS:
            return self.warn("cloud", f"point cloud rejected: {xyz.shape[0]} points "
                                      f"(max {MAX_CLOUD_POINTS})")
        if xyz.shape[0] == 0:
            return
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        self.ros.publish_points(xyz)

    def on_external_pose(self, x, y, yaw):
        if not _finite(x, y, yaw) or abs(x) > MAX_COORD or abs(y) > MAX_COORD:
            return self.warn("ext", "external_pose rejected: non-finite or out of range")
        if self.config.localization_mode != "external":
            return
        if not self.ros.set_external_pose(x, y, yaw):
            self.warn("ext", "external_pose ignored: no odometry received yet")

    # ------------------------------------------------------------- map
    def load_map(self, resolution, width, height, origin, data):
        resolution, width, height = float(resolution), int(width), int(height)
        if not math.isfinite(resolution) or not 0.01 <= resolution <= 1.0:
            raise InputError("resolution must be within [0.01, 1.0] m")
        if width < 1 or height < 1 or width * height > MAX_MAP_CELLS:
            raise InputError(f"width*height must be within [1, {MAX_MAP_CELLS}]")
        data = np.asarray(data, dtype=np.int8).reshape(-1)
        if data.size != width * height:
            raise InputError(f"data length {data.size} != width*height {width * height}")
        if ((data < -1) | (data > 100)).any():
            raise InputError("cell values must be -1 (unknown) or 0..100")
        check_pose(*origin, what="origin")
        self.ros.publish_map(resolution, width, height, *origin, data)
        with self.lock:
            m = self.map
            m.resolution, m.width, m.height = resolution, width, height
            m.origin = tuple(float(v) for v in origin)
            m.data = data.copy()
            m.stamp = time.time()
            m.version += 1
            version = m.version
        for f in list(self.map_listeners):
            try:
                f(version)
            except Exception:
                log.exception("map listener failed")
        return version


# ---------------------------------------------------------------------------- lifecycle

RUNTIME_DIR = os.environ.get("RR_NAV2_RUNTIME_DIR", "/tmp/rr_nav2")
STARTUP, PAUSE, RESUME, RESET, SHUTDOWN = 0, 1, 2, 3, 4
EXT_POSE_TIMEOUT = 1.0


class Lifecycle:
    """start()/pause()/reset() of the Nav2 stack. One lifecycle call at a time."""

    def __init__(self, core: BridgeCore):
        self.core = core
        self.proc = Nav2Process()
        self._busy = threading.Lock()
        self.cancel_all = lambda: None  # replaced by the task manager
        self._mon = threading.Thread(target=self._monitor, name="nav2-monitor", daemon=True)
        self._mon.start()

    def _set(self, state, message):
        with self.core.lock:
            self.core.state = state
            self.core.message = message
        log.info("stack %s: %s", state, message)

    def missing_preconditions(self) -> list:
        c, cfg = self.core, self.core.config
        ages = c.ros.input_ages()
        missing = []
        if c.map.version == 0:
            missing.append("no map loaded (call map.load_map())")
        if ages["odometry"] > cfg.odometry_timeout:
            missing.append("odometry not streaming")
        if ages["sensors"] > cfg.sensor_timeout:
            missing.append("laser_scan_in (or point_cloud_in) not streaming")
        if cfg.localization_mode == "external" and ages["external_pose"] > EXT_POSE_TIMEOUT:
            missing.append("external_pose not streaming (localization_mode is external)")
        if cfg.localization_mode == "amcl" and c.initial_pose is None:
            missing.append("initialize_localization() not called (localization_mode is amcl)")
        return missing

    def start(self):
        if not self._busy.acquire(timeout=1.0):
            raise StateError("another start()/pause()/reset() is in progress")
        try:
            state = self.core.state
            if state == "active":
                return
            if state == "error":
                raise StateError(f"stack is in error state ({self.core.message}); call reset() first")
            if state == "paused":
                if not self.core.ros.lifecycle("navigation", RESUME, 60.0):
                    self._set("error", "resume failed")
                    raise RuntimeError("resume failed")
                self._set("active", "active")
                return
            missing = self.missing_preconditions()
            if missing:
                raise StateError("cannot start: " + "; ".join(missing))
            self._activate()
        finally:
            self._busy.release()

    def _activate(self):
        c, cfg = self.core, self.core.config
        self._set("activating", "starting navigation stack")
        try:
            c.ros.set_laser_mount(*cfg.laser_mount)
            amcl = cfg.localization_mode == "amcl"
            with c.ros._lock:
                c.ros.publish_map_to_odom = not amcl
                if amcl:
                    c.ros.map_to_odom = None
            params = write_params(cfg, os.path.join(RUNTIME_DIR, "nav2_params.yaml"),
                                  c.initial_pose if amcl else None)
            self.proc.launch(params, amcl, os.environ.get("RR_NAV2_ROS_LOG_LEVEL", "warn"))
            for mgr in (["localization"] if amcl else []) + ["navigation"]:
                ok = c.ros.lifecycle(mgr, STARTUP, timeout=45.0)
                if not ok or not self.proc.alive():
                    raise RuntimeError(f"{mgr} startup failed")
            # Wait for the action servers the RR members rely on.
            for name in ("navigate_to_pose", "compute_path_to_pose", "follow_path"):
                if not c.ros.actions[name].wait_for_server(timeout_sec=20.0):
                    raise RuntimeError(f"action server {name} did not come up")
            self._set("active", "active")
        except Exception as e:
            detail = self.proc.tail(3)
            self.proc.stop()
            msg = f"start failed: {e}" + (f" | nav2 log: {detail}" if detail else "")
            self._set("error", msg)
            raise RuntimeError(msg) from None

    def pause(self):
        if not self._busy.acquire(timeout=1.0):
            raise StateError("another start()/pause()/reset() is in progress")
        try:
            if self.core.state != "active":
                raise StateError(f"pause() requires state active (state is {self.core.state})")
            self.cancel_all()
            if not self.core.ros.lifecycle("navigation", PAUSE, 60.0):
                self._set("error", "pause failed")
                raise RuntimeError("pause failed")
            self._set("paused", "paused: call start() to resume")
        finally:
            self._busy.release()

    def reset(self):
        with self._busy:
            self.cancel_all()
            self.proc.stop()
            with self.core.ros._lock:
                self.core.ros.costmaps = {"global": None, "local": None}
                self.core.ros.cmd_vel = None
                self.core.ros.global_plan = self.core.ros.local_plan = None
            self._set("inactive", "inactive: configuration may be changed; call start()")

    def shutdown(self):
        self.cancel_all()
        self.proc.stop(timeout=5.0)

    def _monitor(self):
        while True:
            time.sleep(1.0)
            # Skip while start()/pause()/reset() runs: they replace the process themselves.
            if not self._busy.acquire(blocking=False):
                continue
            try:
                if self.core.state in ("active", "paused") and not self.proc.alive():
                    code = self.proc.exit_code()
                    self.cancel_all()
                    self._set("error", f"navigation stack exited unexpectedly (code {code}); "
                                       f"call reset() then start()")
            finally:
                self._busy.release()
