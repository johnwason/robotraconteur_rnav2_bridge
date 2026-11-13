"""Bridge state shared by all RR objects: configuration, map, input validation,
lifecycle state. RR adapters in service.py translate RR values to calls on BridgeCore."""
from __future__ import annotations

import logging
import math
import threading
import time

import numpy as np

from .params import NavConfig

log = logging.getLogger("rr_nav2_bridge.core")

# Input size caps (protect the container from bad clients).
MAX_SCAN_RANGES = 10000
MAX_CLOUD_POINTS = 200000
MAX_MAP_CELLS = 16_000_000
MAX_COORD = 1.0e6          # |x|, |y| in meters for any pose
MAX_SPEED_IN = 100.0       # odometry velocity sanity bound


class InputError(ValueError):
    """Client sent invalid data (maps to InvalidArgumentException)."""


class StateError(RuntimeError):
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
