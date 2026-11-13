"""RR service objects for experimental.nav2_bridge (thin adapters over BridgeCore)."""
from __future__ import annotations

import functools
import logging
import threading

import numpy as np
import RobotRaconteur as RR

from .convert import Types, pose2d_tuple
from .core import BridgeCore, InputError, Lifecycle, StateError, check_pose
from . import params as P
from .docs import DocIndex

log = logging.getLogger("rr_nav2_bridge.service")
NS = "experimental.nav2_bridge."


def rr_errors(f):
    """Map bridge exceptions onto RR exceptions with client-facing messages."""
    @functools.wraps(f)
    def w(*a, **k):
        try:
            return f(*a, **k)
        except RR.RobotRaconteurException:
            raise
        except (InputError, ValueError, TypeError) as e:
            raise RR.InvalidArgumentException(str(e))
        except StateError as e:
            raise RR.InvalidOperationException(str(e))
        except TimeoutError as e:
            raise RR.OperationTimeoutException(str(e) or "timed out")
        except Exception as e:
            log.exception("unexpected error in %s", f.__name__)
            raise RR.OperationFailedException(f"{f.__name__} failed: {e}")
    return w


def _not_yet(name):
    def f(self, *args):
        raise RR.NotImplementedException(f"{name} is not implemented yet")
    f.__name__ = name
    return f


class SwapMemory:
    """Readonly RR memory backed by a numpy array that can be replaced atomically."""

    def __init__(self, dtype):
        self._a = np.zeros((0,), dtype=dtype)
        self._lock = threading.Lock()

    def set(self, arr: np.ndarray):
        with self._lock:
            self._a = arr

    @property
    def Length(self):
        return int(self._a.shape[0])

    def Read(self, memorypos, buffer, bufferpos, count):
        a = self._a
        if memorypos + count > a.shape[0]:
            raise RR.OutOfRangeException("read past end of memory")
        # The server-side director passes a fresh buffer of exactly `count` elements.
        start = bufferpos if len(buffer) >= bufferpos + count else 0
        buffer[start:start + count] = a[memorypos:memorypos + count]

    def Write(self, memorypos, buffer, bufferpos, count):
        raise RR.ReadOnlyMemberException("memory is read-only")


def grid_info(t: Types, resolution, width, height, origin, stamp, version):
    s = t.struct("GridInfo")
    s.resolution = float(resolution)
    s.width, s.height = int(width), int(height)
    s.origin = t.pose2d(*origin)
    s.stamp = float(stamp)
    s.version = int(version)
    return s


class OccupancyMapObj:
    def __init__(self, core: BridgeCore, types: Types):
        self._core = core
        self._t = types
        self.data = SwapMemory(np.int8)
        self.map_changed = RR.EventHook()
        core.map_listeners.append(self._changed)

    def _changed(self, version):
        self.data.set(self._core.map.data)
        self.map_changed.fire(version)

    @rr_errors
    def load_map(self, grid):
        self._core.load_map(grid.resolution, grid.width, grid.height,
                            pose2d_tuple(grid.origin), grid.data)

    @rr_errors
    def read_map(self):
        m = self._core.map
        if m.version == 0:
            raise StateError("no map loaded; call map.load_map() first")
        g = self._t.struct("OccupancyGrid")
        g.resolution, g.width, g.height = m.resolution, m.width, m.height
        g.origin = self._t.pose2d(*m.origin)
        g.data = m.data.copy()
        return g

    @property
    def info(self):
        m = self._core.map
        return grid_info(self._t, m.resolution, m.width, m.height, m.origin, m.stamp, m.version)


class NavConfigurationObj:
    """Properties map 1:1 onto params.NavConfig; writes only while inactive."""

    CONTROLLERS = {0: "mppi", 1: "dwb"}
    MODES = {0: "external", 1: "amcl"}

    def __init__(self, core: BridgeCore, types: Types):
        self._core = core
        self._t = types

    def _write(self, name, value):
        with self._core.lock:
            if self._core.state != "inactive":
                raise StateError(f"configuration can only be changed while inactive "
                                 f"(state is {self._core.state}); call reset() first")
            setattr(self._core.config, name, value)

    def _float_prop(name):  # noqa: N805 - descriptor factory
        def get(self):
            return float(getattr(self._core.config, name))

        @rr_errors
        def set_(self, v):
            self._write(name, P.check_float(name, v))
        return property(get, set_)

    robot_radius = _float_prop("robot_radius")
    max_linear_velocity = _float_prop("max_linear_velocity")
    max_reverse_velocity = _float_prop("max_reverse_velocity")
    max_angular_velocity = _float_prop("max_angular_velocity")
    max_linear_acceleration = _float_prop("max_linear_acceleration")
    max_angular_acceleration = _float_prop("max_angular_acceleration")
    inflation_radius = _float_prop("inflation_radius")
    goal_xy_tolerance = _float_prop("goal_xy_tolerance")
    goal_yaw_tolerance = _float_prop("goal_yaw_tolerance")
    odometry_timeout = _float_prop("odometry_timeout")
    sensor_timeout = _float_prop("sensor_timeout")
    del _float_prop

    @property
    def footprint(self):
        return self._t.point2d_array(self._core.config.footprint)

    @footprint.setter
    @rr_errors
    def footprint(self, v):
        a = np.asarray(v).reshape(-1)
        self._write("footprint", P.check_footprint([(p["x"], p["y"]) for p in a]))

    @property
    def holonomic(self):
        return bool(self._core.config.holonomic)

    @holonomic.setter
    @rr_errors
    def holonomic(self, v):
        self._write("holonomic", bool(v))

    @property
    def controller(self):
        return {v: k for k, v in self.CONTROLLERS.items()}[self._core.config.controller]

    @controller.setter
    @rr_errors
    def controller(self, v):
        if int(v) not in self.CONTROLLERS:
            raise InputError("controller must be mppi (0) or dwb (1)")
        self._write("controller", self.CONTROLLERS[int(v)])

    @property
    def localization_mode(self):
        return {v: k for k, v in self.MODES.items()}[self._core.config.localization_mode]

    @localization_mode.setter
    @rr_errors
    def localization_mode(self, v):
        if int(v) not in self.MODES:
            raise InputError("localization_mode must be external (0) or amcl (1)")
        self._write("localization_mode", self.MODES[int(v)])

    @property
    def laser_mount(self):
        x, y, yaw = self._core.config.laser_mount
        return self._t.pose2d(x, y, yaw)

    @laser_mount.setter
    @rr_errors
    def laser_mount(self, v):
        self._write("laser_mount", P.check_laser_mount(*pose2d_tuple(v)))

    @rr_errors
    def reset_to_defaults(self):
        with self._core.lock:
            if self._core.state != "inactive":
                raise StateError("configuration can only be changed while inactive")
            self._core.config = P.NavConfig()

    def describe(self):
        out = {}
        for k, v in self._core.config.as_dict().items():
            if isinstance(v, bool):
                out[k] = RR.VarValue(v, "bool")
            elif isinstance(v, (int, float)):
                out[k] = RR.VarValue(float(v), "double")
            elif isinstance(v, str):
                out[k] = RR.VarValue(v, "string")
            else:  # footprint [[x,y],...] / laser_mount [x,y,yaw]
                flat = np.asarray(v, dtype=np.float64).reshape(-1)
                out[k] = RR.VarValue(flat, "double[]")
        return out


class Nav2Navigator:
    """Root object."""

    def __init__(self, ros, types: Types, robdef_text: str):
        self._ros = ros
        self._t = types
        self._core = BridgeCore(ros)
        self._docs = DocIndex(robdef_text)
        self._map = OccupancyMapObj(self._core, types)
        self._config = NavConfigurationObj(self._core, types)
        self._life = Lifecycle(self._core)

    # ---------------------------------------------------------------- RR plumbing
    def RRServiceObjectInit(self, context, service_path):
        self.odometry.InValueChanged += self._on_odometry
        self.external_pose.InValueChanged += self._on_external_pose
        self.laser_scan_in.PipeConnectCallback = self._scan_connect
        self.point_cloud_in.PipeConnectCallback = self._cloud_connect
        log.info("service object initialized at %s", service_path)

    def get_map(self):
        return self._map, NS + "OccupancyMap"

    def get_configuration(self):
        return self._config, NS + "NavConfiguration"

    def shutdown(self):
        self._life.shutdown()

    # ---------------------------------------------------------------- lifecycle
    @rr_errors
    def start(self):
        try:
            self._life.start()
        except RuntimeError as e:
            raise RR.OperationFailedException(str(e))

    @rr_errors
    def pause(self):
        self._life.pause()

    @rr_errors
    def reset(self):
        self._life.reset()

    @rr_errors
    def initialize_localization(self, pose):
        x, y, yaw = pose2d_tuple(pose)
        check_pose(x, y, yaw)
        if self._core.config.localization_mode != "amcl":
            raise StateError("initialize_localization() is only used in localization_mode amcl")
        with self._core.lock:
            self._core.initial_pose = (x, y, yaw)
        if self._core.state in ("active", "paused"):
            self._ros.publish_initial_pose(x, y, yaw)

    # ---------------------------------------------------------------- docs
    def documentation(self, name):
        try:
            return self._docs.lookup(name)
        except KeyError as e:
            raise RR.InvalidArgumentException(str(e.args[0]))

    # ---------------------------------------------------------------- inputs
    def _on_odometry(self, value, ts, ep):
        try:
            x, y, yaw = pose2d_tuple(value.pose)
            v = np.asarray(value.velocity).reshape(-1)[0]
            self._core.on_odometry(x, y, yaw, float(v["linear"]["x"]), float(v["linear"]["y"]),
                                   float(v["angular"]["z"]))
        except Exception as e:
            self._core.warn("odom", f"odometry rejected: {e}")

    def _on_external_pose(self, value, ts, ep):
        try:
            self._core.on_external_pose(*pose2d_tuple(value))
        except Exception as e:
            self._core.warn("ext", f"external_pose rejected: {e}")

    def _scan_connect(self, ep):
        ep.PacketReceivedEvent += self._scan_packets

    def _scan_packets(self, ep):
        while ep.Available > 0:
            s = ep.ReceivePacket()
            try:
                i = s.scan_info
                info = dict(angle_min=i.angle_min, angle_max=i.angle_max,
                            angle_increment=i.angle_increment, time_increment=i.time_increment,
                            scan_time=i.scan_time, range_min=i.range_min, range_max=i.range_max)
                ranges = s.ranges if s.ranges is not None else []
                inten = s.intensities if s.intensities is not None else []
                self._core.on_scan(info, ranges, inten)
            except Exception as e:
                self._core.warn("scan", f"laser scan rejected: {e}")

    def _cloud_connect(self, ep):
        ep.PacketReceivedEvent += self._cloud_packets

    def _cloud_packets(self, ep):
        while ep.Available > 0:
            c = ep.ReceivePacket()
            try:
                p = c.points
                if p is None or len(p) == 0:
                    continue
                xyz = np.stack([p["x"], p["y"], p["z"]], axis=1).astype(np.float32)
                self._core.on_points(xyz)
            except Exception as e:
                self._core.warn("cloud", f"point cloud rejected: {e}")

    # ---------------------------------------------------------------- not yet
    navigate_to_pose = _not_yet("navigate_to_pose")
    navigate_through_poses = _not_yet("navigate_through_poses")
    follow_waypoints = _not_yet("follow_waypoints")
    follow_path = _not_yet("follow_path")
    spin = _not_yet("spin")
    backup = _not_yet("backup")
    drive_on_heading = _not_yet("drive_on_heading")
    wait = _not_yet("wait")
    compute_path_to_pose = _not_yet("compute_path_to_pose")
    compute_path_from_pose = _not_yet("compute_path_from_pose")
    compute_path_through_poses = _not_yet("compute_path_through_poses")
    cancel_all = _not_yet("cancel_all")
    clear_costmaps = _not_yet("clear_costmaps")
