"""RR service objects for experimental.nav2_bridge (thin adapters over BridgeCore)."""
from __future__ import annotations

import functools
import logging
import math
import threading
import time

import numpy as np
import RobotRaconteur as RR

from .convert import Types, pose2d_tuple
from . import tasks as T
from .core import (EXT_POSE_TIMEOUT, BridgeCore, InputError, Lifecycle, StateError,
                   check_pose)
from .ros_side import compose, path_poses, pose_of_msg
from .tasks import TaskFailed
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


def _check_allowance(v):
    if not math.isfinite(v) or not 0 < v <= 3600:
        raise InputError("time_allowance must be within (0, 3600] s")


def _age(a: float) -> str:
    return "never received" if math.isinf(a) else f"{a:.1f} s"


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


class _TaskGenerator:
    """RR generator over one ActionTask. Next() blocks up to ~1 s and returns the latest
    feedback; the final item has status complete; then StopIteration. Failures raise
    OperationFailedException; cancellation raises OperationAbortedException."""

    def __init__(self, nav, task):
        self._nav, self._task = nav, task
        self._seq = -1
        self._final_sent = False
        self._closed = False

    def Next(self):
        if self._closed or self._final_sent:
            raise RR.StopIterationException("")
        task = self._task
        self._seq = task.wait_update(self._seq, 1.0)
        if task.done:
            try:
                result = task.outcome()
            except T.TaskFailed as e:
                self._nav._tasks.finished(task, "failed")
                self._closed = True
                raise RR.OperationFailedException(str(e))
            except T.TaskAborted as e:
                self._nav._tasks.finished(task, "cancelled")
                self._closed = True
                raise RR.OperationAbortedException(str(e))
            self._nav._tasks.finished(task, "complete")
            self._final_sent = True
            return self._item(True, result)
        return self._item(False, None)

    def Abort(self):
        self._closed = True
        self._task.cancel("generator aborted by client")

    def Close(self):
        self._closed = True
        self._task.cancel("generator closed by client")


class NavGenerator(_TaskGenerator):
    def _item(self, complete, result):
        t = self._nav._t
        v = T.nav_feedback_values(self._task, self._nav._ros.robot_pose())
        s = t.struct("NavigationFeedback")
        s.status = t.status["complete" if complete else "running"]
        s.current_pose = t.pose2d(*v["pose"])
        s.distance_remaining = 0.0 if complete else v["distance_remaining"]
        s.navigation_time = v["navigation_time"]
        s.estimated_time_remaining = 0.0 if complete else v["estimated_time_remaining"]
        s.number_of_recoveries = v["number_of_recoveries"]
        s.waypoint_index = v["waypoint_index"]
        s.speed = v["speed"]
        missed = []
        if complete and result is not None and hasattr(result, "missed_waypoints"):
            missed = [int(m.index) for m in result.missed_waypoints]
        s.missed_waypoints = np.asarray(missed, dtype=np.int32)
        return s


class BehaviorGenerator(_TaskGenerator):
    def _item(self, complete, result):
        t = self._nav._t
        s = t.struct("BehaviorFeedback")
        s.status = t.status["complete" if complete else "running"]
        s.progress = T.behavior_progress(self._task)
        s.elapsed_time = self._task.elapsed
        return s


class CostmapObj:
    """Latest Nav2 costmap (nav2_msgs/Costmap on <which>_costmap/costmap_raw)."""

    def __init__(self, which, core: BridgeCore, ros, types: Types):
        self._which, self._core, self._ros, self._t = which, core, ros, types
        self.data = SwapMemory(np.uint8)
        self._info = (0.0, 0, 0, (0.0, 0.0, 0.0), 0.0, 0)  # res, w, h, origin(map), stamp, ver
        self._frame_origin = (0.0, 0.0, 0.0)               # origin in the costmap's own frame
        self._frame = "map"
        self._lock = threading.Lock()

    def _to_map(self, frame, pose):
        if frame == "map":
            return pose
        tf = self._ros.lookup_2d("map", frame)
        return compose(tf, pose) if tf is not None else pose

    def update(self, msg):
        md = msg.metadata
        frame = msg.header.frame_id or "map"
        origin_frame = pose_of_msg(md.origin)
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8).copy()
        with self._lock:
            ver = self._info[5] + 1
            self._frame, self._frame_origin = frame, origin_frame
            self._info = (float(md.resolution), int(md.size_x), int(md.size_y),
                          self._to_map(frame, origin_frame), self._ros.wall(), ver)
            self.data.set(data)

    @property
    def info(self):
        with self._lock:
            return grid_info(self._t, *self._info)

    @rr_errors
    def clear(self):
        if self._core.state not in ("active", "paused"):
            raise StateError("costmaps exist only while the stack is active or paused")
        self._ros.clear_costmap(self._which)

    @rr_errors
    def cost_at(self, x, y):
        if not T.finite_all(x, y):
            raise InputError("x and y must be finite")
        with self._lock:
            res, w, h, _, _, ver = self._info
            frame, (ox, oy, oyaw) = self._frame, self._frame_origin
            data = self.data._a
        if ver == 0:
            raise StateError("costmap not available yet (stack inactive?)")
        px, py = x, y
        if frame != "map":
            tf = self._ros.lookup_2d(frame, "map")
            if tf is None:
                raise StateError(f"no transform between map and {frame}")
            px, py, _ = compose(tf, (x, y, 0.0))
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        lx, ly = c * (px - ox) - s * (py - oy), s * (px - ox) + c * (py - oy)
        i, j = int(math.floor(lx / res)), int(math.floor(ly / res))
        if i < 0 or j < 0 or i >= w or j >= h:
            return 255
        return int(data[j * w + i])


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
        self._tasks = T.TaskManager(ros)
        self._life.cancel_all = lambda: self._tasks.cancel_all("stack paused/reset")
        self._costmaps = {w: CostmapObj(w, self._core, ros, types) for w in ("global", "local")}
        self._stop = threading.Event()
        self._cmd_seq = 0
        self._local_plan_nonempty = False
        self._out_thread = threading.Thread(target=self._output_loop, name="rr-outputs",
                                            daemon=True)

    # ---------------------------------------------------------------- RR plumbing
    def RRServiceObjectInit(self, context, service_path):
        self.odometry.InValueChanged += self._on_odometry
        self.external_pose.InValueChanged += self._on_external_pose
        self.laser_scan_in.PipeConnectCallback = self._scan_connect
        self.point_cloud_in.PipeConnectCallback = self._cloud_connect
        self._ros.output_listeners.append(self._on_ros_output)
        self._out_thread.start()
        log.info("service object initialized at %s", service_path)

    def get_map(self):
        return self._map, NS + "OccupancyMap"

    def get_configuration(self):
        return self._config, NS + "NavConfiguration"

    def shutdown(self):
        self._stop.set()
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

    # ---------------------------------------------------------------- task helpers
    def _require_active(self, need_localized=True):
        st = self._core.state
        if st != "active":
            raise StateError(f"requires stack_state active (state is {st}); call start() first")
        stale = self._stale_inputs()
        if stale:
            raise StateError("inputs are stale: " + ", ".join(stale))
        if need_localized and self._ros.robot_pose() is None:
            raise StateError("robot is not localized (no map->base_link transform yet)")

    def _poses(self, arr, what, lo=1, hi=100):
        a = np.asarray(arr).reshape(-1)
        if not lo <= a.shape[0] <= hi:
            raise InputError(f"{what} must contain {lo}..{hi} poses (got {a.shape[0]})")
        out = [pose2d_tuple(a[i:i + 1]) for i in range(a.shape[0])]
        for p in out:
            check_pose(*p, what=what)
        return out

    def _start_task(self, name, goal, gen_cls):
        try:
            task = self._tasks.start(name, goal)
        except TaskFailed as e:
            raise RR.OperationFailedException(str(e))
        return gen_cls(self, task)

    # ---------------------------------------------------------------- navigation generators
    @rr_errors
    def navigate_to_pose(self, goal):
        self._require_active()
        p = self._poses(goal, "goal", 1, 1)
        return self._start_task("navigate_to_pose", T.nav_goal("navigate_to_pose", p), NavGenerator)

    @rr_errors
    def navigate_through_poses(self, poses):
        self._require_active()
        p = self._poses(poses, "poses")
        return self._start_task("navigate_through_poses",
                                T.nav_goal("navigate_through_poses", p), NavGenerator)

    @rr_errors
    def follow_waypoints(self, waypoints):
        self._require_active()
        p = self._poses(waypoints, "waypoints")
        return self._start_task("follow_waypoints", T.nav_goal("follow_waypoints", p),
                                NavGenerator)

    @rr_errors
    def follow_path(self, path):
        self._require_active()
        p = self._poses(path.poses, "path.poses", 2, 20000)
        return self._start_task("follow_path", T.follow_path_goal(p), NavGenerator)

    # ---------------------------------------------------------------- behavior generators
    @rr_errors
    def spin(self, target_yaw, time_allowance):
        self._require_active(need_localized=False)
        if not T.finite_all(target_yaw, time_allowance) or abs(target_yaw) > 6 * math.pi:
            raise InputError("target_yaw must be finite with |target_yaw| <= 6*pi")
        _check_allowance(time_allowance)
        return self._start_task("spin", T.behavior_goal("spin", target_yaw=target_yaw,
                                                        time_allowance=time_allowance),
                                BehaviorGenerator)

    def _straight(self, name, distance, speed, time_allowance):
        self._require_active(need_localized=False)
        if not T.finite_all(distance, speed) or not 0 < distance <= 100.0:
            raise InputError("distance must be within (0, 100] m")
        vmax = max(self._core.config.max_linear_velocity, self._core.config.max_reverse_velocity)
        if not 0 < speed <= max(vmax, 0.05):
            raise InputError(f"speed must be within (0, {max(vmax, 0.05):g}] m/s "
                             f"(configuration velocity limit)")
        _check_allowance(time_allowance)
        return self._start_task(name, T.behavior_goal(name, distance=distance, speed=speed,
                                                      time_allowance=time_allowance),
                                BehaviorGenerator)

    @rr_errors
    def backup(self, distance, speed, time_allowance):
        return self._straight("backup", distance, speed, time_allowance)

    @rr_errors
    def drive_on_heading(self, distance, speed, time_allowance):
        return self._straight("drive_on_heading", distance, speed, time_allowance)

    @rr_errors
    def wait(self, duration):
        self._require_active(need_localized=False)
        if not T.finite_all(duration) or not 0 < duration <= 3600:
            raise InputError("duration must be within (0, 3600] s")
        return self._start_task("wait", T.behavior_goal("wait", duration=duration),
                                BehaviorGenerator)

    # ---------------------------------------------------------------- planning
    def _plan(self, name, goal):
        try:
            res = self._tasks.plan(name, goal)
        except (T.TaskFailed, T.TaskAborted) as e:
            raise RR.OperationFailedException(str(e))
        poses = path_poses(res.path)
        self._publish_plan("global", poses)
        return self._t.path2d(poses, self._ros.wall())

    @rr_errors
    def compute_path_to_pose(self, goal):
        self._require_active()
        return self._plan("compute_path_to_pose",
                          T.compute_to_pose_goal(self._poses(goal, "goal", 1, 1)[0]))

    @rr_errors
    def compute_path_from_pose(self, start, goal):
        self._require_active(need_localized=False)
        s = self._poses(start, "start", 1, 1)[0]
        g = self._poses(goal, "goal", 1, 1)[0]
        return self._plan("compute_path_to_pose", T.compute_to_pose_goal(g, start=s))

    @rr_errors
    def compute_path_through_poses(self, goals):
        self._require_active()
        return self._plan("compute_path_through_poses",
                          T.compute_through_goal(self._poses(goals, "goals")))

    # ---------------------------------------------------------------- misc
    @rr_errors
    def cancel_all(self):
        self._tasks.cancel_all("cancel_all() called")

    @rr_errors
    def clear_costmaps(self):
        if self._core.state not in ("active", "paused"):
            raise StateError("costmaps exist only while the stack is active or paused")
        for which in ("global", "local"):
            self._ros.clear_costmap(which)

    def get_global_costmap(self):
        return self._costmaps["global"], NS + "Costmap"

    def get_local_costmap(self):
        return self._costmaps["local"], NS + "Costmap"

    # ---------------------------------------------------------------- outputs
    def _publish_plan(self, which, poses):
        wire = self.global_plan if which == "global" else self.local_plan
        try:
            wire.OutValue = self._t.path2d(poses, self._ros.wall())
        except Exception:
            log.debug("plan wire publish failed", exc_info=True)

    def _on_ros_output(self, kind, value):
        if kind == "global_plan":
            self._publish_plan("global", value[0])
        elif kind == "local_plan":
            if self._tasks.running() is not None:
                self._publish_plan("local", value[0])
        elif kind in ("global_costmap", "local_costmap"):
            self._costmaps[kind.split("_")[0]].update(value)

    def _stale_inputs(self) -> list:
        cfg, ages = self._core.config, self._ros.input_ages()
        out = []
        if ages["odometry"] > cfg.odometry_timeout:
            out.append(f"odometry {_age(ages['odometry'])} old (limit {cfg.odometry_timeout:g} s)")
        if ages["sensors"] > cfg.sensor_timeout:
            out.append(f"laser/point cloud {_age(ages['sensors'])} old "
                       f"(limit {cfg.sensor_timeout:g} s)")
        if cfg.localization_mode == "external" and ages["external_pose"] > EXT_POSE_TIMEOUT:
            out.append(f"external_pose {_age(ages['external_pose'])} old "
                       f"(limit {EXT_POSE_TIMEOUT:g} s)")
        return out

    def _output_loop(self):
        period, tick = 0.05, 0
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self._output_tick(tick)
            except Exception:
                log.exception("output loop error")
            tick += 1
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def _output_tick(self, tick):
        ros, core = self._ros, self._core
        running = self._tasks.running()
        stale = self._stale_inputs() if core.state == "active" else []
        # Watchdog: stale inputs abort every running task.
        if running is not None and stale:
            reason = "watchdog: " + "; ".join(stale)
            core.warn("watchdog", reason + " - all tasks aborted, cmd_vel zeroed")
            threading.Thread(target=self._tasks.cancel_all, args=(reason,), daemon=True).start()
            running = None
        # cmd_vel: forward the final Nav2 command only while a task runs and inputs are fresh.
        vx = vy = wz = 0.0
        with ros._lock:
            cv = ros.cmd_vel
        if running is not None and not stale and cv is not None and time.monotonic() - cv[3] < 0.5:
            vx, vy, wz = cv[0], cv[1], cv[2]
        self._cmd_seq += 1
        cmd = self._t.struct("CmdVel")
        cmd.velocity = self._t.velocity(vx, vy, wz)
        cmd.stamp = ros.wall()
        cmd.active = running is not None
        cmd.seqno = self._cmd_seq
        self.cmd_vel.OutValue = cmd
        pose = ros.robot_pose()
        if pose is not None:
            self.robot_pose.OutValue = self._t.pose2d(*pose)
        if running is None and self._local_plan_nonempty:
            self._publish_plan("local", [])
        self._local_plan_nonempty = running is not None
        if tick % 4 == 0:
            self.nav_status.OutValue = self._status(pose, running, stale)

    def _status(self, pose, running, stale):
        core, t = self._core, self._t
        s = t.struct("NavStatus")
        s.stack_state = t.consts["NavStackState"][core.state]
        s.active_task = running.name if running is not None else ""
        name = "running" if running is not None else self._tasks.last_status
        s.task_status = t.status.get(name, 0)
        s.robot_pose = t.pose2d(*(pose or (0.0, 0.0, 0.0)))
        s.localized = pose is not None
        if running is not None:
            v = T.nav_feedback_values(running, pose)
            s.distance_remaining = v["distance_remaining"]
            s.estimated_time_remaining = v["estimated_time_remaining"]
        ages = self._ros.input_ages()
        s.odometry_ok = ages["odometry"] <= core.config.odometry_timeout
        s.sensors_ok = ages["sensors"] <= core.config.sensor_timeout
        s.map_ok = core.map.version > 0
        s.message = core.message
        s.stamp = self._ros.wall()
        return s

