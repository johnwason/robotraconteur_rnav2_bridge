"""ROS 2 side of the bridge: one rclpy node spun by a MultiThreadedExecutor on a
background thread. RR handler threads never spin rclpy; they call the thread-safe
publish helpers here or wait on rclpy futures with timeouts.

Frames: map -> odom -> base_link -> laser. All ROS names stay inside this module.
"""
from __future__ import annotations

import logging
import math
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as RosDuration
from geometry_msgs.msg import (Point, PoseStamped, PoseWithCovarianceStamped, Quaternion,
                               TransformStamped, TwistStamped)
from nav2_msgs.action import (BackUp, ComputePathThroughPoses, ComputePathToPose,
                              DriveOnHeading, FollowPath, FollowWaypoints,
                              NavigateThroughPoses, NavigateToPose, Spin, Wait)
from nav2_msgs.msg import Costmap as RosCostmap
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid as RosGrid
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan as RosScan
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from .convert import quat_to_yaw, wrap_angle, yaw_to_quat

log = logging.getLogger("rr_nav2_bridge.ros")

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
LATEST_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

ACTIONS = {
    "navigate_to_pose": (NavigateToPose, "navigate_to_pose"),
    "navigate_through_poses": (NavigateThroughPoses, "navigate_through_poses"),
    "follow_waypoints": (FollowWaypoints, "follow_waypoints"),
    "follow_path": (FollowPath, "follow_path"),
    "compute_path_to_pose": (ComputePathToPose, "compute_path_to_pose"),
    "compute_path_through_poses": (ComputePathThroughPoses, "compute_path_through_poses"),
    "spin": (Spin, "spin"),
    "backup": (BackUp, "backup"),
    "drive_on_heading": (DriveOnHeading, "drive_on_heading"),
    "wait": (Wait, "wait"),
}


def compose(a, b):
    """2D transform composition a*b, each (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap_angle(-a[2]))


def duration_msg(seconds: float) -> RosDuration:
    sec = int(math.floor(seconds))
    return RosDuration(sec=sec, nanosec=int((seconds - sec) * 1e9))


def duration_s(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def pose_stamped(frame: str, x: float, y: float, yaw: float, stamp=None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    if stamp is not None:
        ps.header.stamp = stamp
    ps.pose.position.x, ps.pose.position.y = float(x), float(y)
    qx, qy, qz, qw = yaw_to_quat(yaw)
    ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return ps


def pose_of(ps) -> tuple:
    p, q = ps.pose.position, ps.pose.orientation
    return float(p.x), float(p.y), quat_to_yaw(q.x, q.y, q.z, q.w)


def path_poses(path: Path) -> list:
    return [pose_of(ps) for ps in path.poses]


class RosSide:
    def __init__(self, node_name: str = "rr_nav2_bridge"):
        rclpy.init(args=None)
        self.node = Node(node_name)
        self.cb = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name="rclpy-executor", daemon=True)
        self._lock = threading.Lock()
        n = self.node

        # ---- inputs
        self.odom_pub = n.create_publisher(Odometry, "/odom", SENSOR_QOS)
        self.scan_pub = n.create_publisher(RosScan, "/scan", SENSOR_QOS)
        self.points_pub = n.create_publisher(PointCloud2, "/points", SENSOR_QOS)
        self.map_pub = n.create_publisher(RosGrid, "/map", MAP_QOS)
        self.initialpose_pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.tf_pub = TransformBroadcaster(n)
        self.static_tf_pub = StaticTransformBroadcaster(n)

        # latest input state (guarded by _lock)
        self.last_odom_wall = 0.0       # time.monotonic() of last odometry
        self.last_sensor_wall = 0.0     # time.monotonic() of last scan / cloud
        self.last_ext_pose_wall = 0.0
        self.last_odom_stamp = None     # rclpy Time used for the odom TF
        self.odom_pose = None           # (x, y, yaw) of base_link in odom
        self.map_to_odom = None         # (x, y, yaw), external localization mode
        self.publish_map_to_odom = True

        # ---- outputs
        self.cmd_vel = None             # (vx, vy, wz, monotonic time)
        self.global_plan = None         # (poses, monotonic time)
        self.local_plan = None
        self.costmaps = {"global": None, "local": None}  # RosCostmap
        self.output_listeners = []      # callables(kind, value)
        n.create_subscription(TwistStamped, "/cmd_vel", self._on_cmd_vel, LATEST_QOS,
                              callback_group=self.cb)
        n.create_subscription(Path, "/plan", lambda m: self._on_plan("global", m), LATEST_QOS,
                              callback_group=self.cb)
        for topic in ("/local_plan", "/transformed_global_plan"):
            n.create_subscription(Path, topic, lambda m: self._on_plan("local", m), LATEST_QOS,
                                  callback_group=self.cb)
        for which in ("global", "local"):
            n.create_subscription(RosCostmap, f"/{which}_costmap/costmap_raw",
                                  lambda m, w=which: self._on_costmap(w, m), LATEST_QOS,
                                  callback_group=self.cb)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, n, spin_thread=False)

        # ---- actions and services
        self.actions = {k: ActionClient(n, t, name, callback_group=self.cb)
                        for k, (t, name) in ACTIONS.items()}
        self.manage = {
            "navigation": n.create_client(ManageLifecycleNodes,
                                          "/lifecycle_manager_navigation/manage_nodes",
                                          callback_group=self.cb),
            "localization": n.create_client(ManageLifecycleNodes,
                                            "/lifecycle_manager_localization/manage_nodes",
                                            callback_group=self.cb),
        }
        self.clear_srv = {
            "global": n.create_client(ClearEntireCostmap,
                                      "/global_costmap/clear_entirely_global_costmap",
                                      callback_group=self.cb),
            "local": n.create_client(ClearEntireCostmap,
                                     "/local_costmap/clear_entirely_local_costmap",
                                     callback_group=self.cb),
        }
        n.create_timer(0.05, self._tick_tf, callback_group=self.cb)
        self.set_laser_mount(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        self._thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # pragma: no cover
            log.exception("rclpy executor stopped")

    def shutdown(self):
        try:
            self.executor.shutdown(timeout_sec=2.0)
        finally:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    # ------------------------------------------------------------------ helpers
    def now(self) -> Time:
        return self.node.get_clock().now()

    @staticmethod
    def wall() -> float:
        """Service timestamp (UNIX seconds) reported to RR clients."""
        return time.time()

    @staticmethod
    def wait_future(fut, timeout: float):
        """Block (on an RR thread) until an rclpy future completes; TimeoutError if not."""
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            raise TimeoutError()
        return fut.result()

    def _sensor_stamp(self):
        """Stamp sensor data with the latest odometry time so TF lookups never need to
        extrapolate into the future (data is stamped on arrival, not by the client)."""
        with self._lock:
            st = self.last_odom_stamp
            fresh = (time.monotonic() - self.last_odom_wall) < 0.5
        return st.to_msg() if (st is not None and fresh) else self.now().to_msg()

    def _tf(self, parent, child, x, y, yaw, stamp) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y = float(x), float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return t

    # ------------------------------------------------------------------ inputs
    def publish_odometry(self, x, y, yaw, vx, vy, wz):
        now = self.now()
        stamp = now.to_msg()
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        m.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        m.twist.twist.linear.x, m.twist.twist.linear.y = float(vx), float(vy)
        m.twist.twist.angular.z = float(wz)
        with self._lock:
            self.odom_pose = (float(x), float(y), float(yaw))
            self.last_odom_stamp = now
            self.last_odom_wall = time.monotonic()
        self.odom_pub.publish(m)
        self.tf_pub.sendTransform(self._tf("odom", "base_link", x, y, yaw, stamp))

    def publish_scan(self, angle_min, angle_max, angle_increment, time_increment, scan_time,
                     range_min, range_max, ranges, intensities):
        m = RosScan()
        m.header.stamp = self._sensor_stamp()
        m.header.frame_id = "laser"
        m.angle_min, m.angle_max = float(angle_min), float(angle_max)
        m.angle_increment = float(angle_increment)
        m.time_increment, m.scan_time = float(time_increment), float(scan_time)
        m.range_min, m.range_max = float(range_min), float(range_max)
        m.ranges = np.asarray(ranges, dtype=np.float32).tolist()
        if len(intensities):
            m.intensities = np.asarray(intensities, dtype=np.float32).tolist()
        with self._lock:
            self.last_sensor_wall = time.monotonic()
        self.scan_pub.publish(m)

    def publish_points(self, xyz: np.ndarray):
        """xyz: (N, 3) float32 points in base_link."""
        xyz = np.ascontiguousarray(xyz, dtype=np.float32)
        m = PointCloud2()
        m.header.stamp = self._sensor_stamp()
        m.header.frame_id = "base_link"
        m.height, m.width = 1, int(xyz.shape[0])
        m.fields = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, c in enumerate("xyz")]
        m.is_bigendian = False
        m.point_step, m.row_step = 12, 12 * m.width
        m.is_dense = True
        m.data = xyz.tobytes()
        with self._lock:
            self.last_sensor_wall = time.monotonic()
        self.points_pub.publish(m)

    def publish_map(self, resolution, width, height, ox, oy, oyaw, data: np.ndarray):
        m = RosGrid()
        m.header.stamp = self.now().to_msg()
        m.header.frame_id = "map"
        m.info.map_load_time = m.header.stamp
        m.info.resolution = float(resolution)
        m.info.width, m.info.height = int(width), int(height)
        m.info.origin.position.x, m.info.origin.position.y = float(ox), float(oy)
        qx, qy, qz, qw = yaw_to_quat(oyaw)
        m.info.origin.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        m.data = np.asarray(data, dtype=np.int8).tolist()
        self.map_pub.publish(m)

    def set_laser_mount(self, x, y, yaw):
        self.static_tf_pub.sendTransform(
            self._tf("base_link", "laser", x, y, yaw, self.now().to_msg()))

    def set_external_pose(self, x, y, yaw) -> bool:
        """Robot pose in map (external localization). Returns False if no odometry yet."""
        with self._lock:
            if self.odom_pose is None:
                return False
            self.map_to_odom = compose((x, y, yaw), inverse(self.odom_pose))
            self.last_ext_pose_wall = time.monotonic()
        return True

    def publish_initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.now().to_msg()
        m.header.frame_id = "map"
        m.pose.pose = pose_stamped("map", x, y, yaw).pose
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.07
        m.pose.covariance = cov
        self.initialpose_pub.publish(m)

    def _tick_tf(self):
        with self._lock:
            mo = self.map_to_odom if self.publish_map_to_odom else None
        if mo is not None:
            stamp = (self.now() + Duration(seconds=0.1)).to_msg()  # future-dated like AMCL
            self.tf_pub.sendTransform(self._tf("map", "odom", *mo, stamp))

    def input_ages(self) -> dict:
        now = time.monotonic()
        with self._lock:
            def age(t):
                return now - t if t > 0 else math.inf
            return {"odometry": age(self.last_odom_wall), "sensors": age(self.last_sensor_wall),
                    "external_pose": age(self.last_ext_pose_wall)}

    # ------------------------------------------------------------------ outputs
    def _notify(self, kind, value):
        for f in list(self.output_listeners):
            try:
                f(kind, value)
            except Exception:
                log.exception("output listener failed")

    def _on_cmd_vel(self, m: TwistStamped):
        v = (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z, time.monotonic())
        with self._lock:
            self.cmd_vel = v
        self._notify("cmd_vel", v)

    def _on_plan(self, which, m: Path):
        v = (path_poses(m), time.monotonic())
        with self._lock:
            if which == "global":
                self.global_plan = v
            else:
                self.local_plan = v
        self._notify(f"{which}_plan", v)

    def _on_costmap(self, which, m: RosCostmap):
        with self._lock:
            self.costmaps[which] = m
        self._notify(f"{which}_costmap", m)

    def robot_pose(self):
        """(x, y, yaw) of base_link in map, or None."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return float(tr.x), float(tr.y), quat_to_yaw(q.x, q.y, q.z, q.w)

    # ------------------------------------------------------------------ services
    def call_service(self, client, request, timeout: float):
        if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
            raise TimeoutError(f"service {client.srv_name} not available")
        return self.wait_future(client.call_async(request), timeout)

    def lifecycle(self, manager: str, command: int, timeout: float = 90.0) -> bool:
        req = ManageLifecycleNodes.Request()
        req.command = command
        return bool(self.call_service(self.manage[manager], req, timeout).success)

    def clear_costmap(self, which: str, timeout: float = 5.0):
        self.call_service(self.clear_srv[which], ClearEntireCostmap.Request(), timeout)


__all__ = ["RosSide", "ACTIONS", "pose_stamped", "pose_of", "path_poses", "duration_msg",
           "duration_s", "Point", "ManageLifecycleNodes"]
