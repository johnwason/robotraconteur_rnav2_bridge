#!/usr/bin/env python3
"""Fake robot RR client (host side, no ROS): unicycle model, ray-cast laser scans,
built-in test map. Streams odometry / scans / external pose to the bridge and applies
the bridge's cmd_vel to the simulated robot.

Run with the project venv:  .venv/bin/python -I tests/fake_robot.py [--url URL]
"""
from __future__ import annotations

import argparse
import math
import threading
import time

import numpy as np
from RobotRaconteur.Client import RRN

URL = "rr+tcp://127.0.0.1:62354?service=nav2"
RES = 0.05


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def make_test_map(width_m=10.0, height_m=8.0, res=RES) -> np.ndarray:
    """int8 grid (rows = y). Outer walls, an inner wall with a doorway, two boxes."""
    w, h = int(round(width_m / res)), int(round(height_m / res))
    g = np.zeros((h, w), dtype=np.int8)
    t = 2  # wall thickness in cells
    g[:t, :] = g[-t:, :] = 100
    g[:, :t] = g[:, -t:] = 100
    # inner wall at x = 5 m with a 1.2 m doorway centered at y = 2 m
    xw = int(5.0 / res)
    g[:, xw:xw + t] = 100
    d0, d1 = int(1.4 / res), int(2.6 / res)
    g[d0:d1, xw:xw + t] = 0
    # boxes
    g[int(5.0 / res):int(6.0 / res), int(2.0 / res):int(3.0 / res)] = 100
    g[int(1.0 / res):int(1.6 / res), int(7.5 / res):int(8.5 / res)] = 100
    return g


class Sim:
    """Ground truth unicycle (or holonomic) robot with first-order velocity response."""

    def __init__(self, grid, res=RES, start=(1.5, 1.5, 0.0), radius=0.22):
        self.grid, self.res = grid, res
        self.x, self.y, self.yaw = start
        self.start = start
        self.v = np.zeros(3)   # vx, vy, wz (body)
        self.cmd = np.zeros(3)
        self.radius = radius
        self.collided = False
        self.lock = threading.Lock()

    def occupied(self, x, y) -> bool:
        i, j = int(math.floor(y / self.res)), int(math.floor(x / self.res))
        if i < 0 or j < 0 or i >= self.grid.shape[0] or j >= self.grid.shape[1]:
            return True
        return self.grid[i, j] >= 65

    def step(self, dt):
        with self.lock:
            a = min(1.0, dt / 0.1)
            self.v += a * (self.cmd - self.v)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            nx = self.x + (c * self.v[0] - s * self.v[1]) * dt
            ny = self.y + (s * self.v[0] + c * self.v[1]) * dt
            # crude collision check on the robot circle
            hit = any(self.occupied(nx + self.radius * math.cos(t), ny + self.radius * math.sin(t))
                      for t in np.linspace(0, 2 * math.pi, 16, endpoint=False))
            if hit:
                self.collided = True
                self.v[:2] = 0
            else:
                self.x, self.y = nx, ny
            self.yaw = wrap(self.yaw + self.v[2] * dt)

    def pose(self):
        with self.lock:
            return self.x, self.y, self.yaw

    def odom_pose(self):
        """Pose in an odom frame whose origin is the start pose (so map != odom)."""
        sx, sy, syaw = self.start
        x, y, yaw = self.pose()
        c, s = math.cos(-syaw), math.sin(-syaw)
        dx, dy = x - sx, y - sy
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - syaw)

    def scan(self, n=360, rmax=8.0, mount=(0.0, 0.0, 0.0), grid=None):
        """Ray-cast ranges against `grid` (default: the physical world)."""
        grid = self.grid if grid is None else grid
        x, y, yaw = self.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        sx, sy = x + c * mount[0] - s * mount[1], y + s * mount[0] + c * mount[1]
        ang = yaw + mount[2] + np.linspace(-math.pi, math.pi, n, endpoint=False)
        steps = np.arange(0.0, rmax, self.res * 0.5)
        px = sx + np.cos(ang)[:, None] * steps[None, :]
        py = sy + np.sin(ang)[:, None] * steps[None, :]
        i = np.floor(py / self.res).astype(int)
        j = np.floor(px / self.res).astype(int)
        out = (i < 0) | (j < 0) | (i >= grid.shape[0]) | (j >= grid.shape[1])
        occ = np.ones_like(out)
        ok = ~out
        occ[ok] = grid[i[ok], j[ok]] >= 65
        first = np.where(occ.any(axis=1), occ.argmax(axis=1), -1)
        r = np.where(first >= 0, steps[np.clip(first, 0, None)], np.inf)
        return r


def _with_boxes(grid, boxes):
    g = grid.copy()
    for x0, y0, x1, y1 in boxes:
        g[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = 100
    return g


class FakeRobot:
    """unmapped: (x0, y0, x1, y1) boxes in the simulated world, seen by the laser and the
    point cloud, but NOT in the map given to the bridge.
    cloud_only: boxes seen only by the point cloud (e.g. below the laser plane).
    points: also stream a point cloud (body frame) on point_cloud_in."""

    def __init__(self, url=URL, start=(1.5, 1.5, 0.0), external_pose=True, scan_hz=10.0,
                 odom_hz=30.0, unmapped=(), cloud_only=(), points=False):
        self.grid = make_test_map()                                  # what the bridge gets
        self.laser_grid = _with_boxes(self.grid, unmapped)           # what the laser sees
        self.cloud_grid = _with_boxes(self.laser_grid, cloud_only)   # what the cloud sees
        self.sim = Sim(self.cloud_grid, start=start)                 # physics: everything
        self.points = points or bool(cloud_only)
        self.url = url
        self.external = external_pose
        self.scan_hz, self.odom_hz = scan_hz, odom_hz
        self.c = None
        self._stop = threading.Event()
        self.pause_odom = threading.Event()     # tests: simulate stale odometry
        self.pause_scan = threading.Event()
        self.apply_cmd = True
        self.last_cmd = None
        self.threads = []

    # -- RR helpers
    def connect(self, timeout=60.0):
        """Connect and attach all wires/pipes. The whole sequence is retried: right after a
        previous session to the same service disconnects, the new session can lose its
        transport while attaching members (observed with RR 1.2.9)."""
        deadline = time.time() + timeout
        while True:
            try:
                return self._connect_once()
            except Exception:
                try:
                    RRN.DisconnectService(self.c)
                except Exception:
                    pass
                if time.time() > deadline:
                    raise
                time.sleep(1.0)

    def _connect_once(self):
        self.c = c = RRN.ConnectService(self.url)
        c.documentation("start")  # round-trip: the connection really works
        self.pose_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose2D", c)
        self.vel_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialVelocity", c)
        self.odom_w = c.odometry.Connect()
        self.ext_w = c.external_pose.Connect()
        self.scan_p = c.laser_scan_in.Connect(-1)
        self.cmd_w = c.cmd_vel.Connect()
        self.cloud_p = c.point_cloud_in.Connect(-1) if self.points else None
        if self.points:
            self.pointf_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometryf.Point", c)
        return c

    def pose2d(self, x, y, yaw):
        a = np.zeros((1,), dtype=self.pose_dt)
        a[0]["position"]["x"], a[0]["position"]["y"], a[0]["orientation"] = x, y, yaw
        return a

    def poses(self, pts):
        a = np.zeros((len(pts),), dtype=self.pose_dt)
        for k, (x, y, yaw) in enumerate(pts):
            a[k]["position"]["x"], a[k]["position"]["y"], a[k]["orientation"] = x, y, yaw
        return a

    def grid_struct(self):
        g = RRN.NewStructure("experimental.nav2_bridge.OccupancyGrid", self.c)
        g.resolution = RES
        g.height, g.width = self.grid.shape
        g.origin = self.pose2d(0.0, 0.0, 0.0)
        g.data = self.grid.reshape(-1).copy()
        return g

    # -- loops
    def _odom_loop(self):
        period = 1.0 / self.odom_hz
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            self.sim.step(now - last)
            last = now
            if not self.pause_odom.is_set():
                o = RRN.NewStructure("experimental.nav2_bridge.Odometry2D", self.c)
                o.pose = self.pose2d(*self.sim.odom_pose())
                v = np.zeros((1,), dtype=self.vel_dt)
                v[0]["linear"]["x"], v[0]["linear"]["y"] = self.sim.v[0], self.sim.v[1]
                v[0]["angular"]["z"] = self.sim.v[2]
                o.velocity = v
                try:
                    self.odom_w.OutValue = o
                    if self.external:
                        self.ext_w.OutValue = self.pose2d(*self.sim.pose())
                except Exception:
                    pass
            self._read_cmd()
            time.sleep(max(0.0, period - (time.monotonic() - now)))

    def _read_cmd(self):
        try:
            ok, cmd, _ = self.cmd_w.TryGetInValue()
        except Exception:
            ok = False
        if not ok:
            return
        self.last_cmd = cmd
        v = np.asarray(cmd.velocity).reshape(-1)[0]
        fresh = (time.time() - cmd.stamp) < 0.5
        if self.apply_cmd and cmd.active and fresh:
            self.sim.cmd = np.array([v["linear"]["x"], v["linear"]["y"], v["angular"]["z"]])
        else:
            self.sim.cmd = np.zeros(3)

    def _scan_loop(self):
        period = 1.0 / self.scan_hz
        n = 360
        while not self._stop.is_set():
            t0 = time.monotonic()
            if not self.pause_scan.is_set():
                s = RRN.NewStructure("com.robotraconteur.laserscan.LaserScan", self.c)
                i = RRN.NewStructure("com.robotraconteur.laserscan.LaserScanInfo", self.c)
                i.angle_min, i.angle_increment = -math.pi, 2 * math.pi / n
                i.angle_max = i.angle_min + (n - 1) * i.angle_increment
                i.angle_count = n
                i.range_min, i.range_max = 0.05, 8.0
                i.scan_time = period
                s.scan_info = i
                s.ranges = self.sim.scan(n, grid=self.laser_grid)
                s.intensities = np.zeros((0,))
                try:
                    self.scan_p.AsyncSendPacket(s, lambda *a: None)
                    if self.cloud_p is not None:
                        cr = self.sim.scan(n, grid=self.cloud_grid)
                        self.cloud_p.AsyncSendPacket(self._cloud(cr, i), lambda *a: None)
                except Exception:
                    pass
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _cloud(self, ranges, info):
        """Body-frame points at 0.1/0.3/0.6 m height on every laser hit."""
        ang = info.angle_min + np.arange(len(ranges)) * info.angle_increment
        ok = np.isfinite(ranges)
        x, y = (np.cos(ang) * ranges)[ok], (np.sin(ang) * ranges)[ok]
        pts = np.zeros((3 * len(x),), dtype=self.pointf_dt)
        for k, z in enumerate((0.1, 0.3, 0.6)):
            sl = slice(k * len(x), (k + 1) * len(x))
            pts["x"][sl], pts["y"][sl], pts["z"][sl] = x, y, z
        pc = RRN.NewStructure("com.robotraconteur.pointcloud.PointCloudf", self.c)
        pc.points = pts
        pc.is_dense = True
        return pc

    def start_streaming(self):
        for f in (self._odom_loop, self._scan_loop):
            t = threading.Thread(target=f, daemon=True)
            t.start()
            self.threads.append(t)

    def stop(self):
        self._stop.set()
        for t in self.threads:
            t.join(timeout=2)
        try:
            RRN.DisconnectService(self.c)
        except Exception:
            pass
        time.sleep(0.5)  # let the transport close before a new session to the same node


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default=URL)
    ap.add_argument("--goal", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                    default=(8.0, 6.0, 0.0))
    args = ap.parse_args()
    r = FakeRobot(args.url)
    c = r.connect()
    c.get_map().load_map(r.grid_struct())
    r.start_streaming()
    time.sleep(2.0)
    print("starting navigation stack...")
    c.start()
    print("navigating to", args.goal)
    gen = c.navigate_to_pose(r.pose2d(*args.goal))
    try:
        while True:
            fb = gen.Next()
            p = np.asarray(fb.current_pose).reshape(-1)[0]
            print(f"status={fb.status} pose=({p['position']['x']:.2f},{p['position']['y']:.2f}) "
                  f"remaining={fb.distance_remaining:.2f} m")
    except Exception as e:
        if "StopIteration" not in type(e).__name__:
            raise
    print("final truth pose:", r.sim.pose(), "collided:", r.sim.collided)
    r.stop()


if __name__ == "__main__":
    main()
