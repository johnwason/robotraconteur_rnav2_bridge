#!/usr/bin/env python3
"""End-to-end smoke tests against a running bridge (host side, no ROS).

    ./scripts/test.sh            # starts the hardened container, runs this, stops it
    .venv/bin/python -I tests/smoke_test.py [--url URL] [-k substring]
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import traceback

import numpy as np
from RobotRaconteur.Client import RRN

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from fake_robot import URL, FakeRobot  # noqa: E402

RRN.RequestTimeout = 120
GOAL = (8.0, 6.0, 0.0)


# ------------------------------------------------------------------ helpers
def drain(gen, limit_s=150.0):
    """Run a generator to the end. Returns (last_item, exception_or_None)."""
    last, t0 = None, time.time()
    while time.time() - t0 < limit_s:
        try:
            last = gen.Next()
        except Exception as e:
            if "StopIteration" in type(e).__name__:
                return last, None
            return last, e
    gen.Abort()
    return last, TimeoutError("generator did not finish in time")


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Session:
    """One fake robot connected to the bridge, stack started."""

    def __init__(self, url, **robot_kw):
        self.r = FakeRobot(url, **robot_kw)
        self.c = self.r.connect()
        self.c.reset()
        self.c.get_configuration().reset_to_defaults()

    def start(self, configure=None):
        if configure:
            configure(self.c.get_configuration())
        self.c.get_map().load_map(self.r.grid_struct())
        self.r.start_streaming()
        time.sleep(2.0)
        self.c.start()
        time.sleep(1.0)
        return self

    def close(self):
        try:
            self.c.reset()
        finally:
            self.r.stop()


def cmd(s):
    v = s.r.cmd_w.InValue
    return v.active, float(np.asarray(v.velocity).reshape(-1)[0]["linear"]["x"])


# ------------------------------------------------------------------ tests
def test_documentation(url):
    r = FakeRobot(url)
    c = r.connect()
    assert "Expected call order" in c.documentation("")
    assert "generator" in c.documentation("navigate_to_pose")
    assert "Circular robot radius" in c.documentation("configuration.robot_radius")
    r.stop()


def test_start_preconditions(url):
    r = FakeRobot(url)
    c = r.connect()
    c.reset()
    try:
        c.start()
    except Exception as e:
        assert "InvalidOperation" in type(e).__name__ and "no map" in str(e), e
    else:
        raise AssertionError("start() without inputs succeeded")
    r.stop()


def test_planning_only(url):
    s = Session(url).start()
    try:
        p0 = s.r.sim.pose()
        path = s.c.compute_path_to_pose(s.r.pose2d(*GOAL))
        assert len(path.poses) > 50 and 7.0 < path.length < 12.0, path.length
        time.sleep(1.0)
        active, vx = cmd(s)
        assert not active and vx == 0.0, "planning must not command motion"
        assert dist(p0, s.r.sim.pose()) < 0.01, "robot moved during planning"
        try:
            s.c.compute_path_to_pose(s.r.pose2d(50, 50, 0))
        except Exception as e:
            assert "GOAL_OUTSIDE_MAP" in str(e), e
        else:
            raise AssertionError("goal outside the map accepted")
    finally:
        s.close()


def test_navigate_full_loop(url):
    s = Session(url).start()
    try:
        last, err = drain(s.c.navigate_to_pose(s.r.pose2d(*GOAL)))
        assert err is None, f"navigation failed: {err}"
        assert last.status == 3, last.status  # complete
        assert dist(s.r.sim.pose(), GOAL) < 0.35, s.r.sim.pose()
        assert not s.r.sim.collided, "collision"
        time.sleep(0.5)
        assert cmd(s) == (False, 0.0)
    finally:
        s.close()


BOX = (3.4, 1.2, 3.8, 1.9)  # in the approach to the doorway, not in the supplied map


def _avoid(url, **kw):
    s = Session(url, **kw).start()
    try:
        last, err = drain(s.c.navigate_to_pose(s.r.pose2d(*GOAL)))
        assert err is None, f"navigation failed: {err}"
        assert not s.r.sim.collided, "hit the unmapped obstacle"
        assert dist(s.r.sim.pose(), GOAL) < 0.35, s.r.sim.pose()
    finally:
        s.close()


def test_avoid_unmapped_obstacle_laser(url):
    _avoid(url, unmapped=[BOX])


def test_avoid_obstacle_seen_only_by_point_cloud(url):
    _avoid(url, cloud_only=[BOX])


def test_waypoints_and_follow_path(url):
    s = Session(url).start()
    try:
        wps = [(3.0, 3.0, 0.0), (3.5, 1.2, -1.57)]
        last, err = drain(s.c.follow_waypoints(s.r.poses(wps)))
        assert err is None, err
        assert list(last.missed_waypoints) == [], last.missed_waypoints
        assert dist(s.r.sim.pose(), wps[-1]) < 0.35
        path = s.c.compute_path_to_pose(s.r.pose2d(2.0, 3.5, 1.57))
        last, err = drain(s.c.follow_path(path))
        assert err is None, err
        assert dist(s.r.sim.pose(), (2.0, 3.5)) < 0.35
    finally:
        s.close()


def test_behaviors(url):
    s = Session(url, start=(3.0, 3.0, 0.0)).start()
    try:
        y0 = s.r.sim.pose()[2]
        last, err = drain(s.c.spin(1.57, 15.0))
        assert err is None and abs(last.progress - 1.57) < 0.3, (err, last.progress)
        assert abs(abs(math.remainder(s.r.sim.pose()[2] - y0, 2 * math.pi)) - 1.57) < 0.3
        p0 = s.r.sim.pose()
        last, err = drain(s.c.backup(0.3, 0.15, 10.0))
        assert err is None and dist(p0, s.r.sim.pose()) > 0.2, (err, s.r.sim.pose())
        last, err = drain(s.c.wait(1.0))
        assert err is None and last.elapsed_time >= 0.9
    finally:
        s.close()


def test_cancel_and_preempt(url):
    s = Session(url).start()
    try:
        g = s.c.navigate_to_pose(s.r.pose2d(*GOAL))
        for _ in range(3):
            g.Next()
        s.c.cancel_all()
        _, err = drain(g)
        assert err is not None and "OperationAborted" in type(err).__name__, err
        time.sleep(0.5)
        assert cmd(s) == (False, 0.0)
        g1 = s.c.navigate_to_pose(s.r.pose2d(*GOAL))
        g1.Next()
        g2 = s.c.wait(0.5)
        _, err = drain(g1)
        assert err is not None and "preempted" in str(err), err
        drain(g2)
        g3 = s.c.navigate_to_pose(s.r.pose2d(*GOAL))
        g3.Next()
        g3.Abort()
        time.sleep(0.5)
        assert cmd(s)[0] is False
    finally:
        s.close()


def test_pause_resume(url):
    s = Session(url).start()
    try:
        s.c.pause()
        st = s.c.nav_status.PeekInValue()[0]
        try:
            s.c.navigate_to_pose(s.r.pose2d(*GOAL))
        except Exception as e:
            assert "InvalidOperation" in type(e).__name__
        else:
            raise AssertionError("navigation accepted while paused")
        s.c.start()
        assert s.c.compute_path_to_pose(s.r.pose2d(*GOAL)).length > 5
        del st
    finally:
        s.close()


def test_watchdog(url):
    s = Session(url).start()
    try:
        g = s.c.navigate_to_pose(s.r.pose2d(*GOAL))
        for _ in range(3):
            g.Next()
        s.r.pause_odom.set()
        t0 = time.time()
        _, err = drain(g, 10.0)
        assert err is not None and "OperationAborted" in type(err).__name__, err
        assert time.time() - t0 < 3.0
        time.sleep(0.5)
        assert cmd(s) == (False, 0.0)
        msg = s.c.nav_status.PeekInValue()[0].message
        assert "watchdog" in msg, msg
        s.r.pause_odom.clear()
    finally:
        s.close()


def test_dwb_controller(url):
    def dwb(cfg):
        cfg.controller = 1
    s = Session(url).start(dwb)
    try:
        last, err = drain(s.c.navigate_to_pose(s.r.pose2d(3.5, 3.0, 0.0)))
        assert err is None, err
        assert dist(s.r.sim.pose(), (3.5, 3.0)) < 0.35 and not s.r.sim.collided
    finally:
        s.close()


TESTS = [test_documentation, test_start_preconditions, test_planning_only,
         test_navigate_full_loop, test_avoid_unmapped_obstacle_laser,
         test_avoid_obstacle_seen_only_by_point_cloud, test_waypoints_and_follow_path, test_behaviors,
         test_cancel_and_preempt, test_pause_resume, test_watchdog, test_dwb_controller]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=URL)
    ap.add_argument("-k", default="")
    args = ap.parse_args()
    failed = 0
    for t in TESTS:
        if args.k not in t.__name__:
            continue
        t0 = time.time()
        try:
            t(args.url)
            print(f"PASS {t.__name__} ({time.time() - t0:.1f}s)", flush=True)
        except Exception as e:
            failed += 1
            print(f"FAIL {t.__name__} ({time.time() - t0:.1f}s): {e}", flush=True)
            traceback.print_exc(limit=3)
    print("ALL PASSED" if not failed else f"{failed} FAILED")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
