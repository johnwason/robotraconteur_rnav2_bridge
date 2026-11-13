#!/usr/bin/env python3
"""Hardening tests against the running test container (scripts/test.sh hardening).

Checks the container's effective security settings and limits, network exposure, and
that the bridge survives malformed input and floods. Uses read-only podman queries
(inspect/exec/stats) on the project-labeled container only.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
import traceback

import numpy as np
from RobotRaconteur.Client import RRN

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_robot import URL, FakeRobot  # noqa: E402

NAME = os.environ.get("RR_NAV2_NAME", "rr-nav2-test")
PORT = int(os.environ.get("RR_NAV2_PORT", "62354"))
LABEL = "project=rr_nav2_bridge"
RRN.RequestTimeout = 60


def sh(*args, check=True) -> str:
    return subprocess.run(args, check=check, capture_output=True, text=True).stdout


def inspect() -> dict:
    out = json.loads(sh("podman", "inspect", NAME))[0]
    assert out["Config"]["Labels"].get("project") == "rr_nav2_bridge", "not a project container"
    return out


def pexec(*cmd, check=False):
    return subprocess.run(["podman", "exec", NAME, *cmd], capture_output=True, text=True,
                          check=check)


# ------------------------------------------------------------------ container config
def test_limits():
    hc = inspect()["HostConfig"]
    assert hc["Memory"] == 58 * 1024 ** 3, hc["Memory"]
    assert hc["MemorySwap"] == 58 * 1024 ** 3, hc["MemorySwap"]
    assert hc["NanoCpus"] == 10 * 10 ** 9, hc["NanoCpus"]
    assert hc["PidsLimit"] == 512, hc["PidsLimit"]


def test_privileges():
    info = inspect()
    hc = info["HostConfig"]
    assert hc["Privileged"] is False
    assert hc["ReadonlyRootfs"] is True
    assert any("no-new-privileges" in o for o in hc["SecurityOpt"] or []), hc["SecurityOpt"]
    assert not hc.get("CapAdd"), hc.get("CapAdd")
    assert hc["NetworkMode"] not in ("host",), hc["NetworkMode"]
    assert hc["PidMode"] in ("", "private"), hc["PidMode"]
    assert hc["IpcMode"] in ("", "private", "shareable"), hc["IpcMode"]
    assert not hc.get("Devices"), hc.get("Devices")
    # Effective capabilities of the bridge process are empty.
    caps = pexec("grep", "CapEff", "/proc/1/status").stdout + \
        pexec("sh", "-c", "grep CapEff /proc/$(pgrep -f rr_nav2_bridge | head -1)/status").stdout
    for line in caps.splitlines():
        assert line.split()[1] == "0000000000000000", line
    assert pexec("id", "-un").stdout.strip() == "nav"
    # Root filesystem is read-only; tmpfs dirs are writable.
    assert pexec("touch", "/usr/rr_nav2_probe").returncode != 0, "rootfs writable"
    assert pexec("touch", "/opt/rr_nav2/src/probe").returncode != 0, "source writable"
    assert pexec("touch", "/tmp/probe").returncode == 0
    # No mounts of host home, podman socket or /dev.
    for m in info["Mounts"]:
        src = m.get("Source", "")
        assert not src.startswith(os.path.expanduser("~/.ssh")), m
        assert "podman.sock" not in src and src not in ("/", "/dev", os.path.expanduser("~")), m


def test_port_bindings():
    ports = inspect()["HostConfig"]["PortBindings"]
    assert list(ports) == [f"{PORT}/tcp"], ports
    for b in ports[f"{PORT}/tcp"]:
        assert b["HostIp"] == "127.0.0.1", b


def _host_ips():
    out = sh("ip", "-4", "-o", "addr", "show", "scope", "global", check=False)
    return [line.split()[3].split("/")[0] for line in out.splitlines()]


def test_not_reachable_off_loopback():
    with socket.create_connection(("127.0.0.1", PORT), timeout=2):
        pass
    ips = _host_ips()
    assert ips, "no non-loopback host address to test"
    for ip in ips:
        s = socket.socket()
        s.settimeout(2)
        try:
            rc = s.connect_ex((ip, PORT))
        finally:
            s.close()
        assert rc != 0, f"RR port reachable on {ip}"


# ------------------------------------------------------------------ robustness
def test_malformed_input_survival():
    r = FakeRobot(URL)
    c = r.connect()
    c.reset()
    m = c.get_map()
    bad = r.grid_struct()
    bad.data = np.zeros(7, dtype=np.int8)
    for f in (lambda: m.load_map(bad),
              lambda: c.navigate_to_pose(r.pose2d(math.nan, 0, 0)),
              lambda: c.follow_waypoints(r.poses([(0, 0, 0)] * 500)),
              lambda: setattr(c.get_configuration(), "robot_radius", -1.0),
              lambda: c.documentation("x" * 10000)):
        try:
            f()
        except Exception as e:
            assert "Invalid" in type(e).__name__, type(e).__name__
        else:
            raise AssertionError("malformed call accepted")
    # In-stream garbage: NaN odometry, broken scans.
    o = RRN.NewStructure("experimental.nav2_bridge.Odometry2D", c)
    o.pose = r.pose2d(math.inf, math.nan, 0)
    r.odom_w.OutValue = o
    s = RRN.NewStructure("com.robotraconteur.laserscan.LaserScan", c)
    i = RRN.NewStructure("com.robotraconteur.laserscan.LaserScanInfo", c)
    i.angle_min, i.angle_max, i.angle_increment = 0.0, math.nan, 0.0
    s.scan_info = i
    s.ranges = np.full(3, math.nan)
    r.scan_p.SendPacket(s)
    time.sleep(0.5)
    assert c.documentation("start").startswith("Nav2Navigator.start")
    r.stop()


def _bridge_rss_mb() -> float:
    out = pexec("sh", "-c", "ps -o rss= -p $(pgrep -f 'python3 -m rr_nav2_bridge' | head -1)")
    return float(out.stdout.strip() or 0) / 1024.0


def test_flood_and_reconnect():
    rss0 = _bridge_rss_mb()
    r = FakeRobot(URL)
    c = r.connect()
    s = RRN.NewStructure("com.robotraconteur.laserscan.LaserScan", c)
    i = RRN.NewStructure("com.robotraconteur.laserscan.LaserScanInfo", c)
    i.angle_min, i.angle_increment, i.range_min, i.range_max = -1.0, 0.0002, 0.1, 10.0
    i.angle_max = i.angle_min + 9999 * i.angle_increment
    s.scan_info = i
    s.ranges = np.ones(10000)
    t0 = time.time()
    for _ in range(300):  # max-size scans as fast as RR flow control allows
        r.scan_p.SendPacket(s)
    flood_s = time.time() - t0
    big = RRN.NewStructure("com.robotraconteur.laserscan.LaserScan", c)
    big.scan_info = i
    big.ranges = np.ones(50000)  # over the cap: rejected, not processed
    for _ in range(20):
        r.scan_p.SendPacket(big)
    r.stop()
    for _ in range(20):  # rapid connect/disconnect
        cc = RRN.ConnectService(URL)
        RRN.DisconnectService(cc)
    time.sleep(2.0)
    c = RRN.ConnectService(URL)
    assert c.documentation("reset").startswith("Nav2Navigator.reset")
    RRN.DisconnectService(c)
    rss1 = _bridge_rss_mb()
    print(f"    flood: 300 x 10000-range scans in {flood_s:.1f}s; bridge RSS "
          f"{rss0:.0f} -> {rss1:.0f} MB")
    assert rss1 < rss0 + 300, "bridge memory grew by > 300 MB"


TESTS = [test_limits, test_privileges, test_port_bindings, test_not_reachable_off_loopback,
         test_malformed_input_survival, test_flood_and_reconnect]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="")
    args = ap.parse_args()
    failed = 0
    for t in TESTS:
        if args.k not in t.__name__:
            continue
        t0 = time.time()
        try:
            t()
            print(f"PASS {t.__name__} ({time.time() - t0:.1f}s)", flush=True)
        except Exception as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}", flush=True)
            traceback.print_exc(limit=3)
    print("ALL PASSED" if not failed else f"{failed} FAILED")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
