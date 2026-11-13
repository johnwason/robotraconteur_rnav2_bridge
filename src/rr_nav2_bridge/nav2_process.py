"""Supervise the Nav2 launch subprocess (started on start(), stopped on reset())."""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import time

log = logging.getLogger("rr_nav2_bridge.nav2")

LAUNCH_FILE = os.environ.get("RR_NAV2_LAUNCH", "/opt/rr_nav2/launch/nav2.launch.py")
LOG_DIR = os.environ.get("ROS_LOG_DIR", "/home/nav/.ros/log")


class Nav2Process:
    def __init__(self):
        self.proc: subprocess.Popen | None = None
        self.log_path = os.path.join(LOG_DIR, "nav2_launch.log")

    def launch(self, params_path: str, use_amcl: bool, log_level: str = "warn"):
        self.stop()
        os.makedirs(LOG_DIR, exist_ok=True)
        logf = open(self.log_path, "w")  # overwritten on every start; lives in tmpfs
        cmd = ["ros2", "launch", LAUNCH_FILE, f"params_file:={params_path}",
               f"use_amcl:={'true' if use_amcl else 'false'}", f"log_level:={log_level}"]
        log.info("launching Nav2: %s", " ".join(cmd))
        self.proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        logf.close()

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def exit_code(self):
        return None if self.proc is None else self.proc.poll()

    def tail(self, n: int = 5) -> str:
        try:
            with open(self.log_path) as f:
                return "".join(f.readlines()[-n:]).strip()
        except OSError:
            return ""

    def stop(self, timeout: float = 15.0):
        p = self.proc
        if p is None:
            return
        if p.poll() is None:
            pgid = os.getpgid(p.pid)
            for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5.0),
                              (signal.SIGKILL, 5.0)):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                try:
                    p.wait(wait)
                    break
                except subprocess.TimeoutExpired:
                    log.warning("Nav2 did not exit after %s", sig.name)
        # Reap any stragglers left in the process group.
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        self.proc = None
        time.sleep(0.2)
