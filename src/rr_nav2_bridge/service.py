"""RR service objects for experimental.nav2_bridge."""
from __future__ import annotations

import logging
import threading

import RobotRaconteur as RR

from .convert import Types
from .docs import DocIndex

log = logging.getLogger("rr_nav2_bridge.service")


def _not_yet(name):
    def f(self, *args):
        raise RR.NotImplementedException(f"{name} is not implemented yet")
    f.__name__ = name
    return f


class Nav2Navigator:
    """Root object. Members are filled in by the later bridge steps."""

    def __init__(self, ros, types: Types, robdef_text: str):
        self._ros = ros
        self._t = types
        self._lock = threading.RLock()
        self._docs = DocIndex(robdef_text)

    def documentation(self, name):
        try:
            return self._docs.lookup(name)
        except KeyError as e:
            raise RR.InvalidArgumentException(str(e.args[0]))

    # Called by RR once the wires/pipes exist.
    def RRServiceObjectInit(self, context, service_path):
        log.info("service object initialized at %s", service_path)

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
    initialize_localization = _not_yet("initialize_localization")
    start = _not_yet("start")
    pause = _not_yet("pause")
    reset = _not_yet("reset")
