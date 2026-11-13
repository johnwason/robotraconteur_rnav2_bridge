"""Nav2 action goals as tasks. At most one *motion* task runs at a time: starting a new one
cancels (preempts) the previous one. Planning requests run independently."""
from __future__ import annotations

import logging
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav_msgs.msg import Path

from .ros_side import ACTIONS, duration_msg, duration_s, pose_of, pose_stamped

log = logging.getLogger("rr_nav2_bridge.tasks")


def _error_names() -> dict:
    names = {}
    for typ, _ in ACTIONS.values():
        for k in dir(typ.Result):
            v = getattr(typ.Result, k)
            if k.isupper() and isinstance(v, int) and v != 0:
                names[v] = k
    return names


ERROR_NAMES = _error_names()


class TaskFailed(Exception):
    """Nav2 aborted the goal (maps to OperationFailedException)."""


class TaskAborted(Exception):
    """Goal cancelled by Abort()/Close(), cancel_all(), preemption or the watchdog."""


class ActionTask:
    """One Nav2 action goal; thread-safe view of its feedback and result."""

    def __init__(self, ros, name: str, goal, accept_timeout: float = 10.0):
        self.ros, self.name = ros, name
        self.t0 = time.monotonic()
        self._cv = threading.Condition()
        self.feedback = None
        self.fb_seq = 0
        self.status = None          # GoalStatus.* once finished
        self.result = None
        self.cancel_reason = None
        client = ros.actions[name]
        if not client.wait_for_server(timeout_sec=5.0):
            raise TaskFailed(f"{name}: action server not available")
        fut = client.send_goal_async(goal, feedback_callback=self._on_feedback)
        try:
            self.handle = ros.wait_future(fut, accept_timeout)
        except TimeoutError:
            raise TaskFailed(f"{name}: goal was not acknowledged within {accept_timeout:g} s")
        if not self.handle.accepted:
            raise TaskFailed(f"{name}: goal rejected by the navigation stack")
        self.handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        with self._cv:
            self.feedback = msg.feedback
            self.fb_seq += 1
            self._cv.notify_all()

    def _on_result(self, fut):
        try:
            r = fut.result()
            status, result = r.status, r.result
        except Exception as e:  # pragma: no cover
            status, result = GoalStatus.STATUS_ABORTED, None
            log.warning("%s: result error %s", self.name, e)
        with self._cv:
            self.status, self.result = status, result
            self._cv.notify_all()

    @property
    def done(self) -> bool:
        return self.status is not None

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def wait_update(self, last_seq: int, timeout: float) -> int:
        with self._cv:
            self._cv.wait_for(lambda: self.fb_seq != last_seq or self.done, timeout)
            return self.fb_seq

    def wait_done(self, timeout: float) -> bool:
        with self._cv:
            return self._cv.wait_for(lambda: self.done, timeout)

    def cancel(self, reason: str, wait: float = 3.0):
        with self._cv:
            if self.done:
                return
            if self.cancel_reason is None:
                self.cancel_reason = reason
        try:
            self.handle.cancel_goal_async()
        except Exception:  # pragma: no cover
            log.exception("cancel failed")
        self.wait_done(wait)

    def outcome(self):
        """Raise TaskFailed/TaskAborted for unsuccessful completion; return result."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return self.result
        if self.status == GoalStatus.STATUS_CANCELED or self.cancel_reason:
            raise TaskAborted(f"{self.name} cancelled: {self.cancel_reason or 'cancelled'}")
        code = getattr(self.result, "error_code", 0) if self.result is not None else 0
        msg = getattr(self.result, "error_msg", "") if self.result is not None else ""
        name = ERROR_NAMES.get(code, "UNKNOWN")
        text = f"{self.name} failed: {name} ({code})"
        if msg:
            text += f": {msg}"
        raise TaskFailed(text)


class TaskManager:
    def __init__(self, ros):
        self.ros = ros
        self._lock = threading.Lock()
        self.current: ActionTask | None = None
        self.last_name = ""
        self.last_status = "unknown"   # ActionStatusCode name of the latest task

    # ---- motion tasks
    def start(self, name: str, goal) -> ActionTask:
        with self._lock:
            prev = self.current
        if prev is not None and not prev.done:
            prev.cancel(f"preempted by {name}")
        task = ActionTask(self.ros, name, goal)
        with self._lock:
            self.current = task
            self.last_name, self.last_status = name, "running"
        return task

    def finished(self, task: ActionTask, status: str):
        with self._lock:
            if self.current is task:
                self.last_status = status
                self.current = None

    def cancel_all(self, reason: str = "cancel_all() called"):
        with self._lock:
            task = self.current
        if task is not None and not task.done:
            task.cancel(reason)
        with self._lock:
            if self.current is task and task is not None:
                self.current = None
                self.last_status = "cancelled"

    def running(self):
        with self._lock:
            t = self.current
        return t if (t is not None and not t.done) else None

    # ---- planning (no motion)
    def plan(self, name: str, goal, timeout: float = 30.0):
        task = ActionTask(self.ros, name, goal)
        if not task.wait_done(timeout):
            task.cancel("planning timed out")
            raise TaskFailed(f"{name} failed: planning did not finish within {timeout:g} s")
        return task.outcome()


# ------------------------------------------------------------------ goal builders
def nav_goal(name, poses):
    typ = ACTIONS[name][0]
    g = typ.Goal()
    if name == "navigate_to_pose":
        g.pose = pose_stamped("map", *poses[0])
    elif name == "navigate_through_poses":
        g.poses = [pose_stamped("map", *p) for p in poses]
    elif name == "follow_waypoints":
        g.poses = [pose_stamped("map", *p) for p in poses]
    return g


def follow_path_goal(poses):
    g = ACTIONS["follow_path"][0].Goal()
    g.path = make_path(poses)
    g.controller_id = "FollowPath"
    g.goal_checker_id = "general_goal_checker"
    g.progress_checker_id = "progress_checker"
    return g


def make_path(poses) -> Path:
    p = Path()
    p.header.frame_id = "map"
    p.poses = [pose_stamped("map", *q) for q in poses]
    return p


def compute_to_pose_goal(goal, start=None):
    g = ACTIONS["compute_path_to_pose"][0].Goal()
    g.goal = pose_stamped("map", *goal)
    g.planner_id = "GridBased"
    if start is not None:
        g.start = pose_stamped("map", *start)
        g.use_start = True
    return g


def compute_through_goal(goals):
    g = ACTIONS["compute_path_through_poses"][0].Goal()
    g.goals = [pose_stamped("map", *p) for p in goals]
    g.planner_id = "GridBased"
    return g


def behavior_goal(name, **kw):
    g = ACTIONS[name][0].Goal()
    if name == "spin":
        g.target_yaw = float(kw["target_yaw"])
        g.time_allowance = duration_msg(kw["time_allowance"])
    elif name in ("backup", "drive_on_heading"):
        g.target = Point(x=abs(float(kw["distance"])), y=0.0, z=0.0)
        g.speed = abs(float(kw["speed"]))
        g.time_allowance = duration_msg(kw["time_allowance"])
    elif name == "wait":
        g.time = duration_msg(kw["duration"])
    return g


# ------------------------------------------------------------------ feedback extraction
def nav_feedback_values(task: ActionTask, robot_pose) -> dict:
    """Common NavigationFeedback values from the latest Nav2 feedback."""
    fb = task.feedback
    v = dict(pose=robot_pose or (0.0, 0.0, 0.0), distance_remaining=0.0,
             navigation_time=task.elapsed, estimated_time_remaining=0.0,
             number_of_recoveries=0, waypoint_index=0, speed=0.0)
    if fb is None:
        return v
    if hasattr(fb, "current_pose") and fb.current_pose.header.frame_id:
        v["pose"] = pose_of(fb.current_pose)
    if hasattr(fb, "distance_remaining"):
        v["distance_remaining"] = float(fb.distance_remaining)
    if hasattr(fb, "navigation_time"):
        v["navigation_time"] = duration_s(fb.navigation_time)
    if hasattr(fb, "estimated_time_remaining"):
        v["estimated_time_remaining"] = duration_s(fb.estimated_time_remaining)
    if hasattr(fb, "number_of_recoveries"):
        v["number_of_recoveries"] = int(fb.number_of_recoveries)
    if hasattr(fb, "number_of_poses_remaining"):
        v["waypoint_index"] = int(fb.number_of_poses_remaining)
    if hasattr(fb, "current_waypoint"):
        v["waypoint_index"] = int(fb.current_waypoint)
    if hasattr(fb, "distance_to_goal"):
        v["distance_remaining"] = float(fb.distance_to_goal)
    if hasattr(fb, "speed"):
        v["speed"] = float(fb.speed)
    return v


def behavior_progress(task: ActionTask) -> float:
    fb = task.feedback
    if fb is None:
        return 0.0
    if hasattr(fb, "angular_distance_traveled"):
        return abs(float(fb.angular_distance_traveled))
    if hasattr(fb, "distance_traveled"):
        return abs(float(fb.distance_traveled))
    if hasattr(fb, "time_left"):
        return max(0.0, duration_s(fb.time_left))
    return 0.0


def finite_all(*vals) -> bool:
    return all(math.isfinite(float(v)) for v in vals)
