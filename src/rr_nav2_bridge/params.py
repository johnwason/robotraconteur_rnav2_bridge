"""Navigation configuration model and Nav2 parameter-file generation.

The RR configuration object stores a NavConfig; on start() the bridge renders it into a
complete Nav2 params YAML (defaults from config/nav2_params.yaml + the chosen controller
block) written to a tmpfs. ROS parameter names never leave this module.
"""
from __future__ import annotations

import copy
import math
import os
from dataclasses import asdict, dataclass, field, fields

import yaml

CONFIG_DIR = os.environ.get("RR_NAV2_CONFIG_DIR", "/opt/rr_nav2/config")

CONTROLLERS = ("mppi", "dwb")
LOCALIZATION_MODES = ("external", "amcl")


@dataclass
class NavConfig:
    robot_radius: float = 0.22
    footprint: list = field(default_factory=list)  # [(x, y), ...] body frame
    max_linear_velocity: float = 0.5
    max_reverse_velocity: float = 0.0
    max_angular_velocity: float = 1.5
    max_linear_acceleration: float = 1.0
    max_angular_acceleration: float = 3.0
    holonomic: bool = False
    controller: str = "mppi"
    localization_mode: str = "external"
    laser_mount: tuple = (0.0, 0.0, 0.0)  # (x, y, yaw)
    inflation_radius: float = 0.35
    goal_xy_tolerance: float = 0.25
    goal_yaw_tolerance: float = 0.25
    odometry_timeout: float = 0.5
    sensor_timeout: float = 1.0

    def as_dict(self) -> dict:
        d = asdict(self)
        d["footprint"] = [list(p) for p in self.footprint]
        d["laser_mount"] = list(self.laser_mount)
        return d


# name -> (min, max) for float properties (inclusive)
FLOAT_RANGES = {
    "robot_radius": (0.05, 3.0),
    "max_linear_velocity": (0.01, 5.0),
    "max_reverse_velocity": (0.0, 5.0),
    "max_angular_velocity": (0.05, 10.0),
    "max_linear_acceleration": (0.05, 10.0),
    "max_angular_acceleration": (0.05, 20.0),
    "inflation_radius": (0.0, 5.0),
    "goal_xy_tolerance": (0.01, 2.0),
    "goal_yaw_tolerance": (0.01, 3.15),
    "odometry_timeout": (0.1, 5.0),
    "sensor_timeout": (0.1, 5.0),
}

FIELD_NAMES = tuple(f.name for f in fields(NavConfig))


def check_float(name: str, value) -> float:
    """Validate a float property; raises ValueError with a client-facing message."""
    v = float(value)
    lo, hi = FLOAT_RANGES[name]
    if not math.isfinite(v) or v < lo or v > hi:
        raise ValueError(f"{name} must be within [{lo}, {hi}], got {value}")
    return v


def check_footprint(points) -> list:
    pts = [(float(x), float(y)) for x, y in points]
    if not pts:
        return []
    if not 3 <= len(pts) <= 64:
        raise ValueError("footprint must have 3..64 points (or be empty to use robot_radius)")
    for x, y in pts:
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > 5.0 or abs(y) > 5.0:
            raise ValueError("footprint points must be finite and within 5 m of the robot center")
    # shoelace area; require non-degenerate polygon
    area = 0.5 * sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))
    if abs(area) < 1e-4:
        raise ValueError("footprint polygon has (near) zero area")
    return pts


def check_laser_mount(x: float, y: float, yaw: float) -> tuple:
    vals = (float(x), float(y), float(yaw))
    if not all(math.isfinite(v) for v in vals) or abs(vals[0]) > 5 or abs(vals[1]) > 5:
        raise ValueError("laser_mount must be finite with |x|,|y| <= 5 m")
    return vals


def _load(name: str) -> dict:
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return yaml.safe_load(f)


def _footprint_str(pts) -> str:
    return "[" + ", ".join(f"[{x:.4f}, {y:.4f}]" for x, y in pts) + "]"


def render_params(cfg: NavConfig, initial_pose=None) -> dict:
    """Return the full Nav2 parameter dict for this configuration."""
    p = copy.deepcopy(_load("nav2_params.yaml"))
    ctrl = copy.deepcopy(_load(f"controller_{cfg.controller}.yaml"))["FollowPath"]

    vx, vrev, wz = cfg.max_linear_velocity, cfg.max_reverse_velocity, cfg.max_angular_velocity
    ax, aw = cfg.max_linear_acceleration, cfg.max_angular_acceleration
    vy = vx if cfg.holonomic else 0.0
    ay = ax if cfg.holonomic else 0.0

    if cfg.controller == "mppi":
        ctrl.update(vx_max=vx, vx_min=-vrev, vy_max=max(vy, 0.0), wz_max=wz,
                    ax_max=ax, ax_min=-ax, ay_max=max(ay, 0.0), az_max=aw,
                    motion_model="Omni" if cfg.holonomic else "DiffDrive")
        ctrl["CostCritic"]["consider_footprint"] = bool(cfg.footprint)
        if vrev <= 0.0:
            ctrl["PreferForwardCritic"]["enabled"] = True
    else:
        ctrl.update(min_vel_x=-vrev, max_vel_x=vx, min_vel_y=-vy, max_vel_y=vy,
                    max_vel_theta=wz, max_speed_xy=max(vx, vy), acc_lim_x=ax,
                    acc_lim_y=ay, acc_lim_theta=aw, decel_lim_x=-ax, decel_lim_y=-ay,
                    decel_lim_theta=-aw, vy_samples=5 if cfg.holonomic else 1,
                    xy_goal_tolerance=cfg.goal_xy_tolerance)

    cs = p["controller_server"]["ros__parameters"]
    cs["FollowPath"] = ctrl
    cs["general_goal_checker"]["xy_goal_tolerance"] = cfg.goal_xy_tolerance
    cs["general_goal_checker"]["yaw_goal_tolerance"] = cfg.goal_yaw_tolerance
    if cfg.holonomic:
        cs["min_y_velocity_threshold"] = 0.001

    for outer in ("local_costmap", "global_costmap"):
        cm = p[outer][outer]["ros__parameters"]
        cm["inflation_layer"]["inflation_radius"] = max(cfg.inflation_radius, 0.0)
        if cfg.footprint:
            cm.pop("robot_radius", None)
            cm["footprint"] = _footprint_str(cfg.footprint)
        else:
            cm["robot_radius"] = cfg.robot_radius
    # Keep the local window comfortably larger than the robot.
    extent = max([cfg.robot_radius] + [math.hypot(x, y) for x, y in cfg.footprint])
    lc = p["local_costmap"]["local_costmap"]["ros__parameters"]
    lc["width"] = lc["height"] = max(3, int(math.ceil(extent * 2 + 2.5)))

    vs = p["velocity_smoother"]["ros__parameters"]
    vs["max_velocity"] = [vx, vy, wz]
    # The smoother sits after both the controller and the behaviors; it must let backup()
    # reverse even when the controller may not (max_reverse_velocity limits the controller).
    vs["min_velocity"] = [-max(vrev, vx), -vy, -wz]
    vs["max_accel"] = [ax, ay, aw]
    vs["max_decel"] = [-ax, -ay, -aw]

    bs = p["behavior_server"]["ros__parameters"]
    bs["max_rotational_vel"] = wz
    bs["min_rotational_vel"] = min(0.4, wz)
    bs["rotational_acc_lim"] = aw

    am = p["amcl"]["ros__parameters"]
    am["robot_model_type"] = ("nav2_amcl::OmniMotionModel" if cfg.holonomic
                              else "nav2_amcl::DifferentialMotionModel")
    if initial_pose is not None:
        x, y, yaw = initial_pose
        am["set_initial_pose"] = True
        am["initial_pose"] = {"x": float(x), "y": float(y), "z": 0.0, "yaw": float(yaw)}
    return p


def write_params(cfg: NavConfig, path: str, initial_pose=None) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(render_params(cfg, initial_pose), f, sort_keys=False)
    return path
