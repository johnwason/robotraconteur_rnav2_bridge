#!/usr/bin/env python3
"""Unit test: configuration -> Nav2 parameter rendering. Runs inside the image:
    ./scripts/oneshot.sh python3 /opt/rr_nav2/tests/test_params.py
"""
import math

from rr_nav2_bridge import params as P


def test_defaults():
    p = P.render_params(P.NavConfig())
    fp = p["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["plugin"] == "nav2_mppi_controller::MPPIController"
    assert fp["vx_max"] == 0.5 and fp["vx_min"] == 0.0 and fp["motion_model"] == "DiffDrive"
    for cm in ("local_costmap", "global_costmap"):
        q = p[cm][cm]["ros__parameters"]
        assert q["robot_radius"] == 0.22 and "footprint" not in q
        assert q["inflation_layer"]["inflation_radius"] == 0.35
    assert p["bt_navigator"]["ros__parameters"]["global_frame"] == "map"
    assert p["lifecycle_manager_navigation"]["ros__parameters"]["autostart"] is False
    for node in ("controller_server", "velocity_smoother", "collision_monitor", "behavior_server"):
        assert p[node]["ros__parameters"]["enable_stamped_cmd_vel"] is True, node
    vs = p["velocity_smoother"]["ros__parameters"]
    assert vs["min_velocity"][0] == -0.5, "smoother must allow backup() reverse"
    assert p["amcl"]["ros__parameters"]["set_initial_pose"] is False


def test_dwb_holonomic_footprint():
    cfg = P.NavConfig(controller="dwb", holonomic=True, max_linear_velocity=0.8,
                      max_reverse_velocity=0.2, footprint=[(0.3, 0.2), (-0.3, 0.2),
                                                           (-0.3, -0.2), (0.3, -0.2)],
                      goal_xy_tolerance=0.1)
    p = P.render_params(cfg)
    fp = p["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["plugin"] == "dwb_core::DWBLocalPlanner"
    assert fp["max_vel_x"] == 0.8 and fp["min_vel_x"] == -0.2 and fp["max_vel_y"] == 0.8
    assert fp["vy_samples"] == 5 and fp["xy_goal_tolerance"] == 0.1
    q = p["global_costmap"]["global_costmap"]["ros__parameters"]
    assert "robot_radius" not in q and q["footprint"].startswith("[[0.3000, 0.2000]")
    assert p["amcl"]["ros__parameters"]["robot_model_type"] == "nav2_amcl::OmniMotionModel"


def test_amcl_initial_pose():
    p = P.render_params(P.NavConfig(localization_mode="amcl"), initial_pose=(1.0, 2.0, 0.5))
    a = p["amcl"]["ros__parameters"]
    assert a["set_initial_pose"] is True and a["initial_pose"]["yaw"] == 0.5


def test_validation():
    for name, bad in (("robot_radius", 0.0), ("max_linear_velocity", math.nan),
                      ("odometry_timeout", 10.0)):
        try:
            P.check_float(name, bad)
        except ValueError:
            continue
        raise AssertionError(f"{name}={bad} accepted")
    for bad in ([(0, 0), (1, 1)], [(0, 0), (1, 0), (2, 0)], [(0, 0), (9, 0), (0, 1)]):
        try:
            P.check_footprint(bad)
        except ValueError:
            continue
        raise AssertionError(f"footprint {bad} accepted")


if __name__ == "__main__":
    for name, f in list(globals().items()):
        if name.startswith("test_"):
            f()
            print("ok", name)
