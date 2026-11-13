"""Launch the Nav2 servers used by the bridge (no map_server, autostart off).

Started and stopped by the bridge on start()/reset(); not meant to be run by users.
Arguments:
  params_file  generated parameter file (written by the bridge into a tmpfs)
  use_amcl     'true' to also launch AMCL and its lifecycle manager

Velocity chain:
  controller_server / behavior_server -> cmd_vel_nav -> velocity_smoother
  -> cmd_vel_smoothed -> collision_monitor -> /cmd_vel (TwistStamped, read by the bridge)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    use_amcl = LaunchConfiguration("use_amcl")
    log_level = LaunchConfiguration("log_level")
    common = dict(output="screen", respawn=False, parameters=[params],
                  arguments=["--ros-args", "--log-level", log_level])
    nav_remap = [("cmd_vel", "cmd_vel_nav")]

    nav_nodes = [
        Node(package="nav2_controller", executable="controller_server",
             remappings=nav_remap, **common),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", **common),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", remappings=nav_remap, **common),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", **common),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", **common),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother",
             remappings=[("cmd_vel", "cmd_vel_nav")], **common),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", **common),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", **common),
    ]
    amcl_nodes = GroupAction(condition=IfCondition(use_amcl), actions=[
        Node(package="nav2_amcl", executable="amcl", name="amcl", **common),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", **common),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("use_amcl", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        *nav_nodes,
        amcl_nodes,
    ])
