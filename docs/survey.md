# Survey: standard RR types vs ROS 2 Jazzy `nav2_msgs`

Verified 2026-11-13 against `RobotRaconteurCompanion==0.4.0` (StdRobDef, stdver 0.10)
and `ros-jazzy-navigation2 1.3.13` inside the project image.

## Standard `com.robotraconteur.*` types that exist and are used

| Need | Standard type | Notes |
|---|---|---|
| 2D pose (goals, robot pose, map origin) | `geometry.Pose2D` (namedarray: `orientation` yaw rad, `position` Point2D) | Nav2 is planar; Pose2D avoids quaternions for clients/LLMs |
| 2D point (footprint) | `geometry.Point2D` | |
| Velocity (odom twist, cmd_vel) | `geometry.SpatialVelocity` (`angular` Vector3, `linear` Vector3) | Only linear.x/y, angular.z used |
| Laser scan input | `laserscan.LaserScan` (+ `LaserScanInfo`) | Maps 1:1 onto `sensor_msgs/LaserScan`; frame comes from configuration (mount pose) |
| Point cloud input | `pointcloud.PointCloudf` | Has no header/frame; we define points as base-frame |
| Task status | `action.ActionStatusCode` (error, failed, cancelled, unknown, queued, running, complete) | Used in all feedback structs |

## Not available in the standard robdefs (custom structs in `experimental.nav2_bridge`)

- **Occupancy grid** – no standard type. Custom `OccupancyGrid` (resolution, width, height, origin Pose2D, int8[] data) mirroring `nav_msgs/OccupancyGrid`.
- **Path** – no standard type (`robotics.trajectory` is joint-space). Custom `Path2D` (Pose2D[] + stamp + length).
- **Odometry** – no standard type. Custom `Odometry2D` (Pose2D + SpatialVelocity).
- **Stamped cmd_vel** – custom `CmdVel` (SpatialVelocity + service stamp + active flag + seqno).
- **Costmap metadata** – custom `GridInfo`; cells exposed via `memory`.

`sensordata.SensorDataHeader` uses `TimeSpec2` + `Identifier`; it is heavy for clients and
the bridge stamps on arrival anyway, so custom structs carry a plain `double stamp`.

## Robdef docstrings

RR 1.2.9 accepts `##` comment lines *preceding* a service, struct, field, object or member
and stores them in `DocString`. Trailing `##` after a member on the same line is a parse
error. `ServiceDefinition.ToString()` drops docstrings, so the bridge must register the
original text (verified in step 7).

## Jazzy `nav2_msgs` actions → RR mapping

| Nav2 action | Goal | Feedback | Result / errors | RR member |
|---|---|---|---|---|
| NavigateToPose | pose, behavior_tree | current_pose, navigation_time, estimated_time_remaining, number_of_recoveries, distance_remaining | error_code, error_msg | `navigate_to_pose(Pose2D)` → `NavigationFeedback{generator}` |
| NavigateThroughPoses | poses[], behavior_tree | + number_of_poses_remaining | error_code, error_msg | `navigate_through_poses(Pose2D[])` |
| FollowWaypoints | number_of_loops, goal_index, poses[] | current_waypoint | missed_waypoints[], error_code (600s) | `follow_waypoints(Pose2D[])` |
| FollowPath | path, controller_id, goal_checker_id, progress_checker_id | distance_to_goal, speed | error_code (100s) | `follow_path(Path2D)` |
| ComputePathToPose | goal, start, planner_id, use_start | – | path, planning_time, error_code (200s) | `compute_path_to_pose`, `compute_path_from_pose` (functions) |
| ComputePathThroughPoses | goals[], start, planner_id, use_start | – | path, planning_time, error_code (300s) | `compute_path_through_poses` (function) |
| Spin | target_yaw, time_allowance | angular_distance_traveled | total_elapsed_time, error_code (700s) | `spin(double, double)` → `BehaviorFeedback{generator}` |
| BackUp | target (Point, x<0 not needed: distance), speed, time_allowance | distance_traveled | error_code (710s) | `backup(double, double, double)` |
| DriveOnHeading | target, speed, time_allowance | distance_traveled | error_code (720s) | `drive_on_heading(double, double, double)` |
| Wait | time | time_left | total_elapsed_time | `wait(double)` |

Error codes and `error_msg` are folded into the RR exception message
(`OperationFailedException: <task> failed: <ERROR_NAME> (<code>): <error_msg>`).

## Services used internally

- `nav2_msgs/srv/ManageLifecycleNodes`: STARTUP, PAUSE, RESUME, RESET, SHUTDOWN, CONFIGURE, CLEANUP → `start`/`pause`/`reset`.
- `nav2_msgs/srv/ClearEntireCostmap` → `clear_costmaps`, `*_costmap.clear()`.
- `nav2_msgs/srv/SetInitialPose` is AMCL-only; we publish `/initialpose` instead (works for AMCL in all modes).

## Packages present in Jazzy 1.3.13

amcl, behaviors, bt_navigator, collision_monitor, controller, dwb_controller, lifecycle_manager,
map_server, mppi_controller, planner, route, smoother, velocity_smoother, waypoint_follower.

**Correction to HANDOFF:** `nav2_route` *is* shipped in this Jazzy release. It remains out of
scope for the prototype.

## Other findings

- System numpy is 1.26.4 (apt); the RR 1.2.9 wheel imports fine against it.
- The `navigation2` metapackage pulls in rviz/LLVM (image 3.13 GB). Step 8 installs the
  needed packages explicitly instead.
- Podman 4.9 `--tmpfs` has no `uid=` option; `U` (chown to container user) is used.
