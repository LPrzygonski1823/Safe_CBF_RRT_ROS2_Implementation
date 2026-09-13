Note on Original Source:
This repository is built on top of the husarion/rosbot-xl-autonomy project.
The original codebase is licensed under the Apache License 2.0 by Husarion.

Modifications:
The primary addition to this fork is the src/cbf_rrt_planner package and related tools, which implement a Safe-CBF-RRT* path planner for the engineering thesis purposes. The original Husarion license has been preserved in the LICENSE file.

## Terminal commands

`just` recipes work from any directory inside the repository. Compose commands assume the
repository root, where `$COMPOSE` stands for
`docker compose -f compose.simulation.yaml -f compose.cbf.yaml`.

### Everyday workflow

| Command | Description |
|---|---|
| `just` | List all available recipes. |
| `just cbf-up` | Build the planner, start the whole stack, follow its logs. |
| `just cbf-restart` | Rebuild the planner and recreate only its container. |
| `just cbf-build` | Build the `cbf_rrt_planner` package only. |
| `just cbf-refresh-map` | Freeze the current SLAM map and regenerate the PSF. |
| `just cbf-goal X Y` | Send a planning goal, e.g. `just cbf-goal 2.5 -1.0`. |
| `just cbf-logs` | Follow the planner logs. |
| `just cbf-shell` | Interactive ROS 2 shell on the stack network. |
| `just cbf-down` | Stop the whole stack. |

`cbf-up` and `cbf-restart` end by following logs, so Ctrl-C only stops the log stream - the
stack keeps running. Rebuilding is required after editing anything under `src/`, including
`config/cbf_rrt_planner.yaml`, because colcon copies it into `install/`.

### Stack inspection

| Command | Description |
|---|---|
| `$COMPOSE ps -a` | Container states, including ones that already exited. |
| `$COMPOSE logs -f navigation` | Nav2 logs - the real reason behind a `FollowPath` abort. |
| `$COMPOSE up -d --force-recreate --no-deps navigation` | Restart Nav2 after editing `config/nav2_*_params.yaml`. |
| `$COMPOSE up -d --force-recreate --no-deps cbf-planner` | Restart the planner without rebuilding. |
| `cat src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv` | Per-run planning metrics. |

### ROS 2 introspection

Run these from `just cbf-shell`, or prefix them with
`docker exec rosbot-xl-autonomy-foxglove-cbf-planner-1 bash -c 'source /opt/ros/humble/setup.bash && ...'`.

| Command | Description |
|---|---|
| `ros2 topic list` | List all topics. |
| `ros2 topic info /odom` | Publisher and subscriber counts of a topic. |
| `ros2 topic echo /cmd_vel_nav --csv` | Controller output, before the velocity smoother. |
| `ros2 topic echo /cmd_vel --csv` | Velocity actually sent to the base. |
| `ros2 topic echo /lookahead_point --csv` | RPP carrot in `base_link`; `atan2(y,x)` is the tracked heading error. |
| `ros2 topic echo /received_global_plan` | Plan as RPP received it - use to check path density. |
| `ros2 topic echo /odometry/filtered --csv` | Fused odometry used as the planning start. |
| `ros2 topic echo /planned_path` | Path published by the planner. |
| `ros2 topic hz /clock` | Verify Gazebo publishes simulated time. |
| `ros2 param get /planner_node use_sim_time` | Verify the planner runs on simulated time. |
| `ros2 param get /controller_server FollowPath.lookahead_dist` | Read a live Nav2 parameter. |
| `ros2 service call /refresh_map std_srvs/srv/Trigger` | Regenerate the PSF. |
| `ros2 topic pub --once /cbf_goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'` | Send a goal manually. |
| `ros2 run tf2_tools view_frames` | Dump the TF tree to a PDF. |