# Simulation guide (Gazebo)

Step-by-step instructions for running Safe-CBF-RRT\* in the Gazebo simulation of the ROSbot XL:
setup, start-up, sending goals, recording maps, running experiments and diagnosing problems.
For the architecture, algorithms, parameters and metrics see [`README.md`](README.md). For the
physical robot see [`REAL_GUIDE.md`](REAL_GUIDE.md).

---

## 1. What runs in simulation

`just cbf-*` recipes drive `docker compose -f compose.simulation.yaml -f compose.cbf.yaml`. Below,
`$COMPOSE` stands for this command, run from the repository root:

```bash
COMPOSE="docker compose -f compose.simulation.yaml -f compose.cbf.yaml"
```

| Container | Role |
|---|---|
| `rosbot` | Gazebo simulation of the ROSbot XL; loads `worlds/custom_world_3.sdf` |
| `navigation` | Nav2: `map_server`, AMCL, `controller_server`, `velocity_smoother`, costmaps |
| `cbf-planner` | `planner_node`: PSF generation + RRT\* + `FollowPath` client |
| `foxglove`, `foxglove-ds` | Web visualisation on `localhost:8080`, bridge on `8765` |
| `static-tf` | Publishes an identity `map → odom` transform (only valid when `SLAM=False`) |

`just` recipes work from any directory inside the repository. Compose commands assume the
repository root.

---

## 2. Prerequisites

- Docker with Compose v2 (`docker compose`).
- [`just`](https://just.systems).
- An X server for the Gazebo window: recipes run `xhost +local:docker`, and
  `compose.simulation.yaml` requires `DISPLAY` to be set.
- GPU: the `rosbot` service uses the NVIDIA runtime (`<<: *gpu-config`). Without an NVIDIA GPU
  and the NVIDIA Container Toolkit, change it to `<<: *cpu-config` in `compose.simulation.yaml`
  (software rendering).
- Run recipes as a normal user, not root.

---

## 3. Configuration

### 3.1 `.env`

| Variable | Value | Notes |
|---|---|---|
| `SLAM` | `False` | Uses the pre-recorded `maps/map.yaml`. With `True`, comment out the `static-tf` service. |
| `CONTROLLER` | `rpp` | Picks which `config/nav2_*_params.yaml` is mounted. |
| `MECANUM` | `True` | Holonomic base in Gazebo. RPP itself never commands `vy`. |
| `SAVE_MAP_PERIOD` | 15 | Map autosave, only when `SLAM=True` (set 0 to disable). |
| `USE_SIM_TIME` | unset or `True` | Leave unset. If it is `False` (left over from robot work), the planner ignores `/clock` and its time comparisons break. |

`LIDAR_BAUDRATE` and `ROBOT_NAMESPACE` are used only on the physical robot.

### 3.2 World selection

Edit **two** lines in `compose.simulation.yaml`, the bind mount and the `world:=` argument:

```yaml
- ./worlds/custom_world_3.sdf:/custom_world_3.sdf
command: ... world:=/custom_world_3.sdf headless:=False
```

Changing the world invalidates `maps/map.pgm`: record a new map first (§5).

---

## 4. Step-by-step: first run

### Step 1. Build and start everything

```bash
just cbf-up
```

This builds the workspace inside the dev container (`colcon build --symlink-install`, Release),
starts Gazebo, Nav2, Foxglove and the planner, recreates the planner container so the freshly
built binary runs, and follows the planner logs. The stack is started detached: Ctrl-C only stops
following the logs, the stack keeps running.

Expected planner log:

```
Planner node initialized, waiting for the map, odometry and goal...
First map received. Awaiting /refresh_map to generate PSF.
```

`TF transform 'odom' -> 'map' failed` at start-up is harmless: it appears before the
`map → odom` transform exists, and is throttled to one line.

### Step 2. Open Foxglove

Open `http://localhost:8080`. In the 3D panel set **Display frame** to `map` and enable `/map`,
`/planned_path`, `/scan` and `/local_costmap/costmap`.

> The 3D panel's "Publish pose" tool publishes on `/goal_pose` by default, which makes Nav2's own
> planner drive the robot and bypasses Safe-CBF-RRT\*. Change its topic to `/cbf_goal_pose`
> (Publish → Pose topic) or send goals from the terminal.

### Step 3. Freeze the map and generate the PSF

```bash
just cbf-refresh-map
```

Wait for `PSF ready (N obstacles). Exported.` in the planner log.

### Step 4. Send a goal

Coordinates are in the `map` frame. They are **not** Gazebo world coordinates: for the bundled
map, `map ≈ world + (0.05, −1.94)` m, because the map recording started at the robot's spawn
point.

```bash
just cbf-goal 2.0 1.5
```

Expected log sequence:

```
Planning SUCCESS: length=... mean_h=... min_h=... rejection_ratio=...
Sent path to controller_server (FollowPath action).
FollowPath: goal accepted, the controller is driving.
FollowPath: robot reached the end of the path.
```

### Step 5. Iterate

| You edited | Do |
|---|---|
| anything under `src/` (C++) | `just cbf-restart`: rebuild + recreate only the planner |
| `src/cbf_rrt_planner/config/cbf_rrt_planner.yaml` or the launch file | `$COMPOSE up -d --force-recreate --no-deps cbf-planner` (symlinked, no rebuild needed) |
| a planner parameter, for this session only | `ros2 param set /planner_node <name> <value>` (§8) |
| `config/nav2_rpp_params.yaml` | `$COMPOSE up -d --force-recreate --no-deps navigation` |
| `.env` or a compose file | `just cbf-down`, then `just cbf-up` |

If a stale CMake cache is suspected, run `just cbf-rebuild`.

### Step 6. Stop

```bash
just cbf-down
```

---

## 5. Recording a new map (new or modified world)

1. Set `SLAM=True` in `.env` and comment out the `static-tf` service in
   `compose.simulation.yaml` (slam_toolbox publishes `map → odom` itself).
2. `just cbf-up`.
3. Drive the robot with the Foxglove **Teleop** panel (publishes `/cmd_vel`) until `/map` covers
   the world. With `SAVE_MAP_PERIOD` > 0 the map is also autosaved periodically.
4. Save the map:

   ```bash
   docker compose exec navigation bash -c "source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f /maps/map --ros-args -p save_map_timeout:=15.0 -p use_sim_time:=true"
   ```

   This writes `maps/map.pgm` and `maps/map.yaml` (the `maps/` directory is mounted).
5. Set `SLAM=False`, uncomment `static-tf`, then `just cbf-down` and `just cbf-up`.
6. `just cbf-refresh-map` to generate the PSF on the new map.

The new `map` frame starts where the robot spawned. `static-tf` is only valid if the robot spawns
in the same place and orientation as during mapping.

---

## 6. Recipe reference (`justfile`)

### 6.1 Everyday workflow

| Command | Description |
|---|---|
| `just` | List all available recipes. |
| `just cbf-up` | Build the planner, start the whole stack, follow its logs. |
| `just cbf-restart` | Rebuild the planner and recreate only its container. |
| `just cbf-build` | Build the workspace (`colcon build --symlink-install`, Release) in the dev container. |
| `just cbf-rebuild` | Wipe `build/ install/ log/` and build from scratch. |
| `just cbf-refresh-map` | Freeze the current map and regenerate the PSF. |
| `just cbf-goal X Y` | Send a planning goal, e.g. `just cbf-goal 2.5 -1.0`. |
| `just cbf-logs` | Follow the planner logs. |
| `just cbf-shell` | Interactive ROS 2 shell on the stack network. |
| `just cbf-down` | Stop the whole stack. |

`cbf-up` and `cbf-restart` end by following logs, so Ctrl-C only stops the log stream: the stack
keeps running.

### 6.2 Original Husarion recipes

These come from the upstream project and run the stack **without** the planner:
`just start-simulation [gazebo|webots]`, `just restart-navigation`. Robot-side upstream recipes
(`start-rosbot`, `flash-firmware`, `sync`, `connect-husarnet`) are described in `REAL_GUIDE.md`.

### 6.3 Stack inspection

| Command | Description |
|---|---|
| `$COMPOSE ps -a` | Container states, including ones that already exited. |
| `$COMPOSE logs -f navigation` | Nav2 logs: the real reason behind a `FollowPath` abort. |
| `$COMPOSE up -d --force-recreate --no-deps navigation` | Restart Nav2 after editing `config/nav2_*_params.yaml`. |
| `$COMPOSE up -d --force-recreate --no-deps cbf-planner` | Restart the planner without rebuilding. |
| `cat src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv` | Per-run planning metrics. |

### 6.4 ROS 2 introspection

Run these from `just cbf-shell` (after `source /opt/ros/humble/setup.bash`), or prefix them with
`docker exec rosbot-xl-autonomy-foxglove-cbf-planner-1 bash -c 'source /opt/ros/humble/setup.bash && ...'`.

| Command | Description |
|---|---|
| `ros2 topic list` | List all topics. |
| `ros2 topic info /odom` | Publisher and subscriber counts of a topic. |
| `ros2 topic echo /cmd_vel_nav --csv` | Controller output, before the velocity smoother. |
| `ros2 topic echo /cmd_vel --csv` | Velocity actually sent to the base. |
| `ros2 topic echo /lookahead_point --csv` | RPP carrot in `base_link`; `atan2(y,x)` is the tracked heading error. |
| `ros2 topic echo /received_global_plan` | Plan as RPP received it: use to check path density. |
| `ros2 topic echo /odometry/filtered --csv` | Fused odometry used as the planning start. |
| `ros2 topic echo /planned_path` | Path published by the planner. |
| `ros2 topic hz /clock` | Verify Gazebo publishes simulated time. |
| `ros2 param get /planner_node use_sim_time` | Verify the planner runs on simulated time. |
| `ros2 param get /controller_server FollowPath.lookahead_dist` | Read a live Nav2 parameter (launch overrides the YAML). |
| `ros2 service call /refresh_map std_srvs/srv/Trigger` | Regenerate the PSF. |
| `ros2 topic pub --once /cbf_goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'` | Send a goal manually. |
| `ros2 run tf2_tools view_frames` | Dump the TF tree to a PDF. |

---

## 7. Running experiments

The paper compares plain RRT\* against Safe-CBF-RRT\* over a range of `c`. Recommended loop:

```bash
docker exec rosbot-xl-autonomy-foxglove-cbf-planner-1 bash -c \
  'source /opt/ros/humble/setup.bash && ros2 param set /planner_node safety_weight_c 0.5'
just cbf-goal -6.98 -3.38
```

- For the baseline set `enable_cbf: false` (`ros2 param set /planner_node enable_cbf false`). The
  CBF condition is still measured, so `mean_h` and `rejection_ratio` remain comparable columns.
  The PSF must already be generated for these columns to be filled.
- The start is always the robot's **current** position. To repeat a start/goal pair, drive the
  robot back (Teleop) or restart the stack (the robot respawns at the same point).
- Results are appended to `src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv`. Column
  meanings and how to read them are in `README.md` §6.
- `python3 src/cbf_rrt_planner/tools/visualize_psf.py` renders the current PSF to
  `src/cbf_rrt_planner/maps/psf_debug/psf_visualization.png`.

---

## 8. Diagnostics

Useful probes (run inside `just cbf-shell`):

| Command | What it tells you |
|---|---|
| `ros2 topic echo /lookahead_point --csv` | RPP carrot in `base_link`. Distance should equal `lookahead_dist`; `(0,0)` means the plan was truncated to one pose. |
| `ros2 topic echo /cmd_vel_nav --csv` | Raw controller output. `linear.x = 0` with oscillating `angular.z` = rotate/lurch limit cycle. |
| `ros2 topic info /odom` | Publisher count: a zero-publisher topic silently breaks RPP's velocity ramp. |
| `ros2 topic echo /received_global_plan` | The plan as RPP received it; check density. |
| `ros2 param get /controller_server FollowPath.lookahead_dist` | Live Nav2 value (launch overrides the YAML). |
| `$COMPOSE logs navigation` | The actual reason behind a `FollowPath` abort. |

Common situations:

- **`Planning REJECTED: goal falls into an occupied or unknown cell`**: the goal is outside the
  frozen map's free space. Legitimate; pick a reachable point.
- **`MAP MISMATCH DETECTED`**: the live map no longer matches the frozen one (only possible with
  `SLAM=True`). Call `just cbf-refresh-map`.
- **`PSF generation in progress`**: the first goal triggers PSF generation lazily and does not
  plan. Send the goal again once `PSF ready` appears. This auto-trigger fires **once**; later
  refreshes are manual.
- **`TF transform 'odom' -> 'map' failed`** at start-up: harmless, appears before the
  `map → odom` transform exists. Throttled to one line.
- **`FollowPath: ABORTED`**: check `speed` in the feedback. `0.00 m/s` throughout means the
  controller never drove; look at `/lookahead_point` and `/cmd_vel_nav`.
- **`use_sim_time is false`** warning at planner start-up: `USE_SIM_TIME=False` is set in `.env`
  or the environment. Remove it for simulation.
- **`workspace not built - run: just cbf-build`**: the planner container started before the
  first build. Run `just cbf-build`, then `just cbf-restart`.
- **The robot drives, but not along `/planned_path`**: something published on `/goal_pose` and
  Nav2's planner took over (§4 Step 2).
