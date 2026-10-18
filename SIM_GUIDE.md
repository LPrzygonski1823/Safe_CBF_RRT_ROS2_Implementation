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
| `rosbot` | Gazebo simulation of the ROSbot XL; loads the world selected in `compose.simulation.yaml` (default `worlds/custom_world_u.sdf`) |
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
| `SLAM` | `False` | Uses the map named by `MAP_YAML`. With `True`, comment out the `static-tf` service. |
| `MAP_YAML` | `map_u.yaml` | File inside `maps/` loaded when `SLAM=False`. `map_u.yaml` matches `custom_world_u.sdf`; `map.yaml` matches `custom_world_3.sdf`. |
| `CONTROLLER` | `rpp` | Picks which `config/nav2_*_params.yaml` is mounted. |
| `MECANUM` | `True` | Holonomic base in Gazebo. RPP itself never commands `vy`. |
| `SAVE_MAP_PERIOD` | 15 | Map autosave, only when `SLAM=True` (set 0 to disable). |
| `USE_SIM_TIME` | unset or `True` | Leave unset. If it is `False` (left over from robot work), the planner ignores `/clock` and its time comparisons break. |

`LIDAR_BAUDRATE` and `ROBOT_NAMESPACE` are used only on the physical robot.

### 3.2 World selection

The default world is `worlds/custom_world_u.sdf` (U-shaped obstacles and one closed O), with
Nav2 loading `maps/map_u.yaml` via `MAP_YAML`. `worlds/custom_world_3.sdf` and `maps/map.yaml`
are the original box-and-cylinder world. Both share the same outer wall and the same map
`origin`.

`compose.simulation.yaml` binds the world into the `rosbot` container and passes it as
`world:=`. Nav2 loads `map:=/maps/${MAP_YAML:-map.yaml}`. To return to world 3, change those
two lines and set `MAP_YAML=map.yaml` in `.env`:

```yaml
- ./worlds/custom_world_3.sdf:/custom_world_3.sdf
command: ... world:=/custom_world_3.sdf headless:=False
```

Do not overwrite `maps/map.pgm` or `maps/map.yaml` with a recording from the U world. That pair
is the map of world 3.

The `map` frame is the robot's spawn, not the SDF origin. The robot appears at world `(0, 2)`,
which is map `(0, 0)`. Interior obstacles therefore sit at world `(x_map, y_map + 2)`, with no
shift in `x` and no rotation. `just cbf-goal` takes map coordinates. The inside of the large
south-opening U "A" is:

```bash
just cbf-goal -6.0 4.5
```

After start-up, check in Foxglove that `/scan` lies on the walls drawn in `/map`. A uniform 2 m
offset means the world file was built without the y shift.

Regenerate the image and the SDF from the same obstacle list (this does not touch
`maps/map.pgm` or `maps/map.yaml`):

```bash
python3 tools/generate_u_world.py
```

Each U is three static boxes and the O is four, walls 0.3 m thick and 1 m tall. The script
keeps the floor, the sun and the `wall_box_*` shell from `custom_world_3.sdf`.

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

Coordinates are in the `map` frame. On `map_u`, world `(x, y)` is map `(x, y − 2)`: the robot
spawns at world `(0, 2)`. The recorded `map.yaml` of world 3 is the SLAM estimate of that same
spawn, about `map ≈ world + (0.05, −1.94)` m.

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
   While `custom_world_u.sdf` is loaded, pass `-f /maps/map_u_slam` instead, so the world-3
   map and the synthetic `map_u` used by the benchmark stay intact. Point `MAP_YAML` at the
   file you saved.
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
| `just cbf-benchmark [OPTIONS]` | Offline benchmark of the planner core, no simulation needed (`--help` lists the options; `README.md` §6.2). |
| `just cbf-benchmark-plot [RESULTS] [OPTIONS]` | Plots and summary of the benchmark results, on the host (`README.md` §6.3). |
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

The paper compares plain RRT\* against Safe-CBF-RRT\* over a range of `c`. There are two ways to
run that comparison:

- **Offline benchmark:** for the numbers, many plans with identical inputs. It needs no running
  stack and is described in §7.1.
- **Live node in Gazebo:** for the behaviour of the whole chain (planner, RPP, simulated robot).
  It is described in §7.2.

### 7.1 Offline benchmark

```bash
# constant source (as in the YAML), 20 reachable goals, 30 seeds, three c values + plain RRT*
just cbf-benchmark --random-goals 20 --runs 30 --c 0.2,0.5,0.8 --baseline --tag custom_world_3
# the same goals and seeds with the centroid source, appended to the same results.csv
just cbf-benchmark --random-goals 20 --runs 30 --c 0.2,0.5,0.8 --baseline --tag custom_world_3 \
                   --set psf_source=centroid
# without inflation (point robot, as in the paper)
just cbf-benchmark --random-goals 20 --runs 30 --c 0.2,0.5,0.8 --baseline --tag custom_world_3 \
                   --set inflation=false
just cbf-benchmark-plot
```

- The benchmark reads `src/cbf_rrt_planner/config/cbf_rrt_planner.yaml`; `--set key=value`
  overrides a key for one invocation only and does not touch the file.
- It uses `maps/map.yaml` and starts at `(0, 0)` by default. For another map pass
  `--map path/to/map.yaml`, and `--start X,Y` if `(0, 0)` is not free there.
- Your own goals: a text file with one `x,y` per line, passed with `--goals FILE`.
- Results: `src/cbf_rrt_planner/benchmark/results.csv` + `results_paths.csv`; plots and
  `summary.csv` in `src/cbf_rrt_planner/benchmark/plots/`. Column meanings: `README.md` §6.2.

### 7.2 Live node

Recommended loop:

```bash
docker exec rosbot-xl-autonomy-foxglove-cbf-planner-1 bash -c \
  'source /opt/ros/humble/setup.bash && ros2 param set /planner_node safety_weight_c 0.5'
just cbf-goal -6.98 -3.38
```

- For the baseline set `enable_cbf: false` (`ros2 param set /planner_node enable_cbf false`). The
  CBF condition is still measured, so `mean_h` and `rejection_ratio` remain comparable columns.
  The PSF must already be generated for these columns to be filled.
- To switch the PSF source, set it and re-freeze the map; the new source applies only after
  `PSF ready`:

  ```bash
  docker exec rosbot-xl-autonomy-foxglove-cbf-planner-1 bash -c \
    'source /opt/ros/humble/setup.bash && ros2 param set /planner_node psf_source constant'
  just cbf-refresh-map
  ```

  `safety_weight_c` weights safety differently for each source, so sweep `c` separately for
  `centroid` and `constant` (see `README.md` §6).
- The start is always the robot's **current** position. To repeat a start/goal pair, drive the
  robot back (Teleop) or restart the stack (the robot respawns at the same point).
- With `seed` set to a fixed value (`ros2 param set /planner_node seed 7`), the same start/goal
  and parameters give the same path. With the default `-1`, every plan differs.
- Every plan also logs `Clearance (map without inflation): min=... mean=..., footprint min=...`.
  The same values go into the CSV columns `min_clearance`, `mean_clearance` and
  `min_footprint_clearance`.
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
- **`Planning REJECTED: ... (inflated zone)`**: the start or goal is free, but closer to an
  obstacle than `inflation_robot_size` (0.22 m). Pick a point further from the wall, or lower the
  radius and call `just cbf-refresh-map`.
- **`Inflation parameters changed since the PSF was generated`**: the inflation was changed with
  `ros2 param set`. It only takes effect after `just cbf-refresh-map`.
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
