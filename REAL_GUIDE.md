# Physical robot guide (ROSbot XL)

Step-by-step instructions for running Safe-CBF-RRT\* on a real Husarion ROSbot XL: connecting to
the robot, deploying the code, building a map, generating the PSF, sending goals and running
experiments. All robot workflows go through `justfile.real`. For the architecture, algorithms,
parameters and metrics see [`README.md`](README.md). For the Gazebo simulation see
[`SIM_GUIDE.md`](SIM_GUIDE.md).

---

## 1. What runs on the robot

`justfile.real` drives `docker compose -f compose.yaml -f compose.cbf.yaml`:

| Container | Role |
|---|---|
| `rosbot` | ROSbot XL drivers, `robot_localization` EKF (`/odometry/filtered`, TF `odom → base_link`) |
| `microros` | micro-ROS agent connecting the STM32 board (motors, encoders, IMU) |
| `rplidar` | RPLidar driver |
| `navigation` | Nav2 with slam_toolbox (`SLAM=True`) or AMCL + map_server (`SLAM=False`), `controller_server` (RPP), `velocity_smoother` |
| `cbf-planner` | `planner_node`: PSF generation, Safe-CBF-RRT\* planning, `FollowPath` client |
| `foxglove`, `foxglove-ds` | Web UI on port `8080`, WebSocket bridge on port `8765` |

The planner replaces the Nav2 global planner. It sends paths directly to `controller_server`
through the `FollowPath` action. Nav2's `planner_server` and `bt_navigator` still run, but are
only used if something publishes on `/goal_pose` (see §6.3).

Everything runs **on the robot**. The laptop only needs a browser (Foxglove), SSH, and `just`,
`rsync` and `sshpass` for deploying code.

---

## 2. Configuration

### 2.1 `.env` (set by hand on the robot)

| Variable | Robot value | Notes |
|---|---|---|
| `USE_SIM_TIME` | `False` | **Required.** Add this line, it is not in `.env` by default. The planner container defaults to `True`, which means waiting for a `/clock` that does not exist on the robot. Recipes that build or start the stack refuse to run until this is `False`. |
| `SLAM` | `True` | `True` = build the map live with slam_toolbox. `False` = load `maps/map.yaml` and localise with AMCL. |
| `LIDAR_BAUDRATE` | depends on the lidar | `1000000` for S2/S3, `256000` for S1/A2M12/A3, `115200` for A2M8. |
| `MECANUM` | `True` / `False` | Wheel type. |
| `CONTROLLER` | `rpp` | Selects `config/nav2_rpp_params.yaml`. |
| `ROBOT_NAMESPACE` | Husarnet hostname | Only used by the original `just sync` recipe, which is not used here. |

`USE_SIM_TIME=False` must **not** stay in the laptop's `.env` when you go back to simulation.
`cbf-real-sync` copies `.env` from the laptop to the robot, so either edit `.env` on the robot
after every sync, or keep `USE_SIM_TIME=False` and `SLAM=True` in the laptop copy while working
on the robot.

### 2.2 Planner parameters

`src/cbf_rrt_planner/config/cbf_rrt_planner.yaml`: the same parameters as in simulation (full
table in `README.md` §5.1). All except `odom_topic` are re-read on every goal, so
`ros2 param set /planner_node <name> <value>` takes effect without a restart (§8.2).

On the robot, keep in mind:

- `nominal_velocity` (default `1.0`) is the speed **assumed** by the CBF condition. RPP actually
  drives at `desired_linear_vel = 0.4` m/s. The CBF margin is therefore computed for faster motion
  than the robot performs.
- Data written by the node (`planning_metrics.csv`, `h.csv`, `u_x.csv`, `u_y.csv`) goes to
  `src/cbf_rrt_planner/maps/psf_debug/` **on the robot**.

---

## 3. Recipe reference

Run every recipe as `just -f justfile.real <recipe>`. To shorten this, add an alias on the robot:

```bash
echo "alias justr='just -f justfile.real'" >> ~/.bashrc && source ~/.bashrc
justr cbf-real
```

| Recipe | Where | Description |
|---|---|---|
| `cbf-real-sync <ip> [password]` | laptop | Copies the repo to `/home/husarion/<repo-dir>` on the robot. Skips `.git/`, `build/`, `install/`, `log/` and every `maps/` directory. `--delete` removes robot files that no longer exist on the laptop, except the skipped ones. Password defaults to `husarion`. |
| `cbf-real-pull` | robot | Pulls the Husarion images from `compose.yaml` and builds the `cbf-planner` image. Needs internet on the robot. |
| `cbf-real-build` | robot | Builds the workspace inside the planner container (Release, `--symlink-install`). If `install/` was built for another CPU architecture, wipes `build/ install/ log/` first. |
| `cbf-real-rebuild` | robot | Wipes `build/ install/ log/` and builds from scratch. |
| `cbf-real` | robot | `cbf-real-build`, then starts the whole stack, recreates the planner container so the new binary runs, and follows the planner logs. |
| `cbf-real-restart` | robot | Rebuilds and recreates only the planner container. The robot stack keeps running. |
| `cbf-real-check` | robot | Measures `/scan_filtered` and `/odometry/filtered` rates for 5 s each and prints the planner's `use_sim_time`. |
| `cbf-real-refresh-map` | robot | Calls `/refresh_map`: freezes the current map and generates the PSF in the background. |
| `cbf-real-goal <x> <y>` | robot | Publishes a goal on `/cbf_goal_pose` in the `map` frame, yaw 0. |
| `cbf-real-stop-nav` | robot | Stops the `navigation` container, so no more velocity commands are sent. |
| `cbf-real-save-map` | robot | Saves the current SLAM map to `maps/map.pgm` + `maps/map.yaml`. |
| `cbf-real-logs` | robot | Follows the planner logs. |
| `cbf-real-shell` | robot | Interactive shell in a new planner container on the stack network, for `ros2 topic/param/service` commands. |
| `cbf-real-down` | robot | Stops and removes all containers. |

Recipes that follow logs (`cbf-real`, `cbf-real-restart`, `cbf-real-logs`) can be left with
Ctrl-C. This **only stops the log stream**: the stack, and the robot, keep running.

Other useful commands on the robot (from the repository directory):

| Command | Description |
|---|---|
| `docker compose -f compose.yaml -f compose.cbf.yaml ps -a` | Container states, including ones that already exited. |
| `docker compose -f compose.yaml -f compose.cbf.yaml logs -f navigation` | Nav2 logs: the real reason behind a `FollowPath` abort. |
| `docker compose -f compose.yaml -f compose.cbf.yaml logs rplidar` | Lidar driver logs. |
| `docker compose -f compose.yaml -f compose.cbf.yaml up -d --force-recreate --no-deps navigation` | Restart Nav2 after editing `config/nav2_*_params.yaml`. |
| `docker compose -f compose.yaml -f compose.cbf.yaml up -d --force-recreate --no-deps cbf-planner` | Restart the planner without rebuilding (enough after editing the planner YAML, which is symlinked). |
| `cat src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv` | Per-run planning metrics. |

---

## 4. One-time setup

### 4.1 Laptop

```bash
sudo apt install -y just rsync sshpass   # or install just from https://just.systems
```

### 4.2 Robot

- ROSbot XL with an onboard computer running Husarion OS (Docker is preinstalled).
- RPLidar connected and visible as `/dev/ttyRPLIDAR` (udev rule from Husarion OS).
- Battery charged. The power switch must be within reach during every test.
- `just` installed on the robot (Husarion OS normally includes it; otherwise install it as above).
- Firmware: if the base does not react to velocity commands or the `rosbot` container reports
  firmware errors, flash the STM32 once with `just flash-firmware` (from the main `justfile`,
  run on the robot). It stops all running containers.

### 4.3 Network

The laptop and the robot must be able to reach each other:

- the same Wi-Fi/Ethernet network: find the robot IP on your router or in the Husarion
  documentation for your computer variant;
- or a Husarnet VPN: `sudo just connect-husarnet <joincode> <hostname>` on both machines.

Below, `ROBOT_IP` stands for the robot address (IP or Husarnet hostname).

```bash
ssh husarion@ROBOT_IP      # default Husarion OS password: husarion
```

---

## 5. Step-by-step: from a powered-off robot to the first plan

### Step 1. Deploy the code [laptop]

```bash
cd ~/FromZero/rosbot-xl-autonomy-foxglove
just -f justfile.real cbf-real-sync ROBOT_IP
```

Do **not** use `just sync` from the main `justfile`. It copies the laptop's x86-64 `build/` and
`install/`, and its watch loop would keep overwriting the binary built on the robot.

Repeat this step after every code change, then rebuild (Step 4 or `cbf-real-restart`).

### Step 2. Configure `.env` [robot]

```bash
ssh husarion@ROBOT_IP
cd ~/rosbot-xl-autonomy-foxglove
nano .env
```

Set `SLAM=True`, add `USE_SIM_TIME=False`, check `LIDAR_BAUDRATE` and `MECANUM` (§2.1).

### Step 3. Pull and build images [robot, once and after image changes]

```bash
just -f justfile.real cbf-real-pull
```

### Step 4. Build and start the stack [robot]

```bash
just -f justfile.real cbf-real
```

The first build on a single-board computer may take several minutes. `navigation` only starts
once `rosbot` and `rplidar` report healthy, so the planner may wait for a while.

Expected planner log:

```
Planner node initialized, waiting for the map, odometry and goal...
First map received. Awaiting /refresh_map to generate PSF.
```

`TF transform 'odom' -> 'map' failed` warnings right after start are harmless. They stop as soon
as slam_toolbox publishes `map → odom`. The `use_sim_time is false` warning is expected on the
robot.

### Step 5. Verify the inputs [robot, second SSH session]

```bash
cd ~/rosbot-xl-autonomy-foxglove
just -f justfile.real cbf-real-check
```

Both topics must report a non-zero rate, and `use_sim_time` must be `False`. If
`/scan_filtered` is silent, neither SLAM nor the local costmap gets lidar data. Check the
`rplidar` container (`docker compose -f compose.yaml -f compose.cbf.yaml logs rplidar`) and the
baud rate.

Also check that every container is up:

```bash
docker compose -f compose.yaml -f compose.cbf.yaml ps -a
```

### Step 6. Open Foxglove [laptop]

1. Open `http://ROBOT_IP:8080` in a browser.
2. If asked for a data source, choose **Foxglove WebSocket** with `ws://ROBOT_IP:8765`.
3. In the 3D panel, set **Display frame** to `map` and enable `/map`, `/scan`,
   `/planned_path` and `/local_costmap/costmap`.

> **Warning:** the "Publish pose" tool of the 3D panel publishes on `/goal_pose` by default.
> That topic is consumed by Nav2's `bt_navigator`, which plans with its own Smac planner,
> **bypassing Safe-CBF-RRT\***, and drives the robot. Before using the tool, change its topic to
> `/cbf_goal_pose` in the panel settings (Publish → Pose topic), or send goals from the terminal
> only (Step 9).

### Step 7. Build the map [laptop, Foxglove]

Use the **Teleop** panel (publishes on `/cmd_vel`, 0.3 m/s, 1 rad/s) and drive slowly around
the test area until `/map` covers it.

- The `map` frame is defined by the robot's pose when slam_toolbox started: `(0, 0)` is where the
  robot stood, and the x axis is the direction it faced. All goal coordinates are in this frame.
- Unknown cells (`-1`) count as obstacles for the planner. The planner can only plan inside the
  explored part of the map.
- Do not use Teleop while a path is being executed. Both write to `/cmd_vel`.

### Step 8. Freeze the map and generate the PSF [robot]

```bash
just -f justfile.real cbf-real-refresh-map
```

Wait for `PSF ready (N obstacles). Exported.` in the planner log. On the robot's CPU this can
take much longer than on a laptop.

If the map grows afterwards (more mapping), the next goal logs `MAP MISMATCH DETECTED`. Call
`cbf-real-refresh-map` again.

### Step 9. Send a goal [robot]

Read the target coordinates in the `map` frame in Foxglove, then:

```bash
just -f justfile.real cbf-real-goal 1.0 0.5
```

Start with goals 1–2 m away in open space. Expected log sequence:

```
New goal received: [X: 1.00, Y: 0.50] (frame: map)
Planning from (...) to (1.00, 0.50)...
Planning SUCCESS: length=... mean_h=... min_h=... rejection_ratio=...
Sent path to controller_server (FollowPath action).
FollowPath: goal accepted, the controller is driving.
FollowPath: ... m to go at ... m/s.
FollowPath: robot reached the end of the path.
```

Planning blocks the node for longer than on a laptop. The robot stands still meanwhile.

### Step 10. Stop and shut down [robot]

```bash
just -f justfile.real cbf-real-stop-nav   # stop sending velocity commands
just -f justfile.real cbf-real-down       # stop the whole stack
```

---

## 6. Safety on the physical robot

### 6.1 Stopping the robot

Ctrl-C in a terminal **does not stop the robot**. In order of reliability:

1. The robot's power switch.
2. `just -f justfile.real cbf-real-stop-nav`. Stops the controller and velocity smoother. The base
   should stop once it stops receiving `/cmd_vel`; verify this behaviour in your first test,
   with the robot lifted or in open space.
3. A new goal at the robot's current position. Cancels the running `FollowPath` goal, but only
   if planning succeeds.

### 6.2 Nothing in the chain avoids unmapped obstacles

- The planner inflates the **frozen map** by `inflation_robot_size` (0.22 m by default), so the
  robot body is covered only for obstacles present when the map was frozen. Check that
  `inflation: true` before experiments meant to be safe for the body.
- The PSF is **frozen** at the moment of `/refresh_map`. People, moved furniture and anything
  else not in the frozen map are invisible to the planner.
- RPP has `use_collision_detection: false`, so the controller does not stop for obstacles in the
  local costmap. It also cuts corners at sharp path turns (lookahead 0.6 m).

For the first runs: keep the test area clear, start with `safety_weight_c` ≤ 0.5 (which keeps
paths away from walls), and stay next to the robot.

### 6.3 `/goal_pose` vs `/cbf_goal_pose`

Anything published on `/goal_pose` (Foxglove "Publish pose" with default settings, RViz
"Nav2 Goal") is planned by Nav2, not by Safe-CBF-RRT\*. Planner experiments must use
`/cbf_goal_pose`.

---

## 7. Saving and reusing a map

Saving the map lets you repeat experiments in the same frame, and avoid re-mapping.

```bash
just -f justfile.real cbf-real-save-map     # writes maps/map.pgm + maps/map.yaml on the robot
```

To reuse it: set `SLAM=False` in `.env` and restart the stack (`cbf-real-down`, `cbf-real`).
Nav2 then loads `maps/map.yaml` and localises with AMCL. AMCL starts from the pose `(0, 0, 0)`
(`initial_pose` in `config/nav2_rpp_params.yaml`), so:

- place the robot where mapping started, facing the same direction, **or**
- set the initial pose in Foxglove ("Publish pose estimate" → `/initialpose`).

Check in Foxglove that `/scan` overlaps the walls of `/map` before sending goals.

`cbf-real-sync` never copies `maps/` directories, so a map saved on the robot is not overwritten
or deleted by a sync from the laptop. With `SLAM=False` on the robot, the map must have been
recorded on the robot: maps from simulation describe a different world.

`map_server` reads `maps/map.yaml` with `free_thresh: 0.25`. With this threshold, pixels saved
as "unknown" (value 205) are loaded as **free**, not as unknown. If you want unknown space to
stay an obstacle for the planner, set `free_thresh: 0.19` in `maps/map.yaml` on the robot.

---

## 8. Running experiments

### 8.1 Recommended protocol

1. Map the area (§5 Step 7) and save the map (§7). For repeatable experiments, switch to
   `SLAM=False` so every run uses exactly the same map and the same frame.
2. Generate the PSF once (`cbf-real-refresh-map`) and keep it for the whole series.
3. Choose a fixed set of start/goal pairs. The start is always the robot's **current**
   position, so drive the robot back to the same start (Teleop) before each run, and check its
   pose in Foxglove.
4. For each configuration (baseline, then each value of `safety_weight_c`, `kappa`,
   `nominal_velocity`), run each start/goal pair several times. RRT\* is randomised: the RNG is
   seeded from `std::random_device`, so every run differs.
5. Record the CSV (§8.3) and, for the real-robot part, observe the actual execution: did the
   robot reach the goal, did it touch anything, how close did it get to walls (Foxglove
   recording, or a tape measure).

### 8.2 Changing parameters between runs [robot]

Open a shell on the stack network:

```bash
just -f justfile.real cbf-real-shell
source /opt/ros/humble/setup.bash
```

Then, for example:

```bash
# plain RRT* baseline: the CBF condition is measured but not enforced, cost = length
ros2 param set /planner_node enable_cbf false

# Safe-CBF-RRT* with a given safety weight (1.0 = length only, 0.0 = safety only)
ros2 param set /planner_node enable_cbf true
ros2 param set /planner_node safety_weight_c 0.5

# CBF strictness: only the ratio nominal_velocity / kappa matters (approach margin in metres)
ros2 param set /planner_node kappa 5.0
ros2 param set /planner_node nominal_velocity 0.4

ros2 param get /planner_node safety_weight_c

# PSF source and inflation are frozen with the map: run cbf-real-refresh-map afterwards
ros2 param set /planner_node psf_source constant      # or: centroid
ros2 param set /planner_node inflation_robot_size 0.26
```

`safety_weight_c` weights safety differently for each PSF source, so sweep `c` separately for
`centroid` and `constant` (see `README.md` §6).

The values apply from the next goal. They are lost when the planner container is recreated. To
make them permanent, edit `src/cbf_rrt_planner/config/cbf_rrt_planner.yaml` on the laptop, sync
and run `cbf-real-restart`.

For the baseline, the PSF must already be generated, otherwise the `mean_h`, `min_h` and
`rejection_ratio` columns stay empty.

### 8.3 Metrics

Each successful or failed plan appends one line to
`src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv` on the robot. Column meanings and how
to read them are in `README.md` §6. In short: compare `mean_h` and `min_h` only between runs on
the same frozen PSF (`h` is in m² and depends on the map), and note `kappa` and
`nominal_velocity` yourself, because the run label does not include them.

Copy the results to the laptop:

```bash
scp -r husarion@ROBOT_IP:/home/husarion/rosbot-xl-autonomy-foxglove/src/cbf_rrt_planner/maps/psf_debug ./psf_debug_robot
python3 src/cbf_rrt_planner/tools/visualize_psf.py ./psf_debug_robot
```

`visualize_psf.py` renders `h`, `u_x` and `u_y` of the robot's PSF to `psf_visualization.png`.

### 8.4 Useful probes [`cbf-real-shell`]

| Command | What it tells you |
|---|---|
| `ros2 topic echo /planned_path --once` | Path published by the planner |
| `ros2 topic echo /received_global_plan --once` | Path as RPP received it |
| `ros2 topic echo /lookahead_point --csv` | RPP carrot in `base_link`; `(0, 0)` means the plan was truncated |
| `ros2 topic echo /cmd_vel_nav --csv` | Controller output before the velocity smoother |
| `ros2 topic echo /cmd_vel --csv` | Velocity actually sent to the base |
| `ros2 topic info /odom` | Publisher count: a zero-publisher odometry topic breaks RPP's velocity ramp |
| `ros2 param get /controller_server FollowPath.lookahead_dist` | Live Nav2 value (launch overrides the YAML) |
| `ros2 run tf2_ros tf2_echo map base_link` | Robot pose in the map frame |
| `ros2 run tf2_tools view_frames` | TF tree as a PDF |

---

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `USE_SIM_TIME is 'True (default)'` and the recipe stops | Add `USE_SIM_TIME=False` to `.env` on the robot. |
| `use_sim_time is false` warning in the planner log | Expected on the robot: the warning is meant for the Gazebo stack. |
| `exec format error` when the planner starts | `install/` from another architecture. Run `cbf-real-rebuild`. |
| `workspace not built - run: just cbf-build` | The workspace was never built on the robot. Run `cbf-real-build` (not `just cbf-build`). |
| `cbf-real-check` shows no `/scan_filtered` | Lidar not running: check `/dev/ttyRPLIDAR`, `LIDAR_BAUDRATE` and the `rplidar` logs. |
| `Odometry is X s old ... refusing to plan` | `/odometry/filtered` is not published, or TF `map → odom` is missing. Check the `rosbot` and `navigation` logs. |
| `PSF generation in progress - send the goal again` | The first goal triggers PSF generation. Wait for `PSF ready`, then resend. |
| `Planning REJECTED: start/goal falls into an occupied or unknown cell` | The point is outside the mapped free space. Map more, or choose another point. |
| `Planning REJECTED: start/goal ... (inflated zone)` | The point is free, but closer to an obstacle than `inflation_robot_size`. Drive the robot away from the wall, pick a goal further from it, or lower the radius and call `cbf-real-refresh-map`. |
| `Inflation parameters changed since the PSF was generated` / `PSF source parameters changed ...` | Inflation or PSF source was changed with `ros2 param set`. Run `cbf-real-refresh-map` to apply it. |
| `Planning FAILED after N iterations` | No admissible path within the budget. Try a nearer goal, a larger `max_iterations`, or a weaker CBF (`kappa` up / `nominal_velocity` down). |
| `MAP MISMATCH DETECTED` | The map grew since the PSF was generated. Run `cbf-real-refresh-map`. |
| `FollowPath action server not available` | `navigation` is not running or not healthy yet. Check `ps -a` and its logs. |
| `FollowPath: ABORTED` | Reason is in `docker compose -f compose.yaml -f compose.cbf.yaml logs navigation` (progress checker, controller exception). Check `speed` in the feedback: `0.00 m/s` throughout means the controller never drove; look at `/lookahead_point` and `/cmd_vel_nav`. |
| Robot drives, but not along `/planned_path` | Something published on `/goal_pose` and Nav2 took over (§6.3). |
| Foxglove shows no data | Check that `foxglove-ds` is running and that port `8765` is reachable from the laptop. |
