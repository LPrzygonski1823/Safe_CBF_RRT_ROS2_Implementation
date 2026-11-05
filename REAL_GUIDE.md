# Physical robot guide (ROSbot XL, ROS 2 Jazzy)

Step-by-step instructions for running Safe-CBF-RRT\* on a real Husarion ROSbot XL: connecting to
the robot, deploying the code, building a map, generating the PSF, sending goals and running
experiments. All robot workflows go through `justfile.jazzy`. For the architecture, algorithms,
parameters and metrics see [`README.md`](README.md). For the Gazebo simulation (ROS 2 Humble) see
[`SIM_GUIDE.md`](SIM_GUIDE.md).

The robot runs **ROS 2 Jazzy** natively, so its stack is separate from the Humble simulation:
`compose.jazzy.yaml`, `dockerfile.jazzy`, `.env.real` and `justfile.jazzy`. The planner code is
the same; it is built on the robot against Jazzy.

---

## 1. What runs on the robot

### 1.1 Host (snaps, not managed by this repository)

| Snap | Role |
|---|---|
| `rosbot` | ROSbot XL drivers, `robot_localization` EKF (`/odometry/filtered`, TF `odom → base_link`), robot description, `twist_mux` (selects the velocity command for the base) |
| `husarion-rplidar` | RPLidar driver, `/scan` in frame `laser` |

### 1.2 Containers (`compose.jazzy.yaml`)

All containers except `foxglove` use the `ros_jazzy_cbf` image built from `dockerfile.jazzy`, the
host network, CycloneDDS and `ROS_DOMAIN_ID=0`, so they share one DDS graph with the snaps.

| Container | Role |
|---|---|
| `navigation` | `launch/nav2_real.launch.py`: slam_toolbox (mapping, `/map`, TF `map → odom`), `controller_server` (RPP + local costmap), `velocity_smoother`, lifecycle manager. Waits up to 45 s for TF `cover_link → laser` before starting, so the first SLAM map is not built from a backwards scan. |
| `laser-tf` | Static TF `cover_link → laser`. The snaps do not publish it; yaw π puts RPLidar angle 0 on the robot's +x. |
| `cbf-planner` | `planner_node`: PSF generation, Safe-CBF-RRT\* planning, `FollowPath` client. Builds the image. |
| `foxglove-ds` | Foxglove WebSocket bridge on port `8765`. |
| `teleop-relay` | `scripts/teleop_relay.py`: Foxglove Teleop panel (`Twist`) → `twist_mux` manual input (`TwistStamped`). |
| `foxglove` | Optional Foxglove web UI on port `8080` (`config/layout.json`). The Foxglove desktop app connects to `8765` directly and does not need it. |

Only the Nav2 parts the planner needs are started. There is no `map_server`, AMCL,
`planner_server` or `bt_navigator`: the planner sends its paths straight to `controller_server`
through the `FollowPath` action.

### 1.3 Velocity chain

```
planner_node --FollowPath--> controller_server (RPP) --/cmd_vel_nav--> velocity_smoother
    --/autonomous/cmd_vel (TwistStamped, priority 10)--> twist_mux (snap) --> base

Foxglove Teleop --/manual/cmd_vel_unstamped (Twist)--> teleop-relay
    --/manual/cmd_vel (TwistStamped)--> twist_mux (snap) --> base
```

The snap's `twist_mux` expects `TwistStamped`, hence `enable_stamped_cmd_vel: true` in
`config/nav2_real_params.yaml` and the relay for the Teleop panel. The relay clamps the command
(0.3 m/s, 1.0 rad/s by default). When the panel stops publishing for 0.5 s, it sends a single zero
command and then stays silent, so it does not hold the manual input of `twist_mux` against
autonomous navigation.

Everything runs **on the robot**. The laptop only needs the Foxglove app, SSH, and `just`,
`rsync` and `sshpass` for deploying code.

---

## 2. Configuration

### 2.1 `.env.real`

Read by `justfile.jazzy` and by `compose.jazzy.yaml`. It is synced from the laptop like any other
file, so edit it on the laptop and run `cbf-real-sync`. The simulation keeps its own `.env`; the
two never need to be switched.

| Variable | Value | Notes |
|---|---|---|
| `USE_SIM_TIME` | `False` | **Required.** Clock of the planner. `cbf-real-image`, `cbf-real-build` and `cbf-real-rebuild` refuse to run unless it is `False`. |
| `LASER_X`, `LASER_Y`, `LASER_Z` | `0.0` | Translation of `laser` relative to `cover_link` [m]. |
| `LASER_YAW` | `3.14159265` | Rotation of `laser` relative to `cover_link` [rad]. Verify it once (§5 Step 5). |
| `ROBOT_IP` | e.g. `192.168.8.199` | Only used by the optional `foxglove` web container as the bridge address. |
| `TELEOP_MAX_LIN`, `TELEOP_MAX_ANG`, `TELEOP_TIMEOUT` | `0.3`, `1.0`, `0.5` (compose defaults) | Optional. Teleop relay limits [m/s, rad/s] and silence timeout [s]. |

### 2.2 Nav2 and SLAM

`config/nav2_real_params.yaml` (robot only; the simulation uses `config/nav2_rpp_params.yaml`):

| Parameter | Value | Notes |
|---|---|---|
| `FollowPath.desired_linear_vel` | 0.6 | Cruise speed, equal to the planner's `nominal_velocity` and to the simulation. |
| `velocity_smoother.max_velocity` | `[0.6, 0.0, 1.0]` | Hard limit on what reaches `twist_mux`. |
| `lookahead_dist` (min / max) | 0.6 (0.3 / 0.9) | As in the simulation. |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.10 / 0.30 | Goal checker acceptance. |
| `use_collision_detection` | false | RPP does **not** stop for obstacles (§6.2). |
| `odom_topic` | `/odometry/filtered` | EKF of the `rosbot` snap. |
| `enable_stamped_cmd_vel` | true | `twist_mux` of the snap expects `TwistStamped`. |
| local costmap | 4 × 4 m, `robot_radius` 0.22, inflation 0.35 | Rolling window in `odom`. |

`config/slam_toolbox_real.yaml`: slam_toolbox in `mapping` mode, 0.05 m resolution (as the
simulation map), `/scan`, `base_link`, maximum laser range 12 m, map update every 2 s.

Changes to either file need a restart of `navigation` (§3, other commands). This **discards the
current SLAM map** and starts a new `map` frame.

### 2.3 Planner parameters

`src/cbf_rrt_planner/config/cbf_rrt_planner.yaml`: the same parameters as in simulation (full
table in `README.md` §5.1). All except `odom_topic` are re-read on every goal, so
`ros2 param set /planner_node <name> <value>` takes effect without a restart (§8.2).

On the robot, keep in mind:

- `nominal_velocity` (default `0.6`) is the speed **assumed** by the CBF condition. It matches
  RPP's `desired_linear_vel` and the `velocity_smoother` limit (0.6 m/s). It does not change the
  driving speed: that is set by `desired_linear_vel` (§8.2). RPP does not stop for obstacles
  (§6.2), so keep a clear area and the power switch within reach.
- Data written by the node (`planning_metrics.csv`, `h.csv`, `u_x.csv`, `u_y.csv`) goes to
  `src/cbf_rrt_planner/maps/psf_debug/` **on the robot**.

---

## 3. Recipe reference

Run every recipe as `just -f justfile.jazzy <recipe>`. To shorten this, add an alias on the robot:

```bash
echo "alias j='just -f justfile.jazzy'" >> ~/.bashrc && source ~/.bashrc
j cbf-real
```

Everything except `cbf-real-sync` runs on the robot, in `~/rosbot-xl-autonomy-foxglove`, as the
`husarion` user (not root).

| Recipe | Where | Description |
|---|---|---|
| `cbf-real-sync <ip> [password]` | laptop | Copies the repo to `/home/husarion/<repo-dir>` on the robot. Skips `.git/`, `.vscode/`, `build/`, `install/`, `log/`, `__pycache__/` and every `maps/` directory. `--delete` removes robot files that no longer exist on the laptop, except the skipped ones. Password defaults to `husarion`. |
| `cbf-real-image` | robot | Builds the `ros_jazzy_cbf` image (Nav2, slam_toolbox, Foxglove bridge, planner toolchain). Once, and again only after `dockerfile.jazzy` changes. Needs internet on the robot. |
| `cbf-real-build` | robot | Builds the workspace inside the planner container (Release, `--symlink-install`). If `install/` was built for another CPU architecture or ROS distro, wipes `build/ install/ log/` first. |
| `cbf-real-rebuild` | robot | Wipes `build/ install/ log/` and builds from scratch. |
| `cbf-real` | robot | `cbf-real-build`, then starts the whole stack, recreates the planner container so the new binary runs, and follows the planner logs. |
| `cbf-real-restart` | robot | Rebuilds and recreates only the planner container. SLAM and the rest of the stack keep running. |
| `cbf-real-check` | robot | Rates of `/scan` and `/odometry/filtered` (5 s each), TF `map → base_link` and `base_link → laser`, publishers and type of `/autonomous/cmd_vel`, the planner's `use_sim_time`. |
| `cbf-real-refresh-map` | robot | Calls `/refresh_map`: freezes the current map and generates the PSF in the background. Wait for `PSF ready` in `cbf-real-logs`. |
| `cbf-real-goal <x> <y>` | robot | Publishes a goal on `/cbf_goal_pose` in the `map` frame, yaw 0. |
| `cbf-real-save-map` | robot | Saves the current SLAM map to `maps/map.pgm` + `maps/map.yaml` on the robot. |
| `cbf-real-logs` | robot | Follows the planner logs. |
| `cbf-real-nav-logs` | robot | Follows the Nav2 / slam_toolbox logs. |
| `cbf-real-teleop-logs` | robot | Follows the teleop relay logs. |
| `cbf-real-teleop-restart` | robot | Recreates the teleop relay (after editing `scripts/teleop_relay.py` or the `TELEOP_*` limits). |
| `cbf-real-stop-teleop` | robot | Stops the teleop relay: the Foxglove Teleop panel can no longer drive the base. |
| `cbf-real-stop-nav` | robot | Stops the `navigation` container, so no more autonomous velocity commands are sent. Also stops SLAM. |
| `cbf-real-shell` | robot | Interactive ROS 2 shell in a new planner container, for `ros2 topic/param/service` commands. |
| `cbf-real-down` | robot | Stops and removes all containers. |

Recipes that follow logs can be left with Ctrl-C. This **only stops the log stream**: the stack,
and the robot, keep running.

Other useful commands on the robot (from the repository directory):

```bash
COMPOSE="docker compose --env-file .env.real -f compose.jazzy.yaml"
```

| Command | Description |
|---|---|
| `$COMPOSE ps -a` | Container states, including ones that already exited. |
| `$COMPOSE logs foxglove-ds` | Foxglove bridge logs. |
| `$COMPOSE up -d --force-recreate --no-deps navigation` | Restart Nav2/SLAM after editing `config/nav2_real_params.yaml` or `config/slam_toolbox_real.yaml`. Discards the current map. |
| `$COMPOSE up -d --force-recreate laser-tf` | Apply new `LASER_*` values. |
| `$COMPOSE up -d --force-recreate --no-deps cbf-planner` | Restart the planner without rebuilding (enough after editing the planner YAML, which is symlinked). |
| `snap services rosbot husarion-rplidar` | State of the driver and lidar snaps. |
| `cat src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv` | Per-run planning metrics. |

---

## 4. One-time setup

### 4.1 Laptop

```bash
sudo apt install -y just rsync sshpass   # or install just from https://just.systems
```

Install the Foxglove desktop app (or use the optional web UI, §5 Step 4).

### 4.2 Robot

- ROSbot XL with ROS 2 Jazzy, and the `rosbot` and `husarion-rplidar` snaps installed and running
  (`snap services rosbot husarion-rplidar`). Docker and `just` installed.
- Battery charged. The power switch must be within reach during every test.

### 4.3 Network

The laptop and the robot must be on the same network. Below, `ROBOT_IP` stands for the robot
address (e.g. `192.168.8.199`).

```bash
ssh husarion@ROBOT_IP      # default password: husarion
```

---

## 5. Step-by-step: from a powered-off robot to the first plan

### Step 1. Deploy the code [laptop]

```bash
cd ~/FromZero/rosbot-xl-autonomy-foxglove
just -f justfile.jazzy cbf-real-sync ROBOT_IP
```

Repeat this step after every code or config change, then rebuild (`cbf-real-restart`) or
restart the affected container (§3).

### Step 2. Build the image [robot, once and after `dockerfile.jazzy` changes]

```bash
ssh husarion@ROBOT_IP
cd ~/rosbot-xl-autonomy-foxglove
just -f justfile.jazzy cbf-real-image
```

### Step 3. Build and start the stack [robot]

```bash
just -f justfile.jazzy cbf-real
```

The first build on the robot's computer may take several minutes. Expected planner log:

```
Planner node initialized, waiting for the map, odometry and goal...
First map received. Awaiting /refresh_map to generate PSF.
```

`TF transform 'odom' -> 'map' failed` warnings right after start are harmless. They stop as soon
as slam_toolbox publishes `map → odom`. The `use_sim_time is false` warning is expected on the
robot.

Then check the inputs in a second SSH session:

```bash
just -f justfile.jazzy cbf-real-check
```

`/scan` and `/odometry/filtered` must report a non-zero rate, both TF lookups must print a
translation, `/autonomous/cmd_vel` must be `geometry_msgs/msg/TwistStamped`, and `use_sim_time`
must be `False`.

### Step 4. Connect Foxglove [laptop]

1. Foxglove desktop: **Open connection → Foxglove WebSocket**, URL `ws://ROBOT_IP:8765`.
   (Web UI alternative: `http://ROBOT_IP:8080`.)
2. In the 3D panel, set **Display frame** to `map` and enable `/map`, `/scan`, `/planned_path`
   and `/local_costmap/costmap`.
3. In the **Teleop** panel, set the topic to `/manual/cmd_vel_unstamped`.
4. Optionally, set the 3D panel's **Publish pose** topic to `/cbf_goal_pose`: clicking a pose
   then sends a planner goal in the display frame (`map`). There is no `bt_navigator` on this
   stack, so with the default `/goal_pose` nothing happens.

### Step 5. Verify the lidar orientation [once, and after changing `LASER_*`]

Put a box about 0.5 m in front of the robot. On `/scan` it must appear in front of the robot
model. If it appears behind, the yaw is off by π: change `LASER_YAW` in `.env.real` on the laptop,
sync, then `$COMPOSE up -d --force-recreate laser-tf` and restart `navigation` (§3), because
slam_toolbox keeps the map started with the wrong scan.

### Step 6. First motion test [robot lifted, wheels in the air]

Before driving on the floor:

1. Leave the robot still for about 15 s after start, so SLAM builds a first map.
2. `just -f justfile.jazzy cbf-real-refresh-map`, wait for `PSF ready` in `cbf-real-logs`.
3. `just -f justfile.jazzy cbf-real-goal 2.0 0.0`.
4. The planner must log `Planning SUCCESS` and `FollowPath: goal accepted`, and the wheels must
   turn forward.
5. Stop with `cbf-real-stop-nav` (§6.1). Do not save this map: the EKF reported wheel motion that
   the lidar did not see. Restart `navigation` (or `cbf-real-down` + `cbf-real`) before mapping.

### Step 7. Build the map [laptop, Foxglove]

Drive slowly around the test area with the Teleop panel until `/map` covers it.

- The `map` frame is defined by the robot's pose when slam_toolbox started: `(0, 0)` is where the
  robot stood, and the x axis is the direction it faced. All goal coordinates are in this frame.
- Unknown cells (`-1`) count as obstacles for the planner. The planner can only plan inside the
  explored part of the map.
- Do not use Teleop while a path is being executed.

### Step 8. Freeze the map and generate the PSF [robot]

```bash
just -f justfile.jazzy cbf-real-refresh-map
```

Wait for `PSF ready (N obstacles). Exported.` in the planner log. On the robot's CPU this takes
longer than on a laptop.

If the map grows afterwards (more mapping), the next goal logs `MAP MISMATCH DETECTED`. Call
`cbf-real-refresh-map` again.

### Step 9. Send a goal [robot or Foxglove]

Read the target coordinates in the `map` frame in Foxglove, then:

```bash
just -f justfile.jazzy cbf-real-goal 1.0 0.5
```

or click it with **Publish pose** (Step 4). Start with goals 1–2 m away in open space. Expected
log sequence:

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
just -f justfile.jazzy cbf-real-stop-nav   # stop sending autonomous velocity commands
just -f justfile.jazzy cbf-real-down       # stop the whole stack
```

---

## 6. Safety on the physical robot

### 6.1 Stopping the robot

Ctrl-C in a terminal **does not stop the robot**. In order of reliability:

1. The robot's power switch.
2. A gamepad connected to the robot, if used: holding its enable button (button 4 on our robot)
   takes over control; releasing it stops manual motion.
3. `just -f justfile.jazzy cbf-real-stop-nav`. Stops the controller and velocity smoother. The
   base stops once `twist_mux` stops receiving `/autonomous/cmd_vel`; verify this in the first
   test (§5 Step 6).
4. `just -f justfile.jazzy cbf-real-stop-teleop` cuts off the Foxglove Teleop panel. Closing
   Foxglove while driving also stops the base after `TELEOP_TIMEOUT` (0.5 s).
5. A new goal at the robot's current position. Cancels the running `FollowPath` goal, but only
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

---

## 7. Saving a map

```bash
just -f justfile.jazzy cbf-real-save-map     # writes maps/map.pgm + maps/map.yaml on the robot
```

The Jazzy stack always runs slam_toolbox in mapping mode and has no `map_server` or AMCL, so a
saved map **cannot be loaded back** for localisation. Every restart of `navigation` starts a new
map with a new origin. The saved map is a record of the experiment area and the input for the
offline benchmark (§8.1).

`cbf-real-sync` never copies `maps/` directories, so a map saved on the robot is not overwritten
or deleted by a sync from the laptop, and the robot's `maps/map.pgm` does not touch the
simulation map on the laptop. Copy it to its own directory on the laptop (§8.1).

`map_saver` writes `free_thresh: 0.25`. With this threshold, pixels saved as "unknown" (value 205)
are read back as **free**, also by `cbf_benchmark`. To keep unknown space an obstacle for the
planner, set `free_thresh: 0.19` in the copied `map.yaml`.

---

## 8. Running experiments

### 8.1 Recommended protocol

1. Map the area (§5 Step 7) and save the map (§7). Keep the same SLAM session (do not restart
   `navigation`) for the whole series, so every run uses the same map and frame.
2. Generate the PSF once (`cbf-real-refresh-map`) and keep it for the whole series.
3. Choose a fixed set of start/goal pairs. The start is always the robot's **current**
   position, so drive the robot back to the same start (Teleop) before each run, and check its
   pose in Foxglove.
4. For each configuration (baseline, then each value of `safety_weight_c`, `kappa`,
   `nominal_velocity`), run each start/goal pair several times. RRT\* is randomised: with the
   default `seed: -1` every run differs. Set `seed` to a fixed value
   (`ros2 param set /planner_node seed 7`) to get the same path for the same start and goal.
   The start pose on the real robot never repeats exactly, though, so the paths may still
   differ slightly.
5. Record the CSV (§8.3) and, for the real-robot part, observe the actual execution: did the
   robot reach the goal, did it touch anything, how close did it get to walls (Foxglove
   recording, or a tape measure).
6. For the statistics (many seeds, all configurations, both PSF sources), copy the saved map to
   the laptop and run the offline benchmark there (`README.md` §6.2). Keep both files together
   in their own directory, because `map.yaml` refers to `image: map.pgm`:

   ```bash
   mkdir -p maps/robot
   scp husarion@ROBOT_IP:/home/husarion/rosbot-xl-autonomy-foxglove/maps/map.{yaml,pgm} maps/robot/
   just cbf-benchmark --map maps/robot/map.yaml --random-goals 20 --runs 30 --c 0.2,0.5,0.8 --baseline
   ```

   The benchmark plans from `(0, 0)` by default, which on the robot's map is where SLAM started.
   Keep the robot runs for what the benchmark cannot measure: tracking, localisation and the
   real distance to obstacles.

### 8.2 Changing parameters without a restart [robot]

Open a shell (the ROS environment is sourced by the image's `.bashrc`):

```bash
just -f justfile.jazzy cbf-real-shell
```

`ros2 param set` changes a parameter of a running node. The value is lost when that node's
container is recreated: `cbf-planner` for `/planner_node`, `navigation` for `/controller_server`
and `/velocity_smoother`. To make a value permanent, edit the YAML on the laptop, sync and
restart the container (§3). Read a value back with `ros2 param get <node> <name>`, or all of them
with `ros2 param dump <node>`.

**Planner (`/planner_node`).** Applied from the next goal. Defaults from
`src/cbf_rrt_planner/config/cbf_rrt_planner.yaml`.

| Command | Default | Effect |
|---|---|---|
| `ros2 param set /planner_node safety_weight_c 0.5` | 0.2 | `c` of equation (6): trade-off between length and safety. 1.0 = length only, 0.0 = safety only. Sweep separately for each `psf_source` (`README.md` §6). |
| `ros2 param set /planner_node enable_cbf false` | true | `false` = plain RRT\* baseline: the CBF condition is measured but not enforced, cost = length. The PSF must already be generated, otherwise `mean_h`, `min_h` and `rejection_ratio` stay empty. |
| `ros2 param set /planner_node kappa 3.0` | 5.0 | κ of the CBF condition. Larger = more permissive. Only `nominal_velocity / kappa` matters: the approach margin in metres (0.12 m by default). |
| `ros2 param set /planner_node nominal_velocity 0.6` | 0.6 | Speed **assumed** by the CBF condition. Does not change the driving speed (use `FollowPath.desired_linear_vel` below). |
| `ros2 param set /planner_node goal_tolerance 0.5` | 1.0 | Radius [m] within which a tree node may connect to the goal. Smaller = less of a kink at the end, but more iterations needed. Does not change where the robot stops (`xy_goal_tolerance` below). |
| `ros2 param set /planner_node max_iterations 20000` | 10000 | RRT\* budget, always fully spent: planning time grows with it. Raise after `Planning FAILED`. |
| `ros2 param set /planner_node step_size 0.5` | 1.0 | Max edge length [m]. Smaller = finer paths, needs more iterations. |
| `ros2 param set /planner_node seed 7` | -1 | `>= 0` = the same start/goal gives the same path; `-1` = different every plan. |
| `ros2 param set /planner_node enable_smoothing true` | false | Greedy shortcutting after the search. |

These are frozen with the map and apply only after `cbf-real-refresh-map` (until then the planner
warns on every goal):

| Command | Default | Effect |
|---|---|---|
| `ros2 param set /planner_node psf_source centroid` | constant | Source term of the PSF: `constant` or `centroid`. |
| `ros2 param set /planner_node psf_boundary_flux 2.0` | 1.0 | `constant` only: slope of `h` next to walls. |
| `ros2 param set /planner_node inflation_robot_size 0.26` | 0.22 | Inflation radius [m]. Add a margin on the robot (0.25–0.27). |
| `ros2 param set /planner_node inflation false` | true | Point robot, as in the paper. |

**Controller and velocity smoother (`navigation` container).** Applied immediately, also to a
path that is being driven. Defaults from `config/nav2_real_params.yaml`.

| Command | Default | Effect |
|---|---|---|
| `ros2 param set /controller_server FollowPath.desired_linear_vel 0.4` | 0.6 | **Driving speed** [m/s] on straight segments. Keep it ≤ `max_velocity[0]` of the smoother. |
| `ros2 param set /velocity_smoother max_velocity "[0.4, 0.0, 1.0]"` | `[0.6, 0.0, 1.0]` | Hard limit on `vx`, `vy`, `ωz` sent to `twist_mux`. Give all three values with a decimal point. |
| `ros2 param set /velocity_smoother min_velocity "[-0.4, 0.0, -1.0]"` | `[-0.6, 0.0, -1.0]` | Lower limit; keep it symmetric to `max_velocity`. |
| `ros2 param set /velocity_smoother max_accel "[0.5, 0.0, 2.0]"` | `[1.0, 0.0, 2.0]` | Acceleration limit; lower = gentler start. |
| `ros2 param set /velocity_smoother max_decel "[-0.5, 0.0, -2.0]"` | `[-1.0, 0.0, -2.0]` | Deceleration limit (negative); lower magnitude = longer braking. |
| `ros2 param set /controller_server FollowPath.rotate_to_heading_angular_vel 0.6` | 1.0 | Speed [rad/s] of rotation in place before driving off. Keep it ≤ `max_velocity[2]`. |
| `ros2 param set /controller_server FollowPath.regulated_linear_scaling_min_radius 0.5` | 0.9 | RPP slows down on arcs tighter than this radius [m]. Smaller = less slowdown in turns. |
| `ros2 param set /controller_server FollowPath.approach_velocity_scaling_dist 0.3` | 0.6 | Distance [m] before the end of the path at which RPP starts slowing down. |
| `ros2 param set /controller_server FollowPath.lookahead_dist 0.8` | 0.6 | Pure-pursuit carrot distance [m]. Larger = smoother but cuts corners more. |
| `ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 0.2` | 0.10 | How close [m] to the end of the path the robot must stop. |
| `ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.5` | 0.30 | Accepted heading error [rad] at the end. |

### 8.3 Metrics

Each successful plan appends one line to
`src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv` on the robot. Column meanings and how
to read them are in `README.md` §6.1. In short:

- compare `mean_h` and `min_h` only between runs on the same frozen PSF (`h` depends on the map
  and on the PSF source);
- compare across PSF sources and inflation settings with `length` and the clearance columns
  (`min_clearance`, `mean_clearance`, `min_footprint_clearance`, in metres, measured on the map
  without inflation);
- note `kappa` and `nominal_velocity` yourself, because the run label does not include them.

The clearance columns describe the planned path. How close the robot actually came is a
separate measurement, e.g. the minimum `/scan` range during the run.

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
| `ros2 topic echo /autonomous/cmd_vel --csv` | Smoothed command sent to `twist_mux` |
| `ros2 topic echo /manual/cmd_vel --csv` | Teleop relay output |
| `ros2 param get /controller_server FollowPath.desired_linear_vel` | Live Nav2 value |
| `ros2 run tf2_ros tf2_echo map base_link` | Robot pose in the map frame |
| `ros2 run tf2_tools view_frames` | TF tree as a PDF |

---

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `USE_SIM_TIME is ...` and the recipe stops | Set `USE_SIM_TIME=False` in `.env.real` (laptop), sync. |
| `use_sim_time is false` warning in the planner log | Expected on the robot: the warning is meant for the Gazebo stack. |
| `pull access denied for ros_jazzy_cbf` / image not found | The image was never built on the robot. Run `cbf-real-image`. |
| `exec format error` or library errors when the planner starts | `install/` built elsewhere. `cbf-real-build` wipes it automatically when the architecture or ROS distro differs; otherwise run `cbf-real-rebuild`. |
| `workspace not built - run: ...` | The workspace was never built on the robot. Run `cbf-real-build` (not `just cbf-build`). |
| `cover_link -> laser was not published within 45s` in `cbf-real-nav-logs` | `laser-tf` is not running. Check `$COMPOSE ps -a`; `navigation` retries on restart. |
| `cbf-real-check` shows no `/scan` | Lidar snap not running: `snap services husarion-rplidar`. |
| Scan in Foxglove points backwards or sideways | Wrong `LASER_YAW` (§5 Step 5). |
| `Odometry is X s old ... refusing to plan` | `/odometry/filtered` is not published (`rosbot` snap), or TF `map → odom` is missing (`cbf-real-nav-logs`). |
| `PSF generation in progress - send the goal again` | The first goal triggers PSF generation. Wait for `PSF ready`, then resend. |
| `Planning REJECTED: start/goal falls into an occupied or unknown cell` | The point is outside the mapped free space. Map more, or choose another point. |
| `Planning REJECTED: start/goal ... (inflated zone)` | The point is free, but closer to an obstacle than `inflation_robot_size`. Drive the robot away from the wall, pick a goal further from it, or lower the radius and call `cbf-real-refresh-map`. |
| `Inflation parameters changed since the PSF was generated` / `PSF source parameters changed ...` | Inflation or PSF source was changed with `ros2 param set`. Run `cbf-real-refresh-map` to apply it. |
| `Planning FAILED after N iterations` | No admissible path within the budget. Try a nearer goal, a larger `max_iterations`, or a weaker CBF (`kappa` up / `nominal_velocity` down). |
| `MAP MISMATCH DETECTED` | The map grew since the PSF was generated. Run `cbf-real-refresh-map`. |
| `FollowPath action server not available` | `navigation` is not running or its lifecycle nodes are not active. Check `cbf-real-nav-logs`. |
| `FollowPath: ABORTED` | Reason is in `cbf-real-nav-logs` (progress checker, controller exception). `0.00 m/s` in the feedback throughout means the controller never drove; look at `/lookahead_point` and `/cmd_vel_nav`. |
| Controller drives (`/cmd_vel_nav` non-zero) but the base does not move | `/autonomous/cmd_vel` must be `TwistStamped` (`cbf-real-check`). Check that no higher-priority `twist_mux` input (gamepad, teleop) is active. |
| Teleop panel does nothing | Panel topic must be `/manual/cmd_vel_unstamped`; check `cbf-real-teleop-logs` for `relay up`. |
| Foxglove cannot connect to `ws://ROBOT_IP:8765` | On the robot: `ss -ltnp \| grep 8765` and `$COMPOSE logs foxglove-ds`. From the laptop: `nc -vz ROBOT_IP 8765`; a timeout means another network or a firewall (`sudo ufw status`). |
