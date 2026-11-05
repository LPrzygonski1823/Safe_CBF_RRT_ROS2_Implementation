# Safe-CBF-RRT* for ROSbot XL

Implementation and experimental verification of **Safe-CBF-RRT\***: an RRT\* planner whose edge
cost and edge admissibility are driven by a **Poisson Safety Function (PSF)** `h(x,y)` derived
from the occupancy map. The planner runs as a standalone ROS 2 node that feeds paths to the
Nav2 controller of a Husarion ROSbot XL, both in Gazebo (ROS 2 Humble) and on the physical robot
(ROS 2 Jazzy).

This file describes the project: architecture, algorithms, configuration, metrics, repository
layout and limitations. Step-by-step instructions and command references are in two guides:

| Guide | Contents |
|---|---|
| [`SIM_GUIDE.md`](SIM_GUIDE.md) | Gazebo simulation: setup, start-up, `just cbf-*` recipes, map recording, experiments, diagnostics |
| [`REAL_GUIDE.md`](REAL_GUIDE.md) | Physical ROSbot XL (Jazzy): connection, deployment, `justfile.jazzy` recipes, mapping, teleop, safety, experiments, troubleshooting |

## Note on original source

This repository is built on top of the
[husarion/rosbot-xl-autonomy](https://github.com/husarion/rosbot-xl-autonomy) project. The
original codebase is licensed under the Apache License 2.0 by Husarion.

**Modifications:** the primary addition to this fork is the `src/cbf_rrt_planner` package and
related tools, which implement a Safe-CBF-RRT\* path planner for the engineering thesis
purposes. The original Husarion licence has been preserved in the `LICENSE` file.

## Quick start

Simulation (details in `SIM_GUIDE.md`):

```bash
just cbf-up              # build + start everything, then follow planner logs
just cbf-refresh-map     # freeze the map and generate the PSF
just cbf-goal 2.0 1.5    # send a goal in map coordinates
```

Physical robot (details in `REAL_GUIDE.md`; settings in `.env.real`):

```bash
just -f justfile.jazzy cbf-real-sync ROBOT_IP    # [laptop] deploy the code
just -f justfile.jazzy cbf-real-image            # [robot] build the image (once)
just -f justfile.jazzy cbf-real                  # [robot] build + start, follow planner logs
just -f justfile.jazzy cbf-real-refresh-map      # [robot] freeze the map and generate the PSF
just -f justfile.jazzy cbf-real-goal 1.0 0.5     # [robot] send a goal in map coordinates
```

Offline benchmark of the planner, no simulation needed (details in §6.2–6.3):

```bash
just cbf-benchmark --random-goals 20 --runs 30 --c 0.2,0.5,0.8 --baseline
just cbf-benchmark-plot    # plots + summary in src/cbf_rrt_planner/benchmark/plots/
```

Foxglove runs at `http://localhost:8080` in simulation. On the robot, connect the Foxglove desktop
app to `ws://ROBOT_IP:8765` (a web UI is also served at `http://ROBOT_IP:8080`).

---

## 1. System overview

The stack is a set of Docker containers. The simulation runs ROS 2 **Humble**:
`compose.simulation.yaml` + `compose.cbf.yaml`, driven by `justfile` and `.env`. The physical
robot runs ROS 2 **Jazzy** natively (drivers and lidar as snaps on the host), so it has its own
stack: `compose.jazzy.yaml` + `dockerfile.jazzy`, driven by `justfile.jazzy` and `.env.real`. The
planner source is the same; it is built against the distro of its container.

| Component | Simulation (Humble) | Physical robot (Jazzy) |
|---|---|---|
| robot | `rosbot` container: Gazebo simulation of the ROSbot XL with a custom `.sdf` world (`dockerfile.rosbot`) | `rosbot` snap on the host: drivers, `robot_localization` EKF (`/odometry/filtered`, TF `odom → base_link`), `twist_mux` |
| lidar | simulated in Gazebo | `husarion-rplidar` snap on the host (`/scan`) + `laser-tf` container (static `cover_link → laser`) |
| `navigation` | Husarion Nav2 bringup: `map_server`, AMCL, `planner_server`, `bt_navigator`, `controller_server`, `velocity_smoother`, costmaps (slam_toolbox when `SLAM=True`) | `launch/nav2_real.launch.py`: slam_toolbox (mapping), `controller_server`, `velocity_smoother` only |
| `cbf-planner` | **this project's** `planner_node`: PSF generation + RRT\* + `FollowPath` client (`dockerfile.dev`, `ros:humble`) | the same (`dockerfile.jazzy`, `ros:jazzy`) |
| `foxglove`, `foxglove-ds` | Web visualisation on port `8080`, WebSocket bridge on port `8765` | the same; the desktop app connects to `8765` directly |
| `teleop-relay` | — | Foxglove Teleop panel (`Twist`) → `twist_mux` manual input (`TwistStamped`) |
| `static-tf` | Identity `map → odom` transform (only valid when `SLAM=False`) | — |

Nav2's own global planner and BT navigator are not used for planning. The planner node replaces
the global planner and talks to `controller_server` directly through the `FollowPath` action. In
simulation, `planner_server` and `bt_navigator` still run, and **take over** if anything publishes
on `/goal_pose` (e.g. Foxglove's "Publish pose" with default settings). The robot stack does not
start them. Planner goals go to `/cbf_goal_pose`.

---

## 2. Data flow

```
                maps/map.pgm + map.yaml  (SLAM=False)   or   slam_toolbox  (SLAM=True)
                             |
                     [ navigation: map_server / slam_toolbox ]
                             |  /map  (OccupancyGrid, transient_local)
                             v
  Gazebo / ROSbot EKF --/odometry/filtered--> [ planner_node ] <--/cbf_goal_pose-- user
                                                     |
                                      (1) freeze map, build PSF (async)
                                                     |
                                      (2) RRT* search on the frozen map
                                                     |
                                      (3) resample path to dense poses
                                                     |
                               /planned_path  +  FollowPath action goal
                                                     v
                                        [ controller_server: RPP ]
                                                     |  /cmd_vel_nav
                                                     v
                                        [ velocity_smoother ] --/cmd_vel--> Gazebo base
                                                     |
                                                     +--/autonomous/cmd_vel--> twist_mux --> ROSbot base
```

On the robot, `velocity_smoother` publishes `TwistStamped` on `/autonomous/cmd_vel`, one of the
inputs of the snap's `twist_mux`, and there is no `map_server`: the map always comes from
slam_toolbox.

The robot's pose in the map comes from TF: `map → odom` (static-tf, AMCL or slam_toolbox) composed
with `odom → base_link` (EKF). The planner transforms odometry and goals into `map`.

---

## 3. How the planner works

### 3.1 PSF pipeline (runs in `std::async`)

`PSFGenerator::generate()` executes four stages on a frozen copy of the map. Before that, the
node dilates every occupied or unknown cell by `inflation_robot_size` (`inflateObstacles`, disc
of cell centres), so the PSF is built on the robot's configuration space: `h = 0` where the robot
**body** would touch an obstacle, not where its centre would. The geometric collision check uses
the same inflated map.

1. **`ObstacleSegmentation`**: thresholds cells at `occupied_threshold` (unknown cells count as
   occupied), finds connected components (4-connectivity), and classifies every cell as `Free`,
   `ObstacleInterior`, `ObstacleBoundary` (∂Ω_i) or `DomainBoundary` (∂Ω). Each obstacle gets a
   centroid `c_i`, computed from all of its cells.
2. **`PoissonSolver::solveLaplaceVector`**: solves the Laplace equation twice (for `u_x` and
   `u_y`) with Gauss-Seidel + successive over-relaxation (ω = 1.99, tolerance 1e-6, at most
   20 000 sweeps). The Dirichlet condition on obstacle boundaries is
   `u = boundary_scale · (p − c_i)`, i.e. the vector from the obstacle's centroid to the boundary
   cell, so the field points away from each obstacle; `u = 0` on the domain boundary. This is equation (4) of the paper. `c_i` read as a
   scalar `boundary_scale = 1` plus the centroid, and `u = 0` on ∂Ω, are implementation
   assumptions: the paper does not specify them.
3. **`computeJacobianFrobeniusNorm`**: Frobenius norm of the Jacobian of `u`, used as the source
   term of equation (5). The choice of the Frobenius norm is also an assumption.
4. **`solvePoissonScalar`**: solves the Poisson equation `∇²h = −‖∇u‖` for `h`, with `h = 0` on
   both obstacle and domain boundaries. Then `computeGradient` gives `∇h` (central differences).

**Alternative source (`psf_source: constant`).** Stages 2 and 3 are skipped and the Poisson
equation gets a constant source, `∇²h = −f0`, with `h = 0` on all boundaries. `f0` follows the
average flux method of Bahati et al. (RSS 2025): by the divergence theorem,
`f0 = psf_boundary_flux · perimeter / area` of the free space, so the mean slope of `h` on the
walls equals `psf_boundary_flux` and next to a wall `h ≈ psf_boundary_flux · distance` [m].
`h` then depends only on the shape of the free space, not on how segmentation split it into
obstacles or where their centroids fall. It is also what the author's MATLAB reference
effectively computes: its obstacle boundary condition is overwritten by a constant, so `u` is
constant and the source `‖u‖` is constant too. It needs one SOR solve instead of three.

`getH()` / `getGradientH()` bilinearly interpolate these grids, so the planner can query `h` at
arbitrary continuous coordinates. `exportToCsv()` dumps `h.csv`, `u_x.csv`, `u_y.csv` to
`src/cbf_rrt_planner/maps/psf_debug/` for `tools/visualize_psf.py`.

The PSF is generated **once** and reused for every subsequent goal. It is intentionally frozen:
regenerating it per goal would cost seconds and make results non-reproducible. Generation is
triggered either by the `/refresh_map` service or, once, lazily by the first goal. A 500 ms timer
polls the background task. Each goal compares the live map's origin and size with the frozen
ones and warns `MAP MISMATCH DETECTED` if they differ.

With the centroid source, `h` is in m² (`u` is in metres, `∇u` is dimensionless), so it is not
a distance; its value depends on the size and centroid placement of nearby obstacles, not only
on clearance. With the constant source, `h` is in metres and approximates the distance only
next to walls; in open areas it grows with the square of the local width.

### 3.2 RRT\* search

`RrtStarPlanner::plan()` is classic RRT\* (Karaman & Frazzoli) with two CBF hooks:

- **Sampling**: uniform over the map bounds; with probability `goal_bias`, the goal itself.
- **Nearest / neighbours**: `SpatialGrid` spatial hashing, ~O(1) per query instead of O(n).
- **Neighbour radius**: `r_n = gamma · sqrt(log n / n)`, clamped to `[step_size, neighbor_radius]`.
  The shrinking radius is what preserves asymptotic optimality.
- **Edge admissibility** (`isEdgeAdmissible`): geometric collision test (robot centre on the
  inflated map, samples every half cell), then the CBF condition of equation (3) sampled every `edge_sample_step` along
  the edge: `∇h · v ≥ −κ·h`, with `v` the unit edge direction times `nominal_velocity`. Only the
  ratio `nominal_velocity / kappa` matters: it is the approach margin in metres (0.12 m with the
  defaults). The condition is always *measured* (so the baseline also reports a rejection ratio)
  but only *enforced* when `enable_cbf: true`.
- **Edge cost** (`edgeCost`): equation (6) as a path integral,

  ```
  cost(edge) = L · ( c + (1 − c) · mean(1/h) )
  ```

  `c = safety_weight_c`. The safety term **must** scale with `L`, otherwise it degenerates into a
  flat per-edge penalty: the optimum then depends on how the path is split into nodes, and
  skimming an obstacle costs the same over 5 cm as over 5 m. `1/h` is integrated along the edge
  rather than inverting the mean of `h`, so a brief dip next to a wall is not washed out by the
  clear part of the edge. `h` is clamped to 1e-6 before inverting.
- **Goal handling**: a single goal node is kept and rewired to cheaper parents. There is **no
  early exit**, so every plan burns the full `max_iterations` budget (~0.5 s for 10 000 on a
  laptop).

Optional `PathSmoother` (greedy shortcutting, `enable_smoothing`) runs afterwards and typically
buys only a few percent of length.

### 3.3 Path handover to Nav2

`toPathMsg()` does two things that Nav2 requires:

- **Orientation**: every pose is oriented along its outgoing segment. The last pose takes the
  orientation requested in `/cbf_goal_pose`, which is the yaw the goal checker compares against.
- **Resampling**: every segment is split to `path_pose_spacing` (0.05 m). This is not cosmetic:
  RPP discards plan poses farther than half the local costmap extent (~2 m for a 4×4 m costmap),
  and RRT\* rewiring can produce edges up to `neighbor_radius` long. A sparse path leaves the
  controller with a single pose and no usable lookahead point.

Metrics are logged **before** resampling, so the CSV reflects the raw RRT\* geometry.

RPP then follows the path at 15 Hz: it picks a lookahead point 0.6 m ahead, steers along an arc
of curvature `2y/L²`, rotates in place when the heading error exceeds 45°, and `velocity_smoother`
applies the final limits before `/cmd_vel`.

---

## 4. Runtime interface

| Kind | Name | Type | Notes |
|---|---|---|---|
| sub | `/map` | `nav_msgs/OccupancyGrid` | `transient_local` QoS, required to receive the latched static map |
| sub | `/odometry/filtered` | `nav_msgs/Odometry` | configurable via `odom_topic`; transformed into `map` |
| sub | `/cbf_goal_pose` | `geometry_msgs/PoseStamped` | the planning trigger; orientation becomes the goal yaw |
| pub | `/planned_path` | `nav_msgs/Path` | dense, oriented path (for visualisation) |
| srv | `/refresh_map` | `std_srvs/Trigger` | re-freeze the live map and regenerate the PSF |
| action client | `FollowPath` | `nav2_msgs/FollowPath` | `controller_id: "FollowPath"` |

Goal handling cancels any `FollowPath` goal still in progress, logs acceptance/rejection, and
reports distance-to-goal and speed from the action feedback.

---

## 5. Configuration

### 5.1 Planner: `src/cbf_rrt_planner/config/cbf_rrt_planner.yaml`

| Parameter | Default | Meaning / effect |
|---|---|---|
| `step_size` | 1.0 | Max extension length per iteration. Smaller = finer paths, needs more iterations. |
| `max_iterations` | 10000 | Fixed budget, always fully spent (~0.5 s on a laptop). Raise for better convergence. |
| `goal_bias` | 0.05 | Probability of sampling the goal directly. |
| `goal_tolerance` | 1.0 | Radius within which a node may connect to the goal. Large values put a kink at the end. |
| `neighbor_radius` | 5.0 | Upper bound on `r_n` **and** the `SpatialGrid` cell size. |
| `gamma` | 5.0 | Scale in `r_n = gamma·sqrt(log n / n)`. Larger = more rewiring, slower, straighter. |
| `enable_cbf` | true | `false` = plain RRT\* baseline (CBF measured but not enforced, cost = length). |
| `kappa` | 5.0 | κ of equation (3). Larger = more permissive CBF condition. |
| `safety_weight_c` | 0.2 | **Main experimental knob.** 1.0 = length only, 0.0 = safety only. |
| `nominal_velocity` | 0.6 | Assumed speed along an edge when evaluating the CBF condition. Not sent to the controller (see `desired_linear_vel` in §5.2). |
| `occupied_threshold` | 65 | Occupancy value treated as obstacle (0–100); unknown counts as occupied. |
| `enable_smoothing` | false | Greedy shortcutting after the search. |
| `psf_source` | constant | Source term of the Poisson equation: `centroid` (paper, equations 4–5) or `constant` (average flux, Bahati et al.). |
| `psf_boundary_flux` | 1.0 | `constant` only: mean slope of `h` on the walls; `h ≈ psf_boundary_flux · distance` next to them. |
| `inflation` | true | Dilate obstacles before the PSF is generated. `false` = point robot, as in the paper. |
| `inflation_robot_size` | 0.22 | Inflation radius [m]: distance from the robot centre to its farthest point. 0.22 ≈ circumscribed radius of the ROSbot XL (0.216 m, safe for any heading); 0.14 = inscribed radius. On the physical robot add a margin for map discretisation, localisation and tracking error (e.g. 0.25–0.27). |
| `seed` | -1 | RRT\* sampling seed. `-1` = different every plan; `>= 0` = the same start/goal (and parameters) gives the same path. |
| `footprint_length` / `footprint_width` | 0.33 / 0.28 | Rectangle used by the `min_footprint_clearance` metric [m], length along the heading. Metric only, the planner still uses the inflation disc. |
| `clearance_search_radius` | 2.0 | Obstacles are searched for up to this distance; the clearance metrics are capped at it [m]. |
| `path_frame_id` | map | Frame of the published path. |
| `path_pose_spacing` | 0.05 | Resampling step. Keep ≈ local costmap resolution; larger breaks RPP. |
| `odom_topic` | /odometry/filtered | Source of the planning start pose. |
| `odom_timeout` | 1.0 | Refuse to plan from odometry older than this [s]. |
| `metrics_csv_path` | …/psf_debug/planning_metrics.csv | Where results are appended. |

- Not exposed in YAML: `edge_sample_step`, set in code to `map_resolution · 0.5`, i.e. the
  spacing of CBF samples along an edge.
- The defaults hardcoded in the `PlannerParams` struct are **not** used on the ROS path.
  `loadPlannerParams()` always overrides them from ROS parameters.
- All parameters except `odom_topic` (read once at start-up) are re-read **on every goal**, so
  `ros2 param set /planner_node <name> <value>` takes effect immediately without a restart. This
  is the fastest way to sweep `safety_weight_c`. Ready-made commands for the planner, RPP and the
  velocity smoother (speed, goal tolerance): `SIM_GUIDE.md` §7.3 and `REAL_GUIDE.md` §8.2.
- Exception: `inflation`, `inflation_robot_size`, `psf_source` and `psf_boundary_flux` are
  applied when the map is frozen, because the PSF depends on them. After changing them call `/refresh_map`; until then the node keeps the
  frozen radius (for the collision check too) and warns on every goal.
- `config/` and `launch/` are **symlinked** into `install/` (`colcon build --symlink-install`).
  Editing the YAML or the launch file needs only a planner container restart, not a rebuild.
  Editing C++ needs a rebuild.
- `use_sim_time` comes from the launch file (`USE_SIM_TIME` in the container): `True` in Gazebo,
  `False` on the physical robot.

### 5.2 Controller: `config/nav2_rpp_params.yaml`

Simulation. Selected by `CONTROLLER` in `.env`. Values that matter for path following (already
tuned):

| Parameter | Value | Why |
|---|---|---|
| `odom_topic` | `odometry/filtered` | The Nav2 default `odom` has **no publisher** here; RPP needs a live measurement to ramp its angular command. |
| `lookahead_dist` | 0.6 | Pure-pursuit curvature is `2y/L²`; short lookaheads cause steering oscillation. |
| `rotate_to_heading_min_angle` | 0.785 | Below this heading error, normal steering corrects instead of stopping to rotate. |
| `max_angular_accel` / `rotate_to_heading_angular_vel` | 3.2 / 1.5 | Matched to `velocity_smoother` limits (`max_accel[2]`, `max_velocity[2]`). |
| `movement_time_allowance` | 30.0 | Rotating in place produces no translation, so the progress checker needs slack. |
| `desired_linear_vel` | 0.6 | Cruise speed, equal to `velocity_smoother` `max_velocity[0]` and to the planner's `nominal_velocity`. |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.1 / 0.3 | Goal checker acceptance. |
| `use_collision_detection` | false | RPP does **not** stop for obstacles in the local costmap. |

`config/nav2_dwb_params.yaml`, `nav2_mppi_params.yaml` and their `*_webots_*` variants come from
the original Husarion project.

The physical robot uses `config/nav2_real_params.yaml` (RPP with the same lookahead, but
`desired_linear_vel` 0.6 m/s as in simulation, `velocity_smoother` limit 0.6 m/s / 1.0 rad/s, stamped velocity
output for `twist_mux`) and `config/slam_toolbox_real.yaml`. Both are described in
`REAL_GUIDE.md` §2.2.

### 5.3 Stack: `.env` (simulation) and `.env.real` (robot)

`.env`, simulation:

| Variable | Value | Notes |
|---|---|---|
| `USE_SIM_TIME` | unset or `True` | Clock of the planner container (Gazebo `/clock`). |
| `SLAM` | `False` | `False` loads the map named by `MAP_YAML`. With `True`, comment out the `static-tf` service. |
| `MAP_YAML` | `map_u.yaml` | Map file inside `maps/` loaded when `SLAM=False`. |
| `CONTROLLER` | `rpp` | Picks which `config/nav2_*_params.yaml` is mounted. |
| `MECANUM` | `True` | Holonomic base. RPP itself never commands `vy`. |
| `SAVE_MAP_PERIOD` | 15 | Map autosave, only when `SLAM=True` (passed to Nav2 in `compose.simulation.yaml`). |

`.env.real`, physical robot (details in `REAL_GUIDE.md` §2.1):

| Variable | Value | Notes |
|---|---|---|
| `USE_SIM_TIME` | `False` | Required; the `justfile.jazzy` build recipes refuse to run otherwise. |
| `LASER_X/Y/Z`, `LASER_YAW` | `0, 0, 0`, `3.14159265` | Static TF `cover_link → laser`. |
| `ROBOT_IP` | robot address | Bridge address for the optional Foxglove web UI. |

### 5.4 Maps and worlds

- `maps/map.pgm` + `maps/map.yaml`: pre-recorded static map (0.05 m/pixel, 466 × 466 px,
  origin `[-11.6, -13.6, 0]`) of `worlds/custom_world_3.sdf`. Changing the world invalidates it.
  Recording a new one is described in `SIM_GUIDE.md`.
- `maps/map_u.pgm` + `maps/map_u.yaml`: synthetic map with U-shaped obstacles, same size and
  outer wall as `map.pgm`. `tools/generate_u_world.py` generates it together with the matching
  `worlds/custom_world_u.sdf` (default simulation world, `MAP_YAML=map_u.yaml`) from one obstacle
  list.
- Maps recorded on the physical robot stay on the robot (`maps/` is never synced); copy them to
  their own directory, e.g. `maps/robot/` (`REAL_GUIDE.md` §7–8).
- With `free_thresh: 0.25`, pixels saved as "unknown" (value 205) are loaded as **free**. Set
  `free_thresh` below 0.196 to keep them unknown, and therefore obstacles for the planner.
- The `map` frame's origin is where the robot stood when the map recording started. Goal
  coordinates are always in this frame, not in Gazebo world coordinates.

---

## 6. Metrics

### 6.1 Per-plan CSV (`planner_node`)

Each successful plan appends one line to `planning_metrics.csv`:

| Column | Meaning |
|---|---|
| `run_label` | `cbf_c<value>` or `baseline_rrt_star`, with `_infl<radius>` appended when inflation is on and `_psf-centroid` / `_psf-constant` when a PSF is available |
| `success` | 1 if a path was found |
| `length` | path length [m] |
| `mean_h` | length-weighted mean of `h` along the path |
| `min_h` | smallest `h` along the path, i.e. the tightest point |
| `min_clearance` | smallest distance from the path (robot centre) to an obstacle [m] |
| `mean_clearance` | length-weighted mean of that distance [m] |
| `min_footprint_clearance` | smallest gap between the robot rectangle (`footprint_length` × `footprint_width`, aligned with the path) and an obstacle [m]; `≤ 0` = the body would touch or overlap one |
| `iterations_used` | always `max_iterations` (no early exit) |
| `iterations_to_first_solution` | iteration that first reached the goal: the paper's "Iterations" |
| `cbf_candidates`, `cbf_rejections`, `rejection_ratio` | edges checked by the CBF condition, edges rejected, their ratio |

Read them carefully:

- `iterations_used` is always `max_iterations`. The informative column is
  `iterations_to_first_solution`.
- `mean_h` is length-weighted; `min_h` is the tightest point on the path. **Both are needed**: a
  high mean can still hide a near-obstacle dip.
- `h` is a Poisson function value (m² for the centroid source, m for the constant source), not
  a distance. Compare between runs on the same frozen PSF rather than converting to distance or
  comparing across maps.
- **The PSF source changes the meaning of `c` and of the `h` metrics.** The cost contains `1/h`,
  and the two sources give `h` of different scale and spatial distribution (on the bundled map,
  inflated by 0.22 m: `h` = 2.24 vs 0.58 at the goal `(-6.98, -3.38)`, 0.62 vs 0.51 at the start).
  The same `safety_weight_c` therefore weights safety differently, and `mean_h` / `min_h` cannot
  be compared between `_psf-centroid` and `_psf-constant` runs. Sweep `c` separately for each
  source and compare the sources by `length` and by the clearance columns, not by `h`.
- The clearance columns are the geometric ground truth, independent of `h`, the PSF source and
  the inflation:
  - they are measured on the live map **without inflation**, after smoothing, with unknown
    cells counted as obstacles and every occupied cell treated as its full square;
  - the path is sampled every map cell (0.05 m) and every vertex is checked with the heading
    before and after the turn;
  - `min_footprint_clearance` assumes the robot faces along the path, which is how RPP drives.
    In-place rotations at sharp vertices sweep the circumscribed disc, which only the inflation
    radius covers;
  - values are capped at `clearance_search_radius` (2 m), so in open areas `mean_clearance`
    saturates.
- With inflation on, `min_clearance` cannot drop meaningfully below `inflation_robot_size`.
  The cell discretisation of the inflation and of the collision check allows a few millimetres
  less: 0.216 m measured at 0.22 m. The plain RRT\* baseline sits right at this bound. The
  useful question is how far above it each configuration keeps the path, and at what cost in
  `length`.
- When the column layout changes (as with the clearance columns), the existing file is moved
  to `planning_metrics_old<N>.csv` and a new one is started, so a file never mixes layouts.
- The label includes the inflation radius but not `kappa` or `nominal_velocity`: note those
  values yourself when you change them.
- Measured effect of `c` on one corridor in simulation (3.81 m straight-line): `c=0.2` → length
  7.34, `mean_h` 2.51, `min_h` 1.18; `c=0.9` → length 6.12, `mean_h` 1.57, `min_h` 0.35.

`python3 src/cbf_rrt_planner/tools/visualize_psf.py [directory]` renders `h`, `u_x` and `u_y`
from the CSV dumps to `psf_visualization.png`.

### 6.2 Offline benchmark (`cbf_benchmark`)

`cbf_benchmark` runs the planner core (the same C++ as `planner_node`) without ROS, Gazebo or
the robot. It is the tool for the numerical analysis: many plans with identical inputs, fully
repeatable. It measures the **plans**, not their execution (no controller, no localisation
error); execution has to be measured on the robot or in Gazebo.

What one invocation does:

1. Reads the planner parameters from `src/cbf_rrt_planner/config/cbf_rrt_planner.yaml` (any key
   can be overridden with `--set key=value`) and loads a `map_server` map (`maps/map.yaml` by
   default). The image is converted the way `nav2_map_server` converts it, so the grid is the one
   the node receives on `/map`.
2. Applies the inflation and generates the PSF **once** (source and inflation as configured).
3. Checks that the start (default `0,0`, `--start X,Y`) is free in the inflated map, and exits
   otherwise.
4. Takes goals from a file (`--goals`, one `x,y` per line) or draws `--random-goals N` of them.
   Drawn goals are taken from the free cells that are connected to the start and at least
   `--min-goal-dist` away. Because a path is known to exist, every failure is a planner failure.
5. For every goal and every seed, runs every configuration:
   - each `safety_weight_c` from `--c 0.2,0.5,0.8`;
   - with `--baseline`, also plain RRT\*.

   The seed is `--seed + goal_id · runs + k`. It is the same for all configurations, so the
   results can be compared **in pairs** (same goal, same random sequence, different `c`).
6. Appends one row per plan to `--out` (default `src/cbf_rrt_planner/benchmark/results.csv`)
   and the path vertices to `<out>_paths.csv`. It also prints a per-goal summary.

Main columns of `results.csv`:

| Group | Columns |
|---|---|
| Identification | `batch` (timestamp of the invocation), `tag` (`--tag`), `map`, `run` (row id in the batch, key into `_paths.csv`), `seed` |
| Frozen setup | `psf_source`, `psf_boundary_flux`, `inflation_radius`, `kappa`, `nominal_velocity`, `max_iterations`, `smoothing` |
| Configuration | `config` (`cbf` / `baseline`), `enable_cbf`, `safety_weight_c` (`nan` for the baseline) |
| Query | `goal_id`, `start_x`, `start_y`, `goal_x`, `goal_y`, `straight_dist` |
| Result | `success`, `failure` (`none` / `start_blocked` / `goal_blocked` / `no_path`), `length`, `length_ratio` = `length / straight_dist`, `mean_h`, `min_h`, `min_clearance`, `mean_clearance`, `min_footprint_clearance`, iteration and rejection columns as in §6.1 |
| Timing | `plan_time_s` (search + smoothing), `psf_time_s` (once per batch) |

Running it:

```bash
just cbf-benchmark --random-goals 20 --runs 30 --c 0.2,0.5,0.8 --baseline --tag map1
just cbf-benchmark --random-goals 20 --runs 30 --c 0.2,0.5,0.8 --baseline --tag map1 \
                   --set psf_source=centroid
just cbf-benchmark --goals my_goals.txt --runs 10 --set inflation=false
just cbf-benchmark --help
```

`just cbf-benchmark` builds the workspace and runs the binary in the dev container. The
simulation does not have to be running. All batches append to the same file. The same
`--goal-seed` (default 1) on the same map draws the same goals, so batches with different
PSF sources or inflation can be compared goal by goal.

One plan takes about 0.5 s on a laptop with the default 10 000 iterations. The PSF takes about
1.3 s (constant source) or about 4.6 s (centroid source). 20 goals × 30 seeds × 4
configurations ≈ 2400 plans ≈ 20 minutes.

Things to keep in mind when reading the results:

- The baseline does not use `h` in its cost. Its paths are identical across PSF sources for the
  same seeds, which is a useful sanity check. Only `mean_h`, `min_h` and `rejection_ratio`
  differ, because they are measured on the respective PSF.
- `mean_h` / `min_h` still compare only within one PSF source (§6.1). Across sources, use
  `length_ratio` and the clearance columns.
- A different map needs a free start: pass `--start` if `(0, 0)` is inside an obstacle or
  closer to one than the inflation radius.

### 6.3 Plots

```bash
just cbf-benchmark-plot                                   # default results.csv
just cbf-benchmark-plot path/to/results.csv --batch 20261013-215632 20261013-215653 --goal-id 0 3
```

`tools/plot_benchmark.py` runs on the host (`numpy`, `matplotlib`, `Pillow`). It groups the
rows into configurations, each one a PSF source, an inflation radius and a `c` value (or the
baseline). It writes the following to `<results dir>/plots/`:

| File | Content |
|---|---|
| `pareto.png` | one dot per plan: `length_ratio` vs `min_footprint_clearance`; large dots are the medians. The main length-vs-safety trade-off figure |
| `boxplots.png` | distributions of `length_ratio`, `min_clearance`, `mean_clearance`, `min_footprint_clearance`, `plan_time_s`, plus the success rate |
| `paths_goal<N>.png` | every path to goal N over the map. The median-length path of each configuration is drawn with the robot rectangle every `--footprint-step` (0.3 m), i.e. the area swept by the body, which shows directly how much margin the inflation and the CBF leave |
| `summary.csv` | per configuration: plans, success rate, number of plans with footprint contact, median and quartiles of every metric (also printed) |

---

## 7. Repository structure

```
src/cbf_rrt_planner/
  include/cbf_rrt_planner/
    occupancy_grid_data.hpp   ROS-independent map struct (width, height, resolution, origin, data)
    grid2d.hpp                Grid2D / GridI: dense double / int grids
    obstacle_segmentation.hpp connected components, centroids, per-cell classification
    poisson_solver.hpp        SOR solver: Laplace(u), Jacobian norm, Poisson(h), gradient
    psf_generator.hpp         public PSF facade: generate / getH / getGradientH / exportToCsv
    collision_checker.hpp     geometric point/edge tests + workspace bounds for sampling
    map_inflation.hpp         obstacle dilation by the robot radius (configuration space)
    spatial_grid.hpp          spatial hashing for nearest / radius queries
    rrt_star_types.hpp        PlannerParams, PlanningResult, TreeNode, PlanningFailure
    rrt_star_planner.hpp      the planner itself
    path_smoother.hpp         greedy shortcutting
    cbf_utils.hpp             computeAverageH, computeAverageInverseH, computeMinH, checkCbfCondition
    ros_conversions.hpp       the ONLY ROS<->core boundary: toGridData, toPathMsg
    planning_metrics.hpp      CSV writers (header check, old layouts moved aside)
    path_clearance.hpp        geometric clearance of a path: centre point and oriented rectangle
    map_loader.hpp            ROS-free map.yaml + PGM reader (nav2_map_server conversion)
  src/                        implementations, planner_node.cpp (the ROS node) and
                              cbf_benchmark.cpp (offline benchmark, no ROS)
  CMakeLists.txt              cbf_rrt_planner_core (ROS-free static library) linked by both
                              planner_node and cbf_benchmark
  config/cbf_rrt_planner.yaml planner parameters (symlinked into install/, also read by cbf_benchmark)
  launch/cbf_planner.launch.py  launch with use_sim_time and params_file arguments
  maps/psf_debug/             PSF field dumps + planning_metrics.csv
  benchmark/                  default output of cbf_benchmark (results.csv, results_paths.csv, plots/)
  tools/visualize_psf.py      renders psf_visualization.png from the CSV dumps
  tools/plot_benchmark.py     benchmark plots: Pareto, boxplots, paths with swept footprint, summary

config/nav2_*_params.yaml     simulation Nav2 configs, one per controller (rpp / mppi / dwb)
config/nav2_real_params.yaml  robot Nav2 config: RPP, local costmap, velocity_smoother (Jazzy)
config/slam_toolbox_real.yaml robot slam_toolbox config
config/layout.json            default Foxglove web layout (3D, Teleop, Image, RosOut panels)
launch/nav2_real.launch.py    robot Nav2 subset: slam_toolbox, controller_server, velocity_smoother
worlds/custom_world_*.sdf     custom Gazebo worlds
maps/map*.pgm, maps/map*.yaml pre-recorded static maps of the simulation worlds
tools/generate_u_world.py     generates maps/map_u.* and worlds/custom_world_u.sdf from one obstacle list
compose.simulation.yaml       simulation (Humble): Gazebo + Nav2 + Foxglove + static-tf
compose.cbf.yaml              simulation overlay adding the cbf-planner service
compose.jazzy.yaml            physical robot (Jazzy): Nav2/SLAM, laser TF, planner, Foxglove, teleop relay
dockerfile.dev                simulation planner container (ros:humble + nav2_msgs + rclcpp_action)
dockerfile.rosbot             Gazebo container (rosbot-xl-gazebo + nav2_msgs + rclcpp_action)
dockerfile.jazzy              robot image (ros:jazzy + Nav2 + slam_toolbox + foxglove_bridge), shared by all robot containers
scripts/cbf_planner_entrypoint.sh  planner container startup (build guard + ros2 launch), both distros
scripts/teleop_relay.py       Foxglove Teleop (Twist) -> twist_mux (TwistStamped) with a silence timeout
justfile, .env                simulation recipes (just cbf-*) + original Husarion simulation recipes
justfile.jazzy, .env.real     physical robot recipes (just -f justfile.jazzy cbf-real-*)
SIM_GUIDE.md, REAL_GUIDE.md   step-by-step guides
```

---

## 8. Known limitations

**Algorithm and PSF**

- The PSF is computed once on a frozen map. Dynamic obstacles are not represented; the CBF term
  only reflects the static map at freeze time.
- With `psf_source: centroid`, `h` depends on obstacle size and centroid placement, not only on
  clearance. For non-convex obstacles (U shapes, rooms, the outer wall) the centroid lies in free
  space, and concave pockets get a comparatively high `h`. `psf_source: constant` avoids this,
  but gives very high `h` in large open areas (it grows with the square of the local width), so
  the `1/h` cost pulls paths towards their middle.
- The clearance columns describe the **planned** path. The robot's real gap to obstacles also
  depends on RPP tracking (corner cutting) and on localisation, which only on-robot or Gazebo
  measurements capture.
- The CBF condition restricts the **approach direction**, not clearance: motion parallel to a wall
  always passes. Goals closer to a wall than `nominal_velocity / kappa` can only be reached
  almost parallel to it.
- The robot body is approximated by a disc of radius `inflation_robot_size`. The default 0.22 m
  (circumscribed radius) is conservative for the rectangular 0.33 × 0.28 m footprint, and
  narrows passages by 0.44 m. A robot parked closer to a wall than this radius cannot start
  planning (`Planning REJECTED: start ... (inflated zone)`).
- Every plan spends the full iteration budget, so there is no continuous replanning. Closed-loop
  replanning would need a time budget or a convergence criterion.
- With the default `seed: -1` the RRT\* generator is seeded from `std::random_device`, so runs
  of the node differ. Set `seed` to a fixed value for repeatable node runs. `cbf_benchmark` is
  always seeded.
- `cbf_benchmark` plans from one fixed start. Drawn goals are reachable through 4-connected
  free cells of the inflated map, which is slightly more conservative than the planner's
  straight edges. Its timings are laptop timings, not robot timings.

**Execution**

- RPP is a differential-drive controller: it never commands lateral velocity, so the mecanum base
  is under-used. `velocity_smoother` also caps `vy` at 0.
- RPP has `use_collision_detection: false` and does not know `h`: nothing in the chain stops the
  robot for an obstacle missing from the frozen map. It also cuts corners at sharp path turns.
- `nominal_velocity`, RPP `desired_linear_vel` and the `velocity_smoother` limit are all 0.6 m/s,
  in simulation and on the robot. This is the cruise speed: RPP still slows down in tight turns
  (`use_regulated_linear_velocity_scaling`), near the goal and while rotating in place, so the
  speed along an edge can be lower than the one assumed by the CBF condition.
- On the robot, slam_toolbox always runs in mapping mode and there is no map loading or AMCL:
  every restart of `navigation` starts a new map with a new origin, so a saved robot map is a
  record and benchmark input, not something the robot can localise in again.
- With `SLAM=False` in simulation, the `static-tf` service publishes an identity `map → odom`,
  i.e. perfect localisation. Fine for planner experiments, not representative of the physical
  robot. The standard Nav2 bringup also starts AMCL in this mode; check with `view_frames` that
  `map → odom` has a single publisher.

**Code and tooling**

- The `Planning SUCCESS` log line reports metrics before smoothing. With `enable_smoothing: true`,
  the CSV records `length`, `mean_h` and the clearance columns after smoothing, but `min_h` is
  still the value of the unsmoothed path (same in `cbf_benchmark`, which shares the smoothing
  code).
- `PlannerParams` struct defaults and the `declare_parameter` defaults are two places holding the
  same numbers; changing a default means editing both.
- `metrics_csv_path` and the PSF export directory are absolute `/workspace/...` paths, valid only
  inside the container.
- The `just cbf-*` recipes are simulation-only (Humble); the robot uses `justfile.jazzy`. The two
  stacks share the source tree but not `build/` / `install/`: `cbf-real-build` wipes artifacts
  built for another architecture or ROS distro.
