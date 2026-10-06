# Safe-CBF-RRT* for ROSbot XL

Implementation and experimental verification of **Safe-CBF-RRT\***: an RRT\* planner whose edge
cost and edge admissibility are driven by a **Poisson Safety Function (PSF)** `h(x,y)` derived
from the occupancy map. The planner runs as a standalone ROS 2 node that feeds paths to the
Nav2 controller of a Husarion ROSbot XL, both in Gazebo and on the physical robot.

This file describes the project: architecture, algorithms, configuration, metrics, repository
layout and limitations. Step-by-step instructions and command references are in two guides:

| Guide | Contents |
|---|---|
| [`SIM_GUIDE.md`](SIM_GUIDE.md) | Gazebo simulation: setup, start-up, `just cbf-*` recipes, map recording, experiments, diagnostics |
| [`REAL_GUIDE.md`](REAL_GUIDE.md) | Physical ROSbot XL: connection, deployment, `justfile.real` recipes, mapping, safety, experiments, troubleshooting |

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

Physical robot (details in `REAL_GUIDE.md`; `.env` on the robot needs `USE_SIM_TIME=False`):

```bash
just -f justfile.real cbf-real-sync ROBOT_IP    # [laptop] deploy the code
just -f justfile.real cbf-real                  # [robot] build + start, follow planner logs
just -f justfile.real cbf-real-refresh-map      # [robot] freeze the map and generate the PSF
just -f justfile.real cbf-real-goal 1.0 0.5     # [robot] send a goal in map coordinates
```

Foxglove runs at `http://localhost:8080` in simulation and at `http://ROBOT_IP:8080` on the robot.

---

## 1. System overview

The stack is a set of Docker containers. In simulation it is `compose.simulation.yaml` +
`compose.cbf.yaml`, driven by `justfile`. On the robot it is `compose.yaml` + `compose.cbf.yaml`,
driven by `justfile.real`.

| Container | Simulation | Physical robot |
|---|---|---|
| `rosbot` | Gazebo simulation of the ROSbot XL; loads a custom `.sdf` world (`dockerfile.rosbot`) | ROSbot XL drivers and `robot_localization` EKF (`/odometry/filtered`, TF `odom → base_link`) |
| `microros` | — | micro-ROS agent connecting the STM32 board (motors, encoders, IMU) |
| `rplidar` | — | RPLidar driver |
| `navigation` | Nav2: `map_server`, AMCL, `controller_server`, `velocity_smoother`, costmaps (slam_toolbox when `SLAM=True`) | the same, with `use_sim_time:=False` |
| `cbf-planner` | **this project's** `planner_node`: PSF generation + RRT\* + `FollowPath` client (`dockerfile.dev`) | the same |
| `foxglove`, `foxglove-ds` | Web visualisation on port `8080`, WebSocket bridge on port `8765` | the same |
| `static-tf` | Identity `map → odom` transform (only valid when `SLAM=False`) | — |

Nav2's own global planner and BT navigator are not used for planning. The planner node replaces
the global planner and talks to `controller_server` directly through the `FollowPath` action.
`planner_server` and `bt_navigator` still run, and **take over** if anything publishes on
`/goal_pose` (e.g. Foxglove's "Publish pose" with default settings). Planner goals go to
`/cbf_goal_pose`.

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
                                        [ velocity_smoother ] --/cmd_vel--> Gazebo base / ROSbot base
```

The robot's pose in the map comes from TF: `map → odom` (static-tf, AMCL or slam_toolbox) composed
with `odom → base_link` (EKF). The planner transforms odometry and goals into `map`.

---

## 3. How the planner works

### 3.1 PSF pipeline (runs in `std::async`)

`PSFGenerator::generate()` executes four stages on a frozen copy of the map:

1. **`ObstacleSegmentation`**: thresholds cells at `occupied_threshold` (unknown cells count as
   occupied), finds connected components (4-connectivity), and classifies every cell as `Free`,
   `ObstacleInterior`, `ObstacleBoundary` (∂Ω_i) or `DomainBoundary` (∂Ω). Each obstacle gets a
   centroid `c_i`, computed from all of its cells.
2. **`PoissonSolver::solveLaplaceVector`**: solves the Laplace equation twice (for `u_x` and
   `u_y`) with Gauss-Seidel + successive over-relaxation (ω = 1.8, tolerance 1e-6, at most
   20 000 sweeps). The Dirichlet condition on obstacle boundaries is
   `u = boundary_scale · (p − c_i)`, i.e. the vector from the obstacle's centroid to the boundary
   cell, so the field points away from each obstacle; `u = 0` on the domain boundary. This is equation (4) of the paper. `c_i` read as a
   scalar `boundary_scale = 1` plus the centroid, and `u = 0` on ∂Ω, are implementation
   assumptions: the paper does not specify them.
3. **`computeJacobianFrobeniusNorm`**: Frobenius norm of the Jacobian of `u`, used as the source
   term of equation (5). The choice of the Frobenius norm is also an assumption.
4. **`solvePoissonScalar`**: solves the Poisson equation `∇²h = −‖∇u‖` for `h`, with `h = 0` on
   both obstacle and domain boundaries. Then `computeGradient` gives `∇h` (central differences).

`getH()` / `getGradientH()` bilinearly interpolate these grids, so the planner can query `h` at
arbitrary continuous coordinates. `exportToCsv()` dumps `h.csv`, `u_x.csv`, `u_y.csv` to
`src/cbf_rrt_planner/maps/psf_debug/` for `tools/visualize_psf.py`.

The PSF is generated **once** and reused for every subsequent goal. It is intentionally frozen:
regenerating it per goal would cost seconds and make results non-reproducible. Generation is
triggered either by the `/refresh_map` service or, once, lazily by the first goal. A 500 ms timer
polls the background task. Each goal compares the live map's origin and size with the frozen
ones and warns `MAP MISMATCH DETECTED` if they differ.

`h` is in m² (`u` is in metres, `∇u` is dimensionless), so it is not a distance. Its value
depends on the size and centroid placement of nearby obstacles, not only on clearance.

### 3.2 RRT\* search

`RrtStarPlanner::plan()` is classic RRT\* (Karaman & Frazzoli) with two CBF hooks:

- **Sampling**: uniform over the map bounds; with probability `goal_bias`, the goal itself.
- **Nearest / neighbours**: `SpatialGrid` spatial hashing, ~O(1) per query instead of O(n).
- **Neighbour radius**: `r_n = gamma · sqrt(log n / n)`, clamped to `[step_size, neighbor_radius]`.
  The shrinking radius is what preserves asymptotic optimality.
- **Edge admissibility** (`isEdgeAdmissible`): geometric collision test (point robot, samples
  every half cell), then the CBF condition of equation (3) sampled every `edge_sample_step` along
  the edge: `∇h · v ≥ −κ·h`, with `v` the unit edge direction times `nominal_velocity`. Only the
  ratio `nominal_velocity / kappa` matters: it is the approach margin in metres (0.2 m with the
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
| `nominal_velocity` | 1.0 | Assumed speed along an edge when evaluating the CBF condition. Not sent to the controller (RPP drives at 0.4 m/s). |
| `occupied_threshold` | 65 | Occupancy value treated as obstacle (0–100); unknown counts as occupied. |
| `enable_smoothing` | false | Greedy shortcutting after the search. |
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
  is the fastest way to sweep `safety_weight_c`.
- `config/` and `launch/` are **symlinked** into `install/` (`colcon build --symlink-install`).
  Editing the YAML or the launch file needs only a planner container restart, not a rebuild.
  Editing C++ needs a rebuild.
- `use_sim_time` comes from the launch file (`USE_SIM_TIME` in the container): `True` in Gazebo,
  `False` on the physical robot.

### 5.2 Controller: `config/nav2_rpp_params.yaml`

Selected by `CONTROLLER` in `.env`. Values that matter for path following (already tuned):

| Parameter | Value | Why |
|---|---|---|
| `odom_topic` | `odometry/filtered` | The Nav2 default `odom` has **no publisher** here; RPP needs a live measurement to ramp its angular command. |
| `lookahead_dist` | 0.6 | Pure-pursuit curvature is `2y/L²`; short lookaheads cause steering oscillation. |
| `rotate_to_heading_min_angle` | 0.785 | Below this heading error, normal steering corrects instead of stopping to rotate. |
| `max_angular_accel` / `rotate_to_heading_angular_vel` | 3.2 / 1.5 | Matched to `velocity_smoother` limits (`max_accel[2]`, `max_velocity[2]`). |
| `movement_time_allowance` | 30.0 | Rotating in place produces no translation, so the progress checker needs slack. |
| `desired_linear_vel` | 0.4 | Cruise speed. |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.1 / 0.3 | Goal checker acceptance. |
| `use_collision_detection` | false | RPP does **not** stop for obstacles in the local costmap. |

`config/nav2_dwb_params.yaml`, `nav2_mppi_params.yaml` and their `*_webots_*` variants come from
the original Husarion project.

### 5.3 Stack: `.env`

| Variable | Simulation | Physical robot | Notes |
|---|---|---|---|
| `USE_SIM_TIME` | unset or `True` | `False` (must be added) | Clock of the planner container. `justfile.real` refuses to build or start unless it is `False`. |
| `SLAM` | `False` | `True` (or `False` with a map recorded on the robot) | `False` uses `maps/map.yaml`. With `True` in simulation, comment out the `static-tf` service. |
| `CONTROLLER` | `rpp` | `rpp` | Picks which `config/nav2_*_params.yaml` is mounted. |
| `MECANUM` | `True` | wheel type | Holonomic base. RPP itself never commands `vy`. |
| `SAVE_MAP_PERIOD` | 15 | — | Map autosave, only when `SLAM=True` (passed to Nav2 in `compose.simulation.yaml`). |
| `LIDAR_BAUDRATE` | — | lidar model | `1000000` S2/S3, `256000` S1/A2M12/A3, `115200` A2M8. |
| `ROBOT_NAMESPACE` | — | Husarnet name | Used by the original `just sync` recipe. |

### 5.4 Maps and worlds

- `maps/map.pgm` + `maps/map.yaml`: pre-recorded static map (0.05 m/pixel, 466 × 466 px,
  origin `[-11.6, -13.6, 0]`) of `worlds/custom_world_3.sdf`. Changing the world invalidates it.
  Recording a new one is described in `SIM_GUIDE.md`.
- With `free_thresh: 0.25`, pixels saved as "unknown" (value 205) are loaded as **free**. Set
  `free_thresh` below 0.196 to keep them unknown, and therefore obstacles for the planner.
- The `map` frame's origin is where the robot stood when the map recording started. Goal
  coordinates are always in this frame, not in Gazebo world coordinates.

---

## 6. Metrics

Each plan appends one line to `planning_metrics.csv`:

| Column | Meaning |
|---|---|
| `run_label` | `cbf_c<value>` or `baseline_rrt_star` |
| `success` | 1 if a path was found |
| `length` | path length [m] |
| `mean_h` | length-weighted mean of `h` along the path |
| `min_h` | smallest `h` along the path, i.e. the tightest point |
| `iterations_used` | always `max_iterations` (no early exit) |
| `iterations_to_first_solution` | iteration that first reached the goal: the paper's "Iterations" |
| `cbf_candidates`, `cbf_rejections`, `rejection_ratio` | edges checked by the CBF condition, edges rejected, their ratio |

Read them carefully:

- `iterations_used` is always `max_iterations`. The informative column is
  `iterations_to_first_solution`.
- `mean_h` is length-weighted; `min_h` is the tightest point on the path. **Both are needed**: a
  high mean can still hide a near-obstacle dip.
- `h` is a Poisson function value in m², not metres. Compare between runs on the same frozen PSF
  rather than converting to distance or comparing across maps.
- The label does not include `kappa` or `nominal_velocity`: note those values yourself when you
  change them.
- Measured effect of `c` on one corridor in simulation (3.81 m straight-line): `c=0.2` → length
  7.34, `mean_h` 2.51, `min_h` 1.18; `c=0.9` → length 6.12, `mean_h` 1.57, `min_h` 0.35.

`python3 src/cbf_rrt_planner/tools/visualize_psf.py [directory]` renders `h`, `u_x` and `u_y`
from the CSV dumps to `psf_visualization.png`.

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
    spatial_grid.hpp          spatial hashing for nearest / radius queries
    rrt_star_types.hpp        PlannerParams, PlanningResult, TreeNode, PlanningFailure
    rrt_star_planner.hpp      the planner itself
    path_smoother.hpp         greedy shortcutting
    cbf_utils.hpp             computeAverageH, computeAverageInverseH, computeMinH, checkCbfCondition
    ros_conversions.hpp       the ONLY ROS<->core boundary: toGridData, toPathMsg
    planning_metrics.hpp      CSV writer
  src/                        implementations + planner_node.cpp (the ROS node)
  config/cbf_rrt_planner.yaml planner parameters (symlinked into install/)
  launch/cbf_planner.launch.py  launch with use_sim_time and params_file arguments
  maps/psf_debug/             PSF field dumps + planning_metrics.csv
  tools/visualize_psf.py      renders psf_visualization.png from the CSV dumps

config/nav2_*_params.yaml     Nav2 configs, one per controller (rpp / mppi / dwb)
config/layout.json            default Foxglove layout (3D, Teleop, Image, RosOut panels)
worlds/custom_world_*.sdf     custom Gazebo worlds
maps/map.pgm, maps/map.yaml   pre-recorded static map
compose.simulation.yaml       Gazebo + Nav2 + Foxglove + static-tf
compose.yaml                  physical ROSbot XL: drivers, micro-ROS, RPLidar, Nav2, Foxglove
compose.cbf.yaml              overlay adding the cbf-planner service to either stack
dockerfile.dev                planner container (ros:humble + nav2_msgs + rclcpp_action)
dockerfile.rosbot             Gazebo container (rosbot-xl-gazebo + nav2_msgs + rclcpp_action)
scripts/cbf_planner_entrypoint.sh  container startup (build guard + ros2 launch)
justfile                      simulation recipes (just cbf-*) + original Husarion recipes
justfile.real                 physical robot recipes (just -f justfile.real cbf-real-*)
SIM_GUIDE.md, REAL_GUIDE.md   step-by-step guides
```

---

## 8. Known limitations

**Algorithm and PSF**

- The PSF is computed once on a frozen map. Dynamic obstacles are not represented; the CBF term
  only reflects the static map at freeze time.
- `h` depends on obstacle size and centroid placement, not only on clearance. For non-convex
  obstacles (U shapes, rooms, the outer wall) the centroid lies in free space, and concave
  pockets get a comparatively high `h`.
- The CBF condition restricts the **approach direction**, not clearance: motion parallel to a wall
  always passes. Goals closer to a wall than `nominal_velocity / kappa` can only be reached
  almost parallel to it.
- The planner treats the robot as a point. The footprint (0.33 × 0.29 m) is not used and
  obstacles are not inflated.
- Every plan spends the full iteration budget, so there is no continuous replanning. Closed-loop
  replanning would need a time budget or a convergence criterion.
- The RRT\* random generator is seeded from `std::random_device`: runs are not repeatable with
  the same seed.

**Execution**

- RPP is a differential-drive controller: it never commands lateral velocity, so the mecanum base
  is under-used. `velocity_smoother` also caps `vy` at 0.
- RPP has `use_collision_detection: false` and does not know `h`: nothing in the chain stops the
  robot for an obstacle missing from the frozen map. It also cuts corners at sharp path turns.
- `nominal_velocity` (1.0 m/s) differs from the actual cruise speed (0.4 m/s).
- With `SLAM=False` in simulation, the `static-tf` service publishes an identity `map → odom`,
  i.e. perfect localisation. Fine for planner experiments, not representative of the physical
  robot. The standard Nav2 bringup also starts AMCL in this mode; check with `view_frames` that
  `map → odom` has a single publisher.

**Code and tooling**

- The `Planning SUCCESS` log line reports metrics before smoothing. With `enable_smoothing: true`,
  the CSV records `length` and `mean_h` after smoothing, but `min_h` is still the value of the
  unsmoothed path.
- `PlannerParams` struct defaults and the `declare_parameter` defaults are two places holding the
  same numbers; changing a default means editing both.
- `metrics_csv_path` and the PSF export directory are absolute `/workspace/...` paths, valid only
  inside the container.
- The `just cbf-*` recipes are simulation-only; the robot uses `justfile.real`.
