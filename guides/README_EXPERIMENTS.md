# Experiments: verification of the research hypotheses

This file describes the experiments of Chapter 5 of the thesis: what each one checks, how it
decides whether a hypothesis holds, and the exact commands to run it. Hypotheses H1-H3 are
verified offline with `cbf_benchmark`, the planner core without ROS. H4 is a timing measurement
on the robot. Two further offline runs give material for the thesis without testing a
hypothesis: the ablation of the two safety mechanisms and the figures of the two safety
functions. The demonstration in Gazebo and on the physical robot, which the main thesis refers
to, is described in §7. It is a demonstration and not a statistical test; a recorder and a plot
script turn it into figures and a small table.

| Hypothesis / purpose | Thesis | Experiment | Run | Analyse | Figure |
|---|---|---|---|---|---|
| H1: `c = 0.8` gives ≥ 0.2 m more footprint clearance for ≤ 10% more length; monotonic in `c` | 5.3 | A (§2) | `just cbf-exp h1h2` | `just cbf-exp-analyze h1` | `h1.png` |
| H2: `constant` source is no worse than `centroid` and ≥ 3× faster | 5.3 | A (same data) | (as above) | `just cbf-exp-analyze h2` | `h2.png` |
| H3: matching the clearance by inflation loses near-obstacle goals | 5.3 | B (§3) | `just cbf-exp h3` | `just cbf-exp-analyze h3` | `h3.png` |
| H4: PSF and planning each < 1 s on the onboard computer | 5.4 | C (§4) | `just -f justfile.jazzy cbf-real-exp-h4` | `python3 .../analyze_experiments.py h4` | - |
| which mechanism gives the clearance: cost term or CBF condition | 5.3.2 | D (§5) | `just cbf-exp ablation` | `just cbf-exp-analyze ablation` | `ablation.png` |
| the two safety functions and the obstacle centroids | 5.2.1 | E (§6) | `just cbf-exp psf` | `just cbf-exp-analyze psf` | `psf/psf_<map>.png` |
| planned vs driven path in Gazebo / on the robot | 5.4.2 | demonstration (§7) | `just cbf-exp-record LABEL` | `just cbf-exp-plot-execution` | `execution/<run>.png` |

Results go to `src/cbf_rrt_planner/benchmark/experiments/` (`h1h2.csv`, `h3.csv`, `h4.csv`,
`ablation.csv`, a `*_paths.csv` with the path vertices for each, `psf/`, `execution/` and the
figures).

---

## 1. Before the first run

### 1.1 Freeze the planner parameters

All experiments read `src/cbf_rrt_planner/config/cbf_rrt_planner.yaml`. Decide on the final
values **before** running anything, and make sure the thesis quotes the same ones. Changing a
value later means re-running every experiment. The values that matter:

| Key | Current value | Meaning |
|---|---|---|
| `step_size` | 1.0 | RRT* step η [m] |
| `max_iterations` | 10000 | RRT* budget, always fully spent |
| `goal_bias` | 0.05 | probability of sampling the goal |
| `goal_tolerance` | 1.0 | radius within which a node may connect to the goal [m] |
| `neighbor_radius`, `gamma` | 5.0, 5.0 | upper bound and constant γ of the rewiring radius |
| `kappa`, `nominal_velocity` | 5.0, 0.6 | CBF condition; only `nominal_velocity / kappa` = 0.12 m matters |
| `inflation_robot_size` | 0.22 | minimal inflation radius [m] |
| `psf_boundary_flux` | 1.0 | slope of `h` at walls, `constant` source |
| `footprint_length`, `footprint_width` | 0.33, 0.28 | rectangle used by the clearance metric [m] |

The experiment scripts override only what the experiment itself varies (`psf_source`, `c`,
`inflation_robot_size`, `inflation`). Every row of the result files records the full setup
(`kappa`, `nominal_velocity`, `step_size`, `gamma`, `goal_bias`, `goal_tolerance`, ...), so a
result file always tells which parameters produced it.

### 1.2 Build and check

The commands run on the laptop from the repository root. The simulation does **not** have to
be running. `just cbf-exp` builds the workspace in the dev container first.

```bash
cd ~/FromZero/rosbot-xl-autonomy-foxglove
just cbf-build
just cbf-benchmark --help        # lists the benchmark options
```

The analysis runs on the host and needs `numpy` and `matplotlib` (the map figures also need
`Pillow`):

```bash
python3 -c "import numpy, matplotlib, PIL"   # otherwise: pip install numpy matplotlib pillow
```

### 1.3 Quick trial run (optional)

A small run checks the whole chain in a few minutes. It writes to a separate directory, so the
real results are not affected:

```bash
GOALS=3 RUNS=2 just cbf-exp h1h2 && just cbf-exp-analyze h1 && just cbf-exp-analyze h2
# then remove it again (the files are created by the container, hence sudo):
sudo rm -rf src/cbf_rrt_planner/benchmark/experiments
```

### 1.4 Shared definitions

- **Map.** By default the simulation map `maps/map.yaml` (`custom_world_3`, 466 × 466 cells,
  0.05 m). Add `--map FILE` after the experiment name to use another one, e.g.
  `just cbf-exp h3 --map maps/map_u.yaml`. Use `--start X,Y` if `(0, 0)` is not free on it.
- **Plan.** One RRT* run: one goal, one seed, one configuration (`c` value, or plain RRT* =
  "baseline"). Every configuration of a batch uses the same goals and the same seeds, so the
  plans can be compared in pairs.
- **Clearance.** `min_footprint_clearance`: the smallest gap between the robot rectangle
  (0.33 × 0.28 m, aligned with the path) and any obstacle of the map **without** inflation, over
  the whole path [m]. It is independent of `h` and of the PSF source, which is why every
  hypothesis uses it.
- **Statistics.** Medians over the successful plans. No statistical tests: each hypothesis has
  a fixed numerical threshold, and the analysis prints the measured value next to it.
- **Fresh files.** `h1h2.csv`, `h3.csv` and `ablation.csv` must not mix two executions. The
  script refuses to start if the file already exists. Move it away (or delete it) to repeat an experiment:
  `sudo mv src/cbf_rrt_planner/benchmark/experiments src/cbf_rrt_planner/benchmark/experiments_old`.

---

## 2. Experiment A: trade-off sweep (H1 and H2)

### 2.1 What it runs

For each PSF source (`constant`, then `centroid`), one batch:

- 20 goals drawn at random among the free cells reachable from the start, at least 2 m away;
- 10 seeds per goal;
- 10 configurations: `c` = 0.1, 0.2, …, 0.9 and plain RRT*;
- the PSF is generated 5 times and the median time is recorded.

That is 2 × 20 × 10 × 10 = 4000 plans, about **35 minutes** on a laptop. Both batches draw the
same goals (the same goal seed on the same map), and plain RRT* gives identical paths in both
because it does not use `h`.

### 2.2 How H1 is decided

For each PSF source separately, from the medians over all plans of a configuration:

1. clearance gain = median clearance at `c = 0.8` − median clearance of plain RRT*. It must be
   **≥ 0.2 m**.
2. length ratio = median length at `c = 0.8` / median length of plain RRT*. It must be
   **≤ 1.10**.
3. monotonicity: going from `c = 0.1` to `c = 0.9`, the median clearance and the median length
   must both **strictly decrease** at every step.

H1 is confirmed if all three hold for both sources. As a cross-check, the analysis also prints
the paired length ratio: the median of `length / length of plain RRT*` over plans with the same
goal and seed.

### 2.3 How H2 is decided

The sources give `h` of different scale, so the same `c` means something different for each.
The two sources are therefore compared along their curves:

1. For each source and each `c`, one point: x = median of the paired length ratio (length /
   length of plain RRT* with the same goal and seed), y = median clearance.
2. For each `centroid` point, the `constant` curve is linearly interpolated at the same x. The
   difference `constant − centroid` is the clearance the `constant` source gives at the same
   length cost. Points outside the x-range of the `constant` curve are skipped.
3. "No worse": every difference is **≥ −0.02 m**. The tolerance is 0.4 grid cells, below the
   resolution of the map. Change it with `--tol`.
4. Speed-up: median PSF time of `centroid` / median PSF time of `constant`. It must be
   **≥ 3.0**. Change it with `--min-speedup`.

H2 is confirmed if both 3 and 4 hold.

### 2.4 Commands

```bash
just cbf-exp h1h2                 # ~35 min, writes experiments/h1h2.csv
just cbf-exp-analyze h1           # table per source, verdict, experiments/h1.png
just cbf-exp-analyze h2           # curve comparison, speed-up, verdict, experiments/h2.png
```

The number of goals and seeds can be changed from the environment, e.g.
`GOALS=30 RUNS=20 just cbf-exp h1h2`. More plans give smoother curves and take proportionally
longer.

### 2.5 Reading the output

- `h1.png`, left: median clearance vs `c`, one line per source. Dashed lines show plain RRT*;
  the triangles at `c = 0.8` mark the required clearance (baseline + 0.2 m). Right: median
  length relative to plain RRT*, with the 1.10 limit.
- `h2.png`: the two trade-off curves (x = length cost, y = clearance), labelled with `c`. A curve
  that lies higher at the same x is better. The black square is plain RRT*.
- If monotonicity fails at one step (two neighbouring `c` values within noise), re-run with
  more seeds (`RUNS=20`). If it still fails, report the step: the trade-off is then monotonic
  only at a coarser resolution of `c`.

---

## 3. Experiment B: inflation vs Safe-CBF-RRT* (H3)

### 3.1 Idea

The usual way to get more clearance from plain RRT* is a larger inflation radius. The experiment
increases the radius until plain RRT* has about the same clearance as Safe-CBF-RRT* with
`c = 0.8`, and then checks how many near-obstacle goals it can still reach. Two goal sets are
used, because each question needs different goals:

| Goal set | Goals | Used for |
|---|---|---|
| `match` | 20 random goals reachable even with the largest radius (drawn on the map inflated by 0.8 m) | clearance at every radius, on the same goals |
| `near` | 20 random goals 0.25-0.40 m from the nearest obstacle (drawn on the map inflated by 0.22 m) | reachability at every radius |

For each set:

- Safe-CBF-RRT* (`c = 0.8`, `constant` source) and plain RRT*, both with the minimal radius
  0.22 m;
- plain RRT* with radius 0.3, 0.4, 0.5, 0.6, 0.7 and 0.8 m.

Each batch has 20 goals × 10 seeds, about **25 minutes** in total. The goals of a set are the same
in every batch (`--goal-inflation` fixes the map they are drawn on).

### 3.2 How H3 is decided

1. Target clearance = median clearance of Safe-CBF-RRT* on the `match` goals.
2. Comparable radius = the smallest radius at which the median clearance of plain RRT* on the
   `match` goals reaches the target.
3. Lost fraction = share of plain RRT* plans on the `near` goals that fail with this radius
   (`goal_blocked`: the goal itself is inside the inflated obstacles; `no_path`: the goal is free
   but the passages to it are closed).
4. H3 is confirmed if the lost fraction is **≥ 25%** ("significant", change with
   `--significant`) **and** Safe-CBF-RRT* succeeds in **100%** of the `near` plans.

If no radius up to 0.8 m reaches the target, the analysis says so (`NOT VERIFIED`). Then extend
the radius list in `src/cbf_rrt_planner/tools/run_experiment.sh` and the `--goal-inflation`
value of the `match` set (the start must stay free, see §8.4).

### 3.3 Commands

```bash
just cbf-exp h3                   # ~25 min, writes experiments/h3.csv
just cbf-exp-analyze h3           # table per radius, verdict, experiments/h3.png
```

### 3.4 Reading the output

- The table lists, for every radius, the median clearance on the `match` goals, the share of
  `near` plans that reached the goal, and the number of `goal_blocked` / `no_path` failures.
- `h3.png`, left: clearance of plain RRT* vs radius, with Safe-CBF-RRT* as a horizontal line.
  Right: reachability of the `near` goals vs radius. The dotted vertical line is the
  comparable radius.
- A goal closer to a wall than the inflation radius is unreachable by definition, so for the
  `near` set H3 follows from the construction once the comparable radius exceeds 0.40 m. This is
  the mechanism the hypothesis describes, not an artefact; present it in the thesis as such
  ("by definition"), with the measured comparable radius as the actual result. The `no_path`
  column shows the less obvious effect: passages that close although the goal is free.
- The clearance of Safe-CBF-RRT* is measured on open goals only. On the `near` goals the
  minimum clearance is set by the goal itself and says nothing about the rest of the path.

---

## 4. Experiment C: computation time (H4)

### 4.1 What it runs

One batch with the `constant` source and `c = 0.8`: the PSF is generated 10 times (median
recorded), then 10 goals × 3 seeds are planned with the full 10 000 iterations. The plan time
covers the RRT* search (and smoothing, if enabled), not the PSF.

H4 is confirmed on a machine if the median PSF time **and** the median plan time are both
**< 1 s**. The maximum plan time is printed as well.

The PSF time grows with the number of map cells and with the iterations the SOR solver needs,
so it depends on the map more than on the machine. State the map size (cells and resolution)
next to the result in the thesis; the robot's experiment map is the one that counts for H4.

### 4.2 Reference run on the laptop (simulation map)

```bash
just cbf-exp h4                   # under a minute, tag "laptop"
just cbf-exp-analyze h4
```

For comparison the same measurement with the `centroid` source (not part of H4; the analysis
shows it as a separate block, one per machine and source):

```bash
PSF=centroid just cbf-exp h4      # on the robot: PSF=centroid just -f justfile.jazzy cbf-real-exp-h4
```

`--set psf_source=...` after the experiment name has no effect: the script sets the source
itself, after the extra options.

### 4.3 Run on the robot (experiment map)

1. Sync the code from the laptop and log in (see `REAL_GUIDE.md` for the first setup):

   ```bash
   just -f justfile.jazzy cbf-real-sync ROBOT_IP
   ssh husarion@ROBOT_IP
   cd ~/rosbot-xl-autonomy-foxglove
   ```

2. Start the stack as for the real experiments, so the measurement includes the load of SLAM,
   Nav2 and the bridges (`Ctrl-C` only stops following the logs):

   ```bash
   just -f justfile.jazzy cbf-real
   ```

3. Map the experiment area and save the map. Skip this if `maps/map.yaml` on the robot is
   already the experiment map:

   ```bash
   just -f justfile.jazzy cbf-real-save-map
   ```

   Set `free_thresh: 0.19` in `maps/map.yaml`, so unknown space counts as an obstacle as it does
   for the planner (`REAL_GUIDE.md` §7).

4. Run the timing experiment on the robot. It builds the planner if needed and writes
   `src/cbf_rrt_planner/benchmark/experiments/h4.csv` with the tag `robot`:

   ```bash
   just -f justfile.jazzy cbf-real-exp-h4
   # another map file:            just -f justfile.jazzy cbf-real-exp-h4 maps/other/map.yaml
   # start (0,0) not free there:  just -f justfile.jazzy cbf-real-exp-h4 maps/map.yaml --start 1.0,0.5
   ```

   `(0, 0)` is where SLAM started, i.e. where the robot stood, which is normally free.

5. Read the verdict directly on the robot (only the Python standard library is needed):

   ```bash
   python3 src/cbf_rrt_planner/tools/analyze_experiments.py h4
   ```

6. Copy the result to the laptop **before the next `cbf-real-sync`**. The sync mirrors the
   laptop's files with `--delete`, so it would remove or overwrite the robot's `h4.csv`. On the
   laptop, append the robot rows to the laptop file, so one analysis shows both machines:

   ```bash
   scp husarion@ROBOT_IP:/home/husarion/rosbot-xl-autonomy-foxglove/src/cbf_rrt_planner/benchmark/experiments/h4.csv /tmp/h4_robot.csv
   tail -n +2 /tmp/h4_robot.csv | sudo tee -a src/cbf_rrt_planner/benchmark/experiments/h4.csv > /dev/null
   just cbf-exp-analyze h4           # one block for "laptop", one for "robot"
   ```

   Copy the map as well (`scp ...maps/map.{yaml,pgm} maps/robot/`), so it can be shown in the
   thesis next to the timing.

---

## 5. Experiment D: ablation of the two safety mechanisms (thesis 5.3.2)

### 5.1 Idea

Safe-CBF-RRT* has two mechanisms that can push a path away from obstacles: the **cost term**
(edge cost `L · (c + (1 − c) · mean(1/h))`, active for `c < 1`) and the **CBF condition**
(an edge is rejected if `∇h · v < −κ h`). The ablation switches them on one at a time, with
everything else equal, to show which of them produces the clearance:

| Variant | `c` | `κ` | Cost term | CBF condition |
|---|---|---|---|---|
| plain RRT* | - | - | off | off (measured only) |
| CBF condition only | 1.0 | 5 | off (`c = 1` gives the plain length cost) | on |
| cost term only | 0.8 | 10⁶ | on | practically off (`v / κ` ≈ 0, no edge is rejected) |
| Safe-CBF-RRT* | 0.8 | 5 | on | on |

It is not a hypothesis test and has no verdict. The table and the figure are the result.

### 5.2 What it runs

`constant` source, inflation 0.22 m, two goal sets:

- `open`: 20 random goals, drawn exactly as in experiment A (the same goals as there);
- `near`: 20 random goals 0.25-0.40 m from the nearest obstacle, as in experiment B.

For each set two batches with the same goals and seeds: `--c 0.8,1.0 --baseline` (plain RRT*,
CBF condition only, Safe-CBF-RRT*) and `--c 0.8 --set kappa=1e6` (cost term only). In total
2 × 20 × 10 × 4 = 1600 plans, about **20 minutes**.

### 5.3 Commands

```bash
just cbf-exp ablation             # ~20 min, writes experiments/ablation.csv
just cbf-exp-analyze ablation     # tables per goal set, experiments/ablation.png
```

### 5.4 Reading the output

For each goal set and variant the table gives: success rate, median footprint clearance, median
length, median paired length ratio (to plain RRT* with the same goal and seed), median share of
rejected edges, and the share of plans whose path is identical to the plain RRT* path (`= RRT*`)
or to the Safe-CBF-RRT* path (`= full`) for the same goal and seed. Two lines below the table
give the clearance each mechanism adds on top of the other:

- `full − CBF only`: what the cost term adds;
- `full − cost only`: what the CBF condition adds.

How to read it:

- Same path = same length to 10⁻⁶ m. A variant that gives the same path as another one in most
  plans has no effect of its own.
- With the current parameters `v / κ = 0.12 m` is small compared to the clearance the cost term
  produces, so the CBF condition rejects only about 1-2% of the edges. A small trial run gave
  `cost only` = `full` in practically every plan, i.e. the clearance comes from the cost term and
  the CBF condition acts as a constraint that is rarely active. State this plainly in the thesis.
  It does not make the condition useless: it guarantees the property it was designed for, it
  just does not shape the path.
- `rejected` for plain RRT* is measured (what the condition would have rejected), not enforced.
  For `cost term only` it is 0 by construction.
- On the `near` goals the minimum clearance is limited by the goal itself (the robot has to stand
  0.25-0.40 m from a wall), so the clearance differences there are small. Use the `open` set for
  the clearance comparison and the `near` set to show that no variant loses goals.

---

## 6. Experiment E: figures of the two safety functions (thesis 5.2.1)

### 6.1 What it runs

For `maps/map.yaml` and `maps/map_u.yaml`, and for both PSF sources, the benchmark generates the
PSF with the planning setup of the YAML (inflation 0.22 m), plans nothing (`--random-goals 0`),
and exports with `--export-psf DIR`:

- `h.csv`, `u_x.csv`, `u_y.csv`: the fields on the planning grid (row 0 = bottom of the map; for
  the `constant` source `u` is zero);
- `centroids.csv`: per segmented obstacle its centroid, number of cells and whether the centroid
  lies in free space (`in_free_space`). This is what the `centroid` source uses as point
  sources, and why it behaves badly on U-shaped or wall-like obstacles.

### 6.2 Commands

```bash
just cbf-exp psf                  # seconds, writes experiments/psf/<map>_<source>/
just cbf-exp-analyze psf          # experiments/psf/psf_map.png, psf_map_u.png
```

Another map: `just cbf-exp psf` runs both maps of the repository; for a single other map run the
benchmark directly, e.g.
`just cbf-benchmark --map maps/robot/map.yaml --set psf_source=centroid --random-goals 0 --export-psf src/cbf_rrt_planner/benchmark/experiments/psf/map_centroid`
(and the same with `constant` and `map_constant`), then
`just cbf-exp-analyze psf --maps-dir maps/robot`. The directory names must be
`<map yaml name>_centroid` and `<map yaml name>_constant`.

### 6.3 Reading the output

One figure per map, three panels:

1. `centroid` source, `h` [m²], with the obstacle centroids: red dot = centroid in free space,
   white cross = centroid inside the obstacle.
2. `centroid` source, the guidance field `u`: arrows show the direction, the colour its
   magnitude. Where a centroid lies in free space, `u` is organised around that point instead of
   following the boundary of the obstacle.
3. `constant` source, `h` [m].

Inflated obstacles (where `h = 0`) are light grey, the real obstacles black. The analysis prints
how many centroids lie in free space. On `map_u` every obstacle is U-shaped or a long wall, so
all centroids fall outside the obstacles. On `map.yaml` none do. This contrast is the argument
for the `constant` source.

---

## 7. Demonstration in Gazebo and on the robot (main thesis, thesis 5.4.2)

The main thesis states that the extra clearance matters when a real controller follows the path.
This is shown, not tested statistically: a few runs, recorded and compared.

### 7.1 What is recorded and how

`tools/record_execution.py` runs in the planner container next to `planner_node`:

- every new `/planned_path` starts a run `n` and is saved as `<label>_<n>_plan.csv`;
- while the `FollowPath` goal is executing (`/follow_path/_action/status`), the TF
  `map → base_link` is sampled at 20 Hz into `<label>_<n>_driven.csv` (`t, x, y, yaw`).

Files go to `src/cbf_rrt_planner/benchmark/experiments/execution/`. Numbering continues after a
restart of the recorder, so earlier runs are never overwritten. Use one label per
configuration, e.g. `baseline` and `cbf`.

`tools/plot_execution.py` (host) then gives for every run a figure over the map (plan, driven
trajectory, robot footprint with the **recorded** heading every 0.3 m, the pose with the
smallest clearance in red) and a row of `execution/summary.csv`:

| Column | Meaning |
|---|---|
| `plan_fp_clearance` | min footprint clearance of the plan (footprint along the path tangent), the same metric as in experiments A-D |
| `driven_fp_clearance` | the same for the driven poses with their recorded heading |
| `max_dev`, `mean_dev` | distance of the driven positions from the plan polyline [m] |
| `goal_error` | distance of the last driven position from the end of the plan [m] |
| `plan_length`, `driven_length`, `duration` | [m], [m], [s] |

The clearance is computed by a Python port of the C++ metric (map without inflation, unknown
cells count as obstacles). On benchmark paths it agrees with `min_footprint_clearance` of
`cbf_benchmark` to 0.1 mm.

**Caveat.** The driven trajectory is the robot's own pose estimate (TF), not ground truth. In
Gazebo, with the static map, `map → odom` is fixed, so the trajectory is the EKF odometry; on the
robot it is the localisation estimate. The deviation therefore measures how well the
controller tracked the plan, not the localisation error, and `driven_fp_clearance` is the
clearance the robot *believed* it had. Real contact must be observed (Gazebo: visible
collision; robot: observation or video). Say this in the thesis next to the figures.

### 7.2 Protocol (identical in simulation and on the robot)

1. Choose 2-3 goals where plain RRT* passes close to a corner, e.g. from the path plots of
   experiment A (`just cbf-benchmark-plot`, see `README.md` §6.3) or from the planned paths in
   Foxglove.
2. For each goal, plan and drive it with plain RRT* (`enable_cbf false`, label `baseline`) and
   with Safe-CBF-RRT* (`enable_cbf true`, `safety_weight_c 0.8`, label `cbf`), 3 times each,
   returning the robot to the same start in between. Set `seed` to a fixed value, so the same
   configuration gives the same plan from the same start.
3. Report per run: goal reached yes/no, contact yes/no, the row of `summary.csv`, and the figure.

### 7.3 Commands in the simulation

The simulation map is `MAP_YAML` of `.env` (currently `map_u.yaml`).

```bash
just cbf-up                       # terminal 1: whole stack, follows the logs
just cbf-refresh-map              # terminal 2: once the map is up; wait for "PSF ready" in the logs

# terminal 3: the recorder, one label per configuration; Ctrl-C to stop
just cbf-exp-record baseline

# terminal 2: configuration + goal
docker exec rosbot-xl-autonomy-foxglove-cbf-planner-1 bash -c 'source /opt/ros/humble/setup.bash && \
  ros2 param set /planner_node enable_cbf false && ros2 param set /planner_node seed 7'
just cbf-goal -6.98 -3.38         # wait until the robot arrives, drive it back, repeat
```

Then stop the recorder, start it with the other label and switch the planner:

```bash
just cbf-exp-record cbf           # terminal 3
docker exec rosbot-xl-autonomy-foxglove-cbf-planner-1 bash -c 'source /opt/ros/humble/setup.bash && \
  ros2 param set /planner_node enable_cbf true && ros2 param set /planner_node safety_weight_c 0.8'
just cbf-goal -6.98 -3.38
```

Analysis on the host:

```bash
just cbf-exp-plot-execution maps/map_u.yaml          # all runs; add --label cbf for one label
```

### 7.4 Commands on the robot

The same with the robot recipes (`REAL_GUIDE.md` §8): `just -f justfile.jazzy cbf-real`,
`cbf-real-refresh-map`, parameters as in `REAL_GUIDE.md` §8.2, goals with `cbf-real-goal X Y`,
and the recorder (on the robot, in a second SSH session):

```bash
just -f justfile.jazzy cbf-real-record baseline      # use_sim_time false; Ctrl-C to stop
```

Copy the runs and the map to the laptop **before the next `cbf-real-sync`** (it mirrors the
laptop with `--delete`), then plot there:

```bash
mkdir -p maps/robot src/cbf_rrt_planner/benchmark/experiments/execution_robot
scp husarion@ROBOT_IP:/home/husarion/rosbot-xl-autonomy-foxglove/maps/map.{yaml,pgm} maps/robot/
scp 'husarion@ROBOT_IP:/home/husarion/rosbot-xl-autonomy-foxglove/src/cbf_rrt_planner/benchmark/experiments/execution/*' \
    src/cbf_rrt_planner/benchmark/experiments/execution_robot/
just cbf-exp-plot-execution maps/robot/map.yaml src/cbf_rrt_planner/benchmark/experiments/execution_robot
```

---

## 8. Reference

### 8.1 Files

| File | Role |
|---|---|
| `src/cbf_rrt_planner/src/cbf_benchmark.cpp` | offline benchmark (`just cbf-benchmark --help`) |
| `src/cbf_rrt_planner/tools/run_experiment.sh` | the batches of each experiment: `c` values, radii, goal sets, numbers of goals and seeds |
| `src/cbf_rrt_planner/tools/analyze_experiments.py` | verdicts, tables and figures (`--help` lists the thresholds) |
| `src/cbf_rrt_planner/tools/record_execution.py` | recorder of plan + driven trajectory (ROS 2 node, runs in the container) |
| `src/cbf_rrt_planner/tools/plot_execution.py` | figures and `summary.csv` of the recorded runs (host) |
| `src/cbf_rrt_planner/benchmark/experiments/` | results and figures; `psf/` and `execution/` subdirectories |

### 8.2 Benchmark options used by the experiments

| Option | Effect |
|---|---|
| `--c LIST`, `--baseline`, `--baseline-only` | configurations of a batch: Safe-CBF-RRT* for each `c`, plus or only plain RRT* |
| `--random-goals N`, `--runs N` | goals per batch, seeds per goal and configuration |
| `--goal-inflation R` | draw the goals on the map inflated by `R` instead of the planning radius, so batches with different radii get the same goals |
| `--goal-clearance MIN,MAX` | keep only goals whose distance to the nearest obstacle (map without inflation) is in `[MIN, MAX]` |
| `--psf-repeats N` | generate the PSF `N` times, record the median time |
| `--export-psf DIR` | write `h`, `u` and the obstacle centroids to `DIR` (experiment E) |
| `--set KEY=VALUE` | override one planner parameter for this batch (`kappa=1e6` in experiment D) |
| `--tag TEXT` | label stored in every row (`match` / `near` in H3, the machine in H4, `open` / `near` in D) |

### 8.3 Changing an experiment

- Other `c` values, radii or goal bands: edit `run_experiment.sh`. The analysis reads whatever
  values are in the CSV.
- Other thresholds: options of the analysis, e.g.
  `just cbf-exp-analyze h1 --min-gain 0.25 --max-length 1.05`,
  `just cbf-exp-analyze h2 --tol 0.03 --min-speedup 2.5`,
  `just cbf-exp-analyze h3 --significant 0.5`,
  `just cbf-exp-analyze h4 --limit 0.5`.
  Thresholds that differ from the hypotheses in the thesis must be changed in the thesis too.
- Another result file: pass it after the hypothesis, e.g.
  `just cbf-exp-analyze h3 path/to/h3.csv`. The figure is written next to it.

### 8.4 Troubleshooting

| Message | Cause and fix |
|---|---|
| `error: .../h1h2.csv already exists` | a previous execution; move the `experiments/` directory away (§1.4) |
| `goal set 'near': no rows for [...]` (ablation) | `ablation.csv` is incomplete (interrupted run); move it away and re-run |
| recorder prints nothing after `plan ... saved` | `FollowPath` never started: controller_server not running or the goal was rejected (planner logs) |
| recorder: `no TF map -> base_link` | localisation not running or wrong `use_sim_time`; check with `cbf-real-check` / `tf2_echo map base_link` |
| `error: start (...) is within the inflation radius` | the start is not free on this map or radius; pass `--start X,Y` |
| `the start is not free on the map the goals are drawn on` | `--goal-inflation` is too large for the start; lower it or move the start |
| `warning: only N reachable goal cells ...` | fewer goals than requested exist in the band; widen `--goal-clearance` or lower `--min-goal-dist` |
| `NOT VERIFIED` in H3 | no radius of the sweep reaches the clearance of Safe-CBF-RRT*; extend the sweep (§3.2) |
