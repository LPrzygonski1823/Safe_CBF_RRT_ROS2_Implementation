# Experiments: verification of the research hypotheses

This file describes the experiments of Chapter 5 of the thesis: what each one checks, how it
decides whether a hypothesis holds, and the exact commands to run it. Hypotheses H1-H3 are
verified offline with `cbf_benchmark`, the planner core without ROS. H4 is a timing measurement
on the robot. The demonstration in Gazebo and on the physical robot, which the main thesis
refers to, is described at the end. It is a demonstration and not a statistical test.

| Hypothesis | Experiment | Run | Analyse | Figure |
|---|---|---|---|---|
| H1: `c = 0.8` gives ≥ 0.2 m more footprint clearance for ≤ 10% more length; monotonic in `c` | A | `just cbf-exp h1h2` | `just cbf-exp-analyze h1` | `h1.png` |
| H2: `constant` source is no worse than `centroid` and ≥ 3× faster | A (same data) | (as above) | `just cbf-exp-analyze h2` | `h2.png` |
| H3: matching the clearance by inflation loses near-obstacle goals | B | `just cbf-exp h3` | `just cbf-exp-analyze h3` | `h3.png` |
| H4: PSF and planning each < 1 s on the onboard computer | C | `just -f justfile.jazzy cbf-real-exp-h4` | `python3 .../analyze_experiments.py h4` | - |

Results go to `src/cbf_rrt_planner/benchmark/experiments/` (`h1h2.csv`, `h3.csv`, `h4.csv`, a
`*_paths.csv` with the path vertices for each, and the figures).

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

The analysis runs on the host and needs `numpy` and `matplotlib`:

```bash
python3 -c "import numpy, matplotlib"   # otherwise: pip install numpy matplotlib
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
- **Fresh files.** `h1h2.csv` and `h3.csv` must not mix two executions. The script refuses to
  start if the file already exists. Move it away (or delete it) to repeat an experiment:
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
value of the `match` set (the start must stay free, see §5).

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
- A goal closer to a wall than the inflation radius is unreachable by definition. This is the
  mechanism the hypothesis describes, not an artefact; say so in the thesis. The `no_path`
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

The PSF time grows with the number of map cells and with the iterations the SOR solver needs. On
a laptop the 466 × 466 simulation map takes about 1.2 s. The result on the robot depends on the
size of the robot's map, so state the map size next to the result in the thesis.

### 4.2 Reference run on the laptop (simulation map)

```bash
just cbf-exp h4                   # under a minute, tag "laptop"
just cbf-exp-analyze h4
```

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

## 5. Demonstration in Gazebo and on the robot (main thesis)

The main thesis states that the extra clearance matters when a real controller follows the path.
This is shown, not tested statistically: a few runs, recorded and compared visually.

Suggested protocol, identical in simulation and on the robot:

1. Choose 2-3 goals where plain RRT* passes close to a corner, e.g. by looking at the path plots
   of experiment A (`just cbf-benchmark-plot`, see `README.md` §6.3), or at the planned paths in
   Foxglove.
2. For each goal, plan and drive it with plain RRT* (`enable_cbf false`) and with Safe-CBF-RRT*
   (`enable_cbf true`, `safety_weight_c 0.8`), 3 times each, returning the robot to the same
   start in between. Set `seed` to a fixed value so the same configuration gives the same plan.
3. Record the planned path (`/planned_path`) and the driven trajectory (TF `map → base_link` or
   `/odometry/filtered`) in Foxglove, or with `ros2 bag record`.
4. Report, per run: goal reached yes/no, contact with an obstacle yes/no (Gazebo: visible
   collision; robot: observation), and a figure with the planned and driven paths over the map.

Commands:

- simulation: `SIM_GUIDE.md` §7.2 (start with `just cbf-up`, freeze the map with
  `just cbf-refresh-map`, change parameters with `ros2 param set /planner_node ...`, send goals
  with `just cbf-goal X Y`);
- robot: `REAL_GUIDE.md` §8 (`just -f justfile.jazzy cbf-real`, `cbf-real-refresh-map`,
  `cbf-real-goal X Y`, parameters in §8.2).

---

## 6. Reference

### 6.1 Files

| File | Role |
|---|---|
| `src/cbf_rrt_planner/src/cbf_benchmark.cpp` | offline benchmark (`just cbf-benchmark --help`) |
| `src/cbf_rrt_planner/tools/run_experiment.sh` | the batches of each experiment: `c` values, radii, goal sets, numbers of goals and seeds |
| `src/cbf_rrt_planner/tools/analyze_experiments.py` | verdicts, tables and figures (`--help` lists the thresholds) |
| `src/cbf_rrt_planner/benchmark/experiments/` | results and figures |

### 6.2 Benchmark options used by the experiments

| Option | Effect |
|---|---|
| `--c LIST`, `--baseline`, `--baseline-only` | configurations of a batch: Safe-CBF-RRT* for each `c`, plus or only plain RRT* |
| `--random-goals N`, `--runs N` | goals per batch, seeds per goal and configuration |
| `--goal-inflation R` | draw the goals on the map inflated by `R` instead of the planning radius, so batches with different radii get the same goals |
| `--goal-clearance MIN,MAX` | keep only goals whose distance to the nearest obstacle (map without inflation) is in `[MIN, MAX]` |
| `--psf-repeats N` | generate the PSF `N` times, record the median time |
| `--set KEY=VALUE` | override one planner parameter for this batch |
| `--tag TEXT` | label stored in every row (`match` / `near` in H3, the machine in H4) |

### 6.3 Changing an experiment

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

### 6.4 Troubleshooting

| Message | Cause and fix |
|---|---|
| `error: .../h1h2.csv already exists` | a previous execution; move the `experiments/` directory away (§1.4) |
| `error: start (...) is within the inflation radius` | the start is not free on this map or radius; pass `--start X,Y` |
| `the start is not free on the map the goals are drawn on` | `--goal-inflation` is too large for the start; lower it or move the start |
| `warning: only N reachable goal cells ...` | fewer goals than requested exist in the band; widen `--goal-clearance` or lower `--min-goal-dist` |
| `NOT VERIFIED` in H3 | no radius of the sweep reaches the clearance of Safe-CBF-RRT*; extend the sweep (§3.2) |
